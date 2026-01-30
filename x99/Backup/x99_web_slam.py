#!/usr/bin/env python3
"""
X99 Web Interface for SLAM + Camera Streams
View camera feeds and SLAM data in browser
"""

from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
import json
import threading
import time
import base64
from io import BytesIO
from PIL import Image

from x99_headless import OptimizedCameraReceiver
from x99.Backup.stereo_depth_mapping import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance
from persistent_map import PersistentMap

app = Flask(__name__)
app.config['SECRET_KEY'] = 'slam_web_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global server instance
web_server = None

class SLAMWebServer:
    """Web server with camera streaming and SLAM visualization"""
    
    def __init__(self, left_port=9001, right_port=9002):
        self.left_receiver = OptimizedCameraReceiver(left_port, "LEFT")
        self.right_receiver = OptimizedCameraReceiver(right_port, "RIGHT")
        
        # SLAM components
        try:
            from x99_slam_server import ORBFeatureExtractor, YOLOSegmentator
            self.orb_extractor = ORBFeatureExtractor(n_features=3000)
            self.yolo = YOLOSegmentator()
            self.has_slam = True
            print("[Web] SLAM components loaded")
        except ImportError as e:
            print(f"[Web] SLAM not available: {e}")
            self.orb_extractor = None
            self.yolo = None
            self.has_slam = False
        
        # Depth mapping components
        self.depth_mapper = StereoDepthMapper(baseline=0.275, focal_length=500)
        self.grid_mapper = OccupancyGridMapper(grid_size=400, resolution=0.05, max_range=5.0)
        self.avoidance = ObstacleAvoidance(safety_distance=0.5, max_linear_vel=0.3)
        
        # Persistent map builder
        self.persistent_map = PersistentMap(grid_size=800, resolution=0.02, voxel_size=0.05)
        
        self.is_running = False
        self.streaming = False
        
        # Latest depth and grid
        self.latest_depth = None
        self.latest_grid = None
        self.depth_lock = threading.Lock()
        
        # Robot pose estimation (simplified)
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, z
        
        # Stats
        self.stats = {
            'left_fps': 0.0,
            'right_fps': 0.0,
            'left_frames': 0,
            'right_frames': 0,
            'orb_features': 0,
            'matches': 0,
            'uptime': 0,
            'map_points': 0,
            'obstacles_detected': 0,
            'linear_vel': 0.0,
            'angular_vel': 0.0,
            'persistent_map_points': 0,
            'trajectory_length': 0
        }
        
        self.start_time = None
    
    def start_receivers(self):
        """Start camera receivers"""
        print("\n" + "=" * 70)
        print("  X99 SLAM Web Interface")
        print("=" * 70)
        
        # Show IPs
        import subprocess
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            ips = result.stdout.strip().split()
            if ips:
                print(f"X99 IPs: {', '.join(ips)}")
                print(f"Web Interface: http://{ips[0]}:5000")
        except:
            pass
        
        print(f"Camera ports: {self.left_receiver.port}, {self.right_receiver.port}")
        print("=" * 70 + "\n")
        
        # Start receivers
        left_thread = threading.Thread(
            target=lambda: (
                self.left_receiver.start_server() and
                self.left_receiver.receive_frames()
            ),
            daemon=True
        )
        
        right_thread = threading.Thread(
            target=lambda: (
                self.right_receiver.start_server() and
                self.right_receiver.receive_frames()
            ),
            daemon=True
        )
        
        left_thread.start()
        right_thread.start()
        
        # Wait for connections
        print("[WAITING] For Jetson camera connections...")
        
        for i in range(60):
            time.sleep(1)
            if self.left_receiver.is_running and self.right_receiver.is_running:
                break
            
            if i % 5 == 0 and i > 0:
                left_ok = "✓" if self.left_receiver.is_running else "⏳"
                right_ok = "✓" if self.right_receiver.is_running else "⏳"
                print(f"  {i}s - Left: {left_ok} Right: {right_ok}")
        
        if not self.left_receiver.is_running or not self.right_receiver.is_running:
            print("\n[ERROR] Camera connection timeout")
            return False
        
        print("\n[OK] Cameras connected!\n")
        return True
    
    def process_slam(self, frame_left, frame_right):
        """Process SLAM and depth mapping on frames"""
        
        # Compute depth map
        try:
            disparity = self.depth_mapper.compute_disparity(frame_left, frame_right, use_wls=False)
            depth = self.depth_mapper.disparity_to_depth(disparity)
            
            # Create 3D point cloud
            points, colors = self.depth_mapper.depth_to_point_cloud(depth, frame_left)
            
            if len(points) > 100:
                # Add to persistent map - CRITICAL: Accumulate points for SLAM map
                self.persistent_map.add_point_cloud(points, colors, self.robot_pose)
                
                # Update occupancy grid
                self.grid_mapper.update_from_point_cloud(points)
                self.grid_mapper.inflate_obstacles(radius=3)
                
                # Get obstacle scan
                scan = self.grid_mapper.get_obstacle_scan(num_rays=360)
                
                # Compute avoidance velocity
                linear_vel, angular_vel = self.avoidance.compute_velocity(scan)
                
                # Update robot pose (simplified - in real system use odometry/SLAM)
                dt = 0.1  # Assume 10Hz update
                self.robot_pose[0] += linear_vel * np.cos(self.robot_pose[2]) * dt
                self.robot_pose[1] += 0  # Height stays same
                self.robot_pose[2] += angular_vel * dt
                
                # Update stats
                self.stats['map_points'] = len(points)
                self.stats['obstacles_detected'] = np.sum(self.grid_mapper.grid == 100)
                self.stats['linear_vel'] = round(linear_vel, 2)
                self.stats['angular_vel'] = round(angular_vel, 2)
                self.stats['persistent_map_points'] = len(self.persistent_map.voxel_grid)
                self.stats['trajectory_length'] = len(self.persistent_map.trajectory)
            
            # Store latest depth and grid
            with self.depth_lock:
                self.latest_depth = depth
                # Get persistent 2D map instead of temporary one
                self.latest_grid = self.persistent_map.visualize_2d(show_trajectory=True)
                
        except Exception as e:
            print(f"[Web] Depth mapping error: {e}")
            import traceback
            traceback.print_exc()
        
        # ORB-SLAM processing
        if not self.has_slam or self.orb_extractor is None:
            return frame_left, 0, 0, 0
        
        try:
            # Extract features
            kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
            kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
            
            # Match features
            matches = self.orb_extractor.match_features(desc_left, desc_right)
            
            # YOLO segmentation (optional)
            processed = frame_left.copy()
            if self.yolo and self.yolo.model:
                try:
                    results = self.yolo.segment(frame_left, conf=0.5)
                    if results:
                        processed = self.yolo.draw_segments(frame_left, results)
                except Exception as e:
                    print(f"[Web] YOLO error: {e}")
            
            # Draw ORB features
            frame_slam = cv2.drawKeypoints(
                processed, kp_left, None,
                color=(0, 255, 0),
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )
            
            # Update stats
            self.stats['orb_features'] = len(kp_left)
            self.stats['matches'] = len(matches)
            
            return frame_slam, len(kp_left), len(kp_right), len(matches)
            
        except Exception as e:
            print(f"[Web] ORB processing error: {e}")
            import traceback
            traceback.print_exc()
            return frame_left, 0, 0, 0
    
    def generate_left_stream(self):
        """Generate left camera MJPEG stream"""
        while self.is_running and self.streaming:
            if self.left_receiver.latest_frame is not None:
                frame = self.left_receiver.latest_frame.copy()
                
                # Add info overlay
                cv2.putText(frame, f"LEFT CAM - FPS: {self.stats['left_fps']:.1f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Encode to JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)  # ~30 FPS
    
    def generate_right_stream(self):
        """Generate right camera MJPEG stream"""
        while self.is_running and self.streaming:
            if self.right_receiver.latest_frame is not None:
                frame = self.right_receiver.latest_frame.copy()
                
                # Add info overlay
                cv2.putText(frame, f"RIGHT CAM - FPS: {self.stats['right_fps']:.1f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Encode to JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)
    
    def generate_slam_stream(self):
        """Generate SLAM-processed MJPEG stream"""
        while self.is_running and self.streaming:
            if self.left_receiver.latest_frame is not None and self.right_receiver.latest_frame is not None:
                # Process SLAM
                frame_slam, n_kp_left, n_kp_right, n_matches = self.process_slam(
                    self.left_receiver.latest_frame,
                    self.right_receiver.latest_frame
                )
                
                # Add info overlay
                cv2.putText(frame_slam, f"SLAM - Features: {n_kp_left} Matches: {n_matches}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame_slam, f"Map Points: {self.stats['persistent_map_points']}",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Encode to JPEG
                _, buffer = cv2.imencode('.jpg', frame_slam, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)
    
    def generate_depth_stream(self):
        """Generate depth map MJPEG stream"""
        while self.is_running and self.streaming:
            with self.depth_lock:
                if self.latest_depth is not None:
                    # Normalize depth for visualization
                    depth_norm = cv2.normalize(self.latest_depth, None, 0, 255, cv2.NORM_MINMAX)
                    depth_colored = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
                    
                    # Add info overlay
                    cv2.putText(depth_colored, "DEPTH MAP",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # Encode to JPEG
                    _, buffer = cv2.imencode('.jpg', depth_colored, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    frame_bytes = buffer.tobytes()
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)
    
    def generate_grid_stream(self):
        """Generate occupancy grid MJPEG stream"""
        while self.is_running and self.streaming:
            with self.depth_lock:
                if self.latest_grid is not None:
                    # Resize for better visibility
                    grid_large = cv2.resize(self.latest_grid, (640, 640), interpolation=cv2.INTER_NEAREST)
                    
                    # Add info overlay
                    cv2.putText(grid_large, f"PERSISTENT MAP - {self.stats['persistent_map_points']} points",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(grid_large, f"Trajectory: {self.stats['trajectory_length']} poses",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    
                    # Encode to JPEG
                    _, buffer = cv2.imencode('.jpg', grid_large, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    frame_bytes = buffer.tobytes()
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)
    
    def update_stats_loop(self):
        """Update statistics continuously"""
        while self.is_running:
            # Update FPS
            if hasattr(self.left_receiver, 'fps'):
                self.stats['left_fps'] = self.left_receiver.fps
            if hasattr(self.right_receiver, 'fps'):
                self.stats['right_fps'] = self.right_receiver.fps
            
            # Update uptime
            if self.start_time:
                self.stats['uptime'] = time.time() - self.start_time
            
            # Update frame counts
            if hasattr(self.left_receiver, 'frame_count'):
                self.stats['left_frames'] = int(self.left_receiver.frame_count)
            if hasattr(self.right_receiver, 'frame_count'):
                    self.stats['right_frames'] = int(self.right_receiver.frame_count)
            
            # Convert all numpy types to Python types for JSON serialization
            stats_json = {
                'left_fps': float(self.stats['left_fps']),
                'right_fps': float(self.stats['right_fps']),
                'left_frames': int(self.stats['left_frames']),
                'right_frames': int(self.stats['right_frames']),
                'orb_features': int(self.stats['orb_features']),
                'matches': int(self.stats['matches']),
                'uptime': int(self.stats['uptime']),
                'map_points': int(self.stats['map_points']),
                'obstacles_detected': int(self.stats['obstacles_detected']),
                'linear_vel': float(self.stats['linear_vel']),
                'angular_vel': float(self.stats['angular_vel']),
                'persistent_map_points': int(self.stats['persistent_map_points']),
                'trajectory_length': int(self.stats['trajectory_length'])
            }
            
            # Broadcast to all connected clients
            socketio.emit('stats_update', stats_json)
            
            time.sleep(1)

# Flask routes
@app.route('/')
def index():
    """Main page"""
    return render_template('slam_web.html')

@app.route('/video/left')
def video_left():
    """Left camera stream"""
    if web_server and web_server.streaming:
        return Response(web_server.generate_left_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/right')
def video_right():
    """Right camera stream"""
    if web_server and web_server.streaming:
        return Response(web_server.generate_right_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/slam')
def video_slam():
    """SLAM processed stream"""
    if web_server and web_server.streaming:
        return Response(web_server.generate_slam_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/depth')
def video_depth():
    """Depth map stream"""
    if web_server and web_server.streaming:
        return Response(web_server.generate_depth_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/grid')
def video_grid():
    """Occupancy grid stream"""
    if web_server and web_server.streaming:
        return Response(web_server.generate_grid_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/api/stats')
def get_stats():
    """Get current statistics"""
    if web_server:
        return jsonify(web_server.stats)
    return jsonify({'error': 'not running'}), 503

@app.route('/api/status')
def get_status():
    """Get system status"""
    if web_server:
        return jsonify({
            'running': web_server.is_running,
            'streaming': web_server.streaming,
            'left_connected': web_server.left_receiver.is_running,
            'right_connected': web_server.right_receiver.is_running,
            'has_slam': web_server.has_slam
        })
    return jsonify({'running': False})

@app.route('/api/map_data')
def get_map_data():
    """API trả về dữ liệu 3D Point Cloud cho Three.js"""
    if not web_server or not web_server.persistent_map:
        return jsonify({
            'points': [],
            'colors': [],
            'robot_pose': [0, 0, 0]
        })
    
    # Lấy dữ liệu điểm 3D từ PersistentMap
    points, colors = web_server.persistent_map.get_3d_points()
    
    if len(points) == 0:
        return jsonify({
            'points': [],
            'colors': [],
            'robot_pose': web_server.robot_pose.tolist()
        })
        
    # Giới hạn số lượng điểm để tránh lag trình duyệt
    MAX_POINTS = 20000
    if len(points) > MAX_POINTS:
        indices = np.random.choice(len(points), MAX_POINTS, replace=False)
        points = points[indices]
        colors = colors[indices]

    return jsonify({
        'points': points.tolist(),
        'colors': colors.tolist(),
        'robot_pose': web_server.robot_pose.tolist()
    })

@app.route('/api/map3d')
def get_map_3d():
    """Get 3D map data for visualization (alternative API)"""
    if web_server:
        map_data = web_server.persistent_map.get_map_data_for_web()
        return jsonify(map_data)
    return jsonify({'error': 'not running'}), 503

@app.route('/api/save_map')
def save_map():
    """Save current map to file"""
    if web_server:
        filename = f"slam_map_{int(time.time())}.npz"
        web_server.persistent_map.save_map(filename)
        return jsonify({'success': True, 'filename': filename})
    return jsonify({'error': 'not running'}), 503

@app.route('/api/clear_map')
def clear_map():
    """Clear current map"""
    if web_server:
        web_server.persistent_map.clear_map()
        return jsonify({'success': True})
    return jsonify({'error': 'not running'}), 503

# SocketIO events
@socketio.on('connect')
def handle_connect():
    """Client connected"""
    print('[Web] Client connected')
    emit('status', {'connected': True})

@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected"""
    print('[Web] Client disconnected')

@socketio.on('start_streaming')
def handle_start_streaming():
    """Start video streaming"""
    global web_server
    
    if web_server and not web_server.streaming:
        web_server.streaming = True
        emit('streaming_started', {'success': True})
        print('[Web] Streaming started')

@socketio.on('stop_streaming')
def handle_stop_streaming():
    """Stop video streaming"""
    global web_server
    
    if web_server:
        web_server.streaming = False
        emit('streaming_stopped', {'success': True})
        print('[Web] Streaming stopped')

def run_web_server(host='0.0.0.0', port=5000):
    """Run Flask web server"""
    global web_server
    
    # Initialize SLAM server
    web_server = SLAMWebServer(left_port=9001, right_port=9002)
    
    # Start receivers
    if not web_server.start_receivers():
        print("[ERROR] Failed to start receivers")
        return
    
    web_server.is_running = True
    web_server.streaming = True  # Auto-start streaming
    web_server.start_time = time.time()
    
    # Start stats update thread
    stats_thread = threading.Thread(
        target=web_server.update_stats_loop,
        daemon=True
    )
    stats_thread.start()
    
    print(f"\n{'='*70}")
    print(f"  Web Interface Running")
    print(f"{'='*70}")
    print(f"Open browser: http://{host}:{port}")
    print(f"{'='*70}\n")
    
    # Run Flask
    socketio.run(app, host=host, port=port, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 SLAM Web Interface')
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5000)
    
    args = parser.parse_args()
    
    try:
        run_web_server(host=args.host, port=args.port)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        if web_server:
            web_server.is_running = False