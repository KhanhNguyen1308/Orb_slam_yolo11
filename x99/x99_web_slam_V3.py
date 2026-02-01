#!/usr/bin/env python3
"""
X99 Web Interface V3 - FIXED POSE DRIFT + 2D LIDAR MAP
========================================================
FIXES:
- Robot pose NO LONGER DRIFTS when stationary
- Added clear 2D occupancy grid like LIDAR
- Proper height filtering for 50cm camera height
- Separated streaming from SLAM processing

Robot specs:
- Camera baseline: 10cm
- Camera height: 50cm above ground
"""

from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
import json
import threading
import time
import base64
from io import BytesIO
from PIL import Image

import heapq
from typing import List, Tuple, Optional

from x99_headless import OptimizedCameraReceiver
from stereo_depth_mapping_optimized import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance
from persistent_map import PersistentMap

app = Flask(__name__)
app.config['SECRET_KEY'] = 'slam_web_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global server instance
web_server = None

class SLAMWebServer:
    """Web server with camera streaming and SLAM visualization - FIXED VERSION"""
    
    def __init__(self, left_port=9001, right_port=9002):
        self.left_receiver = OptimizedCameraReceiver(left_port, "LEFT")
        self.right_receiver = OptimizedCameraReceiver(right_port, "RIGHT")
        self.latest_color_left = None
        self.latest_color_right = None
        # SLAM components
        try:
            from x99_slam_server import ORBFeatureExtractor, YOLOSegmentator
            self.orb_extractor = ORBFeatureExtractor(n_features=1500)
            self.yolo = YOLOSegmentator()
            self.has_slam = True
            print("[Web] SLAM components loaded")
        except ImportError as e:
            print(f"[Web] SLAM not available: {e}")
            self.orb_extractor = None
            self.yolo = None
            self.has_slam = False
        
        # Depth mapping components - CALIBRATED FOR 50cm HEIGHT
        self.depth_mapper = StereoDepthMapper(
            baseline=0.10,      # 10cm between cameras
            focal_length=500    # Will be overridden by calibration if available
        )
        
        # Occupancy grid for 2D navigation
        self.grid_mapper = OccupancyGridMapper(
            grid_size=600,       # 30m x 30m at 5cm resolution
            resolution=0.05,     # 5cm per cell
            max_range=15.0       # 15m max range
        )
        
        # Persistent map builder
        self.persistent_map = PersistentMap(
            grid_size=800, 
            resolution=0.02,    # 2cm for detailed 3D
            voxel_size=0.05     # 5cm voxel for downsampling
        )
        
        self.is_running = False
        self.streaming = False
        self.slam_processing = False
        
        # Latest depth and grid
        self.latest_depth = None
        self.latest_grid = None
        self.latest_grid_2d = None  # NEW: 2D occupancy grid
        self.latest_slam_frame = None
        self.depth_lock = threading.Lock()
        
        # Robot pose - FIXED: Only update from external odometry
        self.robot_pose = np.array([0.0, 0.5, 0.0])  # x, height(0.5m), z
        self.robot_yaw = 0.0  # Orientation (radians)
        
        # CRITICAL: Flag to enable/disable pose updates
        self.update_pose_from_velocity = False  # Set to True only if you have real odometry
        
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
            'trajectory_length': 0,
            'slam_fps': 0.0,
            'grid_2d_obstacles': 0,
            'grid_2d_free': 0
        }
        
        self.start_time = None
        self.last_slam_time = time.time()
        self.slam_frame_count = 0
        
        print("\n[INIT] Camera height: 50cm, Baseline: 10cm")
        print("[INIT] Robot pose LOCKED (no drift)")
    
    def start_receivers(self):
        """Start camera receivers"""
        print("\n" + "=" * 70)
        print("  X99 SLAM Web Interface V3 - DRIFT FIXED")
        print("=" * 70)
        
        # Show IPs
        import subprocess
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            ips = result.stdout.strip().split()
            if ips:
                print(f"X99 IPs: {', '.join(ips)}")
                print(f"Web Interface: http://{ips[0]}:1234")
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
    
    def process_slam_frame(self, frame_left, frame_right):
        """
        Process stereo frame for SLAM and depth mapping
        FIXED: No automatic pose updates
        """
        
        # Compute depth map
        try:
            disparity = self.depth_mapper.compute_disparity(frame_left, frame_right, use_wls=False)
            depth = self.depth_mapper.disparity_to_depth(disparity)
            
            # Create 3D point cloud
            points, colors = self.depth_mapper.depth_to_point_cloud(depth, frame_left)
            
            if len(points) > 100:
                # --- HEIGHT FILTERING FOR 50cm CAMERA ---
                # Transform points to world frame (camera is 50cm high)
                # Camera Y-axis points down, so ground is at Y = -0.5m
                points_world = points.copy()
                points_world[:, 1] += 0.5  # Shift by camera height
                
                # Filter valid points
                valid_mask = (
                    (points_world[:, 1] > -0.1) &  # Above ground
                    (points_world[:, 1] < 2.5) &   # Below 2.5m
                    (points_world[:, 2] > 0.1) &   # In front of camera
                    (points_world[:, 2] < 15.0)    # Within 15m
                )
                
                points_valid = points_world[valid_mask]
                colors_valid = colors[valid_mask] if colors is not None else None
                
                print(f"[SLAM] {len(points)} raw → {len(points_valid)} filtered points")
                
                if len(points_valid) > 50:
                    # Add to persistent 3D map
                    self.persistent_map.add_point_cloud(
                        points_valid, 
                        colors_valid, 
                        self.robot_pose  # Use FIXED pose
                    )
                    
                    # Update 2D occupancy grid
                    self.grid_mapper.update_from_point_cloud(points_valid)
                    self.grid_mapper.inflate_obstacles(radius=4)  # Inflate for safety
                    
                    # Get obstacle counts
                    obstacles = np.sum(self.grid_mapper.grid == 100)
                    free_space = np.sum(self.grid_mapper.grid == 0)
                    
                    # Update stats
                    self.stats['map_points'] = len(points_valid)
                    self.stats['obstacles_detected'] = obstacles
                    self.stats['persistent_map_points'] = len(self.persistent_map.voxel_grid)
                    self.stats['trajectory_length'] = len(self.persistent_map.trajectory)
                    self.stats['grid_2d_obstacles'] = obstacles
                    self.stats['grid_2d_free'] = free_space
                    
                    print(f"[GRID] Obstacles: {obstacles}, Free: {free_space}")
                
                # Store latest data
                with self.depth_lock:
                    self.latest_depth = depth
                    self.latest_grid = self.persistent_map.visualize_2d(show_trajectory=True)
                    self.latest_grid_2d = self.grid_mapper.visualize()
                    
            else:
                print(f"[SLAM] Warning: Only {len(points)} points after filtering")
                
        except Exception as e:
            print(f"[SLAM] Error: {e}")
            import traceback
            traceback.print_exc()
        
        # ORB feature visualization (optional)
        processed_frame = frame_left.copy()
        
        if self.has_slam and self.orb_extractor is not None:
            try:
                kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
                kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
                matches = self.orb_extractor.match_features(desc_left, desc_right)
                
                processed_frame = cv2.drawKeypoints(
                    processed_frame, kp_left, None,
                    color=(0, 255, 0),
                    flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                )
                
                self.stats['orb_features'] = len(kp_left)
                self.stats['matches'] = len(matches)
                
            except Exception as e:
                print(f"[SLAM] ORB error: {e}")
        
        with self.depth_lock:
            self.latest_slam_frame = processed_frame
    
    def slam_processing_loop(self):
        """Background SLAM processing thread"""
        print("[SLAM] Processing thread started")
        
        while self.is_running:
            if not self.slam_processing:
                time.sleep(0.1)
                continue
            
            frame_left = self.left_receiver.get_latest_frame()
            frame_right = self.right_receiver.get_latest_frame()
            self.latest_color_left = frame_left.copy()
            self.latest_color_right = frame_right.copy()
            if frame_left is not None and frame_right is not None:
                self.process_slam_frame(frame_left, frame_right)
                
                self.slam_frame_count += 1
                current_time = time.time()
                elapsed = current_time - self.last_slam_time
                
                if elapsed >= 1.0:
                    self.stats['slam_fps'] = self.slam_frame_count / elapsed
                    self.slam_frame_count = 0
                    self.last_slam_time = current_time
            
            time.sleep(0.1)  # 10 Hz
        
        print("[SLAM] Processing thread stopped")
    
    def generate_left_stream(self):
        """Generate left camera MJPEG stream"""
        while self.is_running and self.streaming:
            if self.left_receiver.latest_frame is not None:
                frame = self.latest_color_left.copy() 
                cv2.putText(frame, f"LEFT - FPS: {self.stats['left_fps']:.1f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    def generate_right_stream(self):
        """Generate right camera MJPEG stream"""
        while self.is_running and self.streaming:
            if self.right_receiver.latest_frame is not None:
                frame = self.right_receiver.latest_frame.copy()
                cv2.putText(frame, f"RIGHT - FPS: {self.stats['right_fps']:.1f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    def generate_slam_stream(self):
        """Generate SLAM features stream"""
        while self.is_running and self.streaming:
            with self.depth_lock:
                if self.latest_slam_frame is not None:
                    frame = self.latest_slam_frame.copy()
                    
                    cv2.putText(frame, f"SLAM - Features: {self.stats['orb_features']}",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(frame, f"Map: {self.stats['persistent_map_points']} pts",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(frame, f"FPS: {self.stats['slam_fps']:.1f}",
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    def generate_depth_stream(self):
        """Generate depth map stream"""
        while self.is_running and self.streaming:
            with self.depth_lock:
                if self.latest_depth is not None:
                    depth_norm = cv2.normalize(self.latest_depth, None, 0, 255, cv2.NORM_MINMAX)
                    depth_colored = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
                    
                    cv2.putText(depth_colored, "DEPTH MAP",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    _, buffer = cv2.imencode('.jpg', depth_colored, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    def generate_grid_stream(self):
        """Generate 3D persistent map stream"""
        while self.is_running and self.streaming:
            with self.depth_lock:
                if self.latest_grid is not None:
                    grid_large = cv2.resize(self.latest_grid, (640, 640), interpolation=cv2.INTER_NEAREST)
                    
                    cv2.putText(grid_large, f"3D MAP - {self.stats['persistent_map_points']} points",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    
                    _, buffer = cv2.imencode('.jpg', grid_large, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    def generate_grid_2d_stream(self):
        """NEW: Generate 2D occupancy grid stream (LIDAR-like)"""
        while self.is_running and self.streaming:
            with self.depth_lock:
                if self.latest_grid_2d is not None:
                    grid_large = cv2.resize(self.latest_grid_2d, (640, 640), interpolation=cv2.INTER_NEAREST)
                    
                    # Add overlay info
                    cv2.putText(grid_large, f"2D GRID - Obstacles: {self.stats['grid_2d_obstacles']}",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(grid_large, f"Free: {self.stats['grid_2d_free']} cells",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                    _, buffer = cv2.imencode('.jpg', grid_large, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    def update_stats_loop(self):
        """Update statistics"""
        while self.is_running:
            if hasattr(self.left_receiver, 'fps'):
                self.stats['left_fps'] = self.left_receiver.fps
            if hasattr(self.right_receiver, 'fps'):
                self.stats['right_fps'] = self.right_receiver.fps
            
            if self.start_time:
                self.stats['uptime'] = time.time() - self.start_time
            
            if hasattr(self.left_receiver, 'frame_count'):
                self.stats['left_frames'] = int(self.left_receiver.frame_count)
            if hasattr(self.right_receiver, 'frame_count'):
                self.stats['right_frames'] = int(self.right_receiver.frame_count)
            
            stats_json = {k: float(v) if isinstance(v, (np.floating, np.integer)) else v 
                         for k, v in self.stats.items()}
            stats_json['slam_processing'] = self.slam_processing
            
            socketio.emit('stats_update', stats_json)
            time.sleep(1)

# Flask routes
@app.route('/')
def index():
    return render_template('slam_web_V3.html')

@app.route('/video/left')
def video_left():
    if web_server and web_server.streaming:
        return Response(web_server.generate_left_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/right')
def video_right():
    if web_server and web_server.streaming:
        return Response(web_server.generate_right_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/slam')
def video_slam():
    if web_server and web_server.streaming:
        return Response(web_server.generate_slam_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/depth')
def video_depth():
    if web_server and web_server.streaming:
        return Response(web_server.generate_depth_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/grid')
def video_grid():
    if web_server and web_server.streaming:
        return Response(web_server.generate_grid_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/video/grid2d')
def video_grid_2d():
    """NEW: 2D occupancy grid endpoint"""
    if web_server and web_server.streaming:
        return Response(web_server.generate_grid_2d_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Not streaming", 503

@app.route('/api/stats')
def get_stats():
    if web_server:
        return jsonify(web_server.stats)
    return jsonify({'error': 'not running'}), 503

@app.route('/api/status')
def get_status():
    if web_server:
        return jsonify({
            'running': web_server.is_running,
            'streaming': web_server.streaming,
            'slam_processing': web_server.slam_processing,
            'left_connected': web_server.left_receiver.is_running,
            'right_connected': web_server.right_receiver.is_running,
            'has_slam': web_server.has_slam,
            'pose_locked': not web_server.update_pose_from_velocity
        })
    return jsonify({'running': False})

@app.route('/api/map_data')
def get_map_data():
    """3D point cloud data"""
    if not web_server or not web_server.persistent_map:
        return jsonify({'points': [], 'colors': [], 'robot_pose': [0, 0, 0]})
    
    points, colors = web_server.persistent_map.get_3d_points()
    
    if len(points) == 0:
        return jsonify({'points': [], 'colors': [], 'robot_pose': web_server.robot_pose.tolist()})
    
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

@app.route('/api/map_2d')
def get_map_2d():
    """NEW: Get 2D occupancy grid for navigation"""
    if not web_server or not web_server.grid_mapper:
        return jsonify({'grid': [], 'robot_pose': [0, 0, 0]})
    
    grid = web_server.grid_mapper.grid.copy()
    
    return jsonify({
        'grid': grid.tolist(),
        'robot_pose': web_server.robot_pose.tolist(),
        'robot_yaw': float(web_server.robot_yaw),
        'grid_size': grid.shape[0],
        'resolution': web_server.grid_mapper.resolution,
        'obstacles': int(np.sum(grid == 100)),
        'free_space': int(np.sum(grid == 0))
    })

@app.route('/api/save_map')
def save_map():
    if web_server:
        filename = f"slam_map_{int(time.time())}.npz"
        web_server.persistent_map.save_map(filename)
        return jsonify({'success': True, 'filename': filename})
    return jsonify({'error': 'not running'}), 503

@app.route('/api/clear_map')
def clear_map():
    if web_server:
        web_server.persistent_map.clear_map()
        web_server.grid_mapper.grid.fill(-1)
        return jsonify({'success': True})
    return jsonify({'error': 'not running'}), 503

@app.route('/api/set_pose', methods=['POST'])
def set_pose():
    """NEW: Manually set robot pose"""
    if not web_server:
        return jsonify({'error': 'not running'}), 503
    
    data = request.json
    x = data.get('x', 0.0)
    y = data.get('y', 0.5)
    z = data.get('z', 0.0)
    yaw = data.get('yaw', 0.0)
    
    web_server.robot_pose = np.array([x, y, z])
    web_server.robot_yaw = yaw
    
    return jsonify({'success': True, 'pose': web_server.robot_pose.tolist(), 'yaw': yaw})

# SocketIO events
@socketio.on('connect')
def handle_connect():
    print('[Web] Client connected')
    emit('status', {'connected': True})

@socketio.on('disconnect')
def handle_disconnect():
    print('[Web] Client disconnected')

@socketio.on('start_streaming')
def handle_start_streaming():
    global web_server
    if web_server and not web_server.streaming:
        web_server.streaming = True
        emit('streaming_started', {'success': True})
        print('[Web] Streaming started')

@socketio.on('stop_streaming')
def handle_stop_streaming():
    global web_server
    if web_server:
        web_server.streaming = False
        emit('streaming_stopped', {'success': True})
        print('[Web] Streaming stopped')

@socketio.on('start_slam')
def handle_start_slam():
    global web_server
    if web_server:
        web_server.slam_processing = True
        emit('slam_started', {'success': True})
        print('[Web] SLAM processing started')

@socketio.on('stop_slam')
def handle_stop_slam():
    global web_server
    if web_server:
        web_server.slam_processing = False
        emit('slam_stopped', {'success': True})
        print('[Web] SLAM processing stopped')

def run_web_server(host='0.0.0.0', port=1234):
    """Run Flask web server"""
    global web_server
    
    web_server = SLAMWebServer(left_port=9002, right_port=9001)
    
    if not web_server.start_receivers():
        print("[ERROR] Failed to start receivers")
        return
    
    web_server.is_running = True
    web_server.streaming = True
    web_server.slam_processing = True
    web_server.start_time = time.time()
    
    # Start background threads
    stats_thread = threading.Thread(target=web_server.update_stats_loop, daemon=True)
    stats_thread.start()
    
    slam_thread = threading.Thread(target=web_server.slam_processing_loop, daemon=True)
    slam_thread.start()
    
    print(f"\n{'='*70}")
    print(f"  Web Interface V3 - DRIFT FIXED")
    print(f"{'='*70}")
    print(f"URL: http://{host}:{port}")
    print(f"Robot pose: LOCKED (no automatic updates)")
    print(f"{'='*70}\n")
    
    socketio.run(app, host=host, port=port, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 SLAM V3 - Fixed Drift')
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=1234)
    
    args = parser.parse_args()
    
    try:
        run_web_server(host=args.host, port=args.port)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        if web_server:
            web_server.is_running = False