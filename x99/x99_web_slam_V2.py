#!/usr/bin/env python3
"""
X99 Web Interface for SLAM + Camera Streams
SLAM processing runs in background thread independent of streaming
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
from x99.stereo_depth_mapping_optimized import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance
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
        self.depth_mapper = StereoDepthMapper(baseline=0.01, focal_length=500)
        self.grid_mapper = OccupancyGridMapper(grid_size=400, resolution=0.05, max_range=5.0)
        self.avoidance = ObstacleAvoidance(safety_distance=0.5, max_linear_vel=0.3)
        
        # Persistent map builder
        self.persistent_map = PersistentMap(grid_size=800, resolution=0.02, voxel_size=0.05)
        
        self.is_running = False
        self.streaming = False
        self.slam_processing = False  # NEW: Flag for SLAM processing
        
        # Latest depth and grid
        self.latest_depth = None
        self.latest_grid = None
        self.latest_slam_frame = None
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
            'trajectory_length': 0,
            'slam_fps': 0.0
        }
        
        self.start_time = None
        self.last_slam_time = time.time()
        self.slam_frame_count = 0
    
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
        Process a single stereo frame for SLAM and depth mapping
        This runs in background thread independent of video streaming
        """
        
        # Compute depth map
        try:
            disparity = self.depth_mapper.compute_disparity(frame_left, frame_right, use_wls=False)
            depth = self.depth_mapper.disparity_to_depth(disparity)
            
            # Create 3D point cloud
            points, colors = self.depth_mapper.depth_to_point_cloud(depth, frame_left)
            
            print(f"[SLAM] Frame processed: {len(points)} points generated")
            
            if len(points) > 100:
                # Add to persistent map - CRITICAL for map visualization
                self.persistent_map.add_point_cloud(points, colors, self.robot_pose)
                print(f"[SLAM] Added to map. Total voxels: {len(self.persistent_map.voxel_grid)}")
                
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
            else:
                print(f"[SLAM] Warning: Only {len(points)} points generated (need >100)")
            
            # Store latest depth and grid
            with self.depth_lock:
                self.latest_depth = depth
                self.latest_grid = self.persistent_map.visualize_2d(show_trajectory=True)
                
        except Exception as e:
            print(f"[SLAM] Depth mapping error: {e}")
            import traceback
            traceback.print_exc()
        
        # ORB-SLAM processing (optional, for feature visualization)
        processed_frame = frame_left.copy()
        
        if self.has_slam and self.orb_extractor is not None:
            try:
                # Extract features
                kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
                kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
                
                # Match features
                matches = self.orb_extractor.match_features(desc_left, desc_right)
                
                # YOLO segmentation (optional)
                if self.yolo and self.yolo.model:
                    try:
                        results = self.yolo.segment(frame_left, conf=0.5)
                        if results:
                            processed_frame = self.yolo.draw_segments(frame_left, results)
                    except Exception as e:
                        print(f"[SLAM] YOLO error: {e}")
                
                # Draw ORB features
                processed_frame = cv2.drawKeypoints(
                    processed_frame, kp_left, None,
                    color=(0, 255, 0),
                    flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                )
                
                # Update stats
                self.stats['orb_features'] = len(kp_left)
                self.stats['matches'] = len(matches)
                
            except Exception as e:
                print(f"[SLAM] ORB processing error: {e}")
        
        # Store latest SLAM frame
        with self.depth_lock:
            self.latest_slam_frame = processed_frame
    
    def slam_processing_loop(self):
        """
        Background thread that continuously processes SLAM
        Runs independent of video streaming
        """
        print("[SLAM] Processing thread started")
        
        while self.is_running:
            if not self.slam_processing:
                time.sleep(0.1)
                continue
            
            # Get latest frames
            frame_left = self.left_receiver.get_latest_frame()
            frame_right = self.right_receiver.get_latest_frame()
            
            if frame_left is not None and frame_right is not None:
                # Process SLAM
                self.process_slam_frame(frame_left, frame_right)
                
                # Update SLAM FPS
                self.slam_frame_count += 1
                current_time = time.time()
                elapsed = current_time - self.last_slam_time
                
                if elapsed >= 1.0:
                    self.stats['slam_fps'] = self.slam_frame_count / elapsed
                    self.slam_frame_count = 0
                    self.last_slam_time = current_time
            else:
                print("[SLAM] Warning: No frames available")
            
            time.sleep(0.1)  # 10 Hz processing rate
        
        print("[SLAM] Processing thread stopped")
    
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
            with self.depth_lock:
                if self.latest_slam_frame is not None:
                    frame = self.latest_slam_frame.copy()
                    
                    # Add info overlay
                    cv2.putText(frame, f"SLAM - Features: {self.stats['orb_features']} Matches: {self.stats['matches']}",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(frame, f"Map Points: {self.stats['persistent_map_points']}",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(frame, f"SLAM FPS: {self.stats['slam_fps']:.1f}",
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                    # Encode to JPEG
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
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
                'trajectory_length': int(self.stats['trajectory_length']),
                'slam_fps': float(self.stats['slam_fps']),
                'slam_processing': self.slam_processing
            }
            
            # Broadcast to all connected clients
            socketio.emit('stats_update', stats_json)
            
            time.sleep(1)

# Flask routes
@app.route('/')
def index():
    """Main page"""
    return render_template('slam_web_V2.html')

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
            'slam_processing': web_server.slam_processing,
            'left_connected': web_server.left_receiver.is_running,
            'right_connected': web_server.right_receiver.is_running,
            'has_slam': web_server.has_slam
        })
    return jsonify({'running': False})

@app.route('/api/map_data')
def get_map_data():
    """API returns 3D Point Cloud data for Three.js"""
    if not web_server or not web_server.persistent_map:
        return jsonify({
            'points': [],
            'colors': [],
            'robot_pose': [0, 0, 0]
        })
    
    # Get 3D points from PersistentMap
    points, colors = web_server.persistent_map.get_3d_points()
    
    if len(points) == 0:
        return jsonify({
            'points': [],
            'colors': [],
            'robot_pose': web_server.robot_pose.tolist()
        })
        
    # Limit points to avoid browser lag
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

@socketio.on('start_slam')
def handle_start_slam():
    """Start SLAM processing"""
    global web_server
    
    if web_server:
        web_server.slam_processing = True
        emit('slam_started', {'success': True})
        print('[Web] SLAM processing started')

@socketio.on('stop_slam')
def handle_stop_slam():
    """Stop SLAM processing"""
    global web_server
    
    if web_server:
        web_server.slam_processing = False
        emit('slam_stopped', {'success': True})
        print('[Web] SLAM processing stopped')


@app.route('/api/map_2d')
def get_map_2d():
    """Get 2D occupancy grid for navigation"""
    if not web_server or not web_server.persistent_map:
        return jsonify({'grid': [], 'robot_pose': [0, 0, 0]})
    
    # Get 2D occupancy map
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    
    return jsonify({
        'grid': grid.tolist(),
        'robot_pose': web_server.robot_pose.tolist(),
        'grid_size': grid.shape[0],
        'resolution': web_server.persistent_map.resolution
    })

@app.route('/api/plan_path', methods=['POST'])
def plan_path():
    """
    Plan path from start to goal using A*
    POST data: {start: [x, y], goal: [x, y]}
    """
    if not web_server or not web_server.persistent_map:
        return jsonify({'error': 'no map available'}), 503
    
    data = request.json
    start = data.get('start')
    goal = data.get('goal')
    
    if not start or not goal:
        return jsonify({'error': 'missing start or goal'}), 400
    
    # Get occupancy grid
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    
    # Plan path
    path = astar_path_planning(grid, tuple(start), tuple(goal))
    
    if path is None:
        return jsonify({'error': 'no path found'}), 404
    
    # Calculate distance
    distance = len(path) * web_server.persistent_map.resolution
    
    return jsonify({
        'path': path,
        'distance': distance,
        'num_waypoints': len(path)
    })

def astar_path_planning(grid: np.ndarray, start: Tuple[int, int], 
                       goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """
    A* path planning on occupancy grid
    
    Args:
        grid: 2D array where -1=unknown, 0=free, 100=occupied
        start: (x, y) start position in grid coordinates
        goal: (x, y) goal position in grid coordinates
    
    Returns:
        List of (x, y) waypoints or None if no path found
    """
    height, width = grid.shape
    
    # Check if start and goal are valid
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        print(f"Start {start} out of bounds")
        return None
    
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        print(f"Goal {goal} out of bounds")
        return None
    
    # Check if start or goal is occupied
    if grid[start[1], start[0]] == 100:
        print(f"Start {start} is occupied")
        return None
    
    if grid[goal[1], goal[0]] == 100:
        print(f"Goal {goal} is occupied")
        return None
    
    def heuristic(a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(pos):
        """Get valid neighbors (8-directional)"""
        x, y = pos
        neighbors = []
        
        # 8 directions: N, NE, E, SE, S, SW, W, NW
        directions = [
            (0, -1, 1.0),   # N
            (1, -1, 1.414), # NE
            (1, 0, 1.0),    # E
            (1, 1, 1.414),  # SE
            (0, 1, 1.0),    # S
            (-1, 1, 1.414), # SW
            (-1, 0, 1.0),   # W
            (-1, -1, 1.414) # NW
        ]
        
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < width and 0 <= ny < height:
                # Check if free or unknown (allow unknown for exploration)
                if grid[ny, nx] != 100:  # Not occupied
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    # A* algorithm
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    closed_set = set()
    
    while open_set:
        current_f, current = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
        
        closed_set.add(current)
        
        # Goal reached
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        
        # Explore neighbors
        for neighbor, move_cost in get_neighbors(current):
            if neighbor in closed_set:
                continue
            
            tentative_g = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # No path found
    return None

# Also add this helper function to extract clean boundaries

@app.route('/api/get_boundaries')
def get_boundaries():
    """Extract boundary edges from occupancy grid"""
    if not web_server or not web_server.persistent_map:
        return jsonify({'boundaries': []})
    
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    boundaries = extract_boundaries(grid)
    
    return jsonify({
        'boundaries': boundaries,
        'num_segments': len(boundaries)
    })

def extract_boundaries(grid: np.ndarray) -> List[List[Tuple[int, int]]]:
    """
    Extract boundary contours from occupancy grid
    
    Returns:
        List of boundary segments, each segment is a list of (x, y) points
    """
    
    # Create binary image: 255 for occupied, 0 for free/unknown
    binary = np.zeros_like(grid, dtype=np.uint8)
    binary[grid == 100] = 255
    
    # Find contours
    contours, hierarchy = cv2.findContours(
        binary, 
        cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE
    )
    
    # Convert contours to list of points
    boundaries = []
    for contour in contours:
        if len(contour) > 10:  # Filter small contours
            points = [(int(pt[0][0]), int(pt[0][1])) for pt in contour]
            boundaries.append(points)
    
    return boundaries

@app.route('/api/navigate', methods=['POST'])
def start_navigation():
    """
    Start autonomous navigation to goal
    POST data: {goal: [x, y]}
    """
    global navigation_process, navigation_status
    
    data = request.json
    goal = data.get('goal')
    
    if not goal or len(goal) != 2:
        return jsonify({'success': False, 'error': 'Invalid goal'}), 400
    
    goal_x, goal_y = goal
    
    # Stop existing navigation
    if navigation_process and navigation_process.poll() is None:
        navigation_process.terminate()
        navigation_process.wait()
    
    # Start new navigation process on Jetson
    # This assumes jetson_navigation.py is running on Jetson and accessible via SSH
    # Or you can implement a REST API on Jetson and call it here
    
    try:
        # Option 1: If Jetson navigation is a service, call its API
        # import requests
        # response = requests.post('http://jetson_ip:8000/navigate', 
        #                          json={'goal_x': goal_x, 'goal_y': goal_y})
        
        # Option 2: If using SSH to trigger command
        # navigation_process = subprocess.Popen(
        #     ['ssh', 'jetson@jetson_ip', 
        #      f'python3 /path/to/jetson_navigation.py g {goal_x} {goal_y}']
        # )
        
        # Option 3: Store goal and let Jetson poll for it
        navigation_status['active'] = True
        navigation_status['goal'] = goal
        navigation_status['progress'] = 0
        
        # Emit via SocketIO
        socketio.emit('navigation_started', {'goal': goal})
        
        return jsonify({'success': True, 'goal': goal})
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/stop_navigation', methods=['POST'])
def stop_navigation():
    """Stop current navigation"""
    global navigation_process, navigation_status
    
    if navigation_process and navigation_process.poll() is None:
        navigation_process.terminate()
        navigation_process.wait()
    
    navigation_status['active'] = False
    navigation_status['progress'] = 0
    
    socketio.emit('navigation_stopped', {})
    
    return jsonify({'success': True})

@app.route('/api/navigation_status')
def get_navigation_status():
    """Get current navigation status"""
    return jsonify(navigation_status)

@app.route('/control')
def robot_control():
    """Robot remote control interface"""
    return render_template('robot_control.html')


def run_web_server(host='0.0.0.0', port=1234):
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
    web_server.slam_processing = True  # Auto-start SLAM processing
    web_server.start_time = time.time()
    
    # Start stats update thread
    stats_thread = threading.Thread(
        target=web_server.update_stats_loop,
        daemon=True
    )
    stats_thread.start()
    
    # Start SLAM processing thread
    slam_thread = threading.Thread(
        target=web_server.slam_processing_loop,
        daemon=True
    )
    slam_thread.start()
    
    print(f"\n{'='*70}")
    print(f"  Web Interface Running")
    print(f"{'='*70}")
    print(f"Open browser: http://{host}:{port}")
    print(f"SLAM processing: AUTO-STARTED")
    print(f"{'='*70}\n")
    
    # Run Flask
    socketio.run(app, host=host, port=port, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 SLAM Web Interface')
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=1234)
    
    args = parser.parse_args()
    
    try:
        run_web_server(host=args.host, port=args.port)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        if web_server:
            web_server.is_running = False