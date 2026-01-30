#!/usr/bin/env python3
"""
X99 Web Interface for SLAM + Camera Streams - OPTIMIZED (FIXED)
High-performance version with robust error handling.
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

# Import OPTIMIZED components
try:
    from x99_headless import OptimizedCameraReceiver
except ImportError:
    print("[Error] x99_headless.py not found. Make sure it is in the same directory.")
    exit(1)

# Robust import for stereo mapping
try:
    from stereo_depth_mapping_optimized import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance
    print("[Info] Loaded OPTIMIZED stereo mapping module.")
except ImportError:
    print("[Warning] Optimized mapping not found, falling back to standard...")
    from stereo_depth_mapping import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance

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
            self.orb_extractor = ORBFeatureExtractor(n_features=1000)
            self.yolo = YOLOSegmentator()
            self.has_slam = True
            print("[Web] SLAM components loaded (YOLO + ORB)")
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
        self.slam_processing = False
        
        # Latest depth and grid
        self.latest_depth = None
        self.latest_grid = None
        self.latest_slam_frame = None
        self.depth_lock = threading.Lock()
        
        # Robot pose estimation
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, z
        
        # Stats
        self.stats = {
            'left_fps': 0.0, 'right_fps': 0.0,
            'slam_fps': 0.0, 'map_points': 0,
            'orb_features': 0, 'matches': 0,
            'persistent_map_points': 0
        }
        
        self.start_time = None
        self.last_slam_time = time.time()
        self.slam_frame_count = 0

        # Check if optimization is available in the loaded class
        self.use_step_optimization = hasattr(self.depth_mapper.depth_to_point_cloud, '__code__') and \
                                     'step' in self.depth_mapper.depth_to_point_cloud.__code__.co_varnames
        if self.use_step_optimization:
            print("[Info] Vectorized point cloud generation ENABLED.")
        else:
            print("[Warning] Legacy point cloud generation detected. Update stereo_depth_mapping_optimized.py for better FPS.")
    
    def start_receivers(self):
        """Start camera receivers"""
        print(f"Starting receivers on {self.left_receiver.port}, {self.right_receiver.port}")
        
        threading.Thread(target=lambda: (self.left_receiver.start_server() and self.left_receiver.receive_frames()), daemon=True).start()
        threading.Thread(target=lambda: (self.right_receiver.start_server() and self.right_receiver.receive_frames()), daemon=True).start()
        
        # Wait for connections
        for i in range(10):
            time.sleep(1)
            if self.left_receiver.is_running and self.right_receiver.is_running:
                print("[OK] Cameras connected!")
                return True
        return False
    
    def process_slam_frame(self, frame_left, frame_right):
        """Process a single stereo frame for SLAM"""
        
        # 1. Compute depth map
        # Handle method signature difference for compute_disparity
        try:
             disparity = self.depth_mapper.compute_disparity(frame_left, frame_right, use_wls=False)
        except TypeError:
             # Fallback for old class
             disparity = self.depth_mapper.compute_disparity(frame_left, frame_right)

        depth = self.depth_mapper.disparity_to_depth(disparity)
        
        # 2. Create 3D point cloud
        # Use 'step' only if available (Fixes TypeError)
        if self.use_step_optimization:
            points, colors = self.depth_mapper.depth_to_point_cloud(depth, frame_left, step=4)
        else:
            points, colors = self.depth_mapper.depth_to_point_cloud(depth, frame_left)
        
        # 3. Persistent Map Update
        if len(points) > 0:
             # Subsample heavily before sending to PersistentMap (CPU heavy)
             if len(points) > 2000:
                 indices = np.random.choice(len(points), 2000, replace=False)
                 points_small = points[indices]
                 colors_small = colors[indices]
             else:
                 points_small = points
                 colors_small = colors
                 
             self.persistent_map.add_point_cloud(points_small, colors_small, self.robot_pose)
             
             # Update local occupancy grid
             self.grid_mapper.update_from_point_cloud(points)
             self.grid_mapper.inflate_obstacles(radius=2)
             
             # Avoidance
             scan = self.grid_mapper.get_obstacle_scan()
             lin, ang = self.avoidance.compute_velocity(scan)
        
        # 4. ORB / YOLO (Optional)
        processed_frame = frame_left.copy()
        if self.has_slam and self.orb_extractor:
            try:
                kp1, des1 = self.orb_extractor.extract_features(frame_left)
                
                # YOLO - Run less frequently
                if self.yolo and self.yolo.model:
                     if self.slam_frame_count % 5 == 0:
                         results = self.yolo.segment(frame_left, conf=0.5)
                         if results:
                             processed_frame = self.yolo.draw_segments(frame_left, results)
                
                cv2.drawKeypoints(processed_frame, kp1, processed_frame, color=(0,255,0))
                self.stats['orb_features'] = len(kp1)
            except Exception as e:
                print(f"ORB/YOLO Error: {e}")

        # Update visualizations
        with self.depth_lock:
            self.latest_depth = depth
            self.latest_slam_frame = processed_frame
            # Render grid less frequently to save FPS
            if self.slam_frame_count % 5 == 0:
                self.latest_grid = self.persistent_map.visualize_2d(show_trajectory=True)

    def slam_processing_loop(self):
        """Background SLAM loop"""
        print("[SLAM] Processing loop started")
        while self.is_running:
            if not self.slam_processing:
                time.sleep(0.1)
                continue
            
            frame_left = self.left_receiver.get_latest_frame()
            frame_right = self.right_receiver.get_latest_frame()
            
            if frame_left is not None and frame_right is not None:
                start_t = time.time()
                try:
                    self.process_slam_frame(frame_left, frame_right)
                except Exception as e:
                    print(f"[SLAM Error] {e}")
                    import traceback
                    traceback.print_exc()
                
                dt = time.time() - start_t
                
                # Update FPS
                self.slam_frame_count += 1
                if dt > 0:
                    current_fps = 1.0 / dt
                    self.stats['slam_fps'] = 0.9 * self.stats['slam_fps'] + 0.1 * current_fps
            
            time.sleep(0.001)

    # Generators
    def generate_left_stream(self):
        while self.is_running:
            if self.left_receiver.latest_frame is not None:
                ret, buf = cv2.imencode('.jpg', self.left_receiver.latest_frame)
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.04)

    def generate_slam_stream(self):
        while self.is_running:
            with self.depth_lock:
                if self.latest_slam_frame is not None:
                    ret, buf = cv2.imencode('.jpg', self.latest_slam_frame)
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.04)
            
    def generate_depth_stream(self):
        while self.is_running:
            with self.depth_lock:
                if self.latest_depth is not None:
                    depth_norm = cv2.normalize(self.latest_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                    ret, buf = cv2.imencode('.jpg', colored)
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.04)
            
    def generate_grid_stream(self):
        while self.is_running:
            with self.depth_lock:
                if self.latest_grid is not None:
                     ret, buf = cv2.imencode('.jpg', self.latest_grid)
                     yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.1)

    def update_stats_loop(self):
        while self.is_running:
            # FIX: Robustly get FPS (handle missing attribute)
            left_fps = getattr(self.left_receiver, 'fps', 0)
            right_fps = getattr(self.right_receiver, 'fps', 0)
            
            # If fps is 0, try to calculate from history if available
            if left_fps == 0 and hasattr(self.left_receiver, 'fps_history') and len(self.left_receiver.fps_history) > 0:
                 left_fps = sum(self.left_receiver.fps_history) / len(self.left_receiver.fps_history)

            self.stats['left_fps'] = left_fps
            self.stats['right_fps'] = right_fps
            socketio.emit('stats_update', self.stats)
            time.sleep(1)

@app.route('/')
def index():
    return "Optimized SLAM Server Running. <br><a href='/video/slam'>SLAM Stream</a> <br><a href='/video/depth'>Depth Stream</a>"

@app.route('/video/left')
def video_left():
    return Response(web_server.generate_left_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/slam')
def video_slam():
    return Response(web_server.generate_slam_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/depth')
def video_depth():
    return Response(web_server.generate_depth_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/grid')
def video_grid():
    return Response(web_server.generate_grid_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    web_server = SLAMWebServer()
    web_server.is_running = True
    web_server.slam_processing = True
    
    if web_server.start_receivers():
        # Threads
        threading.Thread(target=web_server.slam_processing_loop, daemon=True).start()
        threading.Thread(target=web_server.update_stats_loop, daemon=True).start()
        
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)