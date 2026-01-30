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
            time.sleep(0.1) # Lower FPS for grid

    def update_stats_loop(self):
        while self.is_running:
            # Emit stats via socketio
            self.stats['left_fps'] = self.left_receiver.fps
            self.stats['right_fps'] = self.right_receiver.fps
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