#!/usr/bin/env python3
"""
X99 Web Interface for SLAM + Camera Streams - OPTIMIZED
High-performance version with vectorized depth mapping and reduced overhead.
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
from x99_headless import OptimizedCameraReceiver
from stereo_depth_mapping_optimized import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance
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
            self.orb_extractor = ORBFeatureExtractor(n_features=1000) # Reduced features
            self.yolo = YOLOSegmentator()
            self.has_slam = True
            print("[Web] SLAM components loaded (YOLO + ORB)")
        except ImportError as e:
            print(f"[Web] SLAM not available: {e}")
            self.orb_extractor = None
            self.yolo = None
            self.has_slam = False
        
        # Depth mapping components (Optimized)
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
        
        # 1. Compute depth map (FAST MODE)
        # Disable WLS filter for speed
        disparity = self.depth_mapper.compute_disparity(frame_left, frame_right, use_wls=False)
        depth = self.depth_mapper.disparity_to_depth(disparity)
        
        # 2. Create 3D point cloud (VECTORIZED)
        # step=4 reduces points by 16x, massive speedup
        points, colors = self.depth_mapper.depth_to_point_cloud(depth, frame_left, step=4)
        
        # 3. Persistent Map Update
        # Only add to map if we have points and not too frequently
        # (This is still the bottleneck as PersistentMap is Python-heavy)
        if len(points) > 0:
             # Subsample heavily before sending to PersistentMap
             if len(points) > 2000:
                 indices = np.random.choice(len(points), 2000, replace=False)
                 points_small = points[indices]
                 colors_small = colors[indices]
             else:
                 points_small = points
                 colors_small = colors
                 
             self.persistent_map.add_point_cloud(points_small, colors_small, self.robot_pose)
             
             # Update local occupancy grid
             self.grid_mapper.update_from_point_cloud(points) # Use full points for local safety
             self.grid_mapper.inflate_obstacles(radius=2)
             
             # Avoidance
             scan = self.grid_mapper.get_obstacle_scan()
             lin, ang = self.avoidance.compute_velocity(scan)
        
        # 4. ORB / YOLO (Optional)
        processed_frame = frame_left.copy()
        if self.has_slam and self.orb_extractor:
            try:
                # Limit features to keep FPS high
                kp1, des1 = self.orb_extractor.extract_features(frame_left)
                # kp2, des2 = self.orb_extractor.extract_features(frame_right) # Skip right for speed
                
                # YOLO (Check if model exists)
                if self.yolo and self.yolo.model:
                     # Only run YOLO every 5th frame to save GPU/CPU transfer time
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
            # Only update grid visualization occasionally
            if self.slam_frame_count % 5 == 0:
                self.latest_grid = self.persistent_map.visualize_2d(show_trajectory=True)

    def slam_processing_loop(self):
        """Background SLAM loop"""
        print("[SLAM] Optimized processing loop started")
        while self.is_running:
            if not self.slam_processing:
                time.sleep(0.1)
                continue
            
            frame_left = self.left_receiver.get_latest_frame()
            frame_right = self.right_receiver.get_latest_frame()
            
            if frame_left is not None and frame_right is not None:
                start_t = time.time()
                self.process_slam_frame(frame_left, frame_right)
                dt = time.time() - start_t
                
                # Update FPS
                self.slam_frame_count += 1
                if dt > 0:
                    current_fps = 1.0 / dt
                    # Smooth FPS
                    self.stats['slam_fps'] = 0.9 * self.stats['slam_fps'] + 0.1 * current_fps
            
            # Don't sleep if we want max FPS, but be nice to CPU
            time.sleep(0.001)