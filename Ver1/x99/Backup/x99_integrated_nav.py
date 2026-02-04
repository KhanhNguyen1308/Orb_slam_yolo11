#!/usr/bin/env python3
"""
X99 Integrated SLAM + Path Planning Server
Receives camera streams, runs SLAM, plans paths, sends commands to Jetson
"""

import cv2
import numpy as np
import socket
import struct
import threading
import time
import json
from typing import List, Tuple, Optional

from x99_wifi_optimized import OptimizedCameraReceiver, X99OptimizedSLAMServer
from path_planning import PathPlanner

class NavigationCommandSender:
    """Send navigation commands to Jetson Nano"""
    
    def __init__(self, jetson_ip: str, port: int = 9003):
        self.jetson_ip = jetson_ip
        self.port = port
        self.sock = None
        self.connected = False
    
    def connect(self) -> bool:
        """Connect to Jetson Nano"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.jetson_ip, self.port))
            self.connected = True
            print(f"[NavSender] Connected to Jetson at {self.jetson_ip}:{self.port}")
            return True
        except Exception as e:
            print(f"[NavSender] Connection failed: {e}")
            return False
    
    def send_path(self, path: List[Tuple[float, float]]) -> bool:
        """Send path to Jetson"""
        if not self.connected:
            return False
        
        try:
            message = {
                'type': 'path',
                'path': path
            }
            
            data = json.dumps(message).encode()
            size = struct.pack("!I", len(data))
            
            self.sock.sendall(size + data)
            print(f"[NavSender] Sent path with {len(path)} waypoints")
            return True
            
        except Exception as e:
            print(f"[NavSender] Send error: {e}")
            self.connected = False
            return False
    
    def send_pose(self, x: float, y: float, theta: float) -> bool:
        """Send robot pose update to Jetson"""
        if not self.connected:
            return False
        
        try:
            message = {
                'type': 'pose',
                'pose': [x, y, theta]
            }
            
            data = json.dumps(message).encode()
            size = struct.pack("!I", len(data))
            
            self.sock.sendall(size + data)
            return True
            
        except Exception as e:
            print(f"[NavSender] Pose send error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Jetson"""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.connected = False

class IntegratedSLAMNavigationServer:
    """Integrated SLAM + Path Planning + Navigation system"""
    
    def __init__(self, jetson_ip: str, left_port: int = 9001, 
                 right_port: int = 9002, nav_port: int = 9003):
        
        # Camera receivers
        self.left_receiver = OptimizedCameraReceiver(left_port, "LEFT")
        self.right_receiver = OptimizedCameraReceiver(right_port, "RIGHT")
        
        # SLAM components
        try:
            from x99_slam_server import ORBFeatureExtractor, YOLOSegmentator
            self.orb_extractor = ORBFeatureExtractor(n_features=3000)
            self.yolo = YOLOSegmentator()
        except ImportError:
            print("[WARNING] SLAM components not available")
            self.orb_extractor = None
            self.yolo = None
        
        # Path planner
        self.path_planner = PathPlanner(
            grid_width=300,
            grid_height=300,
            resolution=0.05  # 5cm per cell
        )
        
        # Navigation command sender
        self.nav_sender = NavigationCommandSender(jetson_ip, nav_port)
        
        # Map data
        self.map_points = []
        self.robot_pose = np.eye(4)
        
        self.is_running = False
        
        # Goal
        self.current_goal = None
        self.path_planning_enabled = False
    
    def start_system(self):
        """Start all subsystems"""
        print("\n" + "="*70)
        print("  X99 Integrated SLAM + Navigation Server")
        print("="*70)
        
        # Start camera receivers
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
        
        time.sleep(2)
        
        if not self.left_receiver.is_running or not self.right_receiver.is_running:
            print("[ERROR] Failed to start camera receivers")
            return False
        
        print("[OK] Camera receivers started")
        
        # Connect to Jetson for navigation commands
        if self.nav_sender.connect():
            print("[OK] Connected to Jetson for navigation")
        else:
            print("[WARNING] Could not connect to Jetson navigation")
        
        print("="*70 + "\n")
        
        return True
    
    def process_slam(self, frame_left, frame_right):
        """Process SLAM and update map"""
        if self.orb_extractor is None:
            return frame_left, []
        
        # Extract features
        kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
        kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
        
        # Match features
        matches = self.orb_extractor.match_features(desc_left, desc_right)
        
        # Simple 3D point reconstruction (simplified)
        # In production, use proper triangulation with calibrated cameras
        new_map_points = []
        for match in matches[:50]:  # Limit points
            pt = kp_left[match.queryIdx].pt
            # Simplified depth estimation
            depth = 2.0  # Placeholder
            x = (pt[0] - 320) * depth / 500
            y = (pt[1] - 240) * depth / 500
            z = depth
            
            new_map_points.append({
                'position': [x, y, z],
                'color': [0, 255, 0]
            })
        
        self.map_points = new_map_points
        
        # YOLO segmentation
        processed = frame_left.copy()
        if self.yolo and self.yolo.model:
            results = self.yolo.segment(frame_left, conf=0.5)
            processed = self.yolo.draw_segments(frame_left, results)
        
        # Draw features
        frame_with_features = cv2.drawKeypoints(
            processed, kp_left, None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        
        # Overlay info
        cv2.putText(frame_with_features, f"Features: {len(kp_left)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_with_features, f"Map Points: {len(self.map_points)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if self.current_goal:
            cv2.putText(frame_with_features, 
                       f"Goal: ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f})",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return frame_with_features, new_map_points
    
    def update_path_planning(self):
        """Update path planner with latest map"""
        if len(self.map_points) > 0:
            self.path_planner.update_map(self.map_points, self.robot_pose)
    
    def set_goal_and_plan(self, goal_x: float, goal_y: float):
        """Set navigation goal and plan path"""
        print(f"\n[Navigation] Planning path to ({goal_x:.2f}, {goal_y:.2f})")
        
        self.current_goal = (goal_x, goal_y)
        
        # Update map
        self.update_path_planning()
        
        # Plan path
        path = self.path_planner.plan_to_goal(goal_x, goal_y)
        
        if path:
            print(f"[Navigation] Path found with {len(path)} waypoints")
            
            # Send to Jetson
            if self.nav_sender.connected:
                success = self.nav_sender.send_path(path)
                if success:
                    print("[Navigation] Path sent to Jetson")
                else:
                    print("[Navigation] Failed to send path")
            
            return path
        else:
            print("[Navigation] No path found!")
            return None
    
    def run(self):
        """Main control loop"""
        if not self.start_system():
            return
        
        self.is_running = True
        
        print("[INFO] System running")
        print("[INFO] Commands:")
        print("  - Press 'g' to set goal (click on map)")
        print("  - Press 'p' to toggle path planning visualization")
        print("  - Press 'q' to quit")
        print()
        
        # Mouse callback for setting goals
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and self.path_planning_enabled:
                # Convert click to world coordinates
                grid_x = x * self.path_planner.grid.width // 800
                grid_y = y * self.path_planner.grid.height // 600
                world_x, world_y = self.path_planner.grid.grid_to_world(grid_x, grid_y)
                
                self.set_goal_and_plan(world_x, world_y)
        
        cv2.namedWindow('SLAM View')
        cv2.namedWindow('Path Planning')
        cv2.setMouseCallback('Path Planning', mouse_callback)
        
        frame_count = 0
        last_pose_send = time.time()
        
        try:
            while self.is_running:
                # Get frames
                frame_left = self.left_receiver.get_frame(timeout=0.1)
                frame_right = self.right_receiver.get_frame(timeout=0.1)
                
                if frame_left is None or frame_right is None:
                    continue
                
                # Process SLAM
                processed, new_points = self.process_slam(frame_left, frame_right)
                
                # Update map periodically
                if frame_count % 10 == 0:
                    self.update_path_planning()
                
                # Send pose to Jetson
                if time.time() - last_pose_send > 0.1:  # 10 Hz
                    x, y = self.robot_pose[0, 3], self.robot_pose[1, 3]
                    theta = np.arctan2(self.robot_pose[1, 0], self.robot_pose[0, 0])
                    self.nav_sender.send_pose(x, y, theta)
                    last_pose_send = time.time()
                
                # Display
                cv2.imshow('SLAM View', processed)
                
                # Show path planning
                if self.path_planning_enabled:
                    path_vis = self.path_planner.visualize_path()
                    path_vis = cv2.resize(path_vis, (800, 600))
                    cv2.imshow('Path Planning', path_vis)
                
                frame_count += 1
                
                # Handle keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('p'):
                    self.path_planning_enabled = not self.path_planning_enabled
                    print(f"[INFO] Path planning visualization: {self.path_planning_enabled}")
                elif key == ord('g'):
                    self.path_planning_enabled = True
                    print("[INFO] Click on map to set goal")
                
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted")
        
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown system"""
        self.is_running = False
        
        self.left_receiver.is_running = False
        self.right_receiver.is_running = False
        
        self.nav_sender.disconnect()
        
        cv2.destroyAllWindows()
        
        print("\n[INFO] System shutdown complete")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='X99 Integrated SLAM + Navigation Server'
    )
    parser.add_argument('--jetson-ip', type=str, required=True,
                       help='Jetson Nano IP address')
    parser.add_argument('--left-port', type=int, default=9001)
    parser.add_argument('--right-port', type=int, default=9002)
    parser.add_argument('--nav-port', type=int, default=9003)
    
    args = parser.parse_args()
    
    server = IntegratedSLAMNavigationServer(
        jetson_ip=args.jetson_ip,
        left_port=args.left_port,
        right_port=args.right_port,
        nav_port=args.nav_port
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")

if __name__ == "__main__":
    main()