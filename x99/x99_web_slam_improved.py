#!/usr/bin/env python3
"""
X99 Web SLAM Interface - IMPROVED with Full Pose Tracking
Headless server with web-based visualization
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
from typing import List, Tuple, Optional, Dict

# Import improved SLAM tracker
try:
    from x99_slam_server_improved import StereoSLAMTracker, YOLOSegmentator
    SLAM_AVAILABLE = True
except ImportError:
    print("[Warning] x99_slam_server_improved not found, using fallback")
    SLAM_AVAILABLE = False

# Import supporting modules
from stereo_depth_mapping_optimized import StereoDepthMapper, OccupancyGridMapper, ObstacleAvoidance
from persistent_map import PersistentMap

# Import semantic filtering (NEW!)
try:
    from semantic_filtering import SemanticFilter, SemanticOccupancyGrid
    SEMANTIC_AVAILABLE = True
    print("[Web] Semantic filtering enabled")
except ImportError:
    print("[Warning] semantic_filtering not found")
    SEMANTIC_AVAILABLE = False
    SemanticFilter = None
    SemanticOccupancyGrid = None

try:
    from path_planning import PathPlanner
    PATH_PLANNING_AVAILABLE = True
except ImportError:
    print("[Warning] path_planning not found")
    PATH_PLANNING_AVAILABLE = False

app = Flask(__name__)
app.config['SECRET_KEY'] = 'slam_improved_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global server instance
web_server = None

class CameraReceiver:
    """Simple camera receiver for web interface"""
    
    def __init__(self, port: int, name: str):
        import socket
        import struct
        import queue
        
        self.port = port
        self.name = name
        self.server_socket = None
        self.client_socket = None
        self.is_running = False
        self.frame_queue = queue.Queue(maxsize=5)
        
    def start_server(self):
        """Start TCP server"""
        import socket
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("0.0.0.0", self.port))
            self.server_socket.listen(1)
            
            print(f"[{self.name}] Waiting for connection on port {self.port}...")
            self.client_socket, addr = self.server_socket.accept()
            print(f"[{self.name}] Connected from {addr}")
            return True
        except Exception as e:
            print(f"[{self.name}] Server start failed: {e}")
            return False
    
    def receive_frames(self):
        """Receive frames from client"""
        import struct
        import pickle
        import queue
        
        self.is_running = True
        data = b""
        payload_size = struct.calcsize("!I")
        
        try:
            while self.is_running:
                # Receive message size
                while len(data) < payload_size:
                    packet = self.client_socket.recv(4096)
                    if not packet:
                        return
                    data += packet
                
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("!L", packed_msg_size)[0]
                
                # Receive frame data
                while len(data) < msg_size:
                    packet = self.client_socket.recv(4096)
                    if not packet:
                        return
                    data += packet
                
                frame_data = data[:msg_size]
                data = data[msg_size:]
                
                # Deserialize
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is None:
                    print(f"[{self.name}] Failed to decode JPEG")
                    continue
                
                # Add to queue
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                
                self.frame_queue.put(frame)
                
        except Exception as e:
            print(f"[{self.name}] Receive error: {e}")
        finally:
            self.cleanup()
    
    def get_frame(self):
        """Get latest frame"""
        import queue
        try:
            return self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            return None
    
    def cleanup(self):
        """Clean up resources"""
        self.is_running = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

class WebSLAMServer:
    """Web-based SLAM server with full tracking"""
    
    def __init__(self, left_port=9001, right_port=9002, calibration_file=None):
        # Camera receivers
        self.left_receiver = CameraReceiver(left_port, "LEFT")
        self.right_receiver = CameraReceiver(right_port, "RIGHT")
        
        # SLAM tracker with FULL pose estimation
        if SLAM_AVAILABLE:
            self.slam_tracker = StereoSLAMTracker(
                n_features=1500,
                calibration_file=calibration_file
            )
            self.yolo = YOLOSegmentator()
            print("[Web] SLAM tracker initialized with pose estimation")
        else:
            self.slam_tracker = None
            self.yolo = None
            print("[Web] SLAM not available")
        
        # Semantic filtering (NEW!)
        if SEMANTIC_AVAILABLE and SemanticFilter:
            self.semantic_filter = SemanticFilter(
                filter_dynamic=True,
                min_confidence=0.5
            )
            self.semantic_grid = SemanticOccupancyGrid(
                grid_size=400,
                resolution=0.05
            )
            print("[Web] Semantic filtering enabled")
        else:
            self.semantic_filter = None
            self.semantic_grid = None
        
        # Depth mapping
        self.depth_mapper = StereoDepthMapper(baseline=0.1, focal_length=500)
        self.grid_mapper = OccupancyGridMapper(grid_size=400, resolution=0.05, max_range=5.0)
        
        # Persistent map
        self.persistent_map = PersistentMap(grid_size=800, resolution=0.02)
        
        # Path planner
        if PATH_PLANNING_AVAILABLE:
            self.path_planner = PathPlanner(grid_width=200, grid_height=200, resolution=0.05)
            print("[Web] Path planner enabled")
        else:
            self.path_planner = None
        
        # State
        self.is_running = False
        self.streaming = False
        self.slam_processing = False
        
        # Latest frames and visualizations
        self.latest_frames = {
            'left': None,
            'right': None,
            'slam': None,
            'depth': None,
            'grid': None,
            'map': None,
            'path': None,
            'semantic': None  # NEW: Semantic view
        }
        self.frames_lock = threading.Lock()
        
        # SLAM state
        self.current_pose = np.eye(4, dtype=np.float32)
        self.pose_2d = (0.0, 0.0, 0.0)  # x, y, theta
        self.tracking_quality = 'INIT'
        
        # Statistics
        self.stats = {
            'left_fps': 0.0,
            'right_fps': 0.0,
            'slam_fps': 0.0,
            'left_frames': 0,
            'right_frames': 0,
            'slam_frames': 0,
            'uptime': 0,
            # SLAM stats
            'tracking_quality': 'INIT',
            'num_keyframes': 0,
            'num_map_points': 0,
            'num_tracked_points': 0,
            'num_inliers': 0,
            'pose_x': 0.0,
            'pose_y': 0.0,
            'pose_theta': 0.0,
            # Map stats
            'persistent_map_points': 0,
            'trajectory_length': 0,
            'obstacles_detected': 0,
            # Semantic stats (NEW!)
            'semantic_filtered': 0,
            'dynamic_objects': 0,
            'static_objects': 0
        }
        
        self.start_time = None
        self.last_slam_time = time.time()
    
    def start_receivers(self):
        """Start camera receivers"""
        print("\n" + "=" * 70)
        print("  X99 Web SLAM - Improved with Full Tracking")
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
        
        # Start receivers in threads
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
        Process stereo frame with FULL SLAM tracking + SEMANTIC FILTERING
        """
        if not SLAM_AVAILABLE or self.slam_tracker is None:
            return frame_left
        
        try:
            # Run YOLO segmentation FIRST
            yolo_results = None
            if self.yolo and self.yolo.model:
                try:
                    yolo_results = self.yolo.segment(frame_left, conf=0.5)
                except Exception as e:
                    print(f"[YOLO] Error: {e}")
            
            # Run SLAM tracking with pose estimation
            current_pose, map_points, tracking_quality, slam_stats = \
                self.slam_tracker.process_stereo_frame(frame_left, frame_right)
            
            # Update state
            self.current_pose = current_pose
            self.pose_2d = self.slam_tracker.get_current_pose_2d()
            self.tracking_quality = tracking_quality
            
            # ✨ SEMANTIC FILTERING (NEW!)
            if self.semantic_filter and yolo_results and len(map_points) > 0:
                # Get camera matrix from SLAM tracker
                camera_matrix = self.slam_tracker.K
                image_size = (frame_left.shape[1], frame_left.shape[0])
                
                # Filter dynamic objects and classify
                map_points = self.semantic_filter.filter_and_classify_points(
                    map_points,
                    yolo_results,
                    camera_matrix,
                    current_pose,
                    image_size
                )
                
                # Update stats
                filter_stats = self.semantic_filter.get_statistics()
                self.stats['semantic_filtered'] = filter_stats['filtered_points']
                self.stats['dynamic_objects'] = filter_stats['dynamic_filtered']
                self.stats['static_objects'] = filter_stats['static_classified']
            
            # Update stats
            self.stats.update({
                'tracking_quality': tracking_quality,
                'num_keyframes': slam_stats['num_keyframes'],
                'num_map_points': len(map_points),  # After filtering!
                'num_tracked_points': slam_stats['num_tracked_points'],
                'num_inliers': slam_stats['num_inliers'],
                'pose_x': round(self.pose_2d[0], 3),
                'pose_y': round(self.pose_2d[1], 3),
                'pose_theta': round(self.pose_2d[2], 3)
            })
            
            # ✨ UPDATE SEMANTIC GRID (NEW!)
            if self.semantic_grid and len(map_points) > 0:
                self.semantic_grid.update_from_semantic_points(map_points)
                self.semantic_grid.inflate_obstacles_semantic()
            
            # Update persistent map with FILTERED map points
            if len(map_points) > 0:
                points_array = np.array([mp['position'] for mp in map_points])
                colors_array = np.array([mp.get('color', [128, 128, 128]) for mp in map_points])
                robot_pose_3d = [self.pose_2d[0], 0.0, self.pose_2d[1]]
                
                self.persistent_map.add_point_cloud(
                    points_array, colors_array, robot_pose_3d
                )
                
                self.stats['persistent_map_points'] = len(self.persistent_map.voxel_grid)
                self.stats['trajectory_length'] = len(self.persistent_map.trajectory)
            
            # Update path planner with semantic grid if available
            if self.path_planner and tracking_quality == 'GOOD':
                if self.semantic_grid:
                    # Use semantic occupancy grid (better!)
                    semantic_map_data = self.semantic_grid.get_semantic_map_data()
                    # TODO: Update path planner to use semantic costs
                else:
                    # Fallback to regular map
                    self.path_planner.update_map(map_points, current_pose)
            
            # Create visualization
            vis_frame = self._create_slam_visualization(
                frame_left, map_points, yolo_results, tracking_quality, slam_stats
            )
            
            return vis_frame
            
        except Exception as e:
            print(f"[SLAM] Processing error: {e}")
            import traceback
            traceback.print_exc()
            return frame_left
    
    def _create_slam_visualization(self, frame, map_points, yolo_results, tracking_quality, stats):
        """Create SLAM visualization with overlays + YOLO"""
        # Start with YOLO overlay if available
        if yolo_results and self.yolo:
            vis = self.yolo.draw_segments(frame, yolo_results)
        else:
            vis = frame.copy()
        
        h, w = vis.shape[:2]
        
        # Color based on tracking quality
        if tracking_quality == 'GOOD':
            color = (0, 255, 0)  # Green
        elif tracking_quality == 'LOST':
            color = (0, 0, 255)  # Red
        elif tracking_quality == 'INIT':
            color = (255, 255, 0)  # Yellow
        else:
            color = (128, 128, 128)  # Gray
        
        # Info panel
        info_lines = [
            f"SLAM Status: {tracking_quality}",
            f"Frame: {stats['frame_count']}",
            f"Keyframes: {stats['num_keyframes']}",
            f"Map Points: {len(map_points)}",  # Use actual filtered count
            f"Tracked: {stats['num_inliers']} inliers",
            f"Pose: ({self.pose_2d[0]:.2f}, {self.pose_2d[1]:.2f}, {self.pose_2d[2]:.2f})"
        ]
        
        # Add semantic stats if available
        if self.semantic_filter:
            info_lines.extend([
                f"Filtered: {self.stats.get('semantic_filtered', 0)} pts",
                f"Dynamic: {self.stats.get('dynamic_objects', 0)} obj"
            ])
        
        y_offset = 30
        for line in info_lines:
            # Background
            text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(vis, (5, y_offset - 22), 
                         (15 + text_size[0], y_offset + 5), (0, 0, 0), -1)
            
            # Text
            cv2.putText(vis, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y_offset += 28
        
        # Add FPS
        fps_text = f"FPS: {self.stats['slam_fps']:.1f}"
        cv2.putText(vis, fps_text, (10, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return vis
    
    def slam_processing_loop(self):
        """Main SLAM processing loop"""
        print("[SLAM] Processing loop started")
        
        frame_count = 0
        last_fps_time = time.time()
        fps_counter = 0
        
        while self.is_running:
            if not self.slam_processing:
                time.sleep(0.1)
                continue
            
            # Get frames
            frame_left = self.left_receiver.get_frame()
            frame_right = self.right_receiver.get_frame()
            
            if frame_left is None or frame_right is None:
                continue
            
            # Process SLAM
            slam_vis = self.process_slam_frame(frame_left, frame_right)
            
            # Create other visualizations
            depth_vis = self._create_depth_visualization(frame_left, frame_right)
            grid_vis = self.persistent_map.visualize_2d(show_trajectory=True)
            
            # Create semantic visualization if available
            if self.semantic_grid:
                semantic_vis = self.semantic_grid.visualize_semantic()
                trav_vis = self.semantic_grid.visualize_traversability()
            else:
                semantic_vis = grid_vis.copy()
                trav_vis = grid_vis.copy()
            
            if self.path_planner and self.path_planner.current_path:
                path_vis = self.path_planner.visualize_path()
            else:
                path_vis = grid_vis.copy()
            
            # Update latest frames
            with self.frames_lock:
                self.latest_frames['slam'] = slam_vis
                self.latest_frames['depth'] = depth_vis
                self.latest_frames['grid'] = grid_vis
                self.latest_frames['map'] = semantic_vis  # Semantic map!
                self.latest_frames['path'] = path_vis
                self.latest_frames['semantic'] = trav_vis  # Traversability!
            
            frame_count += 1
            fps_counter += 1
            self.stats['slam_frames'] = frame_count
            
            # Update FPS
            if time.time() - last_fps_time > 1.0:
                self.stats['slam_fps'] = fps_counter / (time.time() - last_fps_time)
                fps_counter = 0
                last_fps_time = time.time()
            
            # Log periodically
            if frame_count % 30 == 0:
                print(f"[SLAM] Frame {frame_count}: {self.tracking_quality}, "
                      f"Pose: ({self.pose_2d[0]:.2f}, {self.pose_2d[1]:.2f})")
    
    def _create_depth_visualization(self, frame_left, frame_right):
        """Create depth map visualization"""
        try:
            disparity = self.depth_mapper.compute_disparity(frame_left, frame_right)
            depth = self.depth_mapper.disparity_to_depth(disparity)
            
            # Normalize for visualization
            depth_vis = depth.copy()
            depth_vis = np.clip(depth_vis, 0, 5.0)
            depth_vis = (depth_vis / 5.0 * 255).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            
            return depth_vis
        except:
            return frame_left
    
    def video_stream_loop(self):
        """Stream video frames"""
        while self.is_running:
            if not self.streaming:
                time.sleep(0.1)
                continue
            
            frame_left = self.left_receiver.get_frame()
            frame_right = self.right_receiver.get_frame()
            
            if frame_left is not None:
                with self.frames_lock:
                    self.latest_frames['left'] = frame_left
                    self.stats['left_frames'] += 1
            
            if frame_right is not None:
                with self.frames_lock:
                    self.latest_frames['right'] = frame_right
                    self.stats['right_frames'] += 1
    
    def update_stats_loop(self):
        """Update statistics"""
        while self.is_running:
            if self.start_time:
                self.stats['uptime'] = int(time.time() - self.start_time)
            time.sleep(1)

# Flask routes
@app.route('/')
def index():
    """Main dashboard"""
    return render_template('index.html')

@app.route('/video_feed/<feed>')
def video_feed(feed):
    """Video streaming endpoint"""
    def generate(feed_name):
        while True:
            if not web_server or not web_server.is_running:
                break
            
            with web_server.frames_lock:
                frame = web_server.latest_frames.get(feed_name)
            
            if frame is None:
                time.sleep(0.033)  # 30 FPS
                continue
            
            # Encode frame
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    return Response(generate(feed),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/stats')
def get_stats():
    """Get current statistics"""
    if web_server:
        return jsonify(web_server.stats)
    return jsonify({})

@app.route('/api/control/start_slam', methods=['POST'])
def start_slam():
    """Start SLAM processing"""
    if web_server:
        web_server.slam_processing = True
        return jsonify({'success': True})
    return jsonify({'success': False})

@app.route('/api/control/stop_slam', methods=['POST'])
def stop_slam():
    """Stop SLAM processing"""
    if web_server:
        web_server.slam_processing = False
        return jsonify({'success': True})
    return jsonify({'success': False})

@app.route('/api/control/reset_slam', methods=['POST'])
def reset_slam():
    """Reset SLAM"""
    if web_server and web_server.slam_tracker:
        # Reinitialize tracker
        web_server.slam_tracker = StereoSLAMTracker(n_features=1500)
        web_server.persistent_map.clear_map()
        return jsonify({'success': True})
    return jsonify({'success': False})

@app.route('/api/plan_path', methods=['POST'])
def plan_path():
    """Plan path to goal"""
    data = request.json
    goal_x = data.get('goal_x', 0)
    goal_y = data.get('goal_y', 0)
    
    if web_server and web_server.path_planner:
        path = web_server.path_planner.plan_to_goal(goal_x, goal_y)
        if path:
            return jsonify({
                'success': True,
                'path': path,
                'num_waypoints': len(path)
            })
    
    return jsonify({'success': False})

@app.route('/api/save_map', methods=['POST'])
def save_map():
    """Save current map"""
    if web_server and web_server.persistent_map:
        filename = f"map_{int(time.time())}.npz"
        web_server.persistent_map.save_map(filename)
        return jsonify({'success': True, 'filename': filename})
    return jsonify({'success': False})

def run_web_server(host='0.0.0.0', port=1234, calibration_file=None):
    """Run Flask web server"""
    global web_server
    
    # Initialize server
    web_server = WebSLAMServer(
        left_port=9001,
        right_port=9002,
        calibration_file=calibration_file
    )
    
    # Start receivers
    if not web_server.start_receivers():
        print("[ERROR] Failed to start receivers")
        return
    
    web_server.is_running = True
    web_server.streaming = True
    web_server.slam_processing = True
    web_server.start_time = time.time()
    
    # Start threads
    threads = [
        threading.Thread(target=web_server.video_stream_loop, daemon=True),
        threading.Thread(target=web_server.slam_processing_loop, daemon=True),
        threading.Thread(target=web_server.update_stats_loop, daemon=True)
    ]
    
    for t in threads:
        t.start()
    
    print(f"\n{'='*70}")
    print(f"  Web Interface Running with FULL SLAM Tracking")
    print(f"{'='*70}")
    print(f"Open browser: http://{host}:{port}")
    print(f"SLAM: AUTO-STARTED with pose estimation")
    print(f"{'='*70}\n")
    
    # Run Flask
    socketio.run(app, host=host, port=port, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 Web SLAM - Improved')
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=1234)
    parser.add_argument('--calibration', type=str, default='calibration_params.npz')
    
    args = parser.parse_args()
    
    try:
        run_web_server(
            host=args.host,
            port=args.port,
            calibration_file=args.calibration
        )
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        if web_server:
            web_server.is_running = False