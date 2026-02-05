#!/usr/bin/env python3
"""
X99 Headless Web Server - SLAM Improved Edition
Chạy trên server không màn hình, hiển thị trên web interface

Features:
- ✅ Drift correction SLAM
- ✅ 2D Navigation map
- ✅ Path planning
- ✅ Real-time 3D visualization
- ✅ WebSocket streaming
- ✅ Camera height adaptation
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
import queue

# Import improved SLAM components
from x99_slam_improved_drift_correction import DriftCorrectedSLAM
from x99_navigation_map_2d import NavigationMap2D
from x99_persistent_map import PersistentMap
from x99_camera_height_adapter import CameraHeightAdapter

# Import existing camera receiver
try:
    from x99_headless import OptimizedCameraReceiver
except ImportError:
    print("[Warning] x99_headless not found, using basic receiver")
    OptimizedCameraReceiver = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'x99_slam_improved_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', ping_timeout=60)

# Global server instance
web_server = None

class HeadlessSLAMServer:
    """
    Headless SLAM server with web interface
    Runs all processing in background threads
    """
    
    def __init__(self, left_port: int = 9002, right_port: int = 9001,
                 camera_height: float = None):
        
        print("\n" + "="*70)
        print("  X99 HEADLESS SLAM SERVER - IMPROVED EDITION")
        print("="*70)
        
        # Camera receivers
        if OptimizedCameraReceiver:
            self.left_receiver = OptimizedCameraReceiver(left_port, "LEFT")
            self.right_receiver = OptimizedCameraReceiver(right_port, "RIGHT")
        else:
            print("[ERROR] Camera receiver not available!")
            return
        
        # ===== IMPROVED SLAM COMPONENTS =====
        print("\n[Init] Loading SLAM Improved components...")
        
        # Drift-corrected SLAM
        self.slam = DriftCorrectedSLAM(
            n_features=3000,
            baseline=0.10,
            focal_length=500
        )
        
        # 2D Navigation Map
        self.nav_map = NavigationMap2D(
            width=20.0,
            height=20.0,
            resolution=0.05
        )
        
        # Persistent 3D Map
        self.persistent_map = PersistentMap(
            grid_size=800,
            resolution=0.02,
            voxel_size=0.05
        )
        
        # Camera Height Adapter
        self.height_adapter = CameraHeightAdapter(
            camera_height=camera_height,
            baseline=0.10,
            focal_length=500,
            auto_calibrate=(camera_height is None)
        )
        
        # ===== STATE =====
        self.is_running = False
        self.slam_active = False
        self.start_time = None
        
        # Frame buffers
        self.latest_left = None
        self.latest_right = None
        self.latest_slam_vis = None
        self.latest_nav_map = None
        self.frame_lock = threading.Lock()
        
        # Navigation
        self.current_path = None
        self.current_goal = None
        
        # Stats
        self.stats = {
            'left_fps': 0.0,
            'right_fps': 0.0,
            'slam_fps': 0.0,
            'frame_count': 0,
            'tracking_quality': 'INIT',
            'keyframes': 0,
            'loop_closures': 0,
            'drift_corrections': 0,
            'map_points': 0,
            'position': [0.0, 0.0, 0.0],
            'camera_height': camera_height if camera_height else 0.0,
            'uptime': 0
        }
        
        # FPS counters
        self.left_fps_counter = 0
        self.right_fps_counter = 0
        self.slam_fps_counter = 0
        self.last_fps_time = time.time()
        
        # Update intervals
        self.map_update_interval = 0.5  # Update nav map every 0.5s
        self.web_update_interval = 0.1  # Send to web every 0.1s
        self.last_map_update = time.time()
        self.last_web_update = time.time()
        
        print("[Init] ✓ Server initialized")
    
    def start_receivers(self):
        """Start camera receivers"""
        print("\n[Receivers] Starting camera receivers...")
        
        # Start left
        if not self.left_receiver.start_server():
            print("[ERROR] Left receiver failed to start")
            return False
        
        left_thread = threading.Thread(
            target=self.left_receiver.receive_frames,
            daemon=True
        )
        left_thread.start()
        
        # Start right
        if not self.right_receiver.start_server():
            print("[ERROR] Right receiver failed to start")
            return False
        
        right_thread = threading.Thread(
            target=self.right_receiver.receive_frames,
            daemon=True
        )
        right_thread.start()
        
        print("[Receivers] ✓ Both cameras connected")
        return True
    
    def start_slam_thread(self):
        """Start SLAM processing thread"""
        self.slam_active = True
        slam_thread = threading.Thread(target=self._slam_processing_loop, daemon=True)
        slam_thread.start()
        print("[SLAM] Processing thread started")
    
    def stop_slam(self):
        """Stop SLAM processing"""
        self.slam_active = False
        print("[SLAM] Processing stopped")
    
    def _slam_processing_loop(self):
        """
        Main SLAM processing loop
        Runs independently of web streaming
        """
        print("[SLAM] Processing loop started")
        
        while self.slam_active:
            try:
                # Get frames
                left = self.left_receiver.get_frame()
                right = self.right_receiver.get_frame()
                
                if left is None or right is None:
                    time.sleep(0.01)
                    continue
                
                # Store latest frames
                with self.frame_lock:
                    self.latest_left = left.copy()
                    self.latest_right = right.copy()
                
                # ===== SLAM PROCESSING =====
                start_time = time.time()
                
                # Process stereo frame
                current_pose, tracking = self.slam.process_frame(left, right)
                
                # Extract position
                pos_x = current_pose[0, 3]
                pos_y = current_pose[1, 3]
                pos_z = current_pose[2, 3]
                
                # Get 3D points from SLAM
                points_3d_camera = self.slam.prev_points_3d
                
                if len(points_3d_camera) > 0:
                    # Process with height adapter
                    points_filtered, classifications = self.height_adapter.process_point_cloud(
                        np.array(points_3d_camera)
                    )
                    
                    if len(points_filtered) > 0:
                        # Transform to world frame
                        points_world = []
                        for pt in points_filtered:
                            pt_h = np.append(pt, 1.0)
                            pt_world = current_pose @ pt_h
                            points_world.append(pt_world[:3])
                        
                        points_world = np.array(points_world)
                        
                        # Update navigation map periodically
                        if time.time() - self.last_map_update > self.map_update_interval:
                            self.nav_map.update_from_point_cloud(
                                points_world,
                                robot_pose_2d=(pos_x, pos_z)
                            )
                            self.nav_map.raycast_free_space()
                            self.last_map_update = time.time()
                        
                        # Update persistent map
                        colors = self._extract_colors(left, points_filtered)
                        robot_pose_3d = np.array([pos_x, pos_y, pos_z])
                        
                        self.persistent_map.add_point_cloud(
                            points_world,
                            colors,
                            robot_pose_3d
                        )
                
                # Create visualization
                vis = self._create_slam_visualization(left, current_pose, tracking)
                nav_vis = self.nav_map.visualize(show_costmap=False, path=self.current_path)
                
                with self.frame_lock:
                    self.latest_slam_vis = vis
                    self.latest_nav_map = nav_vis
                
                # Update stats
                self.stats['frame_count'] += 1
                self.stats['tracking_quality'] = tracking
                self.stats['keyframes'] = len(self.slam.keyframes)
                self.stats['loop_closures'] = len(self.slam.loop_closures)
                self.stats['drift_corrections'] = self.slam.total_drift_corrections
                self.stats['map_points'] = len(self.persistent_map.voxel_grid)
                self.stats['position'] = [float(pos_x), float(pos_y), float(pos_z)]
                
                # Update camera height
                height_status = self.height_adapter.get_status()
                self.stats['camera_height'] = height_status['camera_height']
                
                # FPS calculation
                self.slam_fps_counter += 1
                
                # Sleep to control rate
                elapsed = time.time() - start_time
                if elapsed < 0.033:  # ~30 FPS max
                    time.sleep(0.033 - elapsed)
                
            except Exception as e:
                print(f"[SLAM] Error in processing loop: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _extract_colors(self, image, points_3d_camera):
        """Extract RGB colors for 3D points"""
        if len(points_3d_camera) == 0:
            return np.array([])
        
        colors = []
        h, w = image.shape[:2]
        K = self.slam.K
        
        for pt in points_3d_camera:
            if pt[2] > 0:
                u = int(K[0,0] * pt[0] / pt[2] + K[0,2])
                v = int(K[1,1] * pt[1] / pt[2] + K[1,2])
                
                if 0 <= u < w and 0 <= v < h:
                    color = image[v, u]
                    colors.append(color[::-1])  # BGR to RGB
                else:
                    colors.append([128, 128, 128])
            else:
                colors.append([128, 128, 128])
        
        return np.array(colors)
    
    def _create_slam_visualization(self, img, pose, tracking):
        """Create visualization with overlay"""
        vis = img.copy()
        
        # Info overlay
        info = [
            f"Frame: {self.stats['frame_count']}",
            f"Tracking: {tracking}",
            f"Keyframes: {len(self.slam.keyframes)}",
            f"Loops: {len(self.slam.loop_closures)}",
            f"Drift Fix: {self.slam.total_drift_corrections}",
            f"Height: {self.stats['camera_height']:.2f}m",
            f"Pos: ({pose[0,3]:.2f}, {pose[2,3]:.2f})m"
        ]
        
        y = 25
        for line in info:
            # Background
            text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.rectangle(vis, (5, y-18), (15+text_size[0], y+5), (0,0,0), -1)
            
            # Text color
            if tracking == 'GOOD':
                color = (0, 255, 0)
            elif tracking == 'LOST':
                color = (0, 0, 255)
            else:
                color = (255, 255, 0)
            
            cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y += 22
        
        return vis
    
    def update_fps_stats(self):
        """Update FPS statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed >= 1.0:
            self.stats['left_fps'] = self.left_fps_counter / elapsed
            self.stats['right_fps'] = self.right_fps_counter / elapsed
            self.stats['slam_fps'] = self.slam_fps_counter / elapsed
            
            self.left_fps_counter = 0
            self.right_fps_counter = 0
            self.slam_fps_counter = 0
            self.last_fps_time = current_time
            
            # Uptime
            if self.start_time:
                self.stats['uptime'] = int(current_time - self.start_time)
    
    def get_frame_jpeg(self, frame_type: str):
        """
        Get frame as JPEG bytes for streaming
        frame_type: 'left', 'right', 'slam', 'nav'
        """
        with self.frame_lock:
            if frame_type == 'left':
                frame = self.latest_left
            elif frame_type == 'right':
                frame = self.latest_right
            elif frame_type == 'slam':
                frame = self.latest_slam_vis
            elif frame_type == 'nav':
                frame = self.latest_nav_map
            else:
                return None
        
        if frame is None:
            # Return blank frame
            frame = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(frame, "No Signal", (80, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Encode to JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buffer.tobytes()
    
    def set_navigation_goal(self, goal_x: float, goal_z: float):
        """Set navigation goal and plan path"""
        print(f"[Navigation] Planning path to ({goal_x:.2f}, {goal_z:.2f})")
        
        path = self.nav_map.find_path_astar(goal_x, goal_z)
        
        if path:
            self.current_path = path
            self.current_goal = (goal_x, goal_z)
            print(f"[Navigation] Path found: {len(path)} waypoints")
            return True
        else:
            print("[Navigation] No path found")
            return False
    
    def save_maps(self):
        """Save all maps"""
        timestamp = int(time.time())
        
        nav_file = f"navmap_{timestamp}.npz"
        persist_file = f"persist_{timestamp}.npz"
        
        self.nav_map.save_map(nav_file)
        self.persistent_map.save_map(persist_file)
        
        print(f"[Maps] Saved: {nav_file}, {persist_file}")
        return [nav_file, persist_file]
    
    def reset_slam(self):
        """Reset SLAM system"""
        self.slam.reset()
        self.persistent_map.clear_map()
        self.current_path = None
        self.current_goal = None
        print("[System] SLAM reset")
    
    def start(self):
        """Start the server"""
        self.is_running = True
        self.start_time = time.time()
        
        # Start receivers
        if not self.start_receivers():
            return False
        
        # Start SLAM thread
        self.start_slam_thread()
        
        # Start FPS update thread
        fps_thread = threading.Thread(target=self._fps_update_loop, daemon=True)
        fps_thread.start()
        
        print("\n[Server] ✓ All systems running")
        print("[Server] Web interface ready")
        
        return True
    
    def _fps_update_loop(self):
        """Update FPS stats periodically"""
        while self.is_running:
            self.update_fps_stats()
            time.sleep(1.0)
    
    def stop(self):
        """Stop the server"""
        self.is_running = False
        self.slam_active = False
        self.left_receiver.is_running = False
        self.right_receiver.is_running = False
        print("[Server] Stopped")


# ===== FLASK ROUTES =====

@app.route('/')
def index():
    """Main page"""
    return render_template('slam_improved_web.html')

@app.route('/stats')
def get_stats():
    """Get current statistics"""
    if web_server:
        return jsonify(web_server.stats)
    return jsonify({})

# Video streaming routes
def gen_frames(frame_type):
    """Generate frames for streaming"""
    while True:
        if web_server:
            frame = web_server.get_frame_jpeg(frame_type)
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)  # ~30 FPS

@app.route('/video/left')
def video_left():
    return Response(gen_frames('left'),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/right')
def video_right():
    return Response(gen_frames('right'),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/slam')
def video_slam():
    return Response(gen_frames('slam'),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/nav')
def video_nav():
    return Response(gen_frames('nav'),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

# ===== SOCKETIO EVENTS =====

@socketio.on('connect')
def handle_connect():
    print('[WebSocket] Client connected')
    emit('status', {'connected': True})

@socketio.on('disconnect')
def handle_disconnect():
    print('[WebSocket] Client disconnected')

@socketio.on('set_goal')
def handle_set_goal(data):
    """Handle navigation goal request"""
    if web_server:
        goal_x = float(data.get('x', 0))
        goal_z = float(data.get('z', 0))
        
        success = web_server.set_navigation_goal(goal_x, goal_z)
        emit('goal_response', {'success': success})

@socketio.on('save_maps')
def handle_save_maps():
    """Save maps"""
    if web_server:
        files = web_server.save_maps()
        emit('save_response', {'success': True, 'files': files})

@socketio.on('reset_slam')
def handle_reset():
    """Reset SLAM"""
    if web_server:
        web_server.reset_slam()
        emit('reset_response', {'success': True})

@socketio.on('get_map_data')
def handle_get_map_data():
    """Get 3D map data for visualization"""
    if web_server:
        map_data = web_server.persistent_map.get_map_data_for_web()
        emit('map_data', map_data)


# ===== MAIN =====
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 Headless SLAM Server')
    parser.add_argument('--left-port', type=int, default=9002,
                       help='Left camera port')
    parser.add_argument('--right-port', type=int, default=9001,
                       help='Right camera port')
    parser.add_argument('--web-port', type=int, default=1234,
                       help='Web server port')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Web server host')
    parser.add_argument('--camera-height', type=float, default=None,
                       help='Camera height in meters (auto-detect if not provided)')
    
    args = parser.parse_args()
    
    # Create global server instance
    global web_server
    web_server = HeadlessSLAMServer(
        left_port=args.left_port,
        right_port=args.right_port,
        camera_height=args.camera_height
    )
    
    # Start server
    if web_server.start():
        print(f"\n{'='*70}")
        print(f"  🌐 Web Interface: http://{args.host}:{args.web_port}")
        print(f"{'='*70}\n")
        
        # Run Flask app
        socketio.run(app, host=args.host, port=args.web_port, debug=False)
    else:
        print("[ERROR] Server failed to start")

if __name__ == "__main__":
    main()