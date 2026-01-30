#!/usr/bin/env python3
"""
X99 Web Server with Network Stream Receiver
Receives streams from Jetson Nano and provides web visualization
"""

from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
import json
import threading
import time
import socket
import pickle
import struct
import queue

from x99_slam_server import CameraReceiver, ORBFeatureExtractor, YOLOSegmentator, ServerConfig

app = Flask(__name__)
app.config['SECRET_KEY'] = 'slam_secret_key_2024'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global instances
left_receiver = None
right_receiver = None
orb_extractor = None
yolo_segmentator = None
slam_running = False

class SLAMWebServer:
    """Web server with network stream integration"""
    
    def __init__(self, config: ServerConfig):
        self.config = config
        self.left_receiver = CameraReceiver(config.left_port, "LEFT")
        self.right_receiver = CameraReceiver(config.right_port, "RIGHT")
        self.orb_extractor = ORBFeatureExtractor(n_features=3000)
        self.yolo = YOLOSegmentator()
        
        self.is_streaming = False
        self.map_data = {
            'points': [],
            'camera_poses': [],
            'stats': {
                'num_points': 0,
                'num_poses': 0,
                'fps': 0
            }
        }
    
    def start_receivers(self):
        """Start camera receivers"""
        print("[WEB] Starting camera receivers...")
        
        # Start left camera
        left_thread = threading.Thread(
            target=lambda: (
                self.left_receiver.start_server() and
                self.left_receiver.receive_frames()
            ),
            daemon=True
        )
        left_thread.start()
        
        # Start right camera
        right_thread = threading.Thread(
            target=lambda: (
                self.right_receiver.start_server() and
                self.right_receiver.receive_frames()
            ),
            daemon=True
        )
        right_thread.start()
        
        time.sleep(2)
        return self.left_receiver.is_running and self.right_receiver.is_running
    
    def process_frames(self):
        """Process frames and generate video feed"""
        frame_count = 0
        start_time = time.time()
        
        while self.is_streaming:
            frame_left = self.left_receiver.get_frame()
            frame_right = self.right_receiver.get_frame()
            
            if frame_left is None or frame_right is None:
                continue
            
            # Extract features
            kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
            kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
            
            # Match features
            matches = self.orb_extractor.match_features(desc_left, desc_right)
            
            # YOLO segmentation
            processed = frame_left.copy()
            if self.yolo and self.yolo.model:
                results = self.yolo.segment(frame_left)
                processed = self.yolo.draw_segments(frame_left, results)
            
            # Draw features
            frame_with_features = cv2.drawKeypoints(
                processed, kp_left, None,
                color=(0, 255, 0),
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )
            
            # Add overlay info
            cv2.putText(frame_with_features, f"Features: {len(kp_left)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame_with_features, f"Matches: {len(matches)}", (10, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Update stats
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                self.map_data['stats']['fps'] = int(fps)
                self.map_data['stats']['num_points'] = len(matches)
            
            # Encode and yield
            ret, buffer = cv2.imencode('.jpg', frame_with_features)
            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)  # ~30 FPS

web_server = None

@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    if web_server and web_server.is_streaming:
        return Response(web_server.process_frames(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    return "SLAM not running", 503

@app.route('/api/status')
def get_status():
    """Get system status"""
    if web_server:
        return jsonify({
            'running': web_server.is_streaming,
            'left_connected': web_server.left_receiver.is_running,
            'right_connected': web_server.right_receiver.is_running,
            'stats': web_server.map_data['stats']
        })
    return jsonify({'running': False})

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('[WEB] Client connected')
    emit('status', {'connected': True})

@socketio.on('start_slam')
def handle_start_slam():
    """Start SLAM system"""
    global web_server
    
    if web_server is None:
        config = ServerConfig(
            left_port=9001,
            right_port=9002,
            baseline=0.10
        )
        
        web_server = SLAMWebServer(config)
        
        if web_server.start_receivers():
            web_server.is_streaming = True
            emit('slam_started', {'success': True})
            
            # Start map update broadcast
            def broadcast_updates():
                while web_server and web_server.is_streaming:
                    socketio.emit('map_update', web_server.map_data)
                    time.sleep(0.1)
            
            threading.Thread(target=broadcast_updates, daemon=True).start()
        else:
            emit('slam_started', {'success': False, 'error': 'Connection failed'})

@socketio.on('stop_slam')
def handle_stop_slam():
    """Stop SLAM system"""
    global web_server
    
    if web_server:
        web_server.is_streaming = False
        emit('slam_stopped', {'success': True})

def run_web_server(host='0.0.0.0', port=5000):
    """Run Flask web server"""
    print(f"\n{'='*60}")
    print(f"  X99 SLAM Web Interface")
    print(f"{'='*60}")
    print(f"Server: http://{host}:{port}")
    print(f"Waiting for Jetson Nano streams on ports 9001, 9002")
    print(f"{'='*60}\n")
    
    socketio.run(app, host=host, port=port, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    run_web_server()
