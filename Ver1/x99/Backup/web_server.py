from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import cv2
import numpy as np
import json
import base64
from threading import Lock, Thread
import time


app = Flask(__name__)
app.config['SECRET_KEY'] = 'slam_secret_key'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')


class SLAMServer:
    """Web server for SLAM visualization"""
    
    def __init__(self):
        self.lock = Lock()
        
        # Current frame data
        self.current_frame = None
        self.depth_map = None
        self.semantic_map = None
        
        # SLAM state
        self.trajectory = []
        self.map_points = []
        self.map_colors = []
        self.semantic_labels = []
        
        # Statistics
        self.stats = {
            'fps': 0,
            'num_features': 0,
            'num_matches': 0,
            'num_map_points': 0,
            'num_detections': 0
        }
        
        # Client tracking
        self.clients = set()
        
    def update_frame(self, frame, depth=None, semantic=None):
        """Update current frame"""
        with self.lock:
            self.current_frame = frame.copy()
            if depth is not None:
                self.depth_map = depth.copy()
            if semantic is not None:
                self.semantic_map = semantic.copy()
    
    def update_trajectory(self, trajectory):
        """Update camera trajectory"""
        with self.lock:
            self.trajectory = trajectory.tolist() if isinstance(trajectory, np.ndarray) else trajectory
    
    def update_map(self, points, colors=None, semantic=None):
        """Update 3D map points"""
        with self.lock:
            self.map_points = points.tolist() if isinstance(points, np.ndarray) else points
            if colors is not None:
                self.map_colors = colors.tolist() if isinstance(colors, np.ndarray) else colors
            if semantic is not None:
                self.semantic_labels = semantic.tolist() if isinstance(semantic, np.ndarray) else semantic
    
    def update_stats(self, stats_dict):
        """Update statistics"""
        with self.lock:
            self.stats.update(stats_dict)
    
    def get_frame_jpeg(self):
        """Get current frame as JPEG"""
        with self.lock:
            if self.current_frame is None:
                return None
            
            # Encode frame
            _, buffer = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return buffer.tobytes()
    
    def get_depth_jpeg(self):
        """Get depth map as colored JPEG"""
        with self.lock:
            if self.depth_map is None:
                return None
            
            # Normalize depth for visualization
            depth_normalized = cv2.normalize(self.depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            
            # Encode
            _, buffer = cv2.imencode('.jpg', depth_colored, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return buffer.tobytes()
    
    def get_map_data(self):
        """Get map data for visualization"""
        with self.lock:
            return {
                'trajectory': self.trajectory,
                'points': self.map_points[:10000],  # Limit points for performance
                'colors': self.map_colors[:10000] if self.map_colors else None,
                'semantic': self.semantic_labels[:10000] if self.semantic_labels else None
            }
    
    def get_stats(self):
        """Get current statistics"""
        with self.lock:
            return self.stats.copy()


# Global SLAM server instance
slam_server = SLAMServer()


@app.route('/')
def index():
    """Serve main page"""
    return render_template('index.html')


@app.route('/api/stats')
def get_stats():
    """API endpoint for statistics"""
    return jsonify(slam_server.get_stats())


@app.route('/api/map_data')
def get_map_data():
    """API endpoint for map data"""
    return jsonify(slam_server.get_map_data())


def generate_video_stream():
    """Generate video stream"""
    while True:
        frame = slam_server.get_frame_jpeg()
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)  # ~30 FPS


def generate_depth_stream():
    """Generate depth stream"""
    while True:
        depth = slam_server.get_depth_jpeg()
        if depth is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + depth + b'\r\n')
        time.sleep(0.033)  # ~30 FPS


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_video_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/depth_feed')
def depth_feed():
    """Depth streaming route"""
    return Response(generate_depth_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f'Client connected: {id}')
    slam_server.clients.add(id)
    emit('connection_response', {'status': 'connected'})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f'Client disconnected')
    if id in slam_server.clients:
        slam_server.clients.remove(id)


@socketio.on('request_update')
def handle_update_request():
    """Handle update request from client"""
    # Send map data
    map_data = slam_server.get_map_data()
    emit('map_update', map_data)
    
    # Send stats
    stats = slam_server.get_stats()
    emit('stats_update', stats)


def broadcast_updates():
    """Background thread to broadcast updates to all clients"""
    while True:
        time.sleep(0.1)  # 10 Hz update rate
        
        if len(slam_server.clients) > 0:
            with app.app_context():
                # Get data
                map_data = slam_server.get_map_data()
                stats = slam_server.get_stats()
                
                # Broadcast to all clients
                socketio.emit('map_update', map_data, broadcast=True)
                socketio.emit('stats_update', stats, broadcast=True)


def run_server(host='0.0.0.0', port=5000, debug=False):
    """Run Flask server"""
    # Start background thread for broadcasting
    broadcast_thread = Thread(target=broadcast_updates, daemon=True)
    broadcast_thread.start()
    
    print(f"Starting SLAM visualization server on http://{host}:{port}")
    socketio.run(app, host=host, port=port, debug=debug, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    run_server()
