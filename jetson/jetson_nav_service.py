#!/usr/bin/env python3
"""
Jetson Navigation Service - REST API
Allows X99 server to trigger navigation remotely
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import navigation controller
try:
    from jetson_navigation import NavigationController
except ImportError:
    print("Error: jetson_navigation.py not found")
    sys.exit(1)

app = Flask(__name__)
CORS(app)

# Configuration
X99_SERVER_URL = "http://192.168.1.100:5000"  # Change to your X99 IP
PICO_SERIAL_PORT = "/dev/ttyACM0"             # Change if needed

# Global controller
nav_controller = None
nav_thread = None
nav_lock = threading.Lock()

def init_controller():
    """Initialize navigation controller"""
    global nav_controller
    
    print("=" * 60)
    print("  JETSON NAVIGATION SERVICE")
    print("=" * 60)
    print(f"\nX99 Server: {X99_SERVER_URL}")
    print(f"Pico Port: {PICO_SERIAL_PORT}\n")
    
    nav_controller = NavigationController(X99_SERVER_URL, PICO_SERIAL_PORT)
    
    if not nav_controller.start():
        print("\n✗ Failed to initialize navigation controller")
        return False
    
    print("\n✓ Navigation service ready")
    return True

@app.route('/')
def index():
    """Service info"""
    return jsonify({
        'service': 'Jetson Navigation Service',
        'version': '1.0',
        'endpoints': {
            '/navigate': 'POST - Start navigation',
            '/stop': 'POST - Stop navigation',
            '/status': 'GET - Get status'
        }
    })

@app.route('/navigate', methods=['POST'])
def navigate():
    """
    Start navigation to goal
    POST JSON: {goal_x: int, goal_y: int}
    """
    if not nav_controller:
        return jsonify({
            'success': False, 
            'error': 'Controller not initialized'
        }), 500
    
    data = request.json
    goal_x = data.get('goal_x')
    goal_y = data.get('goal_y')
    
    if goal_x is None or goal_y is None:
        return jsonify({
            'success': False, 
            'error': 'Missing goal_x or goal_y'
        }), 400
    
    # Stop existing navigation
    with nav_lock:
        global nav_thread
        if nav_thread and nav_thread.is_alive():
            print("[Service] Stopping existing navigation...")
            nav_controller.stop_navigation()
            nav_thread.join(timeout=2)
    
    # Start new navigation in background thread
    def nav_task():
        try:
            print(f"[Service] Starting navigation to ({goal_x}, {goal_y})")
            success = nav_controller.navigate_to_goal(int(goal_x), int(goal_y))
            if success:
                print("[Service] Navigation completed successfully")
            else:
                print("[Service] Navigation failed")
        except Exception as e:
            print(f"[Service] Navigation error: {e}")
            import traceback
            traceback.print_exc()
    
    with nav_lock:
        nav_thread = threading.Thread(target=nav_task, daemon=True)
        nav_thread.start()
    
    return jsonify({
        'success': True, 
        'goal': [goal_x, goal_y],
        'message': 'Navigation started'
    })

@app.route('/stop', methods=['POST'])
def stop():
    """Stop current navigation"""
    if not nav_controller:
        return jsonify({'success': False, 'error': 'Controller not initialized'}), 500
    
    print("[Service] Stop requested")
    nav_controller.stop_navigation()
    
    return jsonify({'success': True, 'message': 'Navigation stopped'})

@app.route('/status')
def status():
    """Get navigation status"""
    if not nav_controller:
        return jsonify({
            'ready': False,
            'error': 'Controller not initialized'
        })
    
    with nav_lock:
        is_navigating = nav_thread and nav_thread.is_alive()
    
    return jsonify({
        'ready': True,
        'pico_connected': nav_controller.pico.connected,
        'navigating': is_navigating,
        'navigation_active': nav_controller.navigation_active,
        'goal_reached': nav_controller.goal_reached,
        'current_path_length': len(nav_controller.current_path) if nav_controller.current_path else 0,
        'robot_pose': nav_controller.robot_pose.tolist()
    })

@app.route('/pose')
def get_pose():
    """Get current robot pose"""
    if not nav_controller:
        return jsonify({'error': 'Not initialized'}), 500
    
    pose = nav_controller.get_robot_pose()
    
    return jsonify({
        'x': pose[0],
        'y': pose[1],
        'theta': pose[2]
    })

@app.route('/health')
def health():
    """Health check endpoint"""
    if not nav_controller:
        return jsonify({'status': 'unhealthy', 'reason': 'Not initialized'}), 503
    
    if not nav_controller.pico.connected:
        return jsonify({'status': 'unhealthy', 'reason': 'Pico disconnected'}), 503
    
    return jsonify({'status': 'healthy'})

@app.before_first_request
def startup():
    """Called before first request"""
    pass  # Controller already initialized in main()

def shutdown_handler():
    """Cleanup on shutdown"""
    global nav_controller
    if nav_controller:
        print("\n[Service] Shutting down...")
        nav_controller.shutdown()

if __name__ == '__main__':
    import atexit
    atexit.register(shutdown_handler)
    
    # Initialize controller
    if not init_controller():
        print("\n✗ Service initialization failed")
        sys.exit(1)
    
    # Start Flask server
    print("\n" + "=" * 60)
    print("  Starting HTTP server on port 8000")
    print("=" * 60)
    print("\nEndpoints:")
    print("  POST /navigate - Start navigation")
    print("  POST /stop - Stop navigation")
    print("  GET  /status - Get status")
    print("  GET  /pose - Get robot pose")
    print("  GET  /health - Health check")
    print("\nPress Ctrl+C to stop")
    print("=" * 60 + "\n")
    
    try:
        app.run(
            host='0.0.0.0', 
            port=8000, 
            threaded=True,
            debug=False
        )
    except KeyboardInterrupt:
        print("\n\n[Service] Interrupted by user")
    finally:
        shutdown_handler()