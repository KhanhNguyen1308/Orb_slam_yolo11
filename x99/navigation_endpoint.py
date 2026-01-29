#!/usr/bin/env python3
"""
Autonomous Navigation API for X99 Server
Add these endpoints to x99_web_slam.py
"""

import subprocess
import threading
import json
from flask import request, jsonify

# Global navigation process
navigation_process = None
navigation_status = {
    'active': False,
    'progress': 0,
    'goal': None
}

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

# ===== BETTER APPROACH: Jetson Navigation Service =====
# Create a simple REST service on Jetson that X99 can call

# On Jetson, create jetson_nav_service.py:
"""
from flask import Flask, request, jsonify
import threading

app = Flask(__name__)
nav_controller = None  # Initialize your NavigationController here

@app.route('/navigate', methods=['POST'])
def navigate():
    data = request.json
    goal_x = data.get('goal_x')
    goal_y = data.get('goal_y')
    
    # Start navigation in background thread
    thread = threading.Thread(
        target=nav_controller.navigate_to_goal,
        args=(goal_x, goal_y)
    )
    thread.start()
    
    return jsonify({'success': True})

@app.route('/stop', methods=['POST'])
def stop():
    nav_controller.stop_navigation()
    return jsonify({'success': True})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
"""

# Then in x99_web_slam.py, call Jetson API:
"""
import requests

JETSON_URL = "http://192.168.1.50:8000"  # Jetson IP

@app.route('/api/navigate', methods=['POST'])
def start_navigation():
    data = request.json
    goal = data.get('goal')
    
    try:
        response = requests.post(
            f"{JETSON_URL}/navigate",
            json={'goal_x': goal[0], 'goal_y': goal[1]},
            timeout=5
        )
        
        if response.status_code == 200:
            return jsonify({'success': True})
        else:
            return jsonify({'success': False, 'error': 'Jetson request failed'}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500
"""