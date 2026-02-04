#!/usr/bin/env python3
"""
Web Controller Server for X99 (192.168.2.10)
Serves web interface and forwards commands to Jetson Nano
"""
from flask import Flask, render_template, jsonify, request
import requests
import logging

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)

# Configuration
JETSON_URL = "http://192.168.2.13:5000"
TIMEOUT = 2.0

@app.route('/')
def index():
    """Serve main control interface"""
    return render_template('index.html')

@app.route('/api/velocity', methods=['POST'])
def send_velocity():
    """Forward velocity command to Jetson"""
    try:
        data = request.get_json()
        linear = data.get('linear', 0)
        angular = data.get('angular', 0)
        
        response = requests.post(
            f"{JETSON_URL}/motor/velocity",
            json={'linear': linear, 'angular': angular},
            timeout=TIMEOUT
        )
        return jsonify(response.json()), response.status_code
    except requests.RequestException as e:
        logging.error(f"Jetson connection error: {e}")
        return jsonify({'error': 'Jetson unreachable'}), 503

@app.route('/api/enable', methods=['POST'])
def enable_motors():
    """Enable motors"""
    try:
        response = requests.post(f"{JETSON_URL}/motor/enable", timeout=TIMEOUT)
        return jsonify(response.json()), response.status_code
    except requests.RequestException as e:
        return jsonify({'error': str(e)}), 503

@app.route('/api/disable', methods=['POST'])
def disable_motors():
    """Disable motors"""
    try:
        response = requests.post(f"{JETSON_URL}/motor/disable", timeout=TIMEOUT)
        return jsonify(response.json()), response.status_code
    except requests.RequestException as e:
        return jsonify({'error': str(e)}), 503

@app.route('/api/stop', methods=['POST'])
def stop_motors():
    """Emergency stop"""
    try:
        response = requests.post(f"{JETSON_URL}/motor/stop", timeout=TIMEOUT)
        return jsonify(response.json()), response.status_code
    except requests.RequestException as e:
        return jsonify({'error': str(e)}), 503

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get robot status"""
    try:
        response = requests.get(f"{JETSON_URL}/motor/status", timeout=TIMEOUT)
        return jsonify(response.json()), response.status_code
    except requests.RequestException as e:
        return jsonify({'error': 'Jetson unreachable', 'connected': False}), 503

if __name__ == '__main__':
    print("=" * 60)
    print("X99 Web Controller Starting...")
    print("=" * 60)
    print(f"Web Interface: http://192.168.2.10:8080")
    print(f"Jetson Target: {JETSON_URL}")
    print("=" * 60)
    app.run(host='0.0.0.0', port=8080, debug=False)