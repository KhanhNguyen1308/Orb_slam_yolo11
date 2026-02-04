#!/usr/bin/env python3
"""
Jetson Nano Serial Bridge (192.168.2.13)
Receives HTTP commands and forwards to RP2040 via serial
"""
from flask import Flask, jsonify, request
import serial
import json
import time
import threading
import logging

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)

# Serial configuration
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed (check with 'ls /dev/tty*')
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# Global serial connection
ser = None
serial_lock = threading.Lock()

def init_serial():
    """Initialize serial connection to RP2040"""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
        time.sleep(2)  # Wait for RP2040 to initialize
        logging.info(f"Serial connected: {SERIAL_PORT} @ {SERIAL_BAUD}")
        return True
    except serial.SerialException as e:
        logging.error(f"Serial connection failed: {e}")
        return False

def send_command(cmd_dict):
    """Send JSON command to RP2040"""
    global ser
    if ser is None or not ser.is_open:
        return False
    
    try:
        with serial_lock:
            cmd_json = json.dumps(cmd_dict) + '\n'
            ser.write(cmd_json.encode('utf-8'))
            ser.flush()
            logging.debug(f"Sent: {cmd_json.strip()}")
        return True
    except Exception as e:
        logging.error(f"Serial write error: {e}")
        return False

@app.route('/motor/velocity', methods=['POST'])
def set_velocity():
    """Set robot velocity (linear, angular)"""
    try:
        data = request.get_json()
        linear = float(data.get('linear', 0))
        angular = float(data.get('angular', 0))
        
        cmd = {
            'cmd': 'velocity',
            'linear': linear,
            'angular': angular
        }
        
        if send_command(cmd):
            return jsonify({'status': 'ok', 'linear': linear, 'angular': angular})
        else:
            return jsonify({'error': 'Serial communication failed'}), 500
            
    except (ValueError, TypeError) as e:
        return jsonify({'error': f'Invalid data: {e}'}), 400

@app.route('/motor/enable', methods=['POST'])
def enable_motors():
    """Enable motors"""
    if send_command({'cmd': 'enable'}):
        return jsonify({'status': 'ok', 'enabled': True})
    else:
        return jsonify({'error': 'Serial communication failed'}), 500

@app.route('/motor/disable', methods=['POST'])
def disable_motors():
    """Disable motors"""
    if send_command({'cmd': 'disable'}):
        return jsonify({'status': 'ok', 'enabled': False})
    else:
        return jsonify({'error': 'Serial communication failed'}), 500

@app.route('/motor/stop', methods=['POST'])
def stop_motors():
    """Emergency stop"""
    if send_command({'cmd': 'stop'}):
        return jsonify({'status': 'ok'})
    else:
        return jsonify({'error': 'Serial communication failed'}), 500

@app.route('/motor/status', methods=['GET'])
def get_status():
    """Get motor status"""
    if send_command({'cmd': 'status'}):
        # Note: In production, you'd read response from RP2040
        # For now, just confirm serial is working
        return jsonify({
            'status': 'ok',
            'enabled': True,  # Would parse from RP2040 response
            'serial': ser.is_open if ser else False
        })
    else:
        return jsonify({'error': 'Serial not connected'}), 503

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        'status': 'ok',
        'serial_connected': ser.is_open if ser else False,
        'serial_port': SERIAL_PORT
    })

if __name__ == '__main__':
    print("=" * 60)
    print("Jetson Nano Serial Bridge Starting...")
    print("=" * 60)
    
    if init_serial():
        print(f"✓ Serial: {SERIAL_PORT} @ {SERIAL_BAUD}")
        print(f"✓ HTTP Server: http://192.168.2.13:5000")
        print("=" * 60)
        app.run(host='0.0.0.0', port=5000, debug=False)
    else:
        print("✗ Failed to initialize serial connection")
        print("  Check:")
        print(f"  - RP2040 connected to {SERIAL_PORT}")
        print("  - USB cable is data-capable")
        print("  - User has permissions: sudo usermod -a -G dialout $USER")
        print("=" * 60)