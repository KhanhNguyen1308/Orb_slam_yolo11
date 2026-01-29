# AUTONOMOUS NAVIGATION SETUP GUIDE

## Tổng quan hệ thống:

```
┌─────────────┐          ┌──────────────┐         ┌────────────────┐
│  X99 Server │◄────────►│ Jetson Nano  │◄───USB──►│ Raspberry Pico │
│  (SLAM +    │  Network │ (Navigation) │  Serial │ (Motors)       │
│   Path      │          │              │         │                │
│   Planning) │          │              │         │                │
└─────────────┘          └──────────────┘         └────────────────┘
      ▲                        │                         │
      │                        │                         │
   Browser                  Cameras                   NEMA17
  (Remote                  OV9832 x2                 Motors x2
   Control)
```

## CÁC THÀNH PHẦN:

### 1. X99 Server (Desktop/Workstation)
- **Chức năng**: SLAM, mapping, path planning
- **Phần mềm**: 
  - `x99_web_slam.py` (đã có)
  - `persistent_map.py` (đã có)
  - `navigation_endpoints.py` (mới)
  - `robot_control.html` (mới)

### 2. Jetson Nano
- **Chức năng**: Navigation controller, camera processing
- **Phần mềm**:
  - `jetson_navigation.py` (mới)
  - `jetson_nav_service.py` (mới - REST API)
  
### 3. Raspberry Pico
- **Chức năng**: Motor control
- **Phần mềm**:
  - `rp2040_stepper_pio.py` (đã có)

---

## SETUP CHI TIẾT:

### BƯỚC 1: Setup Raspberry Pico (Motor Controller)

```bash
# 1. Flash MicroPython lên Pico
# Download: https://micropython.org/download/rp2-pico/

# 2. Copy code lên Pico
# Sử dụng Thonny hoặc rshell
rshell
cp rp2040_stepper_pio.py /pyboard/main.py

# 3. Kết nối phần cứng:
# Pico Pin -> A4988 Driver -> NEMA17 Motor
# GPIO 15 -> LEFT_STEP
# GPIO 14 -> LEFT_DIR  
# GPIO 4  -> LEFT_EN
# GPIO 17 -> RIGHT_STEP
# GPIO 16 -> RIGHT_DIR
# GPIO 8  -> RIGHT_EN

# 4. Cấp nguồn:
# A4988: 12V power supply
# Pico: 5V via USB from Jetson
```

### BƯỚC 2: Setup Jetson Nano (Navigation)

```bash
# 1. Install dependencies
sudo apt-get update
sudo apt-get install python3-pip python3-serial
pip3 install pyserial requests numpy

# 2. Copy navigation code
cd ~
mkdir robot_nav
cd robot_nav
cp jetson_navigation.py .
cp jetson_nav_service.py .

# 3. Tìm Pico serial port
ls /dev/ttyACM*
# Hoặc
ls /dev/ttyUSB*
# Thường là /dev/ttyACM0

# 4. Test Pico connection
python3 << EOF
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
import time
time.sleep(2)
ser.write(b'{"cmd":"status"}\n')
print(ser.readline())
EOF

# Nếu thấy {"status":"ok"} -> Thành công!

# 5. Configure X99 IP
nano jetson_navigation.py
# Sửa dòng: X99_SERVER_URL = "http://192.168.1.100:5000"
# Thành IP của X99 server

# 6. Test standalone navigation
python3 jetson_navigation.py
# Nếu thành công, bạn sẽ thấy:
# ===== SYSTEM READY =====
```

### BƯỚC 3: Setup Jetson Navigation Service (REST API)

```bash
# 1. Tạo service file
nano jetson_nav_service.py
```

```python
#!/usr/bin/env python3
"""
Jetson Navigation Service - REST API
Allows X99 server to trigger navigation remotely
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import sys

# Import navigation controller
from jetson_navigation import NavigationController, X99_SERVER_URL, PICO_SERIAL_PORT

app = Flask(__name__)
CORS(app)

# Initialize controller
nav_controller = None
nav_thread = None

def init_controller():
    global nav_controller
    nav_controller = NavigationController(X99_SERVER_URL, PICO_SERIAL_PORT)
    
    if not nav_controller.start():
        print("Failed to initialize navigation controller")
        sys.exit(1)
    
    print("Navigation controller ready")

@app.route('/navigate', methods=['POST'])
def navigate():
    """Start navigation to goal"""
    if not nav_controller:
        return jsonify({'success': False, 'error': 'Controller not initialized'}), 500
    
    data = request.json
    goal_x = data.get('goal_x')
    goal_y = data.get('goal_y')
    
    if goal_x is None or goal_y is None:
        return jsonify({'success': False, 'error': 'Missing goal coordinates'}), 400
    
    # Start navigation in background thread
    def nav_task():
        nav_controller.navigate_to_goal(int(goal_x), int(goal_y))
    
    global nav_thread
    if nav_thread and nav_thread.is_alive():
        nav_controller.stop_navigation()
        nav_thread.join(timeout=1)
    
    nav_thread = threading.Thread(target=nav_task, daemon=True)
    nav_thread.start()
    
    return jsonify({'success': True, 'goal': [goal_x, goal_y]})

@app.route('/stop', methods=['POST'])
def stop():
    """Stop current navigation"""
    if nav_controller:
        nav_controller.stop_navigation()
    return jsonify({'success': True})

@app.route('/status')
def status():
    """Get navigation status"""
    if not nav_controller:
        return jsonify({'ready': False})
    
    return jsonify({
        'ready': True,
        'connected': nav_controller.pico.connected,
        'navigating': nav_controller.navigation_active
    })

if __name__ == '__main__':
    init_controller()
    app.run(host='0.0.0.0', port=8000, threaded=True)
```

```bash
# 2. Install Flask
pip3 install flask flask-cors

# 3. Run service
python3 jetson_nav_service.py

# Nếu thành công:
# Navigation controller ready
# * Running on http://0.0.0.0:8000

# 4. Test API
curl http://localhost:8000/status
# Output: {"ready": true, "connected": true, "navigating": false}
```

### BƯỚC 4: Setup X99 Server (SLAM + Web Interface)

```bash
# 1. Copy new files
cd ~/x99_slam
cp robot_control.html templates/
cp navigation_endpoints.py .

# 2. Integrate navigation endpoints
nano x99_web_slam.py

# Thêm vào đầu file:
import requests

JETSON_NAV_URL = "http://192.168.1.50:8000"  # Change to Jetson IP

# 3. Thêm endpoints (copy từ navigation_endpoints.py)
# Paste before if __name__ == '__main__':

@app.route('/api/navigate', methods=['POST'])
def start_navigation():
    """Start navigation via Jetson"""
    data = request.json
    goal = data.get('goal')
    
    if not goal:
        return jsonify({'success': False, 'error': 'No goal provided'}), 400
    
    try:
        response = requests.post(
            f"{JETSON_NAV_URL}/navigate",
            json={'goal_x': goal[0], 'goal_y': goal[1]},
            timeout=5
        )
        
        if response.status_code == 200:
            socketio.emit('navigation_started', {'goal': goal})
            return jsonify({'success': True})
        else:
            return jsonify({'success': False, 'error': 'Jetson error'}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/stop_navigation', methods=['POST'])
def stop_navigation():
    """Stop navigation"""
    try:
        requests.post(f"{JETSON_NAV_URL}/stop", timeout=2)
        socketio.emit('navigation_stopped', {})
        return jsonify({'success': True})
    except:
        return jsonify({'success': False}), 500

@app.route('/control')
def robot_control():
    """Robot control interface"""
    return render_template('robot_control.html')

# 4. Restart X99 server
python3 x99_web_slam.py
```

### BƯỚC 5: Test Toàn Bộ Hệ Thống

```bash
# 1. Kiểm tra Pico (trên Jetson)
python3 << EOF
import serial, json, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(json.dumps({'cmd': 'enable'}).encode() + b'\n')
time.sleep(0.1)
ser.write(json.dumps({'cmd': 'velocity', 'linear': 0.1, 'angular': 0}).encode() + b'\n')
time.sleep(2)
ser.write(json.dumps({'cmd': 'stop'}).encode() + b'\n')
EOF
# Robot phải di chuyển về phía trước!

# 2. Kiểm tra Jetson service
curl http://localhost:8000/status
# Output: {"ready": true, ...}

# 3. Test navigation từ X99
curl -X POST http://localhost:5000/api/navigate \
  -H "Content-Type: application/json" \
  -d '{"goal": [450, 350]}'
# Output: {"success": true}

# 4. Kiểm tra web interface
# Mở browser: http://<X99_IP>:5000/control
```

---

## SỬ DỤNG:

### Option 1: Web Interface (Khuyến nghị)

```
1. Mở browser: http://<X99_IP>:5000/control
2. Đợi map load
3. Click vào vị trí trên map để chọn goal (màu đỏ)
4. Click "🚀 GO TO GOAL"
5. Robot sẽ tự động tìm đường và di chuyển
6. Click "⏹️ STOP" để dừng bất cứ lúc nào
```

### Option 2: Command Line (Debug)

```bash
# Trên Jetson, chạy trực tiếp:
python3 jetson_navigation.py

# Nhập lệnh:
> g 450 350    # Navigate to grid (450, 350)
> s            # Stop
> q            # Quit
```

### Option 3: API Call

```python
import requests

# Send navigation command
response = requests.post(
    'http://x99_ip:5000/api/navigate',
    json={'goal': [450, 350]}
)

print(response.json())
```

---

## TROUBLESHOOTING:

### Lỗi 1: Pico không kết nối
```bash
# Check USB connection
lsusb | grep -i pico

# Check serial port
ls -l /dev/ttyACM*

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again

# Test manual
minicom -D /dev/ttyACM0 -b 115200
# Type: {"cmd":"status"}
# Should see: {"status":"ok"}
```

### Lỗi 2: Jetson service không start
```bash
# Check dependencies
pip3 list | grep -E "(flask|serial|requests)"

# Check port
sudo netstat -tulpn | grep 8000

# Run with debug
python3 jetson_nav_service.py --debug
```

### Lỗi 3: X99 không kết nối Jetson
```bash
# Check network
ping <jetson_ip>

# Check firewall
sudo ufw allow 8000

# Test API
curl http://<jetson_ip>:8000/status
```

### Lỗi 4: Robot không di chuyển
```bash
# Check motor enable
# A4988 EN pin phải LOW to enable
# Kiểm tra code Pico: en_pin.value(0) = enable

# Check power supply
# A4988 cần 12V riêng, không dùng nguồn Pico

# Check wiring
# Step pulse phải thấy LED flash trên A4988
```

---

## DIAGRAM:

```
USER BROWSER
     │
     │ HTTP
     ▼
┌─────────────────────┐
│   X99 Server        │
│   - SLAM            │◄─── Camera streams
│   - Map 2D          │
│   - Path Planning   │
│   - Web UI          │
└─────────────────────┘
     │
     │ HTTP API
     │ (navigate command)
     ▼
┌─────────────────────┐
│  Jetson Nano        │
│  - Nav Service      │
│  - Nav Controller   │◄─── Cameras (local)
│  - Pure Pursuit     │
└─────────────────────┘
     │
     │ USB Serial (JSON)
     │ {cmd: velocity, linear, angular}
     ▼
┌─────────────────────┐
│  Raspberry Pico     │
│  - PIO Stepper      │
│  - Kinematics       │──► A4988 ──► NEMA17
│  - Watchdog         │
└─────────────────────┘
```

---

## FILES CHECKLIST:

**X99 Server:**
- [ ] x99_web_slam_v2.py
- [ ] templates/robot_control.html
- [ ] navigation endpoints added

**Jetson Nano:**
- [ ] jetson_navigation.py
- [ ] jetson_nav_service.py
- [ ] Pico connected to /dev/ttyACM0

**Raspberry Pico:**
- [ ] rp2040_stepper_pio.py uploaded as main.py
- [ ] A4988 drivers connected
- [ ] NEMA17 motors connected
- [ ] Power supply 12V connected

---

Hệ thống hoàn chỉnh! Robot giờ có thể:
✅ Tự vẽ map môi trường
✅ Tự tìm đường A*
✅ Tự điều khiển động cơ
✅ Điều khiển từ xa qua web

🤖🗺️🚀