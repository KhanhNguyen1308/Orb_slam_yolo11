# Autonomous Navigation System Setup

Hệ thống điều hướng tự động hoàn chỉnh với SLAM, path planning, và motor control.

## 🏗️ Kiến trúc hệ thống

```
┌──────────────────────────────────────────────────────────────┐
│                    X99 Server (Processing)                    │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────────┐  │
│  │ Camera      │→ │ ORB-SLAM +   │→ │ Path Planning      │  │
│  │ Streams     │  │ YOLO         │  │ (A* Algorithm)     │  │
│  │ (WiFi)      │  │              │  │                    │  │
│  └─────────────┘  └──────────────┘  └────────────────────┘  │
│                                              │                 │
│                                              ↓                 │
│                                       Send Path + Pose        │
└──────────────────────────────────────────┼───────────────────┘
                                            │ (Port 9003)
                                            ↓
┌──────────────────────────────────────────────────────────────┐
│              Jetson Nano (Edge Controller)                    │
│  ┌──────────────┐  ┌────────────────┐  ┌─────────────────┐  │
│  │ Navigation   │→ │ Pure Pursuit   │→ │ USB Serial      │  │
│  │ Receiver     │  │ Controller     │  │ to Pico         │  │
│  └──────────────┘  └────────────────┘  └─────────────────┘  │
│                                              │                 │
└──────────────────────────────────────────────┼───────────────┘
                                            │ (USB)
                                            ↓
┌──────────────────────────────────────────────────────────────┐
│              RP2040 Pico (Motor Driver)                       │
│  ┌──────────────┐  ┌────────────────────────────────────┐   │
│  │ Serial       │→ │ Differential Drive Controller      │   │
│  │ Parser       │  │ (Tracked Robot Kinematics)         │   │
│  └──────────────┘  └────────────────────────────────────┘   │
│                              │                 │              │
│                              ↓                 ↓              │
│                      ┌───────────┐     ┌───────────┐         │
│                      │ A4988     │     │ A4988     │         │
│                      │ Driver    │     │ Driver    │         │
│                      └─────┬─────┘     └─────┬─────┘         │
│                            ↓                 ↓                │
│                      ┌───────────┐     ┌───────────┐         │
│                      │ NEMA17    │     │ NEMA17    │         │
│                      │ Left      │     │ Right     │         │
│                      └───────────┘     └───────────┘         │
│                            │                 │                │
│                      Left Track         Right Track          │
│                   (Gear 20/68, 15mm chain pitch)             │
└──────────────────────────────────────────────────────────────┘
```

## 📊 Thông số kỹ thuật

### Robot Mechanics:
- **Bánh xích**: 2 tracks, 275mm apart
- **Bánh nhông chính**: 20 răng
- **Tỉ số truyền**: 20/68 (0.294)
- **Mắt xích**: 15mm pitch
- **Chu vi bánh nhông**: 20 × 15mm = 300mm = 0.3m

### Motors:
- **Động cơ**: NEMA17 stepper (1.8°/step)
- **Driver**: A4988 (1/16 microstepping)
- **Steps/rev**: 200 × 16 = 3200 steps
- **Steps/meter**: ~31,373 steps (after gear ratio)

### Control Parameters:
- **Max linear velocity**: 0.5 m/s
- **Max angular velocity**: 1.0 rad/s
- **Lookahead distance**: 0.3m (Pure Pursuit)
- **Goal tolerance**: 0.1m

## 📦 Installation

### 1. RP2040 Pico Setup

**Cài đặt MicroPython:**
```bash
# Download MicroPython UF2
wget https://micropython.org/download/rp2-pico/rp2-pico-latest.uf2

# Flash Pico (hold BOOTSEL button, connect USB)
# Copy UF2 file to RPI-RP2 drive
```

**Upload code:**
```bash
# Install Thonny IDE hoặc ampy
pip install adafruit-ampy

# Upload file
ampy --port /dev/ttyACM0 put pico_motor_controller.py main.py
```

### 2. Jetson Nano Setup

```bash
# Install pyserial
pip3 install pyserial

# Copy files
# jetson_navigation.py
# jetson_wifi_optimized.py (nếu dùng camera streaming)

# Test Pico connection
ls /dev/ttyACM*
python3 -c "import serial; print('OK')"
```

### 3. X99 Server Setup

```bash
# Install dependencies
pip install -r requirements.txt

# Copy files
# x99_integrated_nav.py
# path_planning.py
# x99_wifi_optimized.py
```

## 🔌 Wiring Diagram

### A4988 → NEMA17 Connections:

```
A4988 Driver          NEMA17 Motor
─────────────        ──────────────
1B ──────────────── Black (A-)
1A ──────────────── Green (A+)
2A ──────────────── Red   (B+)
2B ──────────────── Blue  (B-)

Power:
VMOT ───── 12V (Motor power)
GND ────── Ground
VDD ────── 5V (Logic)
```

### RP2040 Pico → A4988 Connections:

**Left Motor:**
```
Pico GPIO    A4988 Pin
─────────    ─────────
GP2      →   STEP
GP3      →   DIR
GP4      →   ENABLE
```

**Right Motor:**
```
Pico GPIO    A4988 Pin
─────────    ─────────
GP6      →   STEP
GP7      →   DIR
GP8      →   ENABLE
```

**Microstepping (1/16):**
```
A4988 Pin    Connection
─────────    ──────────
MS1      →   3.3V (HIGH)
MS2      →   3.3V (HIGH)
MS3      →   3.3V (HIGH)
```

### Power Supply:

```
12V PSU
  ├─→ A4988 VMOT (Left)
  ├─→ A4988 VMOT (Right)
  └─→ GND
  
5V PSU (or buck converter)
  ├─→ Pico VBUS
  ├─→ A4988 VDD (Left)
  ├─→ A4988 VDD (Right)
  └─→ GND
```

⚠️ **Important**: Share common ground between all components!

## 🚀 Running the System

### Step 1: Start RP2040 Pico

```bash
# Pico should auto-run main.py on power-up
# Or use Thonny to run manually

# Test motor control:
# Set test_mode = True in pico_motor_controller.py
```

### Step 2: Start Jetson Nano Navigation

```bash
# Terminal 1: Camera streaming (if using WiFi cameras)
python3 jetson_wifi_optimized.py \
    --server <X99_IP> \
    --left-camera 0 --right-camera 1 \
    --width 640 --height 480 --quality 75

# Terminal 2: Navigation controller
python3 jetson_navigation.py \
    --pico-port /dev/ttyACM0 \
    --nav-port 9003
```

### Step 3: Start X99 Server

```bash
python3 x99_integrated_nav.py \
    --jetson-ip <JETSON_IP>
```

### Step 4: Set Navigation Goal

1. Press `p` to enable path planning view
2. Press `g` and click on the map to set goal
3. X99 will plan path and send to Jetson
4. Robot will follow path automatically!

## 🎮 Manual Control Testing

### Test Pico Directly (via USB serial):

```bash
# Connect to Pico
screen /dev/ttyACM0 115200

# Send JSON commands:
{"cmd":"enable"}
{"cmd":"velocity","linear":0.2,"angular":0.0}  # Forward
{"cmd":"velocity","linear":0.0,"angular":0.5}  # Turn left
{"cmd":"stop"}
{"cmd":"disable"}
```

### Test from Python:

```python
import serial
import json
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

# Enable motors
ser.write(b'{"cmd":"enable"}\n')
time.sleep(0.5)

# Forward 0.2 m/s
ser.write(b'{"cmd":"velocity","linear":0.2,"angular":0.0}\n')
time.sleep(3)

# Stop
ser.write(b'{"cmd":"stop"}\n')

# Disable
ser.write(b'{"cmd":"disable"}\n')

ser.close()
```

## 🔧 Calibration

### Motor Direction Calibration:

If motors run in wrong direction:

**Method 1: Swap motor wires**
- Swap A+ with A- (or swap B+ with B-)

**Method 2: Change in code**
```python
# In pico_motor_controller.py
def set_direction(self, forward: bool):
    # Invert logic:
    self.dir.value(0 if forward else 1)  # Changed
```

### Speed Calibration:

Test actual robot speed:

```python
# Measure distance traveled
distance = 1.0  # meters
measured_time = 5.0  # seconds

actual_speed = distance / measured_time
commanded_speed = 0.2  # m/s

# Calibration factor
SPEED_CALIBRATION = commanded_speed / actual_speed

# Apply in code:
STEPS_PER_METER = STEPS_PER_METER * SPEED_CALIBRATION
```

### Track Width Calibration:

Test rotation:

```python
# Command 360° rotation
angular_vel = 1.0  # rad/s
duration = 2 * 3.14 / angular_vel  # seconds for 360°

# Measure actual rotation
# Adjust TRACK_WIDTH in pico_motor_controller.py
```

## 📈 Performance Tuning

### Increase Maximum Speed:

```python
# In pico_motor_controller.py
MAX_SPEED_MPS = 0.8  # Increase from 0.5

# In jetson_navigation.py
self.controller = PurePursuitController(
    max_linear_vel=0.5,  # Increase
    max_angular_vel=1.5  # Increase
)
```

### Adjust Lookahead Distance:

```python
# Smaller = tighter following, more oscillation
# Larger = smoother, wider turns

self.controller = PurePursuitController(
    lookahead_distance=0.5,  # Increase from 0.3
)
```

### Path Planning Resolution:

```python
# In x99_integrated_nav.py
self.path_planner = PathPlanner(
    grid_width=400,      # Increase for larger area
    grid_height=400,
    resolution=0.03      # Decrease for finer resolution
)
```

## 🐛 Troubleshooting

### Pico not responding:

```bash
# Check connection
ls /dev/ttyACM*

# Check Pico serial output
screen /dev/ttyACM0 115200

# Reflash if needed
```

### Motors not moving:

1. Check ENABLE pin (should be LOW when enabled)
2. Check motor power (12V connected?)
3. Check wiring (STEP/DIR/ENABLE connected?)
4. Test with multimeter (voltage on motor coils?)

### Motors vibrating/not smooth:

1. Adjust microstepping (try 1/8 or 1/4)
2. Check current limit on A4988 (adjust potentiometer)
3. Lower speed
4. Check mechanical coupling

### Robot not following path:

1. Check pose updates from X99
2. Verify path is being received by Jetson
3. Test Pure Pursuit lookahead distance
4. Check motor directions (left/right swapped?)

### Path planning fails:

1. Check occupancy grid visualization
2. Increase inflation radius if robot too large
3. Check goal is in free space
4. Verify SLAM map quality

## 💡 Advanced Features

### Add Odometry:

```python
# In pico_motor_controller.py
# Track motor steps for odometry

class TrackedRobot:
    def __init__(self):
        # ...
        self.left_steps = 0
        self.right_steps = 0
    
    def update(self):
        # ...
        if stepped_left:
            self.left_steps += 1
        if stepped_right:
            self.right_steps += 1
    
    def get_odometry(self):
        left_dist = self.left_steps / STEPS_PER_METER
        right_dist = self.right_steps / STEPS_PER_METER
        # Calculate x, y, theta from differential drive
        return x, y, theta
```

### Add Emergency Stop:

```python
# Add physical button to Pico
emergency_stop = Pin(15, Pin.IN, Pin.PULL_UP)

def check_emergency_stop():
    if not emergency_stop.value():  # Button pressed (active LOW)
        robot.stop()
        robot.disable()
        return True
    return False
```

### Add IMU Fusion:

- Connect MPU6050 to Jetson I2C
- Fuse SLAM pose with IMU for better odometry
- Improve pose estimation during fast movements

## 📞 Support & Tips

1. **Always test motors individually first**
2. **Start with low speeds during testing**
3. **Use proper power supply (>2A for motors)**
4. **Check all ground connections**
5. **Calibrate speeds and directions before navigation**
6. **Monitor serial output for debugging**
7. **Keep safety margins in path planning**

---

**Quick Test Sequence:**
```bash
# 1. Test Pico motors
python3 pico_motor_controller.py  # with test_mode=True

# 2. Test Jetson → Pico communication
python3 jetson_navigation.py

# 3. Test X99 → Jetson communication
python3 x99_integrated_nav.py --jetson-ip <IP>

# 4. Run full system and set goal!
```

Good luck with your autonomous robot! 🤖🚀