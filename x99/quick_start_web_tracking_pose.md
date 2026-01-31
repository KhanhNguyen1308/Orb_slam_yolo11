# QUICK START - X99 Web SLAM (Headless Server)

## 🌐 WEB INTERFACE VERSION

X99 là **headless server** (không có màn hình), nên tất cả visualization được hiển thị qua **web browser**.

---

## ⚡ CÀI ĐẶT NHANH (5 phút)

### Bước 1: Copy files
```bash
# Main web server
cp x99_web_slam_improved.py /path/to/x99/

# Supporting files (phải có)
# - x99_slam_server_improved.py
# - path_planning.py
# - persistent_map.py
# - stereo_depth_mapping_optimized.py
# - templates/index.html

# Tạo thư mục templates
mkdir -p templates
cp templates/index.html templates/
```

### Bước 2: Install dependencies
```bash
pip install flask flask-socketio opencv-python numpy pillow --break-system-packages
```

### Bước 3: Chạy web server
```bash
python x99_web_slam_improved.py
```

---

## 🖥️ TRUY CẬP WEB INTERFACE

### Từ X99 (local):
```
http://localhost:1234
```

### Từ máy khác trong mạng:
```
http://<X99_IP>:1234

Ví dụ:
http://192.168.1.100:1234
```

**Tìm IP của X99:**
```bash
hostname -I
# Output: 192.168.1.100 ...
```

---

## 📱 GÌ BẠN SẼ THẤY TRÊN WEB

### Dashboard với 4 Video Panels:

#### 1. **📍 SLAM Tracking**
- Video stream với overlay
- Tracking status: GOOD/LOST/INIT
- Frame count, keyframes
- Map points, inliers
- Current pose (x, y, θ)
- FPS counter

#### 2. **🌈 Depth Map**
- Stereo depth visualization
- Color-coded distance
- JET colormap

#### 3. **🗺️ Map & Trajectory**
- 2D top-down view
- Robot trajectory (green)
- Occupied space (black)
- Free space (white)

#### 4. **🎯 Path Planning**
- Occupancy grid
- Planned path (if any)
- Robot position
- Goal marker

### Statistics Panel:
```
┌─────────────────────────────────────┐
│ Tracking Quality: GOOD              │
│ Keyframes: 15                       │
│ Map Points: 3421                    │
│ Tracked Inliers: 87                 │
│ SLAM FPS: 18.5                      │
│ SLAM Frames: 1234                   │
│ Persistent Map: 5678                │
│ Uptime: 0h 5m 23s                   │
└─────────────────────────────────────┘

Current Robot Pose:
X: 1.234 m  Y: 0.567 m  θ: 0.123 rad
```

### Control Panel:
```
▶️ Start SLAM   ⏸️ Stop SLAM   🔄 Reset SLAM   💾 Save Map

🎯 Path Planning
Goal X (m): [2.0]  Goal Y (m): [1.5]  [Plan Path]
```

---

## 🎮 SỬ DỤNG

### 1. Khởi động hệ thống

**Trên Jetson Nano:**
```bash
# Start camera streaming
python jetson_wifi_optimized.py
```

**Trên X99 Server:**
```bash
# Start web SLAM server
python x99_web_slam_improved.py

# Output:
# ======================================
#   X99 Web SLAM - Improved
# ======================================
# X99 IPs: 192.168.1.100 192.168.1.101
# Web Interface: http://192.168.1.100:1234
# Camera ports: 9001, 9002
# ======================================
# 
# [WAITING] For Jetson camera connections...
#   5s - Left: ✓ Right: ✓
# 
# [OK] Cameras connected!
```

### 2. Mở Browser

Từ bất kỳ máy nào trong mạng:
```
http://192.168.1.100:1234
```

### 3. Verify SLAM đang hoạt động

Xem dashboard, kiểm tra:
- **Tracking:** Phải là "GOOD" (màu xanh)
- **Map Points:** Tăng dần
- **Pose:** Thay đổi khi robot di chuyển
- **FPS:** > 10

### 4. Control qua Web

#### Start/Stop SLAM:
Click **▶️ Start SLAM** hoặc **⏸️ Stop SLAM**

#### Reset SLAM:
Click **🔄 Reset SLAM** (xóa toàn bộ map)

#### Save Map:
Click **💾 Save Map** (lưu map hiện tại)

#### Plan Path:
1. Nhập Goal X và Goal Y
2. Click **Plan Path**
3. Xem path trong panel "Path Planning"

---

## 🔍 MONITORING

### Real-time Updates:
- Stats cập nhật **mỗi giây**
- Video streams **15-25 FPS**
- Pose updates **real-time**

### Tracking Quality Indicators:

| Status | Màu | Ý nghĩa |
|--------|-----|---------|
| **GOOD** | 🟢 Xanh | Tracking tốt, ≥15 inliers |
| **LOST** | 🔴 Đỏ | Mất tracking, <15 inliers |
| **INIT** | 🟡 Vàng | Đang khởi tạo |
| **POOR** | ⚪ Xám | Ít features |

---

## 🐛 TROUBLESHOOTING

### 1. "Cannot connect to X99"
**Check:**
```bash
# On X99
ping 192.168.1.100  # Your X99 IP

# Check if port 1234 is open
netstat -tuln | grep 1234
```

### 2. "Tracking: LOST" liên tục
**Fix:**
- Cải thiện lighting
- Thêm texture vào environment
- Giảm tốc độ di chuyển
- Check calibration

### 3. Video lag hoặc freeze
**Fix:**
```bash
# Disable YOLO để tăng FPS
# Edit x99_web_slam_improved.py, line ~30:
self.yolo = None  # Disable YOLO

# Hoặc giảm resolution
# In jetson_wifi_optimized.py
```

### 4. "No camera connection"
**Check Jetson:**
```bash
# Verify Jetson đang stream
python jetson_wifi_optimized.py

# Should show:
# [LEFT] Streaming to X99_IP:9001
# [RIGHT] Streaming to X99_IP:9002
```

---

## 📊 PERFORMANCE

### Expected Performance:

**X99 Server (AMD MI50):**
- SLAM FPS: 15-25
- Feature Extraction: 5-8ms
- PnP RANSAC: 5-10ms
- Web Streaming: 60-80% quality

**Network:**
- Bandwidth: 5-10 Mbps per camera
- Latency: 50-100ms (LAN)

**Browser:**
- Any modern browser
- Chrome/Firefox recommended

---

## 🎯 ADVANCED USAGE

### 1. Remote Access (Outside LAN)

**Setup port forwarding:**
```bash
# On router, forward port 1234 to X99
# Then access via:
http://YOUR_PUBLIC_IP:1234
```

**Or use ngrok:**
```bash
ngrok http 1234
# Gives you: https://xxx.ngrok.io
```

### 2. Multiple Clients

Web interface supports multiple browsers simultaneously!
```
Browser 1: http://192.168.1.100:1234
Browser 2: http://192.168.1.100:1234  # Same time!
Browser 3: http://192.168.1.100:1234  # OK!
```

### 3. Mobile Access

Open on phone browser:
```
http://192.168.1.100:1234
```

Works on iOS Safari, Android Chrome!

### 4. Record Sessions

**Browser screenshot:**
- Right-click → Save As

**Or programmatically:**
```bash
# On X99, save frames
# (Add this feature if needed)
```

---

## 🔧 CUSTOMIZATION

### Change Port:
```bash
python x99_web_slam_improved.py --port 8080
```

### Change Quality:
Edit `x99_web_slam_improved.py`:
```python
# Line ~467
_, buffer = cv2.imencode('.jpg', frame, 
    [cv2.IMWRITE_JPEG_QUALITY, 85])  # 85 = quality (0-100)
```

### Disable YOLO:
```python
# Line ~30
self.yolo = None  # Faster, no semantic segmentation
```

---

## 📋 FILE STRUCTURE

```
x99_project/
├── x99_web_slam_improved.py       # Main web server
├── x99_slam_server_improved.py    # SLAM tracker
├── path_planning.py               # A* planner
├── persistent_map.py              # Map builder
├── stereo_depth_mapping_optimized.py
├── calibration_params.npz         # Calibration
└── templates/
    └── index.html                 # Web UI
```

---

## 🎓 UNDERSTANDING THE FLOW

```
Jetson Nano                X99 Server                 Browser
    │                          │                         │
    ├─► Camera L ─────────────►│                         │
    ├─► Camera R ─────────────►│                         │
    │   (TCP 9001, 9002)        │                         │
    │                          │                         │
    │                   ┌──────┴──────┐                  │
    │                   │ SLAM Tracker│                  │
    │                   │  - Features │                  │
    │                   │  - PnP      │                  │
    │                   │  - Pose     │                  │
    │                   │  - Map      │                  │
    │                   └──────┬──────┘                  │
    │                          │                         │
    │                   ┌──────┴──────┐                  │
    │                   │ Flask Server│                  │
    │                   │  - Streams  │                  │
    │                   │  - Stats    │                  │
    │                   │  - Control  │                  │
    │                   └──────┬──────┘                  │
    │                          │                         │
    │                          │◄────── HTTP ────────────┤
    │                          │      (port 1234)        │
    │                          │                         │
    │                          ├────── Video ───────────►│
    │                          ├────── Stats ───────────►│
    │                          │◄────── Control ─────────┤
    │                          │                         │
```

---

## 🚀 PRODUCTION TIPS

### 1. Auto-start on boot:
```bash
# Create systemd service
sudo nano /etc/systemd/system/x99-slam.service

[Unit]
Description=X99 Web SLAM
After=network.target

[Service]
ExecStart=/usr/bin/python3 /path/to/x99_web_slam_improved.py
WorkingDirectory=/path/to/
Restart=always
User=x99

[Install]
WantedBy=multi-user.target

# Enable
sudo systemctl enable x99-slam
sudo systemctl start x99-slam
```

### 2. Logging:
```bash
python x99_web_slam_improved.py 2>&1 | tee slam.log
```

### 3. Monitor CPU/GPU:
```bash
# Watch resources
watch -n 1 'nvidia-smi'
htop
```

---

## ✅ SUCCESS CHECKLIST

**Before starting:**
- [ ] Jetson streaming cameras
- [ ] X99 can ping Jetson
- [ ] Files in place
- [ ] Dependencies installed

**After starting:**
- [ ] Web opens at http://X99_IP:1234
- [ ] 4 video panels showing streams
- [ ] Tracking: GOOD (green)
- [ ] Pose updating
- [ ] Map points increasing

**Verify SLAM working:**
- [ ] Move robot/cameras
- [ ] Pose changes
- [ ] Keyframes created
- [ ] Map grows
- [ ] Can plan path

---

## 📞 QUICK REFERENCE

### Start System:
```bash
# Jetson
python jetson_wifi_optimized.py

# X99
python x99_web_slam_improved.py

# Browser
http://192.168.1.100:1234
```

### Stop System:
```bash
Ctrl+C  # On X99
Ctrl+C  # On Jetson
```

### Check Status:
```bash
# Web shows real-time stats
# Or check terminal on X99
```

---

**Ready to SLAM with Web Interface! 🌐🤖**

**Next:** Open browser → Monitor SLAM → Control robot!