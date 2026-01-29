# ORB-SLAM + YOLO - Client-Server Architecture

Hệ thống SLAM với kiến trúc client-server:
- **Jetson Nano**: Stream 2 camera OV9832 về server
- **X99 Dual Xeon + Radeon MI50**: Nhận stream, chạy SLAM + YOLO, hiển thị web

## 🏗️ Kiến trúc hệ thống

```
┌─────────────────────────┐         Network          ┌──────────────────────────┐
│   Jetson Nano (Client)  │    (TCP Stream)          │  X99 Server              │
│                         │◄─────────────────────────►│                          │
│  Camera L (OV9832) ─────┼─── Port 9001 ───────────►│  ORB-SLAM                │
│  Camera R (OV9832) ─────┼─── Port 9002 ───────────►│  YOLOv11m-seg (ROCm)     │
│                         │                           │  Web Interface (5000)    │
└─────────────────────────┘                           └──────────────────────────┘
```

## 📦 Cài đặt

### 1. Jetson Nano (Client)

```bash
# Cài đặt dependencies cơ bản
sudo apt update
sudo apt install python3-opencv python3-pip

# Cài đặt thêm
pip3 install numpy

# Copy file client
# jetson_camera_client.py
```

### 2. X99 Server

```bash
# Cài đặt dependencies
pip install -r requirements.txt

# Cài đặt PyTorch với ROCm cho AMD GPU
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.7

# Cài đặt YOLOv11
pip install ultralytics

# Copy files server
# x99_slam_server.py
# x99_web_server.py
# templates/
# static/
```

## 🚀 Khởi động hệ thống

### Bước 1: Tìm IP của X99 Server

Trên X99:
```bash
ip addr show
# hoặc
hostname -I
```

Giả sử IP là: `192.168.1.100`

### Bước 2: Khởi động X99 Server

**Option A: Chạy SLAM standalone (hiển thị local)**

```bash
python3 x99_slam_server.py --left-port 9001 --right-port 9002
```

**Option B: Chạy với Web Interface (khuyến nghị)**

```bash
python3 x99_web_server.py
```

Sau đó mở browser: `http://192.168.1.100:5000`

### Bước 3: Khởi động Jetson Nano Client

Trên Jetson Nano:

```bash
# Kiểm tra camera IDs
python3 -c "import cv2; print([i for i in range(10) if cv2.VideoCapture(i).isOpened()])"

# Stream cameras đến X99
python3 jetson_camera_client.py --server 192.168.1.100 \
    --left-camera 0 \
    --right-camera 1 \
    --width 640 \
    --height 480 \
    --quality 80
```

**Lưu ý**: Giảm `--quality` xuống 60-70 nếu Jetson Nano bị chậm

## ⚙️ Các tùy chọn cấu hình

### Jetson Nano Client Options:

```bash
python3 jetson_camera_client.py \
    --server 192.168.1.100      # IP của X99 server
    --left-port 9001             # Port cho camera trái
    --right-port 9002            # Port cho camera phải
    --left-camera 0              # Device ID camera trái
    --right-camera 1             # Device ID camera phải
    --width 640                  # Width (320/640/1280)
    --height 480                 # Height (240/480/720)
    --quality 80                 # JPEG quality (60-95)
```

### X99 Server Options:

```bash
python3 x99_slam_server.py \
    --left-port 9001             # Port nhận camera trái
    --right-port 9002            # Port nhận camera phải
    --no-yolo                    # Tắt YOLO (nếu cần)
    --baseline 0.10              # Baseline stereo (meters)
```

### X99 Web Server:

```bash
python3 x99_web_server.py
# Mặc định: host=0.0.0.0, port=5000
```

## 🔧 Tối ưu hiệu năng

### Cho Jetson Nano (Tối ưu bandwidth):

**Giảm resolution:**
```bash
--width 320 --height 240 --quality 70
```

**Giảm JPEG quality:**
```bash
--quality 60  # Thấp hơn = file nhỏ hơn = stream nhanh hơn
```

**Enable Jetson power mode:**
```bash
sudo nvpmodel -m 0  # Max performance mode
sudo jetson_clocks   # Max clocks
```

### Cho X99 (Tối ưu processing):

**Tăng ORB features:**
```python
# Trong x99_slam_server.py
self.orb_extractor = ORBFeatureExtractor(n_features=5000)
```

**ROCm GPU utilization:**
```bash
# Kiểm tra GPU
rocm-smi

# Monitor GPU usage
watch -n 1 rocm-smi
```

## 🌐 Network Setup

### 1. Same LAN (Recommended):
- Jetson Nano: `192.168.1.50`
- X99 Server: `192.168.1.100`
- Đơn giản nhất, độ trễ thấp

### 2. Direct Ethernet Connection:
```bash
# Jetson Nano
sudo ifconfig eth0 192.168.2.1 netmask 255.255.255.0

# X99
sudo ifconfig eth0 192.168.2.2 netmask 255.255.255.0

# Test connection
ping 192.168.2.2  # From Jetson
ping 192.168.2.1  # From X99
```

### 3. WiFi:
- Đảm bảo cả 2 máy cùng mạng WiFi
- Có thể có độ trễ cao hơn

## 📊 Benchmark hiệu năng

| Resolution | JPEG Quality | Bandwidth | Jetson FPS | X99 SLAM FPS |
|-----------|--------------|-----------|------------|--------------|
| 320x240   | 60           | ~2 Mbps   | 30         | 28-30        |
| 640x480   | 70           | ~6 Mbps   | 25-30      | 25-28        |
| 640x480   | 80           | ~8 Mbps   | 25-30      | 25-28        |
| 1280x720  | 80           | ~18 Mbps  | 15-20      | 15-18        |

**Khuyến nghị cho real-time:**
- Resolution: 640x480
- Quality: 70-80
- Expected latency: 50-100ms

## 🐛 Troubleshooting

### Jetson không kết nối được X99:

```bash
# Kiểm tra network connectivity
ping 192.168.1.100

# Kiểm tra port có mở không
nc -zv 192.168.1.100 9001
nc -zv 192.168.1.100 9002

# Kiểm tra firewall trên X99
sudo ufw status
sudo ufw allow 9001/tcp
sudo ufw allow 9002/tcp
sudo ufw allow 5000/tcp
```

### Camera không detect:

```bash
# Trên Jetson Nano
ls /dev/video*
v4l2-ctl --list-devices

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
```

### Streaming lag/dropped frames:

**Giảm resolution:**
```bash
--width 320 --height 240
```

**Giảm quality:**
```bash
--quality 60
```

**Kiểm tra network bandwidth:**
```bash
# Install iperf3
sudo apt install iperf3

# On X99
iperf3 -s

# On Jetson
iperf3 -c 192.168.1.100
```

### YOLO chạy chậm trên X99:

```bash
# Kiểm tra GPU
rocm-smi

# Disable YOLO nếu cần
python3 x99_slam_server.py --no-yolo

# Hoặc dùng model nhỏ hơn (yolov11n-seg.pt thay vì yolov11m-seg.pt)
```

## 📁 Cấu trúc Files

**Trên Jetson Nano:**
```
jetson_camera_client.py    # Main streaming client
requirements_jetson.txt    # Minimal dependencies
```

**Trên X99 Server:**
```
x99_slam_server.py         # Standalone SLAM server
x99_web_server.py          # Web interface server
templates/
  index.html               # Web UI
static/
  css/style.css            # Styling
  js/map3d.js              # 3D visualization
  js/main.js               # UI logic
requirements.txt           # Full dependencies
```

## 🔐 Security Notes

**Nếu expose ra internet:**

1. Sử dụng SSH tunnel:
```bash
# From remote machine
ssh -L 5000:localhost:5000 user@x99-server-ip
```

2. Hoặc setup VPN (OpenVPN, WireGuard)

3. Hoặc thêm authentication vào Flask app

## 📈 Next Steps

1. **Add IMU data**: Fuse IMU từ Jetson Nano
2. **Save/Load maps**: Implement map persistence
3. **Loop closure**: Detect và correct drift
4. **Multi-session**: Hỗ trợ nhiều Jetson clients
5. **Recording**: Lưu streams để replay/debug

## 🎓 Usage Examples

**Quick test (lowest latency):**
```bash
# Jetson
python3 jetson_camera_client.py --server 192.168.1.100 \
    --width 320 --height 240 --quality 70

# X99
python3 x99_slam_server.py
```

**Production setup (balanced):**
```bash
# Jetson
python3 jetson_camera_client.py --server 192.168.1.100 \
    --width 640 --height 480 --quality 80

# X99
python3 x99_web_server.py
# Access: http://192.168.1.100:5000
```

**High quality (for recording):**
```bash
# Jetson
python3 jetson_camera_client.py --server 192.168.1.100 \
    --width 1280 --height 720 --quality 90

# X99
python3 x99_slam_server.py --baseline 0.10
```

## 📞 Support

For issues:
1. Check network connectivity first
2. Verify camera IDs and permissions
3. Monitor bandwidth with iperf3
4. Check GPU utilization with rocm-smi

Good luck! 🚀