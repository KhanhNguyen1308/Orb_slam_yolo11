# ORB-SLAM + YOLOv11 Semantic Mapping System

Hệ thống SLAM với khả năng phân đoạn ngữ nghĩa (semantic segmentation) sử dụng camera stereo và hiển thị 3D trên giao diện web.

## 🎯 Tính năng

- **Visual SLAM**: Sử dụng ORB features cho tracking và mapping
- **Stereo Vision**: Tính toán depth map từ camera stereo OV9832
- **Semantic Segmentation**: YOLOv11m-seg để phát hiện và phân đoạn đối tượng
- **3D Visualization**: Hiển thị map và trajectory trên web interface
- **Real-time Processing**: Xử lý thời gian thực trên Jetson Nano

## 🖥️ Hardware Requirements

### Jetson Nano Setup (cho edge deployment)
- Jetson Nano (4GB recommended)
- 2x OV9832 USB cameras cách nhau 10cm
- Power supply 5V/4A
- microSD card 64GB+

### X99 Workstation (cho training/processing)
- Dual Xeon processors
- Radeon MI50 GPU 16BGVRAM
- 32GB+ RAM

## 📦 Installation

### 1. Cài đặt trên Jetson Nano

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y python3-pip python3-opencv
sudo apt install -y libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev
sudo apt install -y libjpeg8-dev liblapack-dev libblas-dev gfortran

# Install PyTorch for Jetson
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl
pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl

# Install other dependencies
pip3 install -r requirements.txt

# For better performance, install jetson-stats
sudo pip3 install jetson-stats
```

### 2. Cài đặt trên X99 Workstation

```bash
# Install Python dependencies
pip install -r requirements.txt

# For AMD Radeon MI50, install ROCm (optional)
# Follow: https://rocmdocs.amd.com/en/latest/Installation_Guide/Installation-Guide.html

# Install PyTorch with ROCm support (if using AMD GPU)
pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.7
```

## 🎮 Usage

### 1. Camera Calibration (Bước đầu tiên - BẮT BUỘC)

```bash
# Chuẩn bị chessboard pattern (in ra giấy)
# Kích thước: 9x6 inner corners, mỗi ô vuông 2.5cm

# Chạy calibration
python main.py --calibrate

# Hướng dẫn:
# 1. Giữ chessboard trước 2 cameras
# 2. Nhấn SPACE để capture (cần ~20-30 samples)
# 3. Di chuyển chessboard đến các góc và khoảng cách khác nhau
# 4. Nhấn ESC khi đủ samples
# 5. File stereo_calibration.yaml sẽ được tạo
```

### 2. Run SLAM System

```bash
# Chạy full system (SLAM + Web visualization)
python main.py

# Hoặc với config file tùy chỉnh
python main.py --config config.yaml

# Chỉ chạy web server (để test giao diện)
python main.py --web-only --host 0.0.0.0 --port 5000
```

### 3. Truy cập Web Interface

Mở trình duyệt và vào:
```
http://localhost:5000
```

Hoặc từ máy khác trong mạng:
```
http://<jetson-ip>:5000
```

### 4. Keyboard Controls (khi chạy SLAM)

- `q` - Quit program
- `s` - Save map to file
- `d` - Downsample map (giảm số điểm)

## 📁 Project Structure

```
slam-yolo-system/
├── main.py                 # Main pipeline
├── stereo_camera.py        # Stereo camera handling
├── yolo_segmentation.py    # YOLO detection & segmentation
├── visual_odometry.py      # ORB-based visual odometry
├── web_server.py          # Flask web server
├── config.yaml            # Configuration file
├── requirements.txt       # Python dependencies
├── templates/
│   └── index.html        # Web interface
└── README.md
```

## ⚙️ Configuration

Edit `config.yaml`:

```yaml
camera:
  left_id: 0          # Device ID của camera trái
  right_id: 1         # Device ID của camera phải
  baseline_cm: 10.0   # Khoảng cách giữa 2 cameras

yolo:
  device: "cuda"      # Hoặc "cpu"
  process_every_n_frames: 2  # Xử lý YOLO mỗi N frames

slam:
  keyframe_interval: 10
```

## 🔧 Optimization Tips

### Cho Jetson Nano:

```bash
# Enable maximum performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor
jtop
```

## 📖 Quick API Reference

```python
# Stereo Camera
cam = StereoCamera(left_id=0, right_id=1, baseline_cm=10.0)
cam.initialize_cameras()
cam.load_calibration('stereo_calibration.yaml')

# YOLO
yolo = YOLOSegmentation(model_path='yolov11m-seg.pt', device='cuda')
results = yolo.predict(frame)

# Visual Odometry
vo = VisualOdometry(camera_matrix=K, baseline=0.1)
vo_result = vo.process_stereo_frame(gray_left, gray_right)
```

## 📝 License

MIT License
