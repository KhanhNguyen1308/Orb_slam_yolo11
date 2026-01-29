# QUICK START - FIX SLAM MAP

## TÓM TẮT VẤN ĐỀ
SLAM map không hiển thị vì có lỗi trong code:
- Duplicate code trong `process_slam()`
- Biến undefined được sử dụng
- Thiếu error handling

## GIẢI PHÁP NHANH (3 BƯỚC)

### Bước 1: Thay thế file Python
```bash
# Backup file cũ
cp x99_web_slam.py x99_web_slam.backup

# Copy file mới (đã download từ Claude)
cp x99_web_slam_fixed.py x99_web_slam.py
```

### Bước 2: Thay thế file HTML
```bash
# Tạo thư mục templates nếu chưa có
mkdir -p templates

# Copy file HTML mới
cp slam_web_fixed.html templates/slam_web.html
```

### Bước 3: Chạy test
```bash
# Test các component
python3 test_slam_debug.py

# Nếu test PASS, khởi động server
python3 x99_web_slam.py
```

## KIỂM TRA KẾT QUẢ

### 1. Mở browser: http://<X99_IP>:5000

### 2. Xem Debug Console (góc dưới trái):
```
✓ Three.js initialized
✓ Connected to server  
✓ Left camera stream loaded
✓ Right camera stream loaded
✓ Map updated: XXX points
```

### 3. Xem Stats Panel (bên trái):
```
Điểm 3D: XXX       <- Phải > 0
FPS: XX.X          <- Phải > 0
Trajectory: XXX    <- Phải tăng dần
```

### 4. Test API trực tiếp:
```bash
# Phải trả về data, không phải []
curl http://localhost:5000/api/map_data | python3 -m json.tool
```

## NẾU VẪN KHÔNG THẤY MAP

### Check 1: Server logs
```bash
# Phải thấy:
[Web] SLAM components loaded
[Web] Depth mapping working
[Web] Map updated: XXX points
```

### Check 2: Browser console (F12)
```javascript
// Không được có lỗi đỏ
// Test manual:
fetch('/api/map_data')
  .then(r => r.json())
  .then(d => console.log('Points:', d.points.length))
```

### Check 3: Cameras connected?
```bash
# Phải thấy:
[LEFT] Connected from XXX.XXX.XXX.XXX
[RIGHT] Connected from XXX.XXX.XXX.XXX
```

## CÁC LỖI THƯỜNG GẶP

### Lỗi: ImportError
```bash
# Cài đặt dependencies
pip3 install flask flask-socketio opencv-python numpy pillow
```

### Lỗi: Port already in use
```bash
# Kill process cũ
sudo lsof -t -i:5000 | xargs kill -9
```

### Lỗi: No camera data
```bash
# Kiểm tra Jetson có chạy không
ping <JETSON_IP>

# Kiểm tra port 9001, 9002
telnet <X99_IP> 9001
```

## FILES CẦN CÓ

```
project/
├── x99_web_slam.py          # ← File đã sửa
├── x99_headless.py
├── stereo_depth_mapping.py
├── persistent_map.py
├── templates/
│   └── slam_web.html       # ← File đã sửa
└── test_slam_debug.py      # ← Script test
```

## SUPPORT

Nếu vẫn không work:
1. Chạy: `python3 test_slam_debug.py`
2. Screenshot Debug Console từ browser
3. Copy Python server logs
4. Check file README_FIX.md để biết chi tiết

---
Updated: 2025-01-29