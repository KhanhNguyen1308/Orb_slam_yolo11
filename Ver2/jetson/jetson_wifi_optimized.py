#!/usr/bin/env python3
"""
Jetson Nano Smart Streaming Client (Auto-Exposure Integrated)
- Stream ảnh qua Wifi TCP/IP
- Tự động điều chỉnh Exposure/Gain dựa trên độ sáng môi trường
- Tối ưu hóa để không làm chặn (blocking) luồng ảnh
"""

import cv2
import numpy as np
import socket
import struct
import time
import threading
import subprocess
import argparse
from dataclasses import dataclass

# --- CẤU HÌNH AUTO EXPOSURE ---
EXPOSURE_UPDATE_INTERVAL = 0.5  # Chỉ chỉnh sáng mỗi 0.5s để tránh dao động (flicker)
TARGET_BRIGHTNESS = 110         # Độ sáng mục tiêu (0-255)
BRIGHTNESS_TOLERANCE = 20       # Chấp nhận sai số +/- 20
MIN_EXPOSURE = 5                # Giới hạn phần cứng camera
MAX_EXPOSURE = 2000             # Giới hạn phần cứng (tùy cam, thường max là 5000)
MIN_GAIN = 0
MAX_GAIN = 100

@dataclass
class StreamConfig:
    server_ip: str = "192.168.1.100" # IP của Server X99
    left_port: int = 9002
    right_port: int = 9001
    left_camera_id: int = 0
    right_camera_id: int = 1
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 60      # Giảm xuống 60 để ưu tiên tốc độ cho SLAM
    buffer_size: int = 65536

class AutoExposureController:
    """Module điều khiển độ sáng chạy ngầm"""
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.current_exposure = 150 # Giá trị khởi điểm
        self.current_gain = 10
        self.last_update = time.time()
        self.lock = threading.Lock()
        
        # Tắt Auto Hardware để chuyển sang Manual Control
        self._run_v4l2("--set-ctrl=exposure_auto=1") 
        self._apply_settings()

    def _run_v4l2(self, cmd_str):
        """Gửi lệnh xuống driver"""
        try:
            cmd = f"v4l2-ctl -d /dev/video{self.camera_id} {cmd_str}"
            subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def _apply_settings(self):
        """Áp dụng thông số mới"""
        # Set Exposure
        self._run_v4l2(f"--set-ctrl=exposure_absolute={int(self.current_exposure)}")
        # Set Gain
        self._run_v4l2(f"--set-ctrl=gain={int(self.current_gain)}")

    def update(self, frame):
        """Hàm này được gọi mỗi frame, nhưng chỉ xử lý theo chu kỳ"""
        now = time.time()
        if now - self.last_update < EXPOSURE_UPDATE_INTERVAL:
            return

        # Tính độ sáng trung bình (resize nhỏ để tính cho nhanh)
        # Chỉ lấy vùng trung tâm ảnh để đo sáng (tránh viền đen)
        h, w = frame.shape[:2]
        center_crop = frame[int(h/3):int(2*h/3), int(w/3):int(2*w/3)]
        gray = cv2.cvtColor(center_crop, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.mean(gray)

        # Logic điều khiển P (Proportional) đơn giản
        err = TARGET_BRIGHTNESS - avg_brightness

        if abs(err) < BRIGHTNESS_TOLERANCE:
            return # Đủ sáng, không chỉnh gì cả

        with self.lock:
            # Điều chỉnh Exposure trước
            change_speed = 20 if abs(err) > 50 else 5
            
            if err > 0: # Ảnh tối -> Tăng sáng
                if self.current_exposure < MAX_EXPOSURE:
                    self.current_exposure += change_speed
                else:
                    # Nếu max exposure rồi mà vẫn tối -> Tăng Gain
                    self.current_gain = min(self.current_gain + 2, MAX_GAIN)
            else: # Ảnh sáng quá -> Giảm sáng
                if self.current_gain > MIN_GAIN:
                    # Ưu tiên giảm Gain trước để bớt nhiễu
                    self.current_gain = max(self.current_gain - 2, MIN_GAIN)
                else:
                    self.current_exposure = max(self.current_exposure - change_speed, MIN_EXPOSURE)

            # Clamp giá trị
            self.current_exposure = max(MIN_EXPOSURE, min(self.current_exposure, MAX_EXPOSURE))
            
            # Gửi lệnh điều khiển
            self._apply_settings()
            
            # Debug log (tùy chọn)
            # print(f"Cam {self.camera_id} | Bright: {avg_brightness:.1f} | Exp: {self.current_exposure} | Gain: {self.current_gain}")
            
        self.last_update = now

class CameraThread(threading.Thread):
    """Thread đọc và gửi camera độc lập"""
    def __init__(self, config: StreamConfig, is_right_cam=False):
        super().__init__()
        self.config = config
        self.camera_id = config.right_camera_id if is_right_cam else config.left_camera_id
        self.port = config.right_port if is_right_cam else config.left_port
        self.is_running = True
        self.exposure_ctrl = AutoExposureController(self.camera_id)
        
    def run(self):
        print(f"[*] Starting Camera {self.camera_id} -> Server Port {self.port}")
        
        # Mở Camera
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        # Kết nối Socket tới Server X99
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # Giảm độ trễ
        
        try:
            client_socket.connect((self.config.server_ip, self.port))
            print(f"[+] Camera {self.camera_id} connected to server!")
        except Exception as e:
            print(f"[!] Camera {self.camera_id} connection failed: {e}")
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.config.jpeg_quality]

        while self.is_running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # --- AUTO EXPOSURE LOGIC ---
            # Gọi hàm này mỗi frame, nó sẽ tự tính toán và điều chỉnh driver
            self.exposure_ctrl.update(frame)
            # ---------------------------

            # Nén và gửi ảnh
            result, encimg = cv2.imencode('.jpg', frame, encode_param)
            data = encimg.tobytes()
            size = len(data)

            try:
                # Gửi kích thước trước (4 bytes unsigned int), sau đó đến dữ liệu ảnh
                client_socket.sendall(struct.pack(">L", size) + data)
            except Exception as e:
                print(f"[!] Connection lost on Camera {self.camera_id}: {e}")
                break
        
        cap.release()
        client_socket.close()

def main():
    parser = argparse.ArgumentParser(description='Jetson Smart Streamer')
    parser.add_argument('--server', type=str, required=True, help='IP của Server X99')
    parser.add_argument('--dual', action='store_true', help='Chạy chế độ 2 camera (Stereo)')
    args = parser.parse_args()

    config = StreamConfig(server_ip=args.server)

    # Khởi động Camera Trái (Luôn chạy)
    t1 = CameraThread(config, is_right_cam=False)
    t1.start()

    # Nếu dùng Stereo mode thì khởi động Camera Phải
    if args.dual:
        t2 = CameraThread(config, is_right_cam=True)
        t2.start()
        t2.join()
    
    t1.join()

if __name__ == "__main__":
    main()