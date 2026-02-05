#!/usr/bin/env python3
"""
[JETSON NANO] Optimized Streamer + Smart Auto Exposure
Gửi ảnh qua Wifi và tự động điều chỉnh ánh sáng camera
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
EXPOSURE_UPDATE_INTERVAL = 0.5  # Chu kỳ chỉnh sáng (giây)
TARGET_BRIGHTNESS = 110         # Độ sáng mục tiêu (0-255)
BRIGHTNESS_TOLERANCE = 15       # Sai số chấp nhận được

@dataclass
class StreamConfig:
    server_ip: str
    left_port: int = 9002
    right_port: int = 9001
    left_camera_id: int = 0
    right_camera_id: int = 1
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 60      # Giảm nhẹ quality để ưu tiên độ mượt cho SLAM
    
class AutoExposureController:
    """Tự động điều chỉnh v4l2 driver dựa trên độ sáng ảnh"""
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.current_exposure = 150
        self.current_gain = 10
        self.last_update = time.time()
        
        # Reset Camera về Manual Mode để code có thể điều khiển
        self._run_v4l2("--set-ctrl=exposure_auto=1") 
        self._apply_settings()

    def _run_v4l2(self, cmd_str):
        try:
            cmd = f"v4l2-ctl -d /dev/video{self.camera_id} {cmd_str}"
            subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except: pass

    def _apply_settings(self):
        # Clamp giá trị trong ngưỡng an toàn của phần cứng
        self.current_exposure = max(5, min(self.current_exposure, 5000))
        self.current_gain = max(0, min(self.current_gain, 100))
        
        self._run_v4l2(f"--set-ctrl=exposure_absolute={int(self.current_exposure)}")
        self._run_v4l2(f"--set-ctrl=gain={int(self.current_gain)}")

    def update(self, frame):
        now = time.time()
        if now - self.last_update < EXPOSURE_UPDATE_INTERVAL: return

        # Chỉ đo sáng vùng trung tâm ảnh (quan trọng nhất cho SLAM)
        h, w = frame.shape[:2]
        center = frame[int(h/3):int(2*h/3), int(w/3):int(2*w/3)]
        if center.size == 0: return
        
        avg_bright = np.mean(cv2.cvtColor(center, cv2.COLOR_BGR2GRAY))
        err = TARGET_BRIGHTNESS - avg_bright

        if abs(err) > BRIGHTNESS_TOLERANCE:
            change = 20 if abs(err) > 50 else 5
            if err > 0: # Quá tối
                if self.current_exposure < 2000: self.current_exposure += change
                else: self.current_gain += 2 # Nếu exposure max thì tăng gain
            else: # Quá sáng
                if self.current_gain > 0: self.current_gain -= 2
                else: self.current_exposure -= change
            
            self._apply_settings()
            self.last_update = now

def run_camera(config, cam_id, port):
    print(f"[*] Starting Cam {cam_id} -> {config.server_ip}:{port}")
    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)
    
    # AE Controller
    ae = AutoExposureController(cam_id)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    
    while True:
        try:
            sock.connect((config.server_ip, port))
            print(f"[+] Connected to port {port}")
            break
        except:
            time.sleep(1)

    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        # Auto Exposure Update
        ae.update(frame)
        
        # Encode & Send
        _, data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), config.jpeg_quality])
        try:
            sock.sendall(struct.pack(">L", len(data)) + data.tobytes())
        except: break # Reconnect logic should be outside
            
    cap.release()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', required=True)
    parser.add_argument('--dual', action='store_true')
    args = parser.parse_args()
    
    conf = StreamConfig(server_ip=args.server)
    
    t1 = threading.Thread(target=run_camera, args=(conf, conf.left_camera_id, conf.left_port))
    t1.start()
    
    if args.dual:
        t2 = threading.Thread(target=run_camera, args=(conf, conf.right_camera_id, conf.right_port))
        t2.start()
        t2.join()
    t1.join()