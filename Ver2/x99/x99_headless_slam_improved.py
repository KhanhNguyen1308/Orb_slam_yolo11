#!/usr/bin/env python3
"""
Server X99 Main Controller - Tích hợp SLAM và Web Stream
"""
from flask import Flask, Response, render_template_string
import cv2
import numpy as np
import threading
import queue
import time

# Import class SLAM đã sửa ở trên
from x99_slam_improved_drift_correction import DriftCorrectedSLAM
# Import class nhận ảnh từ jetson (giữ nguyên file cũ x99_headless.py)
from x99_headless import OptimizedCameraReceiver

app = Flask(__name__)

# Global Objects
frame_queue = queue.Queue(maxsize=2)
slam_system = DriftCorrectedSLAM()
camera_receiver = OptimizedCameraReceiver(port=9002, name="LeftCam") # Cổng trái

def slam_processing_thread():
    """Luồng xử lý SLAM riêng biệt"""
    camera_receiver.start_server()
    print("[Server] Waiting for connection from Jetson...")
    
    while True:
        # Lấy ảnh mới nhất từ bộ đệm receiver
        frame = camera_receiver.get_latest_frame()
        
        if frame is not None:
            # Gọi SLAM processing
            # Hàm này giờ trả về (pose, color_image)
            pose, vis_frame = slam_system.process(frame)
            
            # Đẩy ảnh màu đã vẽ debug vào queue cho Web xem
            if not frame_queue.full():
                frame_queue.put(vis_frame)
            else:
                # Drop frame cũ để stream luôn mượt (realtime)
                try: frame_queue.get_nowait()
                except: pass
                frame_queue.put(vis_frame)
        else:
            time.sleep(0.01)

def generate_web_stream():
    while True:
        try:
            # Lấy ảnh từ queue (timeout ngắn để không treo)
            frame = frame_queue.get(timeout=0.5)
            
            # Encode JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except:
            pass

@app.route('/')
def index():
    return "<html><body><h1>X99 SLAM Dashboard</h1><img src='/video_feed' style='width:100%'></body></html>"

@app.route('/video_feed')
def video_feed():
    return Response(generate_web_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Chạy thread SLAM
    t = threading.Thread(target=slam_processing_thread)
    t.daemon = True
    t.start()
    
    # Chạy Web Server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)