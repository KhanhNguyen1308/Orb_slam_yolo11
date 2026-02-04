#!/usr/bin/env python3
"""
WEB-BASED Stereo Calibration Tool (No Monitor Required)
Chạy trên Server, hiển thị trên trình duyệt.
"""

import os
# Tắt hoàn toàn việc tìm kiếm màn hình của OpenCV/Qt
os.environ["QT_QPA_PLATFORM"] = "offscreen"

from flask import Flask, Response, jsonify, render_template_string
import cv2
import numpy as np
import threading
import time
import json

# Import module nhận ảnh
try:
    from x99_headless import OptimizedCameraReceiver
except ImportError:
    print("Error: x99_headless.py not found.")
    exit(1)

app = Flask(__name__)

# ================= CẤU HÌNH =================
CHECKERBOARD = (9, 6) # Số điểm giao nhau bên trong (Hàng, Cột)
SQUARE_SIZE = 0.024   # Kích thước ô vuông (mét)
LEFT_PORT = 9002
RIGHT_PORT = 9001
# ============================================

class WebCalibrator:
    def __init__(self):
        self.left_receiver = OptimizedCameraReceiver(LEFT_PORT, "LEFT")
        self.right_receiver = OptimizedCameraReceiver(RIGHT_PORT, "RIGHT")
        
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objpoints = [] 
        self.imgpoints_l = [] 
        self.imgpoints_r = []
        
        self.objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        self.objp = self.objp * SQUARE_SIZE

        self.latest_vis = None
        self.last_corners_l = None
        self.last_corners_r = None
        self.valid_pair = False
        self.is_calibrating = False
        self.status_msg = "Ready. Waiting for checkerboard..."
        self.lock = threading.Lock()

    def start(self):
        t1 = threading.Thread(target=lambda: (self.left_receiver.start_server() and self.left_receiver.receive_frames()), daemon=True)
        t2 = threading.Thread(target=lambda: (self.right_receiver.start_server() and self.right_receiver.receive_frames()), daemon=True)
        t1.start()
        t2.start()
        
        # Thread xử lý ảnh nền
        threading.Thread(target=self.processing_loop, daemon=True).start()

    def processing_loop(self):
        while True:
            if self.is_calibrating:
                time.sleep(1)
                continue

            frame_l = self.left_receiver.get_latest_frame()
            frame_r = self.right_receiver.get_latest_frame()

            if frame_l is None or frame_r is None:
                time.sleep(0.05)
                continue

            # Xử lý tìm góc bàn cờ
            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

            ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD, None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD, None)

            # Vẽ visualization
            vis_l = frame_l.copy()
            vis_r = frame_r.copy()
            cv2.putText(vis_l, "LEFT CAM (Goc Trai)", (50, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
            
            # Vẽ chữ RIGHT to đùng màu XANH
            cv2.putText(vis_r, "RIGHT CAM (Goc Phai)", (50, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 4)
            # ===================================
            with self.lock:
                self.valid_pair = False
                if ret_l and ret_r:
                    # Tinh chỉnh subpixel
                    corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), self.criteria)
                    corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), self.criteria)
                    
                    # Lưu tạm để dùng khi chụp
                    self.last_corners_l = corners_l
                    self.last_corners_r = corners_r
                    self.valid_pair = True
                    self.status_msg = "DETECTED! Ready to Capture."

                    cv2.drawChessboardCorners(vis_l, CHECKERBOARD, corners_l, ret_l)
                    cv2.drawChessboardCorners(vis_r, CHECKERBOARD, corners_r, ret_r)
                else:
                    self.status_msg = "Looking for checkerboard..."

                # Ghép ảnh
                h, w = vis_l.shape[:2]
                self.latest_vis = np.hstack((vis_l, vis_r))
                
                # Resize cho nhẹ web
                if w > 640:
                    scale = 640 / w
                    self.latest_vis = cv2.resize(self.latest_vis, (0,0), fx=scale, fy=scale)

            time.sleep(0.05) # Giới hạn 20 FPS xử lý

    def capture(self):
        with self.lock:
            if self.valid_pair and not self.is_calibrating:
                self.objpoints.append(self.objp)
                self.imgpoints_l.append(self.last_corners_l)
                self.imgpoints_r.append(self.last_corners_r)
                return True, len(self.objpoints)
            return False, len(self.objpoints)

    def run_calibration(self):
        self.is_calibrating = True
        self.status_msg = "CALIBRATING... Please wait (1-2 mins)..."
        
        try:
            # Lấy kích thước ảnh gốc từ camera (không phải ảnh resize hiển thị)
            frame = self.left_receiver.get_latest_frame()
            image_size = frame.shape[:2][::-1]

            print("Start Calibrating...")
            ret_l, mtx_l, dist_l, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_l, image_size, None, None)
            ret_r, mtx_r, dist_r, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_r, image_size, None, None)

            flags = cv2.CALIB_FIX_INTRINSIC
            ret_s, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
                self.objpoints, self.imgpoints_l, self.imgpoints_r,
                mtx_l, dist_l, mtx_r, dist_r,
                image_size, None, None, None, None,
                flags=flags, criteria=self.criteria
            )

            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                mtx_l, dist_l, mtx_r, dist_r, image_size, R, T
            )

            filename = "calibration_params.npz"
            np.savez(filename, mtx_l=mtx_l, dist_l=dist_l, mtx_r=mtx_r, dist_r=dist_r,
                     R=R, T=T, Q=Q, R1=R1, R2=R2, P1=P1, P2=P2)
            
            self.status_msg = f"SUCCESS! Saved to {filename}. Error: {ret_s:.4f}"
            self.is_calibrating = False
            return True, ret_s
        except Exception as e:
            self.status_msg = f"FAILED: {str(e)}"
            self.is_calibrating = False
            return False, str(e)

calib = WebCalibrator()

# ================= HTML GIAO DIỆN =================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>X99 Web Calibration</title>
    <style>
        body { background: #111; color: #eee; font-family: sans-serif; text-align: center; }
        .container { max-width: 1200px; margin: 0 auto; }
        img { border: 2px solid #444; max-width: 100%; margin-top: 10px; }
        .btn { padding: 15px 30px; font-size: 18px; cursor: pointer; border: none; border-radius: 5px; margin: 10px; }
        .btn-capture { background: #007bff; color: white; }
        .btn-calib { background: #28a745; color: white; }
        .btn:disabled { background: #555; cursor: not-allowed; }
        #status { font-size: 1.2em; color: #ffc107; margin: 15px; font-weight: bold; }
        #count { font-size: 2em; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🛠️ X99 Stereo Calibration Tool</h1>
        
        <div id="status">Waiting for stream...</div>
        
        <div>
            <img src="/video_feed" id="video">
        </div>

        <div style="margin-top: 20px;">
            <div style="font-size: 1.2em;">Captured Samples: <span id="count" style="color: #00ff88">0</span></div>
            <br>
            <button class="btn btn-capture" onclick="capture()">📸 CAPTURE (Space)</button>
            <button class="btn btn-calib" onclick="calibrate()">⚙️ RUN CALIBRATION</button>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/status').then(r => r.json()).then(data => {
                document.getElementById('status').innerText = data.msg;
                document.getElementById('count').innerText = data.count;
                
                const btnCap = document.querySelector('.btn-capture');
                const btnCal = document.querySelector('.btn-calib');
                
                if (data.is_calibrating) {
                    btnCap.disabled = true;
                    btnCal.disabled = true;
                } else {
                    btnCal.disabled = (data.count < 10);
                    btnCap.disabled = !data.valid;
                }
            });
        }

        function capture() {
            fetch('/capture').then(r => r.json()).then(data => {
                if(data.success) {
                    // Flash effect
                    const img = document.getElementById('video');
                    img.style.opacity = 0;
                    setTimeout(() => img.style.opacity = 1, 100);
                }
            });
        }

        function calibrate() {
            if(!confirm("Start calibration? This will take a while.")) return;
            fetch('/calibrate');
        }

        setInterval(updateStatus, 500);

        // Hotkey Space to capture
        document.addEventListener('keydown', function(event) {
            if (event.code === 'Space') {
                capture();
            }
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

def gen_frames():
    while True:
        with calib.lock:
            if calib.latest_vis is not None:
                ret, buffer = cv2.imencode('.jpg', calib.latest_vis)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    return jsonify({
        'msg': calib.status_msg,
        'count': len(calib.objpoints),
        'valid': calib.valid_pair,
        'is_calibrating': calib.is_calibrating
    })

@app.route('/capture')
def capture_route():
    success, count = calib.capture()
    return jsonify({'success': success, 'count': count})

@app.route('/calibrate')
def calibrate_route():
    threading.Thread(target=calib.run_calibration).start()
    return jsonify({'started': True})

if __name__ == '__main__':
    calib.start()
    print("Web Calibration Tool started on port 5000")
    app.run(host='0.0.0.0', port=5000, debug=False)