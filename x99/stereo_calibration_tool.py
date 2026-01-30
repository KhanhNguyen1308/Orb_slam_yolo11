#!/usr/bin/env python3
"""
Tool Calibrate Stereo Camera qua mạng (Sử dụng stream từ Jetson Nano)
"""

import cv2
import numpy as np
import time
import glob
import threading

# Import module nhận ảnh của bạn
try:
    from x99_headless import OptimizedCameraReceiver
except ImportError:
    print("Không tìm thấy file x99_headless.py. Hãy đặt file này cùng thư mục.")
    exit()

# ================= CẤU HÌNH =================
CHECKERBOARD = (9, 6) # Số điểm giao nhau bên trong (Hàng, Cột)
SQUARE_SIZE = 0.025   # Kích thước ô vuông (mét) - Ví dụ 25mm
LEFT_PORT = 9002
RIGHT_PORT = 9001
# ============================================

class StereoCalibrator:
    def __init__(self):
        self.left_receiver = OptimizedCameraReceiver(LEFT_PORT, "LEFT")
        self.right_receiver = OptimizedCameraReceiver(RIGHT_PORT, "RIGHT")
        
        # Tiêu chí dừng cho thuật toán subpixel
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Mảng lưu điểm 3D thực tế và 2D trên ảnh
        self.objpoints = [] # 3D points in real world space
        self.imgpoints_l = [] # 2D points in left image plane
        self.imgpoints_r = [] # 2D points in right image plane
        
        # Tạo tọa độ giả định cho bàn cờ (0,0,0), (1,0,0), (2,0,0) ...
        self.objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        self.objp = self.objp * SQUARE_SIZE

    def start_cameras(self):
        print("Đang kết nối tới Camera...")
        t1 = threading.Thread(target=lambda: (self.left_receiver.start_server() and self.left_receiver.receive_frames()), daemon=True)
        t2 = threading.Thread(target=lambda: (self.right_receiver.start_server() and self.right_receiver.receive_frames()), daemon=True)
        t1.start()
        t2.start()
        time.sleep(2) # Đợi kết nối

    def capture_loop(self):
        print(f"\n{'='*60}")
        print("HƯỚNG DẪN:")
        print("1. Đưa bàn cờ vào vùng nhìn chung của 2 camera")
        print("2. Giữ yên bàn cờ khi thấy các đường vẽ màu xuất hiện")
        print("3. Nhấn phím 'C' để CHỤP một mẫu")
        print("4. Cần khoảng 15-20 mẫu ở các góc độ/khoảng cách khác nhau")
        print("5. Nhấn 'Q' để KẾT THÚC chụp và bắt đầu tính toán")
        print(f"{'='*60}\n")

        count = 0
        
        while True:
            frame_l = self.left_receiver.get_latest_frame()
            frame_r = self.right_receiver.get_latest_frame()

            if frame_l is None or frame_r is None:
                time.sleep(0.01)
                continue

            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

            # Tìm góc bàn cờ
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD, None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD, None)

            # Copy để vẽ hiển thị (không vẽ lên ảnh gốc)
            vis_l = frame_l.copy()
            vis_r = frame_r.copy()

            valid_pair = False
            
            if ret_l and ret_r:
                valid_pair = True
                # Tinh chỉnh độ chính xác subpixel
                corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), self.criteria)
                corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), self.criteria)

                # Vẽ lên màn hình
                cv2.drawChessboardCorners(vis_l, CHECKERBOARD, corners_l, ret_l)
                cv2.drawChessboardCorners(vis_r, CHECKERBOARD, corners_r, ret_r)
                
                cv2.putText(vis_l, "READY TO CAPTURE", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Ghép 2 ảnh để hiển thị
            h, w = vis_l.shape[:2]
            vis_combined = np.hstack((vis_l, vis_r))
            vis_combined = cv2.resize(vis_combined, (int(w*2*0.8), int(h*0.8))) # Resize cho vừa màn hình
            
            cv2.putText(vis_combined, f"Captured: {count}", (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow('Stereo Calibration Tool', vis_combined)

            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('c'):
                if valid_pair:
                    self.objpoints.append(self.objp)
                    self.imgpoints_l.append(corners_l)
                    self.imgpoints_r.append(corners_r)
                    count += 1
                    print(f"[OK] Đã chụp mẫu số {count}")
                    # Nháy màn hình hiệu ứng chụp
                    cv2.imshow('Stereo Calibration Tool', np.ones_like(vis_combined)*255)
                    cv2.waitKey(50)
                else:
                    print("[WARN] Không tìm thấy bàn cờ ở cả 2 camera!")
            
            elif key == ord('q'):
                if count < 10:
                    print(f"[ERR] Mới có {count} ảnh. Cần ít nhất 10 ảnh để chính xác!")
                else:
                    break
        
        cv2.destroyAllWindows()
        return frame_l.shape[:2][::-1] # Trả về (width, height)

    def calibrate_and_save(self, image_size):
        print("\nĐang tính toán... (Có thể mất vài phút)...")
        
        # 1. Calibrate từng camera riêng lẻ
        print(" -> Calibrating Left Camera...")
        ret_l, mtx_l, dist_l, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_l, image_size, None, None)
        
        print(" -> Calibrating Right Camera...")
        ret_r, mtx_r, dist_r, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_r, image_size, None, None)

        # 2. Calibrate Stereo (Tìm quan hệ giữa 2 cam)
        print(" -> Stereo Calibrating (Finding R, T)...")
        flags = cv2.CALIB_FIX_INTRINSIC
        ret_s, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_l, self.imgpoints_r,
            mtx_l, dist_l,
            mtx_r, dist_r,
            image_size, None, None, None, None,
            flags=flags, criteria=self.criteria
        )

        print(f"\nStereo Error (RMS): {ret_s}")
        print(f"Translation (Baseline): {T.ravel()}")

        # 3. Tính toán Rectification (Làm phẳng ảnh)
        print(" -> Computing Rectification transforms...")
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            mtx_l, dist_l, mtx_r, dist_r, image_size, R, T
        )

        # 4. Lưu lại
        filename = "calibration_params.npz"
        np.savez(filename, 
                 mtx_l=mtx_l, dist_l=dist_l, 
                 mtx_r=mtx_r, dist_r=dist_r,
                 R=R, T=T, Q=Q,
                 R1=R1, R2=R2, P1=P1, P2=P2)
        
        print(f"\n[SUCCESS] Đã lưu tham số vào file '{filename}'")
        print("Hãy sửa file 'stereo_depth_mapping_optimized.py' để load file này!")

if __name__ == "__main__":
    calib = StereoCalibrator()
    calib.start_cameras()
    
    # Chờ stream lên
    print("Waiting for stream...")
    while calib.left_receiver.latest_frame is None:
        time.sleep(0.1)
        
    img_size = calib.capture_loop()
    calib.calibrate_and_save(img_size)