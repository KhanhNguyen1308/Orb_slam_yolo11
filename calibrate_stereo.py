#!/usr/bin/env python3
"""
Stereo Camera Calibration Tool
Calibrates stereo cameras and saves calibration parameters
"""

import cv2
import numpy as np
import glob
import json
import os
from datetime import datetime

class StereoCalibration:
    """Stereo camera calibration"""
    
    def __init__(self, checkerboard_size=(9, 6), square_size=0.025):
        """
        checkerboard_size: (columns, rows) of inner corners
        square_size: Size of checkerboard square in meters (e.g., 2.5cm)
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # Prepare object points
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 
                                     0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Storage for calibration
        self.objpoints = []  # 3D points in real world
        self.imgpoints_left = []  # 2D points in left image
        self.imgpoints_right = []  # 2D points in right image
        
    def capture_calibration_images(self, left_id=0, right_id=1, num_images=20):
        """Capture calibration images from stereo cameras"""
        
        cap_left = cv2.VideoCapture(left_id)
        cap_right = cv2.VideoCapture(right_id)
        
        if not cap_left.isOpened() or not cap_right.isOpened():
            print("Failed to open cameras")
            return False
        
        # Create directory for calibration images
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        calib_dir = f"calibration_images_{timestamp}"
        os.makedirs(calib_dir, exist_ok=True)
        
        print(f"\n📷 Stereo Camera Calibration")
        print(f"Checkerboard: {self.checkerboard_size[0]}x{self.checkerboard_size[1]}")
        print(f"Square size: {self.square_size*1000:.1f}mm")
        print(f"\nInstructions:")
        print("- Position checkerboard in view of both cameras")
        print("- Press SPACE to capture image")
        print("- Press Q to finish capture")
        print(f"- Target: {num_images} images\n")
        
        captured = 0
        
        while captured < num_images:
            ret_left, frame_left = cap_left.read()
            ret_right, frame_right = cap_right.read()
            
            if not ret_left or not ret_right:
                continue
            
            # Convert to grayscale
            gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
            
            # Find checkerboard corners
            found_left, corners_left = cv2.findChessboardCorners(
                gray_left, self.checkerboard_size, None
            )
            found_right, corners_right = cv2.findChessboardCorners(
                gray_right, self.checkerboard_size, None
            )
            
            # Draw corners on display frames
            display_left = frame_left.copy()
            display_right = frame_right.copy()
            
            if found_left:
                cv2.drawChessboardCorners(
                    display_left, self.checkerboard_size, corners_left, found_left
                )
            if found_right:
                cv2.drawChessboardCorners(
                    display_right, self.checkerboard_size, corners_right, found_right
                )
            
            # Add status text
            status = f"Captured: {captured}/{num_images}"
            if found_left and found_right:
                status += " - READY (Press SPACE)"
                color = (0, 255, 0)
            else:
                status += " - Position checkerboard"
                color = (0, 0, 255)
            
            cv2.putText(display_left, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display_right, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # Show images
            combined = np.hstack([display_left, display_right])
            cv2.imshow('Stereo Calibration', combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and found_left and found_right:
                # Save images
                left_path = os.path.join(calib_dir, f"left_{captured:02d}.jpg")
                right_path = os.path.join(calib_dir, f"right_{captured:02d}.jpg")
                
                cv2.imwrite(left_path, frame_left)
                cv2.imwrite(right_path, frame_right)
                
                captured += 1
                print(f"✓ Captured image {captured}/{num_images}")
                
            elif key == ord('q'):
                print("\nCapture stopped by user")
                break
        
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()
        
        print(f"\n✓ Calibration images saved to: {calib_dir}")
        return calib_dir
    
    def calibrate_from_directory(self, calib_dir):
        """Calibrate stereo cameras from saved images"""
        
        left_images = sorted(glob.glob(os.path.join(calib_dir, "left_*.jpg")))
        right_images = sorted(glob.glob(os.path.join(calib_dir, "right_*.jpg")))
        
        if len(left_images) == 0 or len(right_images) == 0:
            print("No calibration images found")
            return None
        
        print(f"\n🔧 Processing {len(left_images)} image pairs...")
        
        img_size = None
        valid_pairs = 0
        
        for left_path, right_path in zip(left_images, right_images):
            img_left = cv2.imread(left_path)
            img_right = cv2.imread(right_path)
            
            if img_size is None:
                img_size = (img_left.shape[1], img_left.shape[0])
            
            gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
            
            # Find corners
            found_left, corners_left = cv2.findChessboardCorners(
                gray_left, self.checkerboard_size, None
            )
            found_right, corners_right = cv2.findChessboardCorners(
                gray_right, self.checkerboard_size, None
            )
            
            if found_left and found_right:
                # Refine corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), 
                                                (-1, -1), criteria)
                corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), 
                                                 (-1, -1), criteria)
                
                self.objpoints.append(self.objp)
                self.imgpoints_left.append(corners_left)
                self.imgpoints_right.append(corners_right)
                valid_pairs += 1
        
        print(f"✓ Valid image pairs: {valid_pairs}/{len(left_images)}")
        
        if valid_pairs < 10:
            print("⚠ Warning: Less than 10 valid pairs, calibration may be inaccurate")
        
        # Calibrate cameras
        print("\n📐 Calibrating cameras...")
        ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_left, img_size, None, None
        )
        ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, img_size, None, None
        )
        
        # Stereo calibration
        flags = cv2.CALIB_FIX_INTRINSIC
        ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = \
            cv2.stereoCalibrate(
                self.objpoints,
                self.imgpoints_left,
                self.imgpoints_right,
                mtx_left, dist_left,
                mtx_right, dist_right,
                img_size,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
                flags=flags
            )
        
        # Stereo rectification
        R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
            mtx_left, dist_left,
            mtx_right, dist_right,
            img_size, R, T,
            alpha=0
        )
        
        baseline = np.linalg.norm(T)
        
        calibration = {
            'image_size': img_size,
            'left_camera_matrix': mtx_left.tolist(),
            'left_distortion': dist_left.tolist(),
            'right_camera_matrix': mtx_right.tolist(),
            'right_distortion': dist_right.tolist(),
            'rotation_matrix': R.tolist(),
            'translation_vector': T.tolist(),
            'baseline': baseline,
            'reprojection_error': ret_stereo
        }
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        calib_file = f"stereo_calibration_{timestamp}.json"
        
        with open(calib_file, 'w') as f:
            json.dump(calibration, f, indent=2)
        
        print(f"\n✅ Calibration complete!")
        print(f"Reprojection error: {ret_stereo:.4f}")
        print(f"Baseline: {baseline*100:.2f} cm")
        print(f"Saved to: {calib_file}")
        
        return calibration

def main():
    """Main calibration workflow"""
    
    calibrator = StereoCalibration(checkerboard_size=(9, 6), square_size=0.025)
    
    print("\nOptions:")
    print("1. Capture new calibration images")
    print("2. Use existing calibration images")
    choice = input("\nSelect option (1/2): ")
    
    if choice == '1':
        calib_dir = calibrator.capture_calibration_images(0, 1, 20)
        if calib_dir:
            calibrator.calibrate_from_directory(calib_dir)
    elif choice == '2':
        calib_dir = input("Enter calibration images directory: ")
        if os.path.exists(calib_dir):
            calibrator.calibrate_from_directory(calib_dir)

if __name__ == "__main__":
    main()
