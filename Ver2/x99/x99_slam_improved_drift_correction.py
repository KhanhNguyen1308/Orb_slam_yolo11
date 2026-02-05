#!/usr/bin/env python3
"""
SLAM Improved with Drift Correction
Giải quyết vấn đề robot trôi trên map bằng:
1. Loop Closure Detection - phát hiện khi robot quay lại vị trí cũ
2. Pose Graph Optimization - tối ưu toàn bộ trajectory
3. Adaptive Keyframe Selection - chọn keyframe thông minh hơn
4. Scale Drift Correction - điều chỉnh scale khi phát hiện sai lệch
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
import time
from dataclasses import dataclass
from collections import deque

@dataclass
class Keyframe:
    """Keyframe data structure"""
    id: int
    timestamp: float
    pose: np.ndarray  # 4x4 transformation matrix
    keypoints: List
    descriptors: np.ndarray
    image: np.ndarray  # Grayscale image for loop closure
    points_3d: List[np.ndarray]  # 3D points visible in this frame
    
@dataclass  
class LoopClosure:
    """Loop closure constraint"""
    kf_id_1: int
    kf_id_2: int
    relative_pose: np.ndarray  # T from kf1 to kf2
    confidence: float

class DriftCorrectedSLAM:
    """
    SLAM với khả năng tự sửa drift
    """
    
    def __init__(self, n_features: int = 2000, 
                 baseline: float = 0.10,
                 focal_length: float = 500,
                 cx: float = 320, cy: float = 240):
        
        # ORB Feature Detector
        try:
            # Try full parameters first
            self.orb = cv2.ORB_create(
                nfeatures=n_features,
                scaleFactor=1.2,
                nLevels=8,
                edgeThreshold=15,
                firstLevel=0,
                WTA_K=2,
                scoreType=cv2.ORB_HARRIS_SCORE,
                patchSize=31,
                fastThreshold=20
            )
        except TypeError:
            # Fallback for older versions
            try:
                self.orb = cv2.ORB_create(
                    nfeatures=n_features,
                    scaleFactor=1.2,
                    nlevels=8,  # lowercase
                    edgeThreshold=15
                )
            except TypeError:
                # Minimal fallback
                self.orb = cv2.ORB_create(nfeatures=n_features)

        print(f"[ORB] Created with {n_features} features")
                
        # Feature Matcher  
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Camera intrinsics
        self.K = np.array([
            [focal_length, 0, cx],
            [0, focal_length, cy],
            [0, 0, 1]
        ], dtype=np.float32)
        self.baseline = baseline
        
        # SLAM State
        self.current_pose = np.eye(4, dtype=np.float32)
        self.keyframes: List[Keyframe] = []
        self.loop_closures: List[LoopClosure] = []
        
        # Previous frame tracking
        self.prev_kp = None
        self.prev_desc = None
        self.prev_points_3d = []
        self.prev_kp_to_3d = {}
        
        # Drift correction params
        self.velocity_buffer = deque(maxlen=10)  # Lưu vận tốc để phát hiện bất thường
        self.last_pose = np.eye(4, dtype=np.float32)
        self.cumulative_drift = np.zeros(3)  # [x, y, z] drift accumulation
        
        # Loop closure detection
        self.loop_closure_min_distance = 1.5  # meters
        self.loop_closure_similarity_threshold = 0.7
        self.last_loop_closure_frame = 0
        self.loop_closure_interval = 30  # Check every N frames
        
        # Statistics
        self.frame_count = 0
        self.tracking_quality = 'INIT'
        self.num_inliers = 0
        self.total_drift_corrections = 0
        
        print("[SLAM+] Drift Correction SLAM Initialized")
        print(f"  Features: {n_features}, Baseline: {baseline}m")
        
    def extract_features(self, image: np.ndarray):
        """Extract ORB features"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
            
        # Apply CLAHE for better feature detection
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        kp, desc = self.orb.detectAndCompute(gray, None)
        return kp, desc
    
    def match_features(self, desc1, desc2, ratio_thresh: float = 0.75):
        """Lowe's ratio test matching"""
        if desc1 is None or desc2 is None:
            return []
        
        matches = self.bf_matcher.knnMatch(desc1, desc2, k=2)
        
        good = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < ratio_thresh * n.distance:
                    good.append(m)
        
        return good
    
    def triangulate_stereo(self, kp_left, desc_left, kp_right, desc_right):
        """Triangulate 3D points from stereo"""
        matches = self.match_features(desc_left, desc_right, ratio_thresh=0.8)
        
        if len(matches) < 10:
            return [], [], []
        
        points_3d = []
        kp_indices = []
        point_descs = []
        
        for m in matches:
            pt_l = kp_left[m.queryIdx].pt
            pt_r = kp_right[m.trainIdx].pt
            
            # Epipolar check
            if abs(pt_l[1] - pt_r[1]) > 2.0:
                continue
            
            disparity = pt_l[0] - pt_r[0]
            if disparity < 1.0:
                continue
            
            # Depth
            depth = (self.baseline * self.K[0,0]) / disparity
            
            if depth < 0.1 or depth > 15.0:
                continue
            
            # Unproject
            x = (pt_l[0] - self.K[0,2]) * depth / self.K[0,0]
            y = (pt_l[1] - self.K[1,2]) * depth / self.K[1,1]
            z = depth
            
            points_3d.append(np.array([x, y, z], dtype=np.float32))
            kp_indices.append(m.queryIdx)
            point_descs.append(desc_left[m.queryIdx])
        
        return points_3d, kp_indices, point_descs
    
    def estimate_motion_with_outlier_rejection(self, kp_current, desc_current):
        """
        Estimate motion with enhanced outlier rejection
        """
        matches = self.match_features(self.prev_desc, desc_current)
        
        if len(matches) < 30:
            return False, None
        
        # Build correspondences
        pts_3d = []
        pts_2d = []
        
        for m in matches:
            if m.queryIdx in self.prev_kp_to_3d:
                pts_3d.append(self.prev_kp_to_3d[m.queryIdx])
                pts_2d.append(kp_current[m.trainIdx].pt)
        
        if len(pts_3d) < 15:
            return False, None
        
        pts_3d = np.array(pts_3d, dtype=np.float32)
        pts_2d = np.array(pts_2d, dtype=np.float32)
        
        # PnP RANSAC
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            objectPoints=pts_3d,
            imagePoints=pts_2d,
            cameraMatrix=self.K,
            distCoeffs=None,
            iterationsCount=200,
            reprojectionError=3.0,  # Stricter threshold
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success or inliers is None or len(inliers) < 20:
            return False, None
        
        # Compute transformation
        R, _ = cv2.Rodrigues(rvec)
        T_motion = np.eye(4, dtype=np.float32)
        T_motion[:3, :3] = R
        T_motion[:3, 3] = tvec.flatten()
        
        self.num_inliers = len(inliers)
        
        # DRIFT DETECTION: Check if motion is reasonable
        translation = np.linalg.norm(tvec)
        rotation_angle = np.linalg.norm(rvec)
        
        # Velocity sanity check (assuming ~30fps, max 1m/s movement)
        max_translation_per_frame = 0.1  # 10cm per frame max
        max_rotation_per_frame = 0.3  # ~17 degrees
        
        if translation > max_translation_per_frame:
            print(f"[SLAM+] WARNING: Excessive translation {translation:.3f}m - possible drift!")
            return False, None
            
        if rotation_angle > max_rotation_per_frame:
            print(f"[SLAM+] WARNING: Excessive rotation {rotation_angle:.3f}rad")
            return False, None
        
        return True, T_motion
    
    def apply_drift_correction(self, T_motion):
        """
        Apply drift correction based on motion consistency
        """
        # Extract translation
        trans = T_motion[:3, 3]
        
        # Add to velocity buffer
        self.velocity_buffer.append(trans)
        
        # If we have enough history, check for drift
        if len(self.velocity_buffer) >= 5:
            velocities = np.array(self.velocity_buffer)
            mean_vel = np.mean(velocities, axis=0)
            std_vel = np.std(velocities, axis=0)
            
            # If current motion is very different from average, scale it down
            deviation = np.abs(trans - mean_vel)
            
            # Apply correction factor
            correction_factor = np.ones(3)
            for i in range(3):
                if std_vel[i] > 0.01:  # Avoid division by zero
                    z_score = deviation[i] / std_vel[i]
                    if z_score > 2.0:  # More than 2 sigma - probably drift
                        correction_factor[i] = 0.5  # Reduce by 50%
                        self.cumulative_drift[i] += deviation[i]
                        print(f"[SLAM+] Drift correction on axis {i}: factor={correction_factor[i]:.2f}")
            
            # Apply correction
            T_corrected = T_motion.copy()
            T_corrected[:3, 3] = trans * correction_factor
            
            return T_corrected
        
        return T_motion
    
    def detect_loop_closure(self):
        """
        Detect loop closure - khi robot quay lại vị trí đã đi qua
        """
        if len(self.keyframes) < 10:
            return None
        
        # Only check periodically
        if self.frame_count - self.last_loop_closure_frame < self.loop_closure_interval:
            return None
        
        current_kf = self.keyframes[-1]
        current_pos = current_kf.pose[:3, 3]
        
        # Check against all previous keyframes (except recent ones)
        for old_kf in self.keyframes[:-5]:
            old_pos = old_kf.pose[:3, 3]
            distance = np.linalg.norm(current_pos - old_pos)
            
            # If close enough, try to match
            if distance < self.loop_closure_min_distance:
                
                # Match descriptors
                matches = self.match_features(
                    old_kf.descriptors, 
                    current_kf.descriptors,
                    ratio_thresh=0.7
                )
                
                if len(matches) < 50:
                    continue
                
                # Compute similarity score
                similarity = len(matches) / min(len(old_kf.descriptors), len(current_kf.descriptors))
                
                if similarity > self.loop_closure_similarity_threshold:
                    print(f"[SLAM+] 🔄 LOOP CLOSURE detected!")
                    print(f"  Current KF {current_kf.id} matched with KF {old_kf.id}")
                    print(f"  Matches: {len(matches)}, Similarity: {similarity:.2f}")
                    
                    # Compute relative pose between the two keyframes
                    relative_pose = np.linalg.inv(old_kf.pose) @ current_kf.pose
                    
                    loop = LoopClosure(
                        kf_id_1=old_kf.id,
                        kf_id_2=current_kf.id,
                        relative_pose=relative_pose,
                        confidence=similarity
                    )
                    
                    self.last_loop_closure_frame = self.frame_count
                    return loop
        
        return None
    
    def optimize_pose_graph(self, loop_closure: LoopClosure):
        """
        Pose Graph Optimization - điều chỉnh toàn bộ trajectory khi phát hiện loop
        Simplified version using direct adjustment
        """
        print("[SLAM+] Optimizing pose graph...")
        
        kf1_id = loop_closure.kf_id_1
        kf2_id = loop_closure.kf_id_2
        
        # Find keyframes
        kf1 = None
        kf2 = None
        for kf in self.keyframes:
            if kf.id == kf1_id:
                kf1 = kf
            if kf.id == kf2_id:
                kf2 = kf
        
        if kf1 is None or kf2 is None:
            return
        
        # Compute error
        measured_relative = loop_closure.relative_pose
        actual_relative = np.linalg.inv(kf1.pose) @ kf2.pose
        
        error_pose = measured_relative @ np.linalg.inv(actual_relative)
        
        # Extract translation error
        trans_error = error_pose[:3, 3]
        error_magnitude = np.linalg.norm(trans_error)
        
        print(f"  Loop closure error: {error_magnitude:.3f}m")
        
        # Distribute error correction across intermediate keyframes
        start_idx = kf1_id
        end_idx = kf2_id
        
        num_kfs = end_idx - start_idx
        if num_kfs <= 0:
            return
        
        # Linear interpolation of correction
        for i, kf in enumerate(self.keyframes):
            if kf.id > start_idx and kf.id <= end_idx:
                # Interpolation factor
                alpha = (kf.id - start_idx) / num_kfs
                
                # Apply partial correction
                correction = alpha * trans_error
                kf.pose[:3, 3] -= correction
        
        # Update current pose
        self.current_pose = self.keyframes[-1].pose.copy()
        
        self.total_drift_corrections += 1
        print(f"  ✓ Pose graph optimized. Total corrections: {self.total_drift_corrections}")
    
    def process_frame(self, img_left: np.ndarray, img_right: np.ndarray):
        """
        Main SLAM processing with drift correction
        """
        self.frame_count += 1
        
        # Extract features
        kp_left, desc_left = self.extract_features(img_left)
        kp_right, desc_right = self.extract_features(img_right)
        
        if len(kp_left) < 50:
            self.tracking_quality = 'POOR'
            return self.current_pose, 'POOR'
        
        # Track previous frame
        if self.prev_desc is not None:
            success, T_motion = self.estimate_motion_with_outlier_rejection(kp_left, desc_left)
            
            if success:
                # Apply drift correction
                T_motion_corrected = self.apply_drift_correction(T_motion)
                
                # Update pose
                T_prev_to_current = np.linalg.inv(T_motion_corrected)
                self.current_pose = self.current_pose @ T_prev_to_current
                
                self.tracking_quality = 'GOOD'
            else:
                self.tracking_quality = 'LOST'
                print(f"[SLAM+] Frame {self.frame_count}: TRACKING LOST")
        else:
            self.tracking_quality = 'INIT'
        
        # Triangulate new 3D points
        pts_3d, kp_indices, _ = self.triangulate_stereo(
            kp_left, desc_left, kp_right, desc_right
        )
        
        # Transform to world frame
        pts_3d_world = []
        for pt in pts_3d:
            pt_h = np.append(pt, 1.0)
            pt_world = self.current_pose @ pt_h
            pts_3d_world.append(pt_world[:3])
        
        # Update tracking data
        self.prev_kp = kp_left
        self.prev_desc = desc_left
        self.prev_points_3d = pts_3d
        self.prev_kp_to_3d = {kp_indices[i]: pts_3d[i] for i in range(len(pts_3d))}
        
        # Keyframe decision (every 10 frames or if tracking is good)
        if self.frame_count % 10 == 0 and self.tracking_quality == 'GOOD':
            self._add_keyframe(img_left, kp_left, desc_left, pts_3d_world)
            
            # Try loop closure detection
            loop = self.detect_loop_closure()
            if loop is not None:
                self.loop_closures.append(loop)
                self.optimize_pose_graph(loop)
        
        return self.current_pose, self.tracking_quality
    
    def _add_keyframe(self, img_left, kp, desc, pts_3d):
        """Add keyframe"""
        kf = Keyframe(
            id=len(self.keyframes),
            timestamp=time.time(),
            pose=self.current_pose.copy(),
            keypoints=kp,
            descriptors=desc,
            image=cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY) if len(img_left.shape) == 3 else img_left.copy(),
            points_3d=pts_3d
        )
        self.keyframes.append(kf)
        print(f"[SLAM+] Keyframe {kf.id} added at {self.current_pose[:3, 3]}")
    
    def get_trajectory(self):
        """Get trajectory from keyframes"""
        trajectory = []
        for kf in self.keyframes:
            pos = kf.pose[:3, 3]
            trajectory.append([pos[0], pos[1], pos[2], kf.timestamp])
        return trajectory
    
    def get_current_position_2d(self):
        """Get current 2D position (x, z)"""
        pos = self.current_pose[:3, 3]
        return pos[0], pos[2]
    
    def reset(self):
        """Reset SLAM"""
        self.current_pose = np.eye(4, dtype=np.float32)
        self.keyframes.clear()
        self.loop_closures.clear()
        self.prev_kp = None
        self.prev_desc = None
        self.prev_points_3d = []
        self.prev_kp_to_3d = {}
        self.velocity_buffer.clear()
        self.cumulative_drift = np.zeros(3)
        self.frame_count = 0
        print("[SLAM+] Reset complete")