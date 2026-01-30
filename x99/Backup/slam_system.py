#!/usr/bin/env python3
"""
ORB-SLAM3 + YOLOv11m-seg System
Supports: X99 Dual Xeon + Radeon MI50, Jetson Nano + Dual USB Cameras
"""

import cv2
import numpy as np
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json

try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: ultralytics not installed. Install with: pip install ultralytics")

@dataclass
class CameraConfig:
    """Camera configuration for stereo setup"""
    left_id: int = 0
    right_id: int = 1
    width: int = 640
    height: int = 480
    fps: int = 30
    baseline: float = 0.10  # 10cm baseline in meters
    focal_length: float = 500  # pixels

class StereoCamera:
    """Stereo camera handler for OV9832 USB cameras"""
    
    def __init__(self, config: CameraConfig):
        self.config = config
        self.left_cap = None
        self.right_cap = None
        self.is_running = False
        
    def initialize(self) -> bool:
        """Initialize stereo cameras"""
        try:
            self.left_cap = cv2.VideoCapture(self.config.left_id)
            self.right_cap = cv2.VideoCapture(self.config.right_id)
            
            # Set camera properties
            for cap in [self.left_cap, self.right_cap]:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
                cap.set(cv2.CAP_PROP_FPS, self.config.fps)
            
            self.is_running = True
            return True
        except Exception as e:
            print(f"Camera initialization error: {e}")
            return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Capture stereo frame pair"""
        if not self.is_running:
            return None, None
            
        ret_left, frame_left = self.left_cap.read()
        ret_right, frame_right = self.right_cap.read()
        
        if ret_left and ret_right:
            return frame_left, frame_right
        return None, None
    
    def release(self):
        """Release camera resources"""
        self.is_running = False
        if self.left_cap:
            self.left_cap.release()
        if self.right_cap:
            self.right_cap.release()

class ORBFeatureExtractor:
    """ORB feature extraction and matching"""
    
    def __init__(self, n_features: int = 2000):
        # Try different parameter names for different OpenCV versions
        try:
            self.orb = cv2.ORB_create(nfeatures=n_features)
        except TypeError:
            try:
                self.orb = cv2.ORB_create(nFeatures=n_features)
            except TypeError:
                self.orb = cv2.ORB_create()
                self.orb.setMaxFeatures(n_features)
        
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
    def extract_features(self, image: np.ndarray) -> Tuple[List, np.ndarray]:
        """Extract ORB keypoints and descriptors"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return keypoints, descriptors
    
    def match_features(self, desc1: np.ndarray, desc2: np.ndarray) -> List:
        """Match features between two frames"""
        if desc1 is None or desc2 is None:
            return []
        matches = self.bf_matcher.match(desc1, desc2)
        matches = sorted(matches, key=lambda x: x.distance)
        return matches[:100]  # Top 100 matches

class YOLOSegmentator:
    """YOLOv11m-seg for semantic segmentation"""
    
    def __init__(self, model_path: str = "yolov11m-seg.pt", device: str = "cuda"):
        """
        Initialize YOLO segmentation model
        device: 'cuda', 'cpu', or 'auto'
        """
        try:
            self.model = YOLO(model_path)
            self.device = device
            self.class_names = self.model.names
        except Exception as e:
            print(f"YOLO initialization error: {e}")
            self.model = None
    
    def segment(self, image: np.ndarray, conf: float = 0.5):
        """Run segmentation on image"""
        if self.model is None:
            return None
            
        results = self.model(image, conf=conf, device=self.device, verbose=False)
        return results[0] if results else None
    
    def draw_segments(self, image: np.ndarray, results) -> np.ndarray:
        """Draw segmentation masks on image"""
        if results is None or results.masks is None:
            return image
            
        annotated = results.plot()
        return annotated

class MapPoint:
    """3D map point representation"""
    
    def __init__(self, position: np.ndarray, descriptor: np.ndarray, 
                 color: np.ndarray = None):
        self.position = position  # [x, y, z]
        self.descriptor = descriptor
        self.color = color if color is not None else np.array([255, 255, 255])
        self.observations = 0
        self.is_valid = True

class Map3D:
    """3D map management"""
    
    def __init__(self, max_points: int = 10000):
        self.points: List[MapPoint] = []
        self.max_points = max_points
        self.camera_poses = deque(maxlen=100)
        self.lock = threading.Lock()
        
    def add_point(self, point: MapPoint):
        """Add new map point"""
        with self.lock:
            if len(self.points) >= self.max_points:
                # Remove oldest/worst points
                self.points = sorted(self.points, 
                                    key=lambda p: p.observations, 
                                    reverse=True)[:self.max_points-1]
            self.points.append(point)
    
    def add_camera_pose(self, pose: np.ndarray):
        """Add camera pose (4x4 transformation matrix)"""
        with self.lock:
            self.camera_poses.append(pose.copy())
    
    def get_map_data(self) -> dict:
        """Get map data for visualization"""
        with self.lock:
            points_data = []
            for p in self.points:
                if p.is_valid:
                    points_data.append({
                        'position': p.position.tolist(),
                        'color': p.color.tolist()
                    })
            
            poses_data = [pose.tolist() for pose in self.camera_poses]
            
            return {
                'points': points_data,
                'camera_poses': poses_data,
                'num_points': len(points_data)
            }

class SLAMSystem:
    """Main SLAM system integrating all components"""
    
    def __init__(self, camera_config: CameraConfig, use_yolo: bool = True):
        self.camera = StereoCamera(camera_config)
        self.orb_extractor = ORBFeatureExtractor()
        self.map = Map3D()
        
        self.yolo = None
        if use_yolo:
            # Auto-detect device
            device = 'cuda' if cv2.cuda.getCudaEnabledDeviceCount() > 0 else 'cpu'
            self.yolo = YOLOSegmentator(device=device)
        
        self.is_running = False
        self.current_pose = np.eye(4)  # 4x4 identity matrix
        self.prev_keypoints = None
        self.prev_descriptors = None
        
    def triangulate_points(self, kp1: List, kp2: List, matches: List) -> List[MapPoint]:
        """Triangulate 3D points from stereo matches"""
        points_3d = []
        
        baseline = self.camera.config.baseline
        focal_length = self.camera.config.focal_length
        
        for match in matches:
            pt1 = kp1[match.queryIdx].pt
            pt2 = kp2[match.trainIdx].pt
            
            disparity = abs(pt1[0] - pt2[0])
            if disparity > 1:  # Minimum disparity threshold
                # Calculate depth
                depth = (baseline * focal_length) / disparity
                
                # Calculate 3D position
                x = (pt1[0] - self.camera.config.width / 2) * depth / focal_length
                y = (pt1[1] - self.camera.config.height / 2) * depth / focal_length
                z = depth
                
                position = np.array([x, y, z])
                
                # Create map point
                map_point = MapPoint(
                    position=position,
                    descriptor=self.prev_descriptors[match.queryIdx],
                    color=np.random.randint(0, 255, 3)
                )
                points_3d.append(map_point)
        
        return points_3d
    
    def process_frame(self, frame_left: np.ndarray, frame_right: np.ndarray):
        """Process stereo frame pair"""
        # Extract ORB features from left frame
        keypoints_left, descriptors_left = self.orb_extractor.extract_features(frame_left)
        keypoints_right, descriptors_right = self.orb_extractor.extract_features(frame_right)
        
        # Match stereo features
        stereo_matches = self.orb_extractor.match_features(descriptors_left, descriptors_right)
        
        # Triangulate new 3D points
        if len(stereo_matches) > 10:
            new_points = self.triangulate_points(keypoints_left, keypoints_right, stereo_matches)
            for point in new_points:
                self.map.add_point(point)
        
        # Track camera pose (simplified)
        if self.prev_keypoints is not None and self.prev_descriptors is not None:
            temporal_matches = self.orb_extractor.match_features(
                self.prev_descriptors, descriptors_left
            )
            # In real implementation, use essential matrix and PnP for pose estimation
        
        self.prev_keypoints = keypoints_left
        self.prev_descriptors = descriptors_left
        
        # Update camera pose
        self.map.add_camera_pose(self.current_pose)
        
        # YOLO segmentation (optional)
        segmented_frame = frame_left.copy()
        if self.yolo is not None:
            yolo_results = self.yolo.segment(frame_left)
            segmented_frame = self.yolo.draw_segments(frame_left, yolo_results)
        
        # Draw features
        frame_with_features = cv2.drawKeypoints(
            segmented_frame, keypoints_left, None, 
            color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        
        return frame_with_features
    
    def run(self):
        """Main SLAM loop"""
        if not self.camera.initialize():
            print("Failed to initialize cameras")
            return
        
        self.is_running = True
        print("SLAM system started")
        
        while self.is_running:
            frame_left, frame_right = self.camera.get_frames()
            
            if frame_left is not None and frame_right is not None:
                processed_frame = self.process_frame(frame_left, frame_right)
                
                # Display
                cv2.imshow('SLAM System', processed_frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.stop()
    
    def stop(self):
        """Stop SLAM system"""
        self.is_running = False
        self.camera.release()
        cv2.destroyAllWindows()
        print("SLAM system stopped")

def main():
    """Main entry point"""
    # Configure camera
    config = CameraConfig(
        left_id=0,
        right_id=1,
        width=640,
        height=480,
        fps=30,
        baseline=0.10,  # 10cm
        focal_length=500
    )
    
    # Initialize SLAM system
    slam = SLAMSystem(camera_config=config, use_yolo=True)
    
    try:
        slam.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
        slam.stop()

if __name__ == "__main__":
    main()
