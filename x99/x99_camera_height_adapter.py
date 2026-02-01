#!/usr/bin/env python3
"""
Camera Height Adaptive SLAM Module
Tự động điều chỉnh SLAM parameters dựa trên chiều cao camera

Usage:
    adapter = CameraHeightAdapter(camera_height=0.45)
    points_filtered = adapter.filter_point_cloud(points_3d)
    grid = adapter.update_navigation_map(points_3d, grid)
"""

import numpy as np
import cv2
from typing import Tuple, List, Optional
from dataclasses import dataclass

@dataclass
class CameraConfig:
    """Camera configuration"""
    height: float  # meters from ground
    baseline: float
    focal_length: float
    tilt_angle: float = 15.0  # degrees, positive = down
    fov_vertical: float = 60.0  # degrees

class GroundPlaneEstimator:
    """
    Tự động estimate camera height từ point cloud
    Dùng RANSAC để fit ground plane
    """
    
    def __init__(self, min_samples: int = 100):
        self.min_samples = min_samples
        self.estimated_height = None
        self.height_history = []
        self.confidence = 0.0
        
    def estimate_height(self, points_3d: np.ndarray, 
                       max_iterations: int = 200) -> Optional[float]:
        """
        Estimate camera height using RANSAC ground plane fitting
        
        Args:
            points_3d: Nx3 array of [x, y, z] in camera frame
            max_iterations: RANSAC iterations
            
        Returns:
            Estimated camera height (meters) or None
        """
        if len(points_3d) < self.min_samples:
            return None
        
        # Filter points that could be ground (negative Y, not too far)
        candidate_mask = (points_3d[:, 1] < 0) & (points_3d[:, 2] < 5.0)
        candidates = points_3d[candidate_mask]
        
        if len(candidates) < self.min_samples:
            return None
        
        best_inliers = 0
        best_height = None
        best_inlier_indices = None
        
        for iteration in range(max_iterations):
            # Random sample 3 points
            sample_idx = np.random.choice(len(candidates), 3, replace=False)
            sample_pts = candidates[sample_idx]
            
            # Fit plane through these points
            # Plane equation: ax + by + cz + d = 0
            # For ground, we expect mostly horizontal: b ≈ 1, a,c ≈ 0
            
            # Normal vector from cross product
            v1 = sample_pts[1] - sample_pts[0]
            v2 = sample_pts[2] - sample_pts[0]
            normal = np.cross(v1, v2)
            
            # Normalize
            normal = normal / (np.linalg.norm(normal) + 1e-8)
            
            # Check if plane is roughly horizontal (Y component dominant)
            if abs(normal[1]) < 0.7:  # Normal should point up/down
                continue
            
            # Make sure normal points up (positive Y)
            if normal[1] < 0:
                normal = -normal
            
            # Distance from origin to plane
            d = -np.dot(normal, sample_pts[0])
            
            # For horizontal plane at height h: y = -h
            # So height = -d / normal[1]
            height = -d / normal[1]
            
            # Sanity check
            if height < 0.05 or height > 2.0:  # 5cm to 2m
                continue
            
            # Count inliers (points close to this plane)
            distances = np.abs(
                np.dot(candidates, normal) + d
            )
            
            inlier_mask = distances < 0.03  # 3cm threshold
            num_inliers = np.sum(inlier_mask)
            
            if num_inliers > best_inliers:
                best_inliers = num_inliers
                best_height = height
                best_inlier_indices = inlier_mask
        
        # Need at least 30% inliers
        if best_inliers > len(candidates) * 0.3:
            # Update history
            self.height_history.append(best_height)
            
            # Keep last 20 estimates
            if len(self.height_history) > 20:
                self.height_history = self.height_history[-20:]
            
            # Use median for robustness
            self.estimated_height = np.median(self.height_history)
            
            # Confidence based on consistency
            if len(self.height_history) >= 5:
                std = np.std(self.height_history[-5:])
                self.confidence = np.clip(1.0 - std * 10, 0.0, 1.0)
            
            return self.estimated_height
        
        return None
    
    def get_height(self) -> Tuple[Optional[float], float]:
        """
        Get estimated height and confidence
        Returns: (height, confidence)
        """
        return self.estimated_height, self.confidence

class AdaptiveGroundClassifier:
    """
    Classify points as ground/obstacle/ceiling
    Automatically adjusts thresholds based on camera height
    """
    
    def __init__(self, camera_height: float):
        self.camera_height = camera_height
        self._update_thresholds()
    
    def _update_thresholds(self):
        """Update classification thresholds"""
        
        # Ground: từ một chút dưới ground đến 20% camera height
        self.ground_y_min = -self.camera_height - 0.08
        self.ground_y_max = -self.camera_height * 0.15
        
        # Obstacle: từ ground_max đến 2.5x camera height
        self.obstacle_y_min = self.ground_y_max
        self.obstacle_y_max = self.camera_height * 2.5
        
        # Low obstacle (cần chú ý đặc biệt): từ ground đến 30cm
        self.low_obstacle_y_max = -self.camera_height + 0.30
        
        print(f"[Classifier] Camera height: {self.camera_height:.2f}m")
        print(f"  Ground Y range: [{self.ground_y_min:.2f}, {self.ground_y_max:.2f}]")
        print(f"  Obstacle Y range: [{self.obstacle_y_min:.2f}, {self.obstacle_y_max:.2f}]")
    
    def update_camera_height(self, new_height: float):
        """Update camera height (e.g., from auto-calibration)"""
        self.camera_height = new_height
        self._update_thresholds()
    
    def classify_point(self, y: float) -> str:
        """
        Classify single point
        
        Returns: 'ground', 'low_obstacle', 'obstacle', or 'ceiling'
        """
        if self.ground_y_min <= y <= self.ground_y_max:
            return 'ground'
        elif self.obstacle_y_min < y <= self.low_obstacle_y_max:
            return 'low_obstacle'  # Dangerous!
        elif self.low_obstacle_y_max < y <= self.obstacle_y_max:
            return 'obstacle'
        else:
            return 'ceiling'  # Too high, ignore
    
    def classify_point_cloud(self, points_3d: np.ndarray) -> np.ndarray:
        """
        Classify entire point cloud
        
        Returns: Array of classifications as integers
            0 = ground
            1 = low_obstacle
            2 = obstacle
            3 = ceiling
        """
        y = points_3d[:, 1]
        
        classifications = np.zeros(len(points_3d), dtype=np.int8)
        
        # Ground
        ground_mask = (y >= self.ground_y_min) & (y <= self.ground_y_max)
        classifications[ground_mask] = 0
        
        # Low obstacle
        low_obs_mask = (y > self.obstacle_y_min) & (y <= self.low_obstacle_y_max)
        classifications[low_obs_mask] = 1
        
        # Regular obstacle
        obs_mask = (y > self.low_obstacle_y_max) & (y <= self.obstacle_y_max)
        classifications[obs_mask] = 2
        
        # Ceiling (everything else)
        classifications[(y < self.ground_y_min) | (y > self.obstacle_y_max)] = 3
        
        return classifications

class HeightAwareDepthFilter:
    """
    Filter depth measurements based on camera height
    Reject physically impossible or unreliable measurements
    """
    
    def __init__(self, camera_height: float, baseline: float):
        self.camera_height = camera_height
        self.baseline = baseline
        
        # Determine depth limits based on camera height
        if camera_height < 0.25:
            self.min_depth = 0.2
            self.max_depth = 2.5
        elif camera_height < 0.40:
            self.min_depth = 0.15
            self.max_depth = 4.0
        elif camera_height < 0.60:
            self.min_depth = 0.15
            self.max_depth = 6.0
        else:
            self.min_depth = 0.15
            self.max_depth = 10.0
        
        print(f"[DepthFilter] Camera height: {camera_height:.2f}m")
        print(f"  Valid depth range: [{self.min_depth}, {self.max_depth}]m")
    
    def is_valid_point(self, x: float, y: float, z: float) -> bool:
        """
        Check if 3D point is valid
        
        Args:
            x, y, z: Point coordinates in camera frame
        """
        # Depth range check
        if z < self.min_depth or z > self.max_depth:
            return False
        
        # Physical constraint: point can't be below ground
        actual_height = y + self.camera_height
        
        if actual_height < -0.10:  # 10cm below ground = invalid
            return False
        
        # For low cameras, reject far ground points (unreliable)
        if self.camera_height < 0.30:
            if z > 2.0 and abs(actual_height) < 0.05:
                return False
        
        # Reject points too far laterally
        if abs(x) > z * 1.5:  # More than 56° from center
            return False
        
        return True
    
    def filter_point_cloud(self, points_3d: np.ndarray) -> np.ndarray:
        """
        Filter point cloud, return only valid points
        """
        if len(points_3d) == 0:
            return points_3d
        
        valid_mask = np.array([
            self.is_valid_point(pt[0], pt[1], pt[2])
            for pt in points_3d
        ])
        
        return points_3d[valid_mask]
    
    def get_depth_confidence(self, z: float, y: float) -> float:
        """
        Get confidence score for depth measurement (0-1)
        
        Lower confidence for:
        - Points near max range
        - Ground points far from camera (if camera low)
        """
        # Distance confidence (decreases with distance)
        dist_conf = np.clip(1.0 - (z - self.min_depth) / self.max_depth, 0.0, 1.0)
        
        # Height confidence
        actual_height = y + self.camera_height
        
        if self.camera_height < 0.30 and abs(actual_height) < 0.05:
            # Low camera + ground point
            # Confidence decreases with distance
            height_conf = np.clip(1.0 - z / 2.0, 0.3, 1.0)
        else:
            height_conf = 1.0
        
        return dist_conf * height_conf

class CameraHeightAdapter:
    """
    Main adapter class - combines all height-aware components
    """
    
    def __init__(self, camera_height: float = None,
                 baseline: float = 0.10,
                 focal_length: float = 500.0,
                 auto_calibrate: bool = True):
        """
        Args:
            camera_height: Camera height in meters (if known)
            baseline: Stereo baseline
            focal_length: Focal length in pixels
            auto_calibrate: Auto-estimate height if True
        """
        
        self.baseline = baseline
        self.focal_length = focal_length
        self.auto_calibrate = auto_calibrate
        
        # Ground plane estimator
        self.height_estimator = GroundPlaneEstimator()
        
        # Initialize with provided or default height
        if camera_height is None:
            self.camera_height = 0.40  # Default assumption
            print("[Adapter] No camera height provided, using default: 0.40m")
            print("[Adapter] Auto-calibration enabled" if auto_calibrate else "")
        else:
            self.camera_height = camera_height
            print(f"[Adapter] Camera height: {camera_height:.2f}m")
        
        # Initialize components
        self.classifier = AdaptiveGroundClassifier(self.camera_height)
        self.depth_filter = HeightAwareDepthFilter(self.camera_height, baseline)
        
        self.calibration_frames = 0
        self.is_calibrated = (camera_height is not None)
    
    def process_point_cloud(self, points_3d: np.ndarray,
                           update_height: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Process point cloud with height-aware filtering and classification
        
        Args:
            points_3d: Nx3 array of [x, y, z] in camera frame
            update_height: Whether to update height estimate
            
        Returns:
            (filtered_points, classifications)
        """
        if len(points_3d) == 0:
            return points_3d, np.array([])
        
        # Auto-calibrate height (first N frames)
        if self.auto_calibrate and not self.is_calibrated:
            estimated_height = self.height_estimator.estimate_height(points_3d)
            
            if estimated_height is not None:
                self.calibration_frames += 1
                
                # After 10 successful estimates, switch to estimated height
                if self.calibration_frames >= 10:
                    height, confidence = self.height_estimator.get_height()
                    
                    if confidence > 0.7:
                        print(f"[Adapter] ✓ Auto-calibrated height: {height:.3f}m (confidence: {confidence:.2f})")
                        self.camera_height = height
                        
                        # Update all components
                        self.classifier.update_camera_height(height)
                        self.depth_filter = HeightAwareDepthFilter(height, self.baseline)
                        
                        self.is_calibrated = True
        
        # Continuous height update (optional)
        elif update_height and self.is_calibrated:
            # Still update occasionally for height changes
            if np.random.rand() < 0.1:  # 10% of frames
                estimated_height = self.height_estimator.estimate_height(points_3d)
                
                if estimated_height is not None:
                    height, confidence = self.height_estimator.get_height()
                    
                    # Only update if confident and change is small
                    if confidence > 0.8 and abs(height - self.camera_height) < 0.05:
                        self.camera_height = height
                        self.classifier.update_camera_height(height)
        
        # Filter invalid points
        filtered_points = self.depth_filter.filter_point_cloud(points_3d)
        
        if len(filtered_points) == 0:
            return filtered_points, np.array([])
        
        # Classify points
        classifications = self.classifier.classify_point_cloud(filtered_points)
        
        return filtered_points, classifications
    
    def update_navigation_map(self, points_3d: np.ndarray, 
                             grid: np.ndarray,
                             world_to_grid_func) -> np.ndarray:
        """
        Update 2D occupancy grid with height-aware classification
        
        Args:
            points_3d: Nx3 points in world frame
            grid: 2D occupancy grid
            world_to_grid_func: Function to convert (x, z) to grid coords
            
        Returns:
            Updated grid
        """
        if len(points_3d) == 0:
            return grid
        
        for point in points_3d:
            x, y, z = point
            
            # Classify in camera frame (need to adjust Y)
            # Assuming points are in world frame where Y=0 is ground
            y_camera = y - self.camera_height
            
            classification = self.classifier.classify_point(y_camera)
            
            grid_x, grid_y = world_to_grid_func(x, z)
            
            # Update grid
            if classification == 'ground':
                grid[grid_y, grid_x] = 0  # Free
            elif classification in ['low_obstacle', 'obstacle']:
                grid[grid_y, grid_x] = 100  # Occupied
            # Ignore ceiling
        
        return grid
    
    def get_status(self) -> dict:
        """Get adapter status"""
        height, confidence = self.height_estimator.get_height()
        
        return {
            'camera_height': self.camera_height,
            'is_calibrated': self.is_calibrated,
            'calibration_frames': self.calibration_frames,
            'estimated_height': height,
            'confidence': confidence,
            'ground_range': (self.classifier.ground_y_min, self.classifier.ground_y_max),
            'depth_range': (self.depth_filter.min_depth, self.depth_filter.max_depth)
        }
    
    def visualize_classification(self, points_3d: np.ndarray, 
                                 classifications: np.ndarray) -> np.ndarray:
        """
        Create colored point cloud visualization
        
        Returns: Nx3 RGB colors
        """
        colors = np.zeros((len(points_3d), 3), dtype=np.uint8)
        
        # Ground = green
        colors[classifications == 0] = [0, 255, 0]
        
        # Low obstacle = red
        colors[classifications == 1] = [255, 0, 0]
        
        # Obstacle = orange
        colors[classifications == 2] = [255, 128, 0]
        
        # Ceiling = gray
        colors[classifications == 3] = [128, 128, 128]
        
        return colors

# ===== USAGE EXAMPLE =====
def test_camera_height_adapter():
    """Test the adapter with simulated data"""
    
    # Create adapter
    adapter = CameraHeightAdapter(
        camera_height=None,  # Will auto-calibrate
        baseline=0.10,
        focal_length=500,
        auto_calibrate=True
    )
    
    # Simulate point clouds
    for frame in range(30):
        # Generate random points
        n_points = 500
        
        # Ground points (around Y = -0.45)
        ground_pts = np.random.randn(n_points // 2, 3)
        ground_pts[:, 1] = np.random.uniform(-0.50, -0.40, n_points // 2)
        ground_pts[:, 2] = np.random.uniform(0.5, 3.0, n_points // 2)
        
        # Obstacle points
        obs_pts = np.random.randn(n_points // 2, 3)
        obs_pts[:, 1] = np.random.uniform(-0.20, 0.50, n_points // 2)
        obs_pts[:, 2] = np.random.uniform(0.5, 3.0, n_points // 2)
        
        points = np.vstack([ground_pts, obs_pts])
        
        # Process
        filtered, classifications = adapter.process_point_cloud(points)
        
        # Print stats
        if frame % 5 == 0:
            status = adapter.get_status()
            print(f"\nFrame {frame}:")
            print(f"  Calibrated: {status['is_calibrated']}")
            print(f"  Height: {status['camera_height']:.3f}m")
            if status['estimated_height']:
                print(f"  Estimated: {status['estimated_height']:.3f}m (conf: {status['confidence']:.2f})")
            
            # Classification breakdown
            n_ground = np.sum(classifications == 0)
            n_low_obs = np.sum(classifications == 1)
            n_obs = np.sum(classifications == 2)
            print(f"  Ground: {n_ground}, Low Obs: {n_low_obs}, Obs: {n_obs}")

if __name__ == "__main__":
    test_camera_height_adapter()