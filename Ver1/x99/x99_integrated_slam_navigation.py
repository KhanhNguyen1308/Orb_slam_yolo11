#!/usr/bin/env python3
"""
Integrated SLAM + Navigation System
Kết hợp:
- SLAM với drift correction
- 2D Navigation map
- Autonomous navigation
- Web visualization
"""

import cv2
import numpy as np
import time
from typing import Tuple, List, Optional, Dict
import json
import threading

from x99_slam_improved_drift_correction import DriftCorrectedSLAM
from x99_navigation_map_2d import NavigationMap2D
from persistent_map import PersistentMap

class IntegratedSLAMNavigation:
    """
    Hệ thống SLAM + Navigation hoàn chỉnh
    """
    
    def __init__(self, 
                 baseline: float = 0.10,
                 focal_length: float = 500,
                 map_size: float = 20.0,
                 map_resolution: float = 0.05):
        
        # SLAM System
        self.slam = DriftCorrectedSLAM(
            n_features=2000,
            baseline=baseline,
            focal_length=focal_length
        )
        
        # Navigation Map (2D)
        self.nav_map = NavigationMap2D(
            width=map_size,
            height=map_size,
            resolution=map_resolution
        )
        
        # Persistent 3D Map
        self.persistent_map = PersistentMap(
            grid_size=800,
            resolution=0.02,
            voxel_size=0.05
        )
        
        # Navigation state
        self.current_path: Optional[List[Tuple[float, float]]] = None
        self.current_goal: Optional[Tuple[float, float]] = None
        self.path_index = 0
        
        # Statistics
        self.frame_count = 0
        self.last_map_update = time.time()
        self.map_update_interval = 0.5  # Update map every 0.5s
        
        # Thread safety
        self.lock = threading.Lock()
        
        print("[System] Integrated SLAM + Navigation initialized")
    
    def process_stereo_frame(self, img_left: np.ndarray, img_right: np.ndarray) -> Dict:
        """
        Process stereo frame - main update loop
        Returns: status dictionary
        """
        self.frame_count += 1
        
        # ===== 1. SLAM TRACKING =====
        current_pose, tracking_quality = self.slam.process_frame(img_left, img_right)
        
        # Extract 2D position
        pos_x = current_pose[0, 3]
        pos_z = current_pose[2, 3]
        
        # ===== 2. BUILD 3D POINT CLOUD =====
        # Get 3D points from SLAM
        points_3d_camera = self.slam.prev_points_3d
        
        if len(points_3d_camera) > 0:
            # Transform to world frame
            points_3d_world = []
            for pt in points_3d_camera:
                pt_h = np.append(pt, 1.0)
                pt_world = current_pose @ pt_h
                points_3d_world.append(pt_world[:3])
            
            points_3d_world = np.array(points_3d_world)
            
            # ===== 3. UPDATE NAVIGATION MAP =====
            # Only update periodically to save computation
            if time.time() - self.last_map_update > self.map_update_interval:
                with self.lock:
                    self.nav_map.update_from_point_cloud(
                        points_3d_world,
                        robot_pose_2d=(pos_x, pos_z)
                    )
                    
                    # Raycast free space (like LIDAR)
                    self.nav_map.raycast_free_space()
                
                self.last_map_update = time.time()
            
            # ===== 4. UPDATE PERSISTENT MAP =====
            # Extract colors from left image
            colors = self._extract_colors_for_points(img_left, points_3d_camera)
            
            robot_pose_3d = np.array([pos_x, current_pose[1, 3], pos_z])
            
            self.persistent_map.add_point_cloud(
                points_3d_world,
                colors,
                robot_pose_3d
            )
        
        # ===== 5. PATH FOLLOWING (if path exists) =====
        cmd_linear = 0.0
        cmd_angular = 0.0
        
        if self.current_path is not None and len(self.current_path) > 0:
            cmd_linear, cmd_angular = self._follow_path(pos_x, pos_z)
        
        # ===== 6. PREPARE STATUS =====
        status = {
            'frame': self.frame_count,
            'tracking': tracking_quality,
            'position': {
                'x': float(pos_x),
                'y': float(current_pose[1, 3]),
                'z': float(pos_z)
            },
            'slam_stats': {
                'keyframes': len(self.slam.keyframes),
                'loop_closures': len(self.slam.loop_closures),
                'drift_corrections': self.slam.total_drift_corrections,
                'inliers': self.slam.num_inliers
            },
            'navigation': {
                'has_path': self.current_path is not None,
                'path_progress': f"{self.path_index}/{len(self.current_path) if self.current_path else 0}",
                'cmd_linear': cmd_linear,
                'cmd_angular': cmd_angular
            }
        }
        
        return status
    
    def _extract_colors_for_points(self, image: np.ndarray, points_3d_camera: List[np.ndarray]) -> np.ndarray:
        """Extract RGB colors for 3D points from image"""
        if len(points_3d_camera) == 0:
            return np.array([])
        
        colors = []
        h, w = image.shape[:2]
        K = self.slam.K
        
        for pt in points_3d_camera:
            # Project to image
            if pt[2] > 0:
                u = int(K[0, 0] * pt[0] / pt[2] + K[0, 2])
                v = int(K[1, 1] * pt[1] / pt[2] + K[1, 2])
                
                if 0 <= u < w and 0 <= v < h:
                    color = image[v, u]
                    colors.append(color[::-1])  # BGR to RGB
                else:
                    colors.append([128, 128, 128])
            else:
                colors.append([128, 128, 128])
        
        return np.array(colors)
    
    def set_navigation_goal(self, goal_x: float, goal_z: float) -> bool:
        """
        Set navigation goal and plan path
        """
        print(f"\n[Navigation] Planning path to ({goal_x:.2f}, {goal_z:.2f})")
        
        with self.lock:
            # Plan path using A*
            path = self.nav_map.find_path_astar(goal_x, goal_z)
            
            if path is None:
                print("[Navigation] Path planning FAILED")
                return False
            
            # Smooth path (optional)
            path = self._smooth_path(path)
            
            self.current_path = path
            self.current_goal = (goal_x, goal_z)
            self.path_index = 0
            
            print(f"[Navigation] Path found: {len(path)} waypoints")
            
        return True
    
    def _smooth_path(self, path: List[Tuple[float, float]], 
                     window_size: int = 5) -> List[Tuple[float, float]]:
        """Smooth path using moving average"""
        if len(path) <= window_size:
            return path
        
        smoothed = []
        for i in range(len(path)):
            start = max(0, i - window_size // 2)
            end = min(len(path), i + window_size // 2 + 1)
            
            avg_x = np.mean([p[0] for p in path[start:end]])
            avg_z = np.mean([p[1] for p in path[start:end]])
            
            smoothed.append((avg_x, avg_z))
        
        return smoothed
    
    def _follow_path(self, robot_x: float, robot_z: float) -> Tuple[float, float]:
        """
        Pure pursuit path following
        Returns: (linear_velocity, angular_velocity)
        """
        if self.current_path is None or len(self.current_path) == 0:
            return 0.0, 0.0
        
        # Find closest point on path ahead
        lookahead_distance = 0.5  # meters
        
        # Current waypoint
        if self.path_index >= len(self.current_path):
            print("[Navigation] Goal reached!")
            self.current_path = None
            return 0.0, 0.0
        
        target_x, target_z = self.current_path[self.path_index]
        
        # Distance to current waypoint
        dx = target_x - robot_x
        dz = target_z - robot_z
        distance = np.sqrt(dx**2 + dz**2)
        
        # If close enough, move to next waypoint
        if distance < 0.2:  # 20cm threshold
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                return 0.0, 0.0
            target_x, target_z = self.current_path[self.path_index]
            dx = target_x - robot_x
            dz = target_z - robot_z
            distance = np.sqrt(dx**2 + dz**2)
        
        # Compute heading error
        target_heading = np.arctan2(dx, dz)  # Note: atan2(x, z) because z is forward
        
        # Assume robot is facing +Z (forward)
        # In a real system, you'd extract heading from SLAM pose
        robot_heading = 0.0  # Simplification
        
        heading_error = target_heading - robot_heading
        
        # Normalize to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Pure pursuit control
        max_linear_vel = 0.3  # m/s
        max_angular_vel = 1.0  # rad/s
        
        # Linear velocity (proportional to distance, but capped)
        linear_vel = min(max_linear_vel, 0.5 * distance)
        
        # Angular velocity (proportional to heading error)
        angular_vel = np.clip(2.0 * heading_error, -max_angular_vel, max_angular_vel)
        
        # If large heading error, rotate in place
        if abs(heading_error) > np.pi / 4:  # 45 degrees
            linear_vel *= 0.3
        
        return linear_vel, angular_vel
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        with self.lock:
            self.current_path = None
            self.current_goal = None
            self.path_index = 0
        print("[Navigation] Cancelled")
    
    def explore_nearest_frontier(self) -> bool:
        """
        Autonomous exploration - find and navigate to nearest unknown area
        """
        with self.lock:
            frontiers = self.nav_map.get_free_space_frontiers()
            
            if len(frontiers) == 0:
                print("[Exploration] No frontiers found - map complete!")
                return False
            
            # Get current position
            pos = self.slam.current_pose[:3, 3]
            robot_x, robot_z = pos[0], pos[2]
            
            # Find nearest frontier
            min_dist = float('inf')
            nearest = None
            
            for fx, fz in frontiers:
                dist = np.sqrt((fx - robot_x)**2 + (fz - robot_z)**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest = (fx, fz)
            
            if nearest is None:
                return False
            
            print(f"[Exploration] Navigating to frontier at ({nearest[0]:.2f}, {nearest[1]:.2f})")
            return self.set_navigation_goal(nearest[0], nearest[1])
    
    def visualize_all(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Create all visualizations
        Returns: (nav_map_vis, persistent_map_vis, path_vis)
        """
        with self.lock:
            # Navigation map
            nav_vis = self.nav_map.visualize(
                show_costmap=False,
                path=self.current_path
            )
            
            # Persistent map (2D top-down)
            persistent_vis = self.persistent_map.visualize_2d(show_trajectory=True)
            
            # Path visualization (separate costmap view)
            path_vis = self.nav_map.visualize(
                show_costmap=True,
                path=self.current_path
            )
        
        return nav_vis, persistent_vis, path_vis
    
    def get_web_data(self) -> Dict:
        """
        Get data for web visualization
        """
        with self.lock:
            # Get persistent map data
            map_data = self.persistent_map.get_map_data_for_web()
            
            # Add navigation data
            map_data['navigation'] = {
                'grid': self.nav_map.grid.tolist(),
                'robot_pos': [self.nav_map.robot_x, self.nav_map.robot_y],
                'resolution': self.nav_map.resolution,
                'path': self.current_path if self.current_path else [],
                'goal': self.current_goal if self.current_goal else None
            }
            
            # SLAM stats
            map_data['slam'] = {
                'keyframes': len(self.slam.keyframes),
                'loop_closures': len(self.slam.loop_closures),
                'tracking': self.slam.tracking_quality,
                'drift_corrections': self.slam.total_drift_corrections
            }
        
        return map_data
    
    def save_session(self, filename_prefix: str = "session"):
        """Save complete session"""
        timestamp = int(time.time())
        
        # Save navigation map
        nav_file = f"{filename_prefix}_navmap_{timestamp}.npz"
        self.nav_map.save_map(nav_file)
        
        # Save persistent map
        persist_file = f"{filename_prefix}_3dmap_{timestamp}.npz"
        self.persistent_map.save_map(persist_file)
        
        # Save SLAM trajectory
        trajectory = self.slam.get_trajectory()
        traj_file = f"{filename_prefix}_trajectory_{timestamp}.json"
        
        with open(traj_file, 'w') as f:
            json.dump({
                'trajectory': trajectory,
                'keyframes': len(self.slam.keyframes),
                'loop_closures': len(self.slam.loop_closures)
            }, f, indent=2)
        
        print(f"[System] Session saved:")
        print(f"  Nav map: {nav_file}")
        print(f"  3D map: {persist_file}")
        print(f"  Trajectory: {traj_file}")
    
    def reset(self):
        """Reset entire system"""
        with self.lock:
            self.slam.reset()
            self.nav_map = NavigationMap2D(
                width=self.nav_map.width_m,
                height=self.nav_map.height_m,
                resolution=self.nav_map.resolution
            )
            self.persistent_map.clear_map()
            self.current_path = None
            self.current_goal = None
            self.path_index = 0
            self.frame_count = 0
        
        print("[System] Reset complete")


# ===== DEMO / TEST =====
def test_integrated_system():
    """Test integrated system with simulated data"""
    
    system = IntegratedSLAMNavigation(
        baseline=0.10,
        focal_length=500,
        map_size=10.0,
        map_resolution=0.05
    )
    
    print("\n[Test] Simulating robot movement and mapping...")
    
    # Simulate movement in a square
    trajectory_2d = [
        (0, 0), (1, 0), (2, 0), (2, 1), (2, 2),
        (1, 2), (0, 2), (0, 1), (0, 0)
    ]
    
    for i, (x, z) in enumerate(trajectory_2d):
        # Create dummy stereo images
        img_left = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        img_right = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Process
        status = system.process_stereo_frame(img_left, img_right)
        
        print(f"\nFrame {status['frame']}:")
        print(f"  Position: ({status['position']['x']:.2f}, {status['position']['z']:.2f})")
        print(f"  Tracking: {status['tracking']}")
        print(f"  Keyframes: {status['slam_stats']['keyframes']}")
        
        # Visualize every 3 frames
        if i % 3 == 0:
            nav_vis, persist_vis, path_vis = system.visualize_all()
            
            # Resize for display
            nav_vis = cv2.resize(nav_vis, (400, 400))
            persist_vis = cv2.resize(persist_vis, (400, 400))
            
            cv2.imshow('Navigation Map', nav_vis)
            cv2.imshow('Persistent Map', persist_vis)
            cv2.waitKey(100)
    
    # Test path planning
    print("\n[Test] Testing path planning to (3.0, 3.0)...")
    success = system.set_navigation_goal(3.0, 3.0)
    
    if success:
        nav_vis, _, _ = system.visualize_all()
        nav_vis = cv2.resize(nav_vis, (600, 600))
        cv2.imshow('Path Planning Result', nav_vis)
        cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    print("\n[Test] Complete!")


if __name__ == "__main__":
    test_integrated_system()