#!/usr/bin/env python3
"""
Stereo Depth Mapping and Obstacle Avoidance - OPTIMIZED + CALIBRATED
Creates 2D occupancy map from stereo cameras (like LIDAR)
Features:
- Vectorized operations for high performance
- Supports Camera Calibration (calibration_params.npz) for rectification
"""

import cv2
import numpy as np
import os
from typing import List, Tuple, Optional

class StereoDepthMapper:
    """Create depth map and 2D occupancy grid from stereo cameras"""
    
    def __init__(self, baseline: float = 0.01, focal_length: float = 500,
                 cx: float = 320, cy: float = 240, 
                 calibration_file: str = "calibration_params.npz"):
        """
        baseline: Distance between cameras in meters
        focal_length: Focal length in pixels (default fallback)
        calibration_file: Path to .npz file from calibration tool
        """
        self.baseline = baseline
        self.focal_length = focal_length
        self.cx = cx
        self.cy = cy
        
        # Meshgrid cache
        self.u_grid = None
        self.v_grid = None
        
        # --- CALIBRATION SETUP ---
        self.use_calib = False
        self.calib_data = None
        self.map_l1 = None
        self.map_l2 = None
        self.map_r1 = None
        self.map_r2 = None
        
        if os.path.exists(calibration_file):
            try:
                self.calib_data = np.load(calibration_file)
                print(f"[Stereo] Found calibration file: {calibration_file}")
                # We will init maps lazily when we receive the first frame to know resolution
                self.use_calib = True
            except Exception as e:
                print(f"[Stereo] Error loading calibration: {e}")
        else:
            print("[Stereo] No calibration file found. Running in RAW mode (Expect distortion).")

        # --- SGBM CONFIGURATION ---
        # Optimized for Speed vs Quality
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,   # Reduced from 128 for speed (must be div by 16)
            blockSize=5,
            P1=8 * 3 * 5**2,     
            P2=32 * 3 * 5**2,    
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
    def _init_rectification_maps(self, width, height):
        """Lazy initialization of rectification maps based on image size"""
        if not self.use_calib or self.calib_data is None:
            return

        try:
            # Generate look-up tables for remapping
            # Using CV_16SC2 is faster for remap
            self.map_l1, self.map_l2 = cv2.initUndistortRectifyMap(
                self.calib_data['mtx_l'], self.calib_data['dist_l'], 
                self.calib_data['R1'], self.calib_data['P1'], 
                (width, height), cv2.CV_16SC2)
            
            self.map_r1, self.map_r2 = cv2.initUndistortRectifyMap(
                self.calib_data['mtx_r'], self.calib_data['dist_r'], 
                self.calib_data['R2'], self.calib_data['P2'], 
                (width, height), cv2.CV_16SC2)
            
            # Update projection matrix Q if available for better 3D reprojection
            if 'Q' in self.calib_data:
                Q = self.calib_data['Q']
                # Q matrix format:
                # [1 0 0 -cx]
                # [0 1 0 -cy]
                # [0 0 0  f ]
                # [0 0 -1/Tx (cx-cx')/Tx]
                self.focal_length = Q[2, 3]
                self.cx = -Q[0, 3]
                self.cy = -Q[1, 3]
                # Baseline T = -1/Q[3,2] (Approximation)
                if Q[3, 2] != 0:
                    self.baseline = 1.0 / abs(Q[3, 2])
            
            print(f"[Stereo] Maps initialized for resolution {width}x{height}")
            print(f"[Stereo] Calibrated Params - F: {self.focal_length:.1f}, B: {self.baseline:.4f}")
            
        except Exception as e:
            print(f"[Stereo] Map init failed: {e}")
            self.use_calib = False

    def compute_disparity(self, left_frame: np.ndarray, 
                         right_frame: np.ndarray,
                         use_wls: bool = False) -> np.ndarray:
        """Compute disparity map from stereo pair"""
        
        height, width = left_frame.shape[:2]
        
        # Init maps if needed
        if self.use_calib and self.map_l1 is None:
            self._init_rectification_maps(width, height)
            
        # 1. RECTIFICATION (Apply Calibration)
        if self.use_calib and self.map_l1 is not None:
            # Remap is very fast with fixed point maps
            img_left = cv2.remap(left_frame, self.map_l1, self.map_l2, cv2.INTER_LINEAR)
            img_right = cv2.remap(right_frame, self.map_r1, self.map_r2, cv2.INTER_LINEAR)
        else:
            img_left = left_frame
            img_right = right_frame

        # 2. DOWNSCALE (Optional optimization)
        scale = 1.0
        if width > 480:
             scale = 0.5
             left_small = cv2.resize(img_left, (0, 0), fx=scale, fy=scale)
             right_small = cv2.resize(img_right, (0, 0), fx=scale, fy=scale)
        else:
             left_small = img_left
             right_small = img_right
             
        # 3. CONVERT TO GRAY
        gray_left = cv2.cvtColor(left_small, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_small, cv2.COLOR_BGR2GRAY)
        
        # 4. COMPUTE DISPARITY
        # Note: WLS filter is disabled by default for speed
        disparity = self.stereo.compute(gray_left, gray_right)
        
        # 5. UPSCALE
        if scale != 1.0:
            disparity = cv2.resize(disparity, (width, height), interpolation=cv2.INTER_NEAREST)
            disparity = disparity * (1.0 / scale)
        
        # Normalize (16 levels)
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """Convert disparity to depth in meters"""
        valid_mask = disparity > 0
        depth = np.zeros_like(disparity)
        
        # Z = (f * B) / d
        depth[valid_mask] = (self.baseline * self.focal_length) / disparity[valid_mask]
        
        depth = np.clip(depth, 0, 10.0)
        return depth
    
    def depth_to_point_cloud(self, depth: np.ndarray, 
                            color_image: np.ndarray = None,
                            step: int = 4) -> Tuple[np.ndarray, np.ndarray]:
        """Convert depth map to 3D point cloud (Vectorized)"""
        
        height, width = depth.shape
        
        if self.u_grid is None or self.u_grid.shape != (height, width):
            u = np.arange(width)
            v = np.arange(height)
            self.u_grid, self.v_grid = np.meshgrid(u, v)
            
        # Downsample by slicing (Fastest method)
        depth_sub = depth[::step, ::step]
        u_sub = self.u_grid[::step, ::step]
        v_sub = self.v_grid[::step, ::step]
        
        if color_image is not None:
            # If we rectified the image, we should use the rectified color image technically,
            # but usually the input color_image here is the raw one from the loop.
            # Ideally, the caller should pass the rectified image, but for visualization
            # raw is usually "okay" enough if distortion isn't massive.
            color_sub = color_image[::step, ::step]
        
        mask = (depth_sub > 0.1) & (depth_sub < 10.0)
        
        z = depth_sub[mask]
        x = (u_sub[mask] - self.cx) * z / self.focal_length
        y = (v_sub[mask] - self.cy) * z / self.focal_length
        
        points = np.stack((x, y, z), axis=1)
        
        if color_image is not None:
            colors = color_sub[mask]
            return points, colors
        
        return points, None

class OccupancyGridMapper:
    """Create 2D occupancy grid from depth map (LIDAR-like)"""
    
    def __init__(self, grid_size: int = 400, resolution: float = 0.05,
                 max_range: float = 5.0):
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_range = max_range
        
        self.grid = np.full((grid_size, grid_size), -1, dtype=np.int8)
        self.robot_x = grid_size // 2
        self.robot_y = grid_size // 2
        
        self.ground_height_min = -0.3
        self.ground_height_max = -0.05
        self.obstacle_height_min = -0.05
        self.obstacle_height_max = 2.0
        
    def update_from_point_cloud(self, points: np.ndarray):
        """Update occupancy grid from 3D point cloud (Vectorized)"""
        self.grid.fill(-1)
        self.grid[self.robot_y, self.robot_x] = 0
        
        if len(points) == 0:
            return

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        
        # Convert to grid coordinates
        # Map: X (World) -> X (Grid), Z (World) -> Y (Grid)
        grid_x = ((x / self.resolution) + self.robot_x).astype(np.int32)
        grid_z = ((z / self.resolution) + self.robot_y).astype(np.int32)
        
        # Filter bounds
        mask = (grid_x >= 0) & (grid_x < self.grid_size) & \
               (grid_z >= 0) & (grid_z < self.grid_size) & \
               (np.sqrt(x**2 + z**2) <= self.max_range)
               
        grid_x = grid_x[mask]
        grid_z = grid_z[mask]
        y_masked = y[mask]
        
        # Vectorized classification
        # Ground points
        ground_mask = (y_masked >= self.ground_height_min) & (y_masked <= self.ground_height_max)
        self.grid[grid_z[ground_mask], grid_x[ground_mask]] = 0
        
        # Obstacle points (overwrite ground if conflict)
        obs_mask = (y_masked >= self.obstacle_height_min) & (y_masked <= self.obstacle_height_max)
        self.grid[grid_z[obs_mask], grid_x[obs_mask]] = 100
    
    def inflate_obstacles(self, radius: int = 3):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius*2+1, radius*2+1))
        occupied = (self.grid == 100).astype(np.uint8) * 255
        inflated = cv2.dilate(occupied, kernel)
        self.grid[inflated > 0] = 100

    def get_obstacle_scan(self, num_rays: int = 360) -> np.ndarray:
        scan = np.full(num_rays, self.max_range, dtype=np.float32)
        angles = np.linspace(0, 2*np.pi, num_rays)
        
        max_dist_cells = int(self.max_range / self.resolution)
        sines = np.sin(angles)
        cosines = np.cos(angles)
        
        for i in range(num_rays):
            # Fast raycast (skip step=2)
            for dist_cells in range(1, max_dist_cells, 2): 
                x = int(self.robot_x + dist_cells * sines[i])
                y = int(self.robot_y - dist_cells * cosines[i])
                
                if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
                    break
                
                if self.grid[y, x] == 100:
                    scan[i] = dist_cells * self.resolution
                    break
        return scan
    
    def visualize(self):
        """High contrast visualization for Web"""
        # Gray background for Unknown
        vis = np.full((self.grid_size, self.grid_size, 3), 128, dtype=np.uint8)
        
        # Free space = White
        vis[self.grid == 0] = [255, 255, 250]
        
        # Obstacles = Dark Color
        vis[self.grid == 100] = [40, 40, 40]
        
        # Robot Position
        center = (self.robot_x, self.robot_y)
        cv2.circle(vis, center, 6, (0, 0, 255), 1)
        cv2.arrowedLine(vis, center, (self.robot_x, self.robot_y - 15), (0, 0, 255), 2, tipLength=0.3)
        
        return vis

class ObstacleAvoidance:
    """Reactive obstacle avoidance controller"""
    
    def __init__(self, safety_distance: float = 0.5, max_linear_vel: float = 0.3):
        self.safety_distance = safety_distance
        self.max_linear_vel = max_linear_vel
        
    def compute_velocity(self, scan: np.ndarray) -> Tuple[float, float]:
        num_rays = len(scan)
        angles = np.linspace(0, 2*np.pi, num_rays)
        
        # Front arc check
        front_mask = (angles > -np.pi/3) & (angles < np.pi/3)
        front_dists = scan[front_mask]
        
        if len(front_dists) == 0:
             return 0.0, 0.0
             
        min_front = np.min(front_dists)
        
        if min_front < self.safety_distance:
            # Too close, turn away
            left_sect = scan[int(num_rays*0.25):int(num_rays*0.5)]
            right_sect = scan[int(num_rays*0.5):int(num_rays*0.75)]
            
            # Simple logic: Turn towards larger average distance
            if np.mean(left_sect) > np.mean(right_sect):
                return 0.0, 0.5 # Turn Left
            else:
                return 0.0, -0.5 # Turn Right
        
        # Proportional speed control
        linear = self.max_linear_vel * min(1.0, min_front / (self.safety_distance * 2))
        return linear, 0.0