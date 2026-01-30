#!/usr/bin/env python3
"""
Stereo Depth Mapping and Obstacle Avoidance - OPTIMIZED
Creates 2D occupancy map from stereo cameras (like LIDAR)
Vectorized operations for high performance
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import threading
import time

class StereoDepthMapper:
    """Create depth map and 2D occupancy grid from stereo cameras"""
    
    def __init__(self, baseline: float = 0.01, focal_length: float = 500,
                 cx: float = 320, cy: float = 240):
        """
        baseline: Distance between cameras in meters
        focal_length: Focal length in pixels
        cx, cy: Principal point (image center)
        """
        self.baseline = baseline
        self.focal_length = focal_length
        self.cx = cx
        self.cy = cy
        
        # Pre-compute meshgrid for 3D reconstruction
        # (Assuming standard VGA resolution, will resize if needed)
        self.u_grid = None
        self.v_grid = None
        
        # Stereo matcher
        # SGBM (Semi-Global Block Matching) - better quality
        # Optimized parameters for speed vs quality
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
        
        # WLS Filter (Optional, creates overhead)
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
        self.wls_filter.setLambda(8000)
        self.wls_filter.setSigmaColor(1.5)
        self.stereo_right = cv2.ximgproc.createRightMatcher(self.stereo)
        
    def compute_disparity(self, left_frame: np.ndarray, 
                         right_frame: np.ndarray,
                         use_wls: bool = False) -> np.ndarray:
        """Compute disparity map from stereo pair"""
        
        # Downscale for speed (optional, but highly recommended for CPU)
        # Using 320x240 is 4x faster than 640x480
        height, width = left_frame.shape[:2]
        
        # If image is large, downscale it
        scale = 1.0
        if width > 480:
             scale = 0.5
             left_small = cv2.resize(left_frame, (0, 0), fx=scale, fy=scale)
             right_small = cv2.resize(right_frame, (0, 0), fx=scale, fy=scale)
        else:
             left_small = left_frame
             right_small = right_frame
             
        # Convert to grayscale
        gray_left = cv2.cvtColor(left_small, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_small, cv2.COLOR_BGR2GRAY)
        
        if use_wls:
            # WLS is SLOW on CPU (computes matching twice)
            disp_left = self.stereo.compute(gray_left, gray_right)
            disp_right = self.stereo_right.compute(gray_right, gray_left)
            disparity = self.wls_filter.filter(disp_left, gray_left, None, disp_right)
        else:
            # Standard SGBM
            disparity = self.stereo.compute(gray_left, gray_right)
        
        # Upscale disparity if we downscaled the image
        if scale != 1.0:
            disparity = cv2.resize(disparity, (width, height), interpolation=cv2.INTER_NEAREST)
            disparity = disparity * (1.0 / scale) # Adjust disparity magnitude
        
        # Convert to float and normalize (16 levels)
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """Convert disparity to depth in meters"""
        # Avoid division by zero
        # Use a mask to keep 0 disparity as 0 depth (infinite)
        valid_mask = disparity > 0
        depth = np.zeros_like(disparity)
        
        depth[valid_mask] = (self.baseline * self.focal_length) / disparity[valid_mask]
        
        # Clip unrealistic values
        depth = np.clip(depth, 0, 10.0)  # Max 10 meters
        return depth
    
    def depth_to_point_cloud(self, depth: np.ndarray, 
                            color_image: np.ndarray = None,
                            step: int = 4) -> Tuple[np.ndarray, np.ndarray]:
        """Convert depth map to 3D point cloud (Vectorized)"""
        
        height, width = depth.shape
        
        # Initialize grid if needed
        if self.u_grid is None or self.u_grid.shape != (height, width):
            u = np.arange(width)
            v = np.arange(height)
            self.u_grid, self.v_grid = np.meshgrid(u, v)
            
        # Downsample by slicing
        # This is much faster than the loop with step
        depth_sub = depth[::step, ::step]
        u_sub = self.u_grid[::step, ::step]
        v_sub = self.v_grid[::step, ::step]
        
        if color_image is not None:
            color_sub = color_image[::step, ::step]
        
        # Mask valid depth
        mask = (depth_sub > 0.1) & (depth_sub < 10.0)
        
        # Calculate 3D coordinates (Vectorized)
        z = depth_sub[mask]
        x = (u_sub[mask] - self.cx) * z / self.focal_length
        y = (v_sub[mask] - self.cy) * z / self.focal_length
        
        # Stack to Nx3 array
        points = np.stack((x, y, z), axis=1)
        
        if color_image is not None:
            colors = color_sub[mask]
            return points, colors
        
        return points, None

class OccupancyGridMapper:
    """Create 2D occupancy grid from depth map"""
    
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
        
        # Calculate grid coordinates
        # grid_x = int(x / res) + robot_x
        # grid_z = int(-z / res) + robot_y (z is forward, so negative for image coords?)
        # Wait, usually Z is forward in camera frame. 
        # In image coords, usually Up is -Y.
        # Let's match the original logic:
        # grid_z = int(x / res) + origin_x (Wait, original mapped x->grid_x, z->grid_y?)
        # Original: grid_x = int(x/res) + orig_x, grid_y = int(-y/res) + orig_y ??
        # No, original said: grid_x, grid_z = self.world_to_grid(x, z)
        # And world_to_grid: int(x/res) + origin, int(z/res) + origin (Actually it used z for y in grid?)
        
        # Re-checking original logic:
        # grid_x = int(x / self.resolution) + self.robot_x
        # grid_y = int(-y / self.resolution) + self.robot_y  <-- This uses Y for the second coord?
        # But in update_from_point_cloud: grid_x, grid_z = self.world_to_grid(x, z)
        # So it maps World Z -> Grid Y.
        
        grid_x = ((x / self.resolution) + self.robot_x).astype(np.int32)
        grid_z = ((z / self.resolution) + self.robot_y).astype(np.int32) # Using Z for grid Y axis
        
        # Filter bounds
        mask = (grid_x >= 0) & (grid_x < self.grid_size) &                (grid_z >= 0) & (grid_z < self.grid_size) &                (np.sqrt(x**2 + z**2) <= self.max_range)
               
        grid_x = grid_x[mask]
        grid_z = grid_z[mask]
        y_masked = y[mask]
        
        # Vectorized classification
        # We need to act on the grid. Since multiple points can hit the same cell,
        # we prioritize obstacles.
        
        # Ground points
        ground_mask = (y_masked >= self.ground_height_min) & (y_masked <= self.ground_height_max)
        self.grid[grid_z[ground_mask], grid_x[ground_mask]] = 0
        
        # Obstacle points (overwrite ground if conflict)
        obs_mask = (y_masked >= self.obstacle_height_min) & (y_masked <= self.obstacle_height_max)
        self.grid[grid_z[obs_mask], grid_x[obs_mask]] = 100
        
        # Ray casting is hard to vectorize fully without complex logic.
        # For performance, we might skip full raycasting or use a simplified version.
        # The original used Bresenham for every point. That is very slow.
        # We will skip it for now to save massive FPS.
    
    def inflate_obstacles(self, radius: int = 3):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius*2+1, radius*2+1))
        occupied = (self.grid == 100).astype(np.uint8) * 255
        inflated = cv2.dilate(occupied, kernel)
        self.grid[inflated > 0] = 100

    def get_obstacle_scan(self, num_rays: int = 360) -> np.ndarray:
        scan = np.full(num_rays, self.max_range, dtype=np.float32)
        angles = np.linspace(0, 2*np.pi, num_rays)
        
        # Simplified scan - just check limited points per ray
        # (This is still a loop, but 360 iters is okay)
        max_dist_cells = int(self.max_range / self.resolution)
        
        # Pre-compute sin/cos
        sines = np.sin(angles)
        cosines = np.cos(angles)
        
        for i in range(num_rays):
            for dist_cells in range(1, max_dist_cells, 2): # Skip some cells
                x = int(self.robot_x + dist_cells * sines[i])
                y = int(self.robot_y - dist_cells * cosines[i])
                
                if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
                    break
                
                if self.grid[y, x] == 100:
                    scan[i] = dist_cells * self.resolution
                    break
        return scan
    
    def visualize(self):
        vis = np.zeros((self.grid_size, self.grid_size, 3), dtype=np.uint8)
        vis[self.grid == -1] = [128, 128, 128]
        vis[self.grid == 0] = [255, 255, 255]
        vis[self.grid == 100] = [0, 0, 0]
        cv2.circle(vis, (self.robot_x, self.robot_y), 5, (0, 255, 0), -1)
        return vis

class ObstacleAvoidance:
    """Reactive obstacle avoidance controller"""
    
    def __init__(self, safety_distance: float = 0.5, max_linear_vel: float = 0.3):
        self.safety_distance = safety_distance
        self.max_linear_vel = max_linear_vel
        
    def compute_velocity(self, scan: np.ndarray) -> Tuple[float, float]:
        num_rays = len(scan)
        angles = np.linspace(0, 2*np.pi, num_rays)
        
        # Front arc (-60 to +60 degrees)
        front_mask = (angles > -np.pi/3) & (angles < np.pi/3)
        front_dists = scan[front_mask]
        
        if len(front_dists) == 0:
             return 0.0, 0.0
             
        min_front = np.min(front_dists)
        
        if min_front < self.safety_distance:
            # Turn
            left_sect = scan[int(num_rays*0.25):int(num_rays*0.5)]
            right_sect = scan[int(num_rays*0.5):int(num_rays*0.75)]
            if np.mean(left_sect) > np.mean(right_sect):
                return 0.0, 0.5 # Left
            else:
                return 0.0, -0.5 # Right
        
        linear = self.max_linear_vel * min(1.0, min_front / (self.safety_distance * 2))
        return linear, 0.0