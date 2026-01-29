#!/usr/bin/env python3
"""
Stereo Depth Mapping and Obstacle Avoidance
Creates 2D occupancy map from stereo cameras (like LIDAR)
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
        baseline: Distance between cameras in meters (27.5cm)
        focal_length: Focal length in pixels
        cx, cy: Principal point (image center)
        """
        self.baseline = baseline
        self.focal_length = focal_length
        self.cx = cx
        self.cy = cy
        
        # Stereo matcher
        # SGBM (Semi-Global Block Matching) - better quality
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5**2,     # Penalty for disparity change by ±1
            P2=32 * 3 * 5**2,    # Penalty for disparity change by more than ±1
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # WLS Filter for disparity refinement
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
        self.wls_filter.setLambda(8000)
        self.wls_filter.setSigmaColor(1.5)
        
        # Right matcher for WLS
        self.stereo_right = cv2.ximgproc.createRightMatcher(self.stereo)
        
    def compute_disparity(self, left_frame: np.ndarray, 
                         right_frame: np.ndarray,
                         use_wls: bool = True) -> np.ndarray:
        """Compute disparity map from stereo pair"""
        
        # Convert to grayscale
        gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
        
        if use_wls:
            # Compute disparity for both images
            disp_left = self.stereo.compute(gray_left, gray_right)
            disp_right = self.stereo_right.compute(gray_right, gray_left)
            
            # WLS filtering
            disparity = self.wls_filter.filter(disp_left, gray_left, None, disp_right)
        else:
            # Simple disparity
            disparity = self.stereo.compute(gray_left, gray_right)
        
        # Convert to float and normalize
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """Convert disparity to depth in meters"""
        
        # Avoid division by zero
        disparity[disparity <= 0] = 0.1
        
        # Depth = (baseline * focal_length) / disparity
        depth = (self.baseline * self.focal_length) / disparity
        
        # Clip unrealistic values
        depth = np.clip(depth, 0, 10.0)  # Max 10 meters
        
        return depth
    
    def depth_to_point_cloud(self, depth: np.ndarray, 
                            color_image: np.ndarray = None) -> np.ndarray:
        """Convert depth map to 3D point cloud"""
        
        height, width = depth.shape
        points = []
        colors = []
        
        for v in range(0, height, 4):  # Sample every 4 pixels
            for u in range(0, width, 4):
                z = depth[v, u]
                
                # Skip invalid depths
                if z <= 0 or z > 10.0:
                    continue
                
                # Compute 3D coordinates
                x = (u - self.cx) * z / self.focal_length
                y = (v - self.cy) * z / self.focal_length
                
                points.append([x, y, z])
                
                if color_image is not None:
                    color = color_image[v, u]
                    colors.append(color)
        
        return np.array(points), np.array(colors) if color_image is not None else None

class OccupancyGridMapper:
    """Create 2D occupancy grid from depth map (like LIDAR)"""
    
    def __init__(self, grid_size: int = 400, resolution: float = 0.05,
                 max_range: float = 5.0):
        """
        grid_size: Size of grid in cells
        resolution: Meters per cell (5cm)
        max_range: Maximum detection range in meters
        """
        self.grid_size = grid_size
        self.resolution = resolution
        self.max_range = max_range
        
        # Occupancy grid: 0 = free, 100 = occupied, -1 = unknown
        self.grid = np.full((grid_size, grid_size), -1, dtype=np.int8)
        
        # Robot position (center of grid)
        self.robot_x = grid_size // 2
        self.robot_y = grid_size // 2
        
        # Ground height threshold (for obstacle detection)
        self.ground_height_min = -0.3  # 30cm below camera
        self.ground_height_max = -0.05  # 5cm below camera
        self.obstacle_height_min = -0.05  # Above ground
        self.obstacle_height_max = 2.0   # Max 2m height
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int(x / self.resolution) + self.robot_x
        grid_y = int(-y / self.resolution) + self.robot_y  # Flip Y
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = (grid_x - self.robot_x) * self.resolution
        y = -(grid_y - self.robot_y) * self.resolution
        return x, y
    
    def update_from_point_cloud(self, points: np.ndarray):
        """Update occupancy grid from 3D point cloud"""
        
        # Reset grid
        self.grid.fill(-1)
        self.grid[self.robot_y, self.robot_x] = 0  # Robot position is free
        
        for point in points:
            x, y, z = point
            
            # Check range
            distance = np.sqrt(x**2 + z**2)
            if distance > self.max_range:
                continue
            
            # Convert to grid coordinates
            grid_x, grid_z = self.world_to_grid(x, z)
            
            # Check bounds
            if not (0 <= grid_x < self.grid_size and 0 <= grid_z < self.grid_size):
                continue
            
            # Classify point
            # y is height (negative = below camera)
            if self.ground_height_min <= y <= self.ground_height_max:
                # Ground - mark as free
                self.grid[grid_z, grid_x] = 0
            elif self.obstacle_height_min <= y <= self.obstacle_height_max:
                # Obstacle - mark as occupied
                self.grid[grid_z, grid_x] = 100
        
        # Ray casting to mark free space between robot and obstacles
        self._ray_cast_free_space()
    
    def _ray_cast_free_space(self):
        """Mark cells as free between robot and detected points"""
        
        # Find all occupied cells
        occupied = np.argwhere(self.grid == 100)
        
        for cell in occupied:
            # Ray from robot to occupied cell
            y0, x0 = self.robot_y, self.robot_x
            y1, x1 = cell[0], cell[1]
            
            # Bresenham's line algorithm
            points = self._bresenham_line(x0, y0, x1, y1)
            
            # Mark all points except last as free
            for px, py in points[:-1]:
                if 0 <= px < self.grid_size and 0 <= py < self.grid_size:
                    if self.grid[py, px] != 100:  # Don't overwrite obstacles
                        self.grid[py, px] = 0
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def inflate_obstacles(self, radius: int = 3):
        """Inflate obstacles for robot safety margin"""
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius*2+1, radius*2+1))
        occupied = (self.grid == 100).astype(np.uint8) * 255
        inflated = cv2.dilate(occupied, kernel)
        self.grid[inflated > 0] = 100
    
    def get_local_costmap(self, radius_cells: int = 50) -> np.ndarray:
        """Get local costmap around robot"""
        x_min = max(0, self.robot_x - radius_cells)
        x_max = min(self.grid_size, self.robot_x + radius_cells)
        y_min = max(0, self.robot_y - radius_cells)
        y_max = min(self.grid_size, self.robot_y + radius_cells)
        
        return self.grid[y_min:y_max, x_min:x_max]
    
    def visualize(self, show_robot: bool = True) -> np.ndarray:
        """Create visualization of occupancy grid"""
        
        # Create color image
        vis = np.zeros((self.grid_size, self.grid_size, 3), dtype=np.uint8)
        
        # Unknown = gray
        vis[self.grid == -1] = [128, 128, 128]
        
        # Free = white
        vis[self.grid == 0] = [255, 255, 255]
        
        # Occupied = black
        vis[self.grid == 100] = [0, 0, 0]
        
        if show_robot:
            # Draw robot position
            cv2.circle(vis, (self.robot_x, self.robot_y), 5, (0, 255, 0), -1)
            
            # Draw robot orientation (forward = up in image)
            cv2.arrowedLine(vis, 
                          (self.robot_x, self.robot_y),
                          (self.robot_x, self.robot_y - 15),
                          (0, 255, 0), 2)
        
        return vis
    
    def get_obstacle_scan(self, num_rays: int = 360) -> np.ndarray:
        """
        Get 360-degree obstacle scan (LIDAR-like)
        Returns: array of distances for each angle
        """
        scan = np.full(num_rays, self.max_range, dtype=np.float32)
        
        for i, angle in enumerate(np.linspace(0, 2*np.pi, num_rays)):
            # Cast ray from robot position
            max_dist_cells = int(self.max_range / self.resolution)
            
            for dist_cells in range(1, max_dist_cells):
                # Calculate cell position
                x = int(self.robot_x + dist_cells * np.sin(angle))
                y = int(self.robot_y - dist_cells * np.cos(angle))
                
                # Check bounds
                if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
                    break
                
                # Check if obstacle
                if self.grid[y, x] == 100:
                    scan[i] = dist_cells * self.resolution
                    break
        
        return scan

class ObstacleAvoidance:
    """Reactive obstacle avoidance controller"""
    
    def __init__(self, safety_distance: float = 0.5, max_linear_vel: float = 0.3):
        """
        safety_distance: Minimum distance to obstacles (meters)
        max_linear_vel: Maximum forward velocity
        """
        self.safety_distance = safety_distance
        self.max_linear_vel = max_linear_vel
        
    def compute_velocity(self, scan: np.ndarray, target_angle: float = 0.0) -> Tuple[float, float]:
        """
        Compute velocity commands from obstacle scan
        scan: 360-degree distance array
        target_angle: Desired direction in radians (0 = forward)
        Returns: (linear_velocity, angular_velocity)
        """
        
        num_rays = len(scan)
        angles = np.linspace(0, 2*np.pi, num_rays)
        
        # Find obstacles in danger zone
        front_indices = np.where(
            (angles > -np.pi/3) & (angles < np.pi/3)  # 120° front arc
        )[0]
        
        front_distances = scan[front_indices]
        min_front_dist = np.min(front_distances)
        
        # Check if obstacle too close
        if min_front_dist < self.safety_distance:
            # Emergency stop or turn
            linear_vel = 0.0
            
            # Find direction with most free space
            left_dist = np.mean(scan[int(num_rays*0.25):int(num_rays*0.5)])
            right_dist = np.mean(scan[int(num_rays*0.5):int(num_rays*0.75)])
            
            # Turn away from obstacle
            if left_dist > right_dist:
                angular_vel = 0.5  # Turn left
            else:
                angular_vel = -0.5  # Turn right
            
            return linear_vel, angular_vel
        
        # Path is clear, move forward
        linear_vel = self.max_linear_vel * min(1.0, min_front_dist / (self.safety_distance * 2))
        
        # Steer towards target
        angular_vel = np.clip(target_angle * 0.5, -1.0, 1.0)
        
        return linear_vel, angular_vel
    
    def get_free_directions(self, scan: np.ndarray, 
                           threshold: float = 1.0) -> List[Tuple[float, float]]:
        """
        Find free directions for navigation
        Returns list of (angle, distance) for free paths
        """
        num_rays = len(scan)
        angles = np.linspace(0, 2*np.pi, num_rays)
        
        free_directions = []
        
        for i, (angle, dist) in enumerate(zip(angles, scan)):
            if dist > threshold:
                free_directions.append((angle, dist))
        
        return free_directions

def test_depth_mapping():
    """Test stereo depth mapping"""
    
    # Initialize
    depth_mapper = StereoDepthMapper(baseline=0.275, focal_length=500)
    grid_mapper = OccupancyGridMapper(grid_size=400, resolution=0.05, max_range=5.0)
    avoidance = ObstacleAvoidance(safety_distance=0.5)
    
    # Open cameras (for testing)
    cap_left = cv2.VideoCapture(0)
    cap_right = cv2.VideoCapture(1)
    
    if not cap_left.isOpened() or not cap_right.isOpened():
        print("Cameras not available - using test data")
        return
    
    print("Starting stereo depth mapping...")
    print("Press 'q' to quit")
    
    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()
        
        if not ret_left or not ret_right:
            break
        
        # Compute disparity and depth
        disparity = depth_mapper.compute_disparity(frame_left, frame_right)
        depth = depth_mapper.disparity_to_depth(disparity)
        
        # Create point cloud
        points, colors = depth_mapper.depth_to_point_cloud(depth, frame_left)
        
        # Update occupancy grid
        if len(points) > 0:
            grid_mapper.update_from_point_cloud(points)
            grid_mapper.inflate_obstacles(radius=3)
        
        # Get obstacle scan
        scan = grid_mapper.get_obstacle_scan(num_rays=360)
        
        # Compute avoidance velocity
        linear_vel, angular_vel = avoidance.compute_velocity(scan)
        
        # Visualize
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        
        grid_vis = grid_mapper.visualize()
        grid_vis = cv2.resize(grid_vis, (600, 600))
        
        # Add velocity info
        cv2.putText(grid_vis, f"V: {linear_vel:.2f} m/s", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(grid_vis, f"W: {angular_vel:.2f} rad/s", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Show
        cv2.imshow('Left Camera', frame_left)
        cv2.imshow('Depth Map', depth_color)
        cv2.imshow('Occupancy Grid (2D Map)', grid_vis)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_depth_mapping()