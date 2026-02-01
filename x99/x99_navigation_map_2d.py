#!/usr/bin/env python3
"""
2D Navigation Map Builder - Giống LIDAR Map
Tạo map 2D từ stereo depth cho navigation, với:
1. Occupancy Grid mapping
2. Free space detection
3. Path planning (A*)
4. Costmap for safe navigation
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import heapq
from collections import deque

class NavigationMap2D:
    """
    2D Occupancy Grid Map cho navigation
    Tương tự như map từ LIDAR
    """
    
    def __init__(self, 
                 width: float = 20.0,  # meters
                 height: float = 20.0,  # meters  
                 resolution: float = 0.05):  # 5cm per cell
        
        self.width_m = width
        self.height_m = height
        self.resolution = resolution
        
        # Grid dimensions
        self.width_cells = int(width / resolution)
        self.height_cells = int(height / resolution)
        
        # Occupancy grid: -1=unknown, 0=free, 100=occupied
        self.grid = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)
        
        # Costmap for planning (inflated obstacles)
        self.costmap = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        
        # Observation counts (for probabilistic mapping)
        self.hit_count = np.zeros((self.height_cells, self.width_cells), dtype=np.int32)
        self.miss_count = np.zeros((self.height_cells, self.width_cells), dtype=np.int32)
        
        # Robot position (grid coordinates)
        self.robot_x = self.width_cells // 2
        self.robot_y = self.height_cells // 2
        
        # Height thresholds for obstacle classification
        self.ground_min = -0.4  # -40cm
        self.ground_max = -0.05  # -5cm
        self.obstacle_min = -0.05
        self.obstacle_max = 2.0
        
        # Inflation radius for costmap
        self.inflation_radius = 0.3  # meters
        
        print(f"[NavMap] Created {self.width_cells}x{self.height_cells} grid")
        print(f"  Real size: {width}x{height}m, Resolution: {resolution}m/cell")
    
    def world_to_grid(self, x: float, z: float) -> Tuple[int, int]:
        """
        Convert world coordinates (x, z) to grid coordinates
        World: x=right, z=forward
        Grid: grid_x=right, grid_y=down (forward)
        """
        grid_x = int(x / self.resolution) + self.robot_x
        grid_y = int(z / self.resolution) + self.robot_y
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid to world coordinates"""
        x = (grid_x - self.robot_x) * self.resolution
        z = (grid_y - self.robot_y) * self.resolution
        return x, z
    
    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid cell is valid"""
        return 0 <= grid_x < self.width_cells and 0 <= grid_y < self.height_cells
    
    def update_from_point_cloud(self, points_3d: np.ndarray, robot_pose_2d: Tuple[float, float] = None):
        """
        Update map from 3D point cloud
        points_3d: Nx3 array [x, y, z] in world frame
        robot_pose_2d: (x, z) robot position in world
        """
        if len(points_3d) == 0:
            return
        
        # Update robot position if provided
        if robot_pose_2d is not None:
            rx, rz = robot_pose_2d
            self.robot_x = int(rx / self.resolution) + self.width_cells // 2
            self.robot_y = int(rz / self.resolution) + self.height_cells // 2
        
        # Process points
        x = points_3d[:, 0]
        y = points_3d[:, 1]
        z = points_3d[:, 2]
        
        # Convert to grid
        grid_x = (x / self.resolution + self.robot_x).astype(np.int32)
        grid_y = (z / self.resolution + self.robot_y).astype(np.int32)
        
        # Filter valid cells
        valid_mask = (grid_x >= 0) & (grid_x < self.width_cells) & \
                     (grid_y >= 0) & (grid_y < self.height_cells)
        
        grid_x = grid_x[valid_mask]
        grid_y = grid_y[valid_mask]
        y_height = y[valid_mask]
        
        # Classify points
        # Ground points (free space)
        ground_mask = (y_height >= self.ground_min) & (y_height <= self.ground_max)
        
        # Obstacle points
        obstacle_mask = (y_height >= self.obstacle_min) & (y_height <= self.obstacle_max)
        
        # Update hit/miss counts
        # Ground = miss (free space)
        for gx, gy in zip(grid_x[ground_mask], grid_y[ground_mask]):
            self.miss_count[gy, gx] += 1
        
        # Obstacle = hit
        for gx, gy in zip(grid_x[obstacle_mask], grid_y[obstacle_mask]):
            self.hit_count[gy, gx] += 2  # Weight obstacles more
        
        # Update occupancy grid with probabilistic model
        self._update_occupancy_probabilities()
        
        # Update costmap
        self._update_costmap()
    
    def _update_occupancy_probabilities(self):
        """
        Update occupancy grid using log-odds
        """
        total_observations = self.hit_count + self.miss_count
        
        # Cells with observations
        observed_mask = total_observations > 0
        
        # Probability of occupied
        prob_occupied = np.zeros_like(self.grid, dtype=np.float32)
        prob_occupied[observed_mask] = self.hit_count[observed_mask] / total_observations[observed_mask]
        
        # Convert to occupancy values
        # Free: prob < 0.3
        # Occupied: prob > 0.6
        # Unknown: 0.3 <= prob <= 0.6 or no observations
        
        free_mask = (prob_occupied < 0.3) & observed_mask
        occupied_mask = (prob_occupied > 0.6) & observed_mask
        
        self.grid[free_mask] = 0
        self.grid[occupied_mask] = 100
        self.grid[~observed_mask] = -1  # Unknown
    
    def _update_costmap(self):
        """
        Update costmap with inflated obstacles
        Used for safe path planning
        """
        self.costmap.fill(0)
        
        # Start with occupied cells = high cost
        self.costmap[self.grid == 100] = 255
        
        # Inflate obstacles
        inflation_cells = int(self.inflation_radius / self.resolution)
        
        if inflation_cells > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, 
                (inflation_cells * 2 + 1, inflation_cells * 2 + 1)
            )
            
            occupied_mask = (self.grid == 100).astype(np.uint8) * 255
            inflated = cv2.dilate(occupied_mask, kernel)
            
            # Create gradient cost around obstacles
            distance_transform = cv2.distanceTransform(
                255 - occupied_mask, 
                cv2.DIST_L2, 
                5
            )
            
            # Normalize distance to cost
            max_dist = inflation_cells
            cost_gradient = np.clip(
                255 * (1.0 - distance_transform / max_dist), 
                0, 
                255
            )
            
            self.costmap = np.maximum(self.costmap, cost_gradient)
    
    def raycast_free_space(self):
        """
        Raycast to mark free space between robot and obstacles
        Giống như LIDAR - đánh dấu vùng giữa robot và obstacle là free
        """
        robot_gx, robot_gy = self.robot_x, self.robot_y
        
        # Find all occupied cells
        occupied_y, occupied_x = np.where(self.grid == 100)
        
        if len(occupied_x) == 0:
            return
        
        # For each occupied cell, raycast from robot
        for ox, oy in zip(occupied_x, occupied_y):
            # Bresenham line
            points = self._bresenham_line(robot_gx, robot_gy, ox, oy)
            
            # Mark all points except last as free
            for px, py in points[:-1]:
                if self.is_valid_cell(px, py):
                    if self.grid[py, px] == -1:  # Unknown -> Free
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
    
    def find_path_astar(self, goal_x: float, goal_z: float) -> Optional[List[Tuple[float, float]]]:
        """
        A* path planning from robot to goal
        Returns path as list of (x, z) world coordinates
        """
        # Convert goal to grid
        goal_gx, goal_gy = self.world_to_grid(goal_x, goal_z)
        
        # Check if goal is valid
        if not self.is_valid_cell(goal_gx, goal_gy):
            print(f"[NavMap] Goal outside map bounds")
            return None
        
        if self.grid[goal_gy, goal_gx] == 100:
            print(f"[NavMap] Goal is occupied!")
            return None
        
        # A* search
        start = (self.robot_x, self.robot_y)
        goal = (goal_gx, goal_gy)
        
        path = self._astar_search(start, goal)
        
        if path is None:
            print(f"[NavMap] No path found!")
            return None
        
        # Convert to world coordinates
        path_world = []
        for gx, gy in path:
            wx, wz = self.grid_to_world(gx, gy)
            path_world.append((wx, wz))
        
        print(f"[NavMap] Path found: {len(path_world)} waypoints")
        return path_world
    
    def _astar_search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* algorithm"""
        
        def heuristic(a, b):
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def get_neighbors(pos):
            x, y = pos
            # 8-connected neighbors
            neighbors = [
                (x+1, y), (x-1, y), (x, y+1), (x, y-1),
                (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)
            ]
            valid = []
            for nx, ny in neighbors:
                if self.is_valid_cell(nx, ny):
                    # Allow free and unknown cells (be optimistic about unknown)
                    if self.grid[ny, nx] != 100:
                        # Check costmap
                        if self.costmap[ny, nx] < 200:  # Not too close to obstacle
                            valid.append((nx, ny))
            return valid
        
        # Priority queue: (f_score, counter, position)
        counter = 0
        open_set = []
        heapq.heappush(open_set, (0, counter, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        open_set_hash = {start}
        
        while open_set:
            current = heapq.heappop(open_set)[2]
            open_set_hash.remove(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            for neighbor in get_neighbors(current):
                # Movement cost (diagonal = sqrt(2))
                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.414 if (dx + dy) == 2 else 1.0
                
                # Add costmap penalty
                cost_penalty = self.costmap[neighbor[1], neighbor[0]] / 255.0
                
                tentative_g = g_score[current] + move_cost * (1.0 + cost_penalty)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    
                    if neighbor not in open_set_hash:
                        counter += 1
                        heapq.heappush(open_set, (f, counter, neighbor))
                        open_set_hash.add(neighbor)
        
        return None  # No path found
    
    def get_free_space_frontiers(self) -> List[Tuple[float, float]]:
        """
        Find frontier cells (boundary between free and unknown)
        Useful for exploration
        """
        frontiers = []
        
        # Find free cells adjacent to unknown cells
        free_mask = (self.grid == 0)
        
        for y in range(1, self.height_cells - 1):
            for x in range(1, self.width_cells - 1):
                if free_mask[y, x]:
                    # Check 8 neighbors
                    neighbors = self.grid[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:  # Has unknown neighbor
                        wx, wz = self.grid_to_world(x, y)
                        frontiers.append((wx, wz))
        
        return frontiers
    
    def visualize(self, show_costmap: bool = False, path: List[Tuple[float, float]] = None) -> np.ndarray:
        """
        Visualize navigation map
        """
        if show_costmap:
            # Show costmap
            vis = cv2.applyColorMap(self.costmap.astype(np.uint8), cv2.COLORMAP_JET)
        else:
            # Show occupancy grid
            vis = np.zeros((self.height_cells, self.width_cells, 3), dtype=np.uint8)
            
            # Unknown = dark gray
            vis[self.grid == -1] = [50, 50, 50]
            
            # Free = light gray
            vis[self.grid == 0] = [200, 200, 200]
            
            # Occupied = black
            vis[self.grid == 100] = [0, 0, 0]
        
        # Draw robot
        if self.is_valid_cell(self.robot_x, self.robot_y):
            cv2.circle(vis, (self.robot_x, self.robot_y), 5, (0, 255, 0), -1)
            # Robot orientation arrow
            cv2.arrowedLine(vis, 
                          (self.robot_x, self.robot_y),
                          (self.robot_x, self.robot_y - 10),
                          (0, 255, 0), 2, tipLength=0.3)
        
        # Draw path if provided
        if path is not None and len(path) > 1:
            path_grid = []
            for wx, wz in path:
                gx, gy = self.world_to_grid(wx, wz)
                if self.is_valid_cell(gx, gy):
                    path_grid.append([gx, gy])
            
            if len(path_grid) > 1:
                pts = np.array(path_grid, dtype=np.int32)
                cv2.polylines(vis, [pts], False, (255, 0, 255), 2)
                
                # Goal
                cv2.circle(vis, tuple(path_grid[-1]), 8, (255, 0, 0), -1)
        
        return vis
    
    def save_map(self, filename: str):
        """Save map to file"""
        np.savez_compressed(
            filename,
            grid=self.grid,
            costmap=self.costmap,
            hit_count=self.hit_count,
            miss_count=self.miss_count,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            resolution=self.resolution,
            width_m=self.width_m,
            height_m=self.height_m
        )
        print(f"[NavMap] Saved to {filename}")
    
    def load_map(self, filename: str) -> bool:
        """Load map from file"""
        try:
            data = np.load(filename)
            self.grid = data['grid']
            self.costmap = data['costmap']
            self.hit_count = data['hit_count']
            self.miss_count = data['miss_count']
            self.robot_x = int(data['robot_x'])
            self.robot_y = int(data['robot_y'])
            self.resolution = float(data['resolution'])
            self.width_m = float(data['width_m'])
            self.height_m = float(data['height_m'])
            
            self.width_cells = self.grid.shape[1]
            self.height_cells = self.grid.shape[0]
            
            print(f"[NavMap] Loaded from {filename}")
            return True
        except Exception as e:
            print(f"[NavMap] Load failed: {e}")
            return False