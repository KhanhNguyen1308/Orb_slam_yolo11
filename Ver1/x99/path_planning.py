#!/usr/bin/env python3
"""
Path Planning Module for X99 Server
Uses SLAM map to generate obstacle-free paths
"""

import numpy as np
import cv2
from collections import deque
from typing import List, Tuple, Optional
import heapq

class OccupancyGrid:
    """2D occupancy grid for path planning"""
    
    def __init__(self, width: int = 200, height: int = 200, resolution: float = 0.05):
        """
        width, height: Grid size in cells
        resolution: meters per cell (5cm default)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Grid: 0 = free, 100 = occupied, -1 = unknown
        self.grid = np.full((height, width), -1, dtype=np.int8)
        
        # Robot starting position (center of grid)
        self.origin_x = width // 2
        self.origin_y = height // 2
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (meters) to grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin_x
        grid_y = int(y / self.resolution) + self.origin_y
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (meters)"""
        x = (grid_x - self.origin_x) * self.resolution
        y = (grid_y - self.origin_y) * self.resolution
        return x, y
    
    def is_valid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are valid"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def is_free(self, grid_x: int, grid_y: int, threshold: int = 30) -> bool:
        """Check if cell is free (not occupied)"""
        if not self.is_valid(grid_x, grid_y):
            return False
        return self.grid[grid_y, grid_x] < threshold
    
    def update_from_slam(self, map_points: List[dict], robot_pose: np.ndarray):
        """Update occupancy grid from SLAM map points"""
        # Reset grid
        self.grid.fill(-1)
        
        # Mark robot position as free
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_pose[0], robot_pose[1])
        if self.is_valid(robot_grid_x, robot_grid_y):
            cv2.circle(self.grid, (robot_grid_x, robot_grid_y), 5, 0, -1)
        
        # Update from map points
        for point in map_points:
            pos = point['position']
            x, y = pos[0], pos[1]
            
            grid_x, grid_y = self.world_to_grid(x, y)
            
            if self.is_valid(grid_x, grid_y):
                # Mark as occupied if point is near ground level
                if abs(pos[2]) < 0.5:  # Within 50cm of ground
                    self.grid[grid_y, grid_x] = 100
                else:
                    # Mark as free if above ground
                    self.grid[grid_y, grid_x] = max(0, self.grid[grid_y, grid_x])
    
    def inflate_obstacles(self, radius: int = 3):
        """Inflate obstacles for robot safety margin"""
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius*2+1, radius*2+1))
        occupied = (self.grid >= 50).astype(np.uint8) * 255
        inflated = cv2.dilate(occupied, kernel)
        self.grid[inflated > 0] = 100
    
    def visualize(self) -> np.ndarray:
        """Create visualization of occupancy grid"""
        vis = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Unknown = gray
        vis[self.grid == -1] = [128, 128, 128]
        
        # Free = white
        vis[self.grid == 0] = [255, 255, 255]
        
        # Occupied = black
        vis[self.grid >= 50] = [0, 0, 0]
        
        # Robot position = green
        cv2.circle(vis, (self.origin_x, self.origin_y), 5, (0, 255, 0), -1)
        
        return vis

class AStarPlanner:
    """A* path planning algorithm"""
    
    def __init__(self, grid: OccupancyGrid):
        self.grid = grid
        
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int, float]]:
        """Get valid neighbors with movement cost"""
        x, y = pos
        neighbors = []
        
        # 8-directional movement
        directions = [
            (0, 1, 1.0),    # Up
            (1, 0, 1.0),    # Right
            (0, -1, 1.0),   # Down
            (-1, 0, 1.0),   # Left
            (1, 1, 1.414),  # Diagonal
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414)
        ]
        
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if self.grid.is_free(nx, ny):
                neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        A* path planning
        Returns list of (x, y) grid coordinates or None if no path found
        """
        if not self.grid.is_free(start[0], start[1]):
            print("[PathPlanner] Start position is not free")
            return None
        
        if not self.grid.is_free(goal[0], goal[1]):
            print("[PathPlanner] Goal position is not free")
            return None
        
        # Priority queue: (f_score, counter, current_pos)
        counter = 0
        open_set = [(0, counter, start)]
        counter += 1
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current_f, _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1
        
        return None  # No path found
    
    def smooth_path(self, path: List[Tuple[int, int]], 
                   iterations: int = 5, weight_data: float = 0.5, 
                   weight_smooth: float = 0.3) -> List[Tuple[int, int]]:
        """Smooth path using gradient descent"""
        if len(path) <= 2:
            return path
        
        # Convert to numpy array
        smooth = np.array(path, dtype=np.float64)
        new_path = smooth.copy()
        
        for _ in range(iterations):
            for i in range(1, len(path) - 1):
                # Data term (stay close to original)
                new_path[i] += weight_data * (smooth[i] - new_path[i])
                
                # Smooth term (stay between neighbors)
                new_path[i] += weight_smooth * (new_path[i-1] + new_path[i+1] - 2 * new_path[i])
        
        # Convert back to integer coordinates
        return [(int(x), int(y)) for x, y in new_path]

class PathPlanner:
    """High-level path planning interface"""
    
    def __init__(self, grid_width: int = 200, grid_height: int = 200, 
                 resolution: float = 0.05):
        self.grid = OccupancyGrid(grid_width, grid_height, resolution)
        self.planner = AStarPlanner(self.grid)
        
        self.current_path = None
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
    
    def update_map(self, map_points: List[dict], camera_pose: np.ndarray):
        """Update occupancy grid from SLAM data"""
        # Extract position from camera pose (4x4 matrix)
        if camera_pose.shape == (4, 4):
            self.robot_pose[0] = camera_pose[0, 3]
            self.robot_pose[1] = camera_pose[1, 3]
            # Extract yaw from rotation matrix
            self.robot_pose[2] = np.arctan2(camera_pose[1, 0], camera_pose[0, 0])
        
        # Update grid
        self.grid.update_from_slam(map_points, self.robot_pose)
        self.grid.inflate_obstacles(radius=3)  # 15cm safety margin
    
    def plan_to_goal(self, goal_x: float, goal_y: float) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from current position to goal
        Returns list of (x, y) waypoints in world coordinates
        """
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        goal_grid = self.grid.world_to_grid(goal_x, goal_y)
        
        print(f"[PathPlanner] Planning from {start_grid} to {goal_grid}")
        
        # Plan path
        path_grid = self.planner.plan(start_grid, goal_grid)
        
        if path_grid is None:
            print("[PathPlanner] No path found!")
            return None
        
        # Smooth path
        path_grid = self.planner.smooth_path(path_grid)
        
        # Convert to world coordinates
        path_world = [self.grid.grid_to_world(x, y) for x, y in path_grid]
        
        self.current_path = path_world
        
        print(f"[PathPlanner] Path found with {len(path_world)} waypoints")
        return path_world
    
    def get_next_waypoint(self, lookahead_distance: float = 0.3) -> Optional[Tuple[float, float]]:
        """
        Get next waypoint for pure pursuit controller
        lookahead_distance: meters
        """
        if self.current_path is None or len(self.current_path) == 0:
            return None
        
        robot_pos = self.robot_pose[:2]
        
        # Find waypoint at lookahead distance
        for i, waypoint in enumerate(self.current_path):
            dist = np.linalg.norm(np.array(waypoint) - robot_pos)
            if dist >= lookahead_distance:
                # Remove passed waypoints
                self.current_path = self.current_path[i:]
                return waypoint
        
        # Return last waypoint if all are closer than lookahead
        return self.current_path[-1]
    
    def visualize_path(self, path: List[Tuple[float, float]] = None) -> np.ndarray:
        """Visualize occupancy grid with path"""
        vis = self.grid.visualize()
        
        if path is None:
            path = self.current_path
        
        if path is not None and len(path) > 0:
            # Draw path
            path_grid = [self.grid.world_to_grid(x, y) for x, y in path]
            
            for i in range(len(path_grid) - 1):
                pt1 = path_grid[i]
                pt2 = path_grid[i + 1]
                cv2.line(vis, pt1, pt2, (0, 255, 255), 2)
            
            # Draw goal
            if len(path_grid) > 0:
                cv2.circle(vis, path_grid[-1], 5, (0, 0, 255), -1)
        
        # Draw robot position and orientation
        robot_grid = self.grid.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        cv2.circle(vis, robot_grid, 7, (0, 255, 0), -1)
        
        # Draw orientation arrow
        arrow_length = 10
        end_x = int(robot_grid[0] + arrow_length * np.cos(self.robot_pose[2]))
        end_y = int(robot_grid[1] + arrow_length * np.sin(self.robot_pose[2]))
        cv2.arrowedLine(vis, robot_grid, (end_x, end_y), (0, 255, 0), 2)
        
        return vis

def test_path_planner():
    """Test path planning"""
    planner = PathPlanner(width=200, height=200, resolution=0.05)
    
    # Create simple map with obstacles
    map_points = []
    
    # Add wall
    for x in np.linspace(-2, 2, 40):
        map_points.append({
            'position': [x, 1.0, 0.1],
            'color': [255, 0, 0]
        })
    
    # Update map
    camera_pose = np.eye(4)
    planner.update_map(map_points, camera_pose)
    
    # Plan path
    path = planner.plan_to_goal(2.0, 2.0)
    
    if path:
        print(f"Path found: {len(path)} waypoints")
        
        # Visualize
        vis = planner.visualize_path(path)
        vis = cv2.resize(vis, (800, 800))
        
        cv2.imshow('Path Planning', vis)
        cv2.waitKey(0)
    else:
        print("No path found")

if __name__ == "__main__":
    test_path_planner()