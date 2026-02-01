#x99/persistent_map.py
#!/usr/bin/env python3
"""
Persistent Map Builder
Accumulates 3D points and creates persistent 2D/3D maps
"""

import numpy as np
import cv2
from typing import List, Tuple, Dict
import time
import threading
import json

class PersistentMap:
    """Persistent 3D point cloud and 2D occupancy map"""
    
    def __init__(self, grid_size: int = 800, resolution: float = 0.02, 
                 voxel_size: float = 0.05):
        """
        grid_size: Size of 2D grid in cells
        resolution: Meters per cell for 2D grid (2cm)
        voxel_size: Voxel size for 3D downsampling (5cm)
        """
        self.grid_size = grid_size
        self.resolution = resolution
        self.voxel_size = voxel_size
        
        # 2D occupancy grid
        self.grid_2d = np.zeros((grid_size, grid_size), dtype=np.int16)
        self.grid_counts = np.zeros((grid_size, grid_size), dtype=np.int16)
        
        # 3D point cloud (downsampled)
        self.points_3d = []  # List of [x, y, z]
        self.colors_3d = []  # List of [r, g, b]
        
        # Voxel grid for downsampling
        self.voxel_grid = {}  # {(vx, vy, vz): [point, color]}
        
        # Robot trajectory
        self.trajectory = []  # List of [x, y, z, timestamp]
        
        # Origin (center of grid)
        self.origin_x = grid_size // 2
        self.origin_y = grid_size // 2
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Stats
        self.total_points_added = 0
        self.last_update_time = time.time()
    
    def world_to_grid_2d(self, x: float, z: float) -> Tuple[int, int]:
        """Convert world XZ to 2D grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin_x
        grid_z = int(-z / self.resolution) + self.origin_y
        return grid_x, grid_z
    
    def world_to_voxel(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """Convert world coordinates to voxel grid"""
        vx = int(x / self.voxel_size)
        vy = int(y / self.voxel_size)
        vz = int(z / self.voxel_size)
        return vx, vy, vz
    
    def add_point_cloud(self, points: np.ndarray, colors: np.ndarray = None,
                       robot_pose: np.ndarray = None):
        """
        Add point cloud to persistent map
        points: Nx3 array of [x, y, z]
        colors: Nx3 array of [r, g, b]
        robot_pose: Current robot position [x, y, z]
        """
        with self.lock:
            for i, point in enumerate(points):
                x, y, z = point
                
                # Filter out invalid points
                if abs(y) > 3.0 or z < 0.1 or z > 10.0:
                    continue
                
                # Add to 3D voxel grid (downsampled)
                voxel_key = self.world_to_voxel(x, y, z)
                
                if voxel_key not in self.voxel_grid:
                    color = colors[i] if colors is not None else [128, 128, 128]
                    self.voxel_grid[voxel_key] = [point.copy(), color]
                    self.total_points_added += 1
                
                # Add to 2D occupancy grid
                grid_x, grid_z = self.world_to_grid_2d(x, z)
                
                if 0 <= grid_x < self.grid_size and 0 <= grid_z < self.grid_size:
                    # Classify as ground or obstacle based on height
                    if -0.3 <= y <= -0.05:  # Ground
                        self.grid_2d[grid_z, grid_x] -= 1  # Decrease (more free)
                    elif -0.05 <= y <= 2.0:  # Obstacle
                        self.grid_2d[grid_z, grid_x] += 2  # Increase (more occupied)
                    
                    self.grid_counts[grid_z, grid_x] += 1
            
            # Update trajectory
            if robot_pose is not None:
                self.trajectory.append([
                    robot_pose[0], robot_pose[1], robot_pose[2],
                    time.time()
                ])
                
                # Keep last 1000 poses
                if len(self.trajectory) > 1000:
                    self.trajectory = self.trajectory[-1000:]
            
            self.last_update_time = time.time()
    
    def get_2d_map(self, normalize: bool = True) -> np.ndarray:
        """Get 2D occupancy map"""
        with self.lock:
            if normalize:
                # Normalize grid values to probability [0, 100]
                grid_copy = self.grid_2d.copy().astype(np.float32)
                
                # Apply threshold
                grid_normalized = np.zeros_like(grid_copy, dtype=np.int8)
                
                # Free space (negative values)
                grid_normalized[grid_copy < -5] = 0
                
                # Occupied space (positive values)
                grid_normalized[grid_copy > 10] = 100
                
                # Unknown (middle values or no observations)
                mask = (grid_copy >= -5) & (grid_copy <= 10)
                grid_normalized[mask] = -1
                
                # Cells with no observations
                grid_normalized[self.grid_counts == 0] = -1
                
                return grid_normalized
            else:
                return self.grid_2d.copy()
    
    def get_3d_points(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get downsampled 3D point cloud"""
        with self.lock:
            if len(self.voxel_grid) == 0:
                return np.array([]), np.array([])
            
            points = []
            colors = []
            
            for voxel_key, (point, color) in self.voxel_grid.items():
                points.append(point)
                colors.append(color)
            
            return np.array(points), np.array(colors)
    
    def get_trajectory(self) -> List[List[float]]:
        """Get robot trajectory"""
        with self.lock:
            return self.trajectory.copy()
    
    def visualize_2d(self, show_trajectory: bool = True) -> np.ndarray:
        """Create 2D map visualization"""
        grid = self.get_2d_map(normalize=True)
        
        # Create color image
        vis = np.zeros((self.grid_size, self.grid_size, 3), dtype=np.uint8)
        
        # Unknown = gray
        vis[grid == -1] = [128, 128, 128]
        
        # Free = white
        vis[grid == 0] = [255, 255, 255]
        
        # Occupied = black
        vis[grid == 100] = [0, 0, 0]
        
        # Draw trajectory
        if show_trajectory and len(self.trajectory) > 1:
            trajectory_2d = []
            for pose in self.trajectory:
                x, y, z, _ = pose
                grid_x, grid_z = self.world_to_grid_2d(x, z)
                
                if 0 <= grid_x < self.grid_size and 0 <= grid_z < self.grid_size:
                    trajectory_2d.append([grid_x, grid_z])
            
            # Draw trajectory line
            if len(trajectory_2d) > 1:
                pts = np.array(trajectory_2d, dtype=np.int32)
                cv2.polylines(vis, [pts], False, (0, 255, 0), 2)
            
            # Draw current position
            if len(trajectory_2d) > 0:
                current = trajectory_2d[-1]
                cv2.circle(vis, tuple(current), 8, (0, 255, 0), -1)
                
                # Draw orientation
                if len(trajectory_2d) > 1:
                    prev = trajectory_2d[-2]
                    cv2.arrowedLine(vis, tuple(prev), tuple(current), (0, 255, 0), 2)
        
        return vis
    
    def get_map_data_for_web(self) -> Dict:
        """Get map data formatted for web visualization"""
        points, colors = self.get_3d_points()
        trajectory = self.get_trajectory()
        
        # Convert to lists for JSON
        points_list = points.tolist() if len(points) > 0 else []
        colors_list = colors.tolist() if len(colors) > 0 else []
        
        # Downsample for web (max 10000 points)
        if len(points_list) > 10000:
            indices = np.random.choice(len(points_list), 10000, replace=False)
            points_list = [points_list[i] for i in indices]
            colors_list = [colors_list[i] for i in indices]
        
        # Trajectory (last 100 poses)
        trajectory_list = trajectory[-100:] if len(trajectory) > 100 else trajectory
        
        return {
            'points': points_list,
            'colors': colors_list,
            'trajectory': trajectory_list,
            'stats': {
                'total_points': len(self.voxel_grid),
                'total_trajectory_points': len(trajectory),
                'grid_size': self.grid_size,
                'resolution': self.resolution,
                'last_update': self.last_update_time
            }
        }
    
    def save_map(self, filename: str = "map.npz"):
        """Save map to file"""
        with self.lock:
            points, colors = self.get_3d_points()
            
            np.savez_compressed(
                filename,
                points=points,
                colors=colors,
                grid_2d=self.grid_2d,
                grid_counts=self.grid_counts,
                trajectory=np.array(self.trajectory),
                grid_size=self.grid_size,
                resolution=self.resolution
            )
            
            print(f"[Map] Saved to {filename}")
    
    def load_map(self, filename: str = "map.npz"):
        """Load map from file"""
        try:
            data = np.load(filename)
            
            with self.lock:
                # Load 2D grid
                self.grid_2d = data['grid_2d']
                self.grid_counts = data['grid_counts']
                self.grid_size = int(data['grid_size'])
                self.resolution = float(data['resolution'])
                
                # Rebuild voxel grid
                points = data['points']
                colors = data['colors']
                
                self.voxel_grid.clear()
                for i, point in enumerate(points):
                    voxel_key = self.world_to_voxel(point[0], point[1], point[2])
                    self.voxel_grid[voxel_key] = [point, colors[i]]
                
                # Load trajectory
                self.trajectory = data['trajectory'].tolist()
                
                print(f"[Map] Loaded from {filename}")
                print(f"  Points: {len(points)}")
                print(f"  Trajectory: {len(self.trajectory)} poses")
                
                return True
                
        except Exception as e:
            print(f"[Map] Load failed: {e}")
            return False
    
    def clear_map(self):
        """Clear entire map"""
        with self.lock:
            self.grid_2d.fill(0)
            self.grid_counts.fill(0)
            self.voxel_grid.clear()
            self.trajectory.clear()
            self.total_points_added = 0
            print("[Map] Cleared")

def test_persistent_map():
    """Test persistent map"""
    
    # Create map
    map_builder = PersistentMap(grid_size=800, resolution=0.02)
    
    # Simulate adding points
    for i in range(100):
        # Random points
        n_points = 1000
        points = np.random.randn(n_points, 3) * 2
        points[:, 1] = np.random.uniform(-0.2, 1.0, n_points)  # Height
        points[:, 2] = np.random.uniform(0.5, 5.0, n_points)  # Depth
        
        colors = np.random.randint(0, 255, (n_points, 3))
        
        # Robot pose
        robot_pose = [i * 0.1, 0, i * 0.1]
        
        # Add to map
        map_builder.add_point_cloud(points, colors, robot_pose)
        
        # Visualize every 10 iterations
        if i % 10 == 0:
            vis_2d = map_builder.visualize_2d(show_trajectory=True)
            vis_2d = cv2.resize(vis_2d, (800, 800))
            
            cv2.imshow('Persistent 2D Map', vis_2d)
            cv2.waitKey(1)
            
            print(f"Iteration {i}: {len(map_builder.voxel_grid)} points in map")
    
    # Save map
    map_builder.save_map("test_map.npz")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_persistent_map()