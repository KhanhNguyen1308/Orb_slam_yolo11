#!/usr/bin/env python3
"""
Additional API endpoints for 2D navigation
Add these to x99_web_slam.py
"""

from flask import jsonify, request
import numpy as np
import heapq
from typing import List, Tuple, Optional

# Add these routes to your x99_web_slam.py Flask app

@app.route('/api/map_2d')
def get_map_2d():
    """Get 2D occupancy grid for navigation"""
    if not web_server or not web_server.persistent_map:
        return jsonify({'grid': [], 'robot_pose': [0, 0, 0]})
    
    # Get 2D occupancy map
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    
    return jsonify({
        'grid': grid.tolist(),
        'robot_pose': web_server.robot_pose.tolist(),
        'grid_size': grid.shape[0],
        'resolution': web_server.persistent_map.resolution
    })

@app.route('/api/plan_path', methods=['POST'])
def plan_path():
    """
    Plan path from start to goal using A*
    POST data: {start: [x, y], goal: [x, y]}
    """
    if not web_server or not web_server.persistent_map:
        return jsonify({'error': 'no map available'}), 503
    
    data = request.json
    start = data.get('start')
    goal = data.get('goal')
    
    if not start or not goal:
        return jsonify({'error': 'missing start or goal'}), 400
    
    # Get occupancy grid
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    
    # Plan path
    path = astar_path_planning(grid, tuple(start), tuple(goal))
    
    if path is None:
        return jsonify({'error': 'no path found'}), 404
    
    # Calculate distance
    distance = len(path) * web_server.persistent_map.resolution
    
    return jsonify({
        'path': path,
        'distance': distance,
        'num_waypoints': len(path)
    })

def astar_path_planning(grid: np.ndarray, start: Tuple[int, int], 
                       goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """
    A* path planning on occupancy grid
    
    Args:
        grid: 2D array where -1=unknown, 0=free, 100=occupied
        start: (x, y) start position in grid coordinates
        goal: (x, y) goal position in grid coordinates
    
    Returns:
        List of (x, y) waypoints or None if no path found
    """
    height, width = grid.shape
    
    # Check if start and goal are valid
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        print(f"Start {start} out of bounds")
        return None
    
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        print(f"Goal {goal} out of bounds")
        return None
    
    # Check if start or goal is occupied
    if grid[start[1], start[0]] == 100:
        print(f"Start {start} is occupied")
        return None
    
    if grid[goal[1], goal[0]] == 100:
        print(f"Goal {goal} is occupied")
        return None
    
    def heuristic(a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(pos):
        """Get valid neighbors (8-directional)"""
        x, y = pos
        neighbors = []
        
        # 8 directions: N, NE, E, SE, S, SW, W, NW
        directions = [
            (0, -1, 1.0),   # N
            (1, -1, 1.414), # NE
            (1, 0, 1.0),    # E
            (1, 1, 1.414),  # SE
            (0, 1, 1.0),    # S
            (-1, 1, 1.414), # SW
            (-1, 0, 1.0),   # W
            (-1, -1, 1.414) # NW
        ]
        
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < width and 0 <= ny < height:
                # Check if free or unknown (allow unknown for exploration)
                if grid[ny, nx] != 100:  # Not occupied
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    # A* algorithm
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    closed_set = set()
    
    while open_set:
        current_f, current = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
        
        closed_set.add(current)
        
        # Goal reached
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        
        # Explore neighbors
        for neighbor, move_cost in get_neighbors(current):
            if neighbor in closed_set:
                continue
            
            tentative_g = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # No path found
    return None

# Also add this helper function to extract clean boundaries

@app.route('/api/get_boundaries')
def get_boundaries():
    """Extract boundary edges from occupancy grid"""
    if not web_server or not web_server.persistent_map:
        return jsonify({'boundaries': []})
    
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    boundaries = extract_boundaries(grid)
    
    return jsonify({
        'boundaries': boundaries,
        'num_segments': len(boundaries)
    })

def extract_boundaries(grid: np.ndarray) -> List[List[Tuple[int, int]]]:
    """
    Extract boundary contours from occupancy grid
    
    Returns:
        List of boundary segments, each segment is a list of (x, y) points
    """
    import cv2
    
    # Create binary image: 255 for occupied, 0 for free/unknown
    binary = np.zeros_like(grid, dtype=np.uint8)
    binary[grid == 100] = 255
    
    # Find contours
    contours, hierarchy = cv2.findContours(
        binary, 
        cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE
    )
    
    # Convert contours to list of points
    boundaries = []
    for contour in contours:
        if len(contour) > 10:  # Filter small contours
            points = [(int(pt[0][0]), int(pt[0][1])) for pt in contour]
            boundaries.append(points)
    
    return boundaries