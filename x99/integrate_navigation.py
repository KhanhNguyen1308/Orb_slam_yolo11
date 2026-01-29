#!/usr/bin/env python3
"""
Auto-integrate navigation API into x99_web_slam.py
"""

import os
import sys

NAVIGATION_CODE = '''
# ========== 2D NAVIGATION API ==========
# Added by integrate_navigation.py

import heapq
from typing import List, Tuple, Optional

@app.route('/api/map_2d')
def get_map_2d():
    """Get 2D occupancy grid for navigation"""
    if not web_server or not web_server.persistent_map:
        return jsonify({'grid': [], 'robot_pose': [0, 0, 0]})
    
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
    
    grid = web_server.persistent_map.get_2d_map(normalize=True)
    path = astar_path_planning(grid, tuple(start), tuple(goal))
    
    if path is None:
        return jsonify({'error': 'no path found'}), 404
    
    distance = len(path) * web_server.persistent_map.resolution
    
    return jsonify({
        'path': path,
        'distance': distance,
        'num_waypoints': len(path)
    })

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

def astar_path_planning(grid: np.ndarray, start: Tuple[int, int], 
                       goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """
    A* path planning on occupancy grid
    """
    height, width = grid.shape
    
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        print(f"[PathPlanning] Start {start} out of bounds")
        return None
    
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        print(f"[PathPlanning] Goal {goal} out of bounds")
        return None
    
    if grid[start[1], start[0]] == 100:
        print(f"[PathPlanning] Start {start} is occupied")
        return None
    
    if grid[goal[1], goal[0]] == 100:
        print(f"[PathPlanning] Goal {goal} is occupied")
        return None
    
    def heuristic(a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(pos):
        x, y = pos
        neighbors = []
        directions = [
            (0, -1, 1.0), (1, -1, 1.414), (1, 0, 1.0), (1, 1, 1.414),
            (0, 1, 1.0), (-1, 1, 1.414), (-1, 0, 1.0), (-1, -1, 1.414)
        ]
        
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                if grid[ny, nx] != 100:
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
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
        
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            print(f"[PathPlanning] Found path with {len(path)} waypoints")
            return path
        
        for neighbor, move_cost in get_neighbors(current):
            if neighbor in closed_set:
                continue
            
            tentative_g = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    print("[PathPlanning] No path found")
    return None

def extract_boundaries(grid: np.ndarray) -> List[List[Tuple[int, int]]]:
    """Extract boundary contours from occupancy grid"""
    import cv2
    
    binary = np.zeros_like(grid, dtype=np.uint8)
    binary[grid == 100] = 255
    
    contours, hierarchy = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    
    boundaries = []
    for contour in contours:
        if len(contour) > 10:
            points = [(int(pt[0][0]), int(pt[0][1])) for pt in contour]
            boundaries.append(points)
    
    return boundaries

# ========== END NAVIGATION API ==========
'''

def integrate():
    """Integrate navigation API into x99_web_slam.py"""
    
    print("=" * 70)
    print("  NAVIGATION API INTEGRATION")
    print("=" * 70)
    
    # Check if file exists
    if not os.path.exists('x99_web_slam.py'):
        print("\n✗ Error: x99_web_slam.py not found in current directory")
        print("  Please run this script from the same directory as x99_web_slam.py")
        return False
    
    # Read existing file
    with open('x99_web_slam.py', 'r') as f:
        content = f.read()
    
    # Check if already integrated
    if '2D NAVIGATION API' in content:
        print("\n⚠ Navigation API already integrated!")
        print("  Skipping...")
        return True
    
    # Find insertion point (before if __name__ == '__main__':)
    if "if __name__ == '__main__':" in content:
        parts = content.split("if __name__ == '__main__':")
        new_content = parts[0] + NAVIGATION_CODE + "\\nif __name__ == '__main__':" + parts[1]
    else:
        # Append to end
        new_content = content + "\\n" + NAVIGATION_CODE
    
    # Backup original
    backup_file = 'x99_web_slam.py.backup'
    print(f"\n✓ Creating backup: {backup_file}")
    with open(backup_file, 'w') as f:
        f.write(content)
    
    # Write new file
    print("✓ Integrating navigation API...")
    with open('x99_web_slam.py', 'w') as f:
        f.write(new_content)
    
    print("✓ Integration complete!")
    
    # Copy HTML template
    if os.path.exists('slam_navigation_2d.html'):
        print("\\n✓ Copying HTML template to templates/")
        os.makedirs('templates', exist_ok=True)
        
        with open('slam_navigation_2d.html', 'r') as f:
            html_content = f.read()
        
        with open('templates/slam_navigation_2d.html', 'w') as f:
            f.write(html_content)
        
        print("✓ HTML template installed")
    
    print("\\n" + "=" * 70)
    print("  INTEGRATION SUCCESSFUL")
    print("=" * 70)
    print("\\nNew endpoints added:")
    print("  • GET  /api/map_2d - Get 2D occupancy grid")
    print("  • POST /api/plan_path - Plan path from start to goal")
    print("  • GET  /api/get_boundaries - Extract boundary edges")
    print("\\nNew interface:")
    print(f"  • http://localhost:5000/slam_navigation_2d.html")
    print("\\nBackup created:")
    print(f"  • {backup_file}")
    print("\\nRestart server to apply changes:")
    print("  python3 x99_web_slam.py")
    print("=" * 70)
    
    return True

if __name__ == '__main__':
    try:
        success = integrate()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\\n✗ Integration failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)