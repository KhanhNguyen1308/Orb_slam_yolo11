# 2D NAVIGATION MAP - Hướng dẫn tích hợp

## TỔNG QUAN

Thay vì hiển thị 3D point cloud phức tạp, hệ thống mới này:
- ✅ Hiển thị **2D occupancy grid** rõ ràng
- ✅ **Edge detection** để tìm đường biên
- ✅ **A* path planning** để tìm đường đi
- ✅ Click trên map để đặt start/goal
- ✅ Visualize path planning trực quan

## CÀI ĐẶT

### Bước 1: Thêm API endpoints vào x99_web_slam.py

Mở file `x99_web_slam.py` và thêm code từ `navigation_api.py`:

```python
# Thêm vào cuối file x99_web_slam.py, trước if __name__ == '__main__':

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
    """Plan path using A*"""
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

# Copy toàn bộ function astar_path_planning từ navigation_api.py
```

### Bước 2: Copy HTML vào templates

```bash
cp slam_navigation_2d.html templates/
```

### Bước 3: Truy cập interface mới

```
http://<X99_IP>:5000/slam_navigation_2d.html
```

## CÁCH SỬ DỤNG

### 1. Xem Occupancy Grid
- Click "Occupancy" mode
- Trắng = free space (robot có thể đi)
- Đen = obstacle (không thể đi qua)
- Xám = unknown (chưa khám phá)

### 2. Xem Edges (Đường biên)
- Click "Edges" mode
- Đỏ = edges/boundaries (ranh giới giữa free và occupied)
- Đây là những đường biên mà robot cần tránh

### 3. Path Planning
- Click "Path" mode
- Click "Set Start" → Click vị trí trên map
- Click "Set Goal" → Click đích đến
- Click "Plan Path" → Hệ thống tính đường đi
- Cyan = đường đi được plan

### 4. Controls
- **🔄 Update Map**: Cập nhật map mới nhất
- **🗑️ Clear Map**: Xóa map hiện tại
- **💾 Save Map**: Lưu map ra file
- **📍 Set Start**: Đặt điểm bắt đầu
- **🎯 Set Goal**: Đặt điểm đích
- **🧭 Plan Path**: Tính đường đi A*

## THUẬT TOÁN

### A* Path Planning

Thuật toán tìm đường đi ngắn nhất từ start → goal:

```
1. Open set = {start}
2. While open set not empty:
   - Pick node with lowest f_score
   - If node == goal → reconstruct path
   - For each neighbor:
     - If not occupied:
       - Calculate g_score (distance from start)
       - Calculate h_score (estimated distance to goal)
       - f_score = g_score + h_score
       - Add to open set
3. Return path or None
```

### Edge Detection

Sử dụng Sobel operator để tìm đường biên:

```
Gx = [-1  0  1]     Gy = [-1 -2 -1]
     [-2  0  2]          [ 0  0  0]
     [-1  0  1]          [ 1  2  1]

Magnitude = sqrt(Gx² + Gy²)
Edge if magnitude > threshold
```

## TÍCH HỢP VỚI NAVIGATION

### 1. Get current path

```python
# API call
import requests

response = requests.post('http://localhost:5000/api/plan_path', 
    json={'start': [400, 400], 'goal': [450, 350]})

path = response.json()['path']
# path = [(400,400), (401,399), ..., (450,350)]
```

### 2. Convert path to robot commands

```python
def follow_path(path, resolution=0.02):
    """Convert grid path to robot waypoints"""
    waypoints = []
    
    for (grid_x, grid_y) in path:
        # Convert grid coordinates to world coordinates
        world_x = (grid_x - 400) * resolution  # 400 = center
        world_y = -(grid_y - 400) * resolution
        
        waypoints.append([world_x, world_y])
    
    return waypoints

# Usage
waypoints = follow_path(path)
# waypoints = [[0.0, 0.0], [0.02, -0.02], ...]

# Send to robot controller
for waypoint in waypoints:
    move_to_waypoint(waypoint[0], waypoint[1])
```

### 3. Dynamic replanning

```python
def navigation_loop():
    while not reached_goal:
        # Update map
        grid = get_map_2d()
        
        # Get current position
        current_pos = get_robot_position()
        
        # Replan if obstacle detected
        path = plan_path(current_pos, goal)
        
        if path:
            # Follow next waypoint
            next_waypoint = path[1]  # path[0] is current
            move_to_waypoint(next_waypoint)
        else:
            # No path found - stop
            stop_robot()
            break
        
        time.sleep(0.1)
```

## TUNING PARAMETERS

### Occupancy Grid

```python
# Trong persistent_map.py
if -0.3 <= y <= -0.05:  # Ground threshold
    self.grid_2d[grid_z, grid_x] -= 1

elif -0.05 <= y <= 2.0:  # Obstacle threshold
    self.grid_2d[grid_z, grid_x] += 2
```

Điều chỉnh:
- Ground threshold: Điểm nào được coi là mặt đất
- Obstacle threshold: Điểm nào được coi là vật cản
- Update weight: Tốc độ cập nhật grid

### Path Planning

```python
# Safety margin
safety_cells = 3  # Tránh xa obstacle 3 cells

# Inflate obstacles
for obstacle in obstacles:
    for r in range(safety_cells):
        mark_neighbors_as_occupied(obstacle, radius=r)
```

### Edge Detection

```python
# Sobel threshold
edge_threshold = 50  # Tăng = ít edge hơn, giảm = nhiều edge hơn
```

## TROUBLESHOOTING

### Vấn đề 1: Không thấy map
**Nguyên nhân**: API /api/map_2d không trả về data

**Kiểm tra**:
```bash
curl http://localhost:5000/api/map_2d | python3 -m json.tool | head -20
```

**Sửa**: Đảm bảo SLAM processing đang chạy

### Vấn đề 2: Path planning thất bại
**Nguyên nhân**: Start hoặc goal nằm trong obstacle

**Kiểm tra**: Click vào vùng free (trắng) trên map

### Vấn đề 3: Edge detection không rõ
**Nguyên nhân**: Threshold quá cao hoặc quá thấp

**Sửa**: 
```javascript
// Trong slam_navigation_2d.html
const magnitude = Math.sqrt(gx*gx + gy*gy);
edgeData[y][x] = magnitude > 30 ? 1 : 0;  // Giảm từ 50 xuống 30
```

### Vấn đề 4: Map cập nhật chậm
**Nguyên nhân**: Auto-update 2 giây

**Sửa**:
```javascript
// Trong slam_navigation_2d.html
setInterval(updateMap, 1000);  // Giảm từ 2000 xuống 1000
```

## SO SÁNH 3D vs 2D

| Feature | 3D Point Cloud | 2D Occupancy Grid |
|---------|---------------|-------------------|
| Hiển thị | Phức tạp, nhiều điểm | Đơn giản, rõ ràng |
| Path planning | Khó | Dễ (A*) |
| CPU usage | Cao | Thấp |
| Dùng cho | Visualization | Navigation |

## KẾT LUẬN

Hệ thống 2D navigation map giúp:
- ✅ Thấy rõ đường biên và obstacle
- ✅ Dễ dàng plan path
- ✅ Hiệu suất cao hơn 3D
- ✅ Phù hợp cho robot navigation

**Khuyến nghị**: Dùng 2D map cho navigation, giữ 3D cho debug và visualization!

---
Version: 1.0
Created: 2025-01-29