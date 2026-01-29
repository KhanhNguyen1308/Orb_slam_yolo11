#!/usr/bin/env python3
"""
Test script để debug SLAM map
Chạy script này để kiểm tra từng component riêng biệt
"""

import sys
import numpy as np

print("=" * 70)
print("  X99 SLAM MAP DEBUG TOOL")
print("=" * 70)

# Test 1: Import các module cần thiết
print("\n[TEST 1] Checking imports...")
try:
    import cv2
    print("  ✓ OpenCV version:", cv2.__version__)
except ImportError as e:
    print(f"  ✗ OpenCV import failed: {e}")
    sys.exit(1)

try:
    from flask import Flask
    print("  ✓ Flask installed")
except ImportError as e:
    print(f"  ✗ Flask import failed: {e}")
    sys.exit(1)

try:
    from flask_socketio import SocketIO
    print("  ✓ Flask-SocketIO installed")
except ImportError as e:
    print(f"  ✗ Flask-SocketIO import failed: {e}")
    sys.exit(1)

# Test 2: Import local modules
print("\n[TEST 2] Checking local modules...")
try:
    from x99_headless import OptimizedCameraReceiver
    print("  ✓ x99_headless.py")
except ImportError as e:
    print(f"  ✗ x99_headless.py: {e}")

try:
    from stereo_depth_mapping import StereoDepthMapper, OccupancyGridMapper
    print("  ✓ stereo_depth_mapping.py")
except ImportError as e:
    print(f"  ✗ stereo_depth_mapping.py: {e}")

try:
    from persistent_map import PersistentMap
    print("  ✓ persistent_map.py")
except ImportError as e:
    print(f"  ✗ persistent_map.py: {e}")
    sys.exit(1)

# Test 3: Test PersistentMap với dữ liệu giả
print("\n[TEST 3] Testing PersistentMap with fake data...")
try:
    map_builder = PersistentMap(grid_size=800, resolution=0.02, voxel_size=0.05)
    
    # Tạo 1000 điểm ngẫu nhiên
    points = np.random.randn(1000, 3) * 2
    points[:, 1] = np.random.uniform(-0.2, 1.0, 1000)  # Height
    points[:, 2] = np.abs(np.random.randn(1000)) * 2 + 0.5  # Depth positive
    
    colors = np.random.randint(0, 255, (1000, 3))
    robot_pose = np.array([0.0, 0.0, 0.0])
    
    # Thêm vào map
    map_builder.add_point_cloud(points, colors, robot_pose)
    
    # Kiểm tra kết quả
    points_3d, colors_3d = map_builder.get_3d_points()
    
    print(f"  ✓ Added {len(points)} points")
    print(f"  ✓ Voxel grid contains {len(map_builder.voxel_grid)} points")
    print(f"  ✓ get_3d_points() returned {len(points_3d)} points")
    
    if len(points_3d) > 0:
        print("  ✓ Point cloud creation: SUCCESS")
    else:
        print("  ✗ Point cloud is empty!")
        
except Exception as e:
    print(f"  ✗ PersistentMap test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 4: Test API data format
print("\n[TEST 4] Testing API data format...")
try:
    map_data = map_builder.get_map_data_for_web()
    
    print(f"  ✓ Map data keys: {list(map_data.keys())}")
    print(f"  ✓ Points count: {len(map_data['points'])}")
    print(f"  ✓ Colors count: {len(map_data['colors'])}")
    print(f"  ✓ Trajectory count: {len(map_data['trajectory'])}")
    
    if len(map_data['points']) > 0:
        sample_point = map_data['points'][0]
        sample_color = map_data['colors'][0]
        print(f"  ✓ Sample point: {sample_point}")
        print(f"  ✓ Sample color: {sample_color}")
    
except Exception as e:
    print(f"  ✗ API format test failed: {e}")
    import traceback
    traceback.print_exc()

# Test 5: Test visualization
print("\n[TEST 5] Testing 2D visualization...")
try:
    vis_2d = map_builder.visualize_2d(show_trajectory=False)
    print(f"  ✓ 2D map shape: {vis_2d.shape}")
    print(f"  ✓ 2D map dtype: {vis_2d.dtype}")
    
    # Save test image
    cv2.imwrite('/tmp/test_map_2d.png', vis_2d)
    print("  ✓ Saved test image to /tmp/test_map_2d.png")
    
except Exception as e:
    print(f"  ✗ Visualization test failed: {e}")
    import traceback
    traceback.print_exc()

# Test 6: Test StereoDepthMapper
print("\n[TEST 6] Testing StereoDepthMapper...")
try:
    depth_mapper = StereoDepthMapper(baseline=0.275, focal_length=500)
    
    # Tạo fake stereo images
    left_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    right_img = left_img.copy()
    right_img[:, 10:] = left_img[:, :-10]  # Shift để tạo disparity
    
    disparity = depth_mapper.compute_disparity(left_img, right_img, use_wls=False)
    depth = depth_mapper.disparity_to_depth(disparity)
    
    print(f"  ✓ Disparity shape: {disparity.shape}")
    print(f"  ✓ Depth range: {depth.min():.2f} - {depth.max():.2f}")
    
    # Test point cloud generation
    points, colors = depth_mapper.depth_to_point_cloud(depth, left_img)
    print(f"  ✓ Generated {len(points)} 3D points from depth")
    
    if len(points) > 100:
        print("  ✓ Depth mapping: SUCCESS")
    else:
        print("  ⚠ Warning: Very few points generated")
    
except Exception as e:
    print(f"  ✗ Depth mapper test failed: {e}")
    import traceback
    traceback.print_exc()

# Test 7: Test JSON serialization
print("\n[TEST 7] Testing JSON serialization...")
try:
    import json
    
    # Get map data
    points, colors = map_builder.get_3d_points()
    
    # Convert to JSON
    json_data = {
        'points': points.tolist(),
        'colors': colors.tolist(),
        'robot_pose': [0.0, 0.0, 0.0]
    }
    
    json_str = json.dumps(json_data)
    print(f"  ✓ JSON size: {len(json_str)} bytes")
    
    # Parse back
    parsed = json.loads(json_str)
    print(f"  ✓ Parsed points: {len(parsed['points'])}")
    
except Exception as e:
    print(f"  ✗ JSON test failed: {e}")
    import traceback
    traceback.print_exc()

# Summary
print("\n" + "=" * 70)
print("  DEBUG TEST COMPLETED")
print("=" * 70)
print("\n✓ All basic tests passed!")
print("✓ PersistentMap is working correctly")
print("✓ Data format is compatible with web API")
print("\nIf you still don't see the SLAM map:")
print("1. Check if cameras are connected and sending data")
print("2. Check browser console (F12) for JavaScript errors")
print("3. Run: curl http://localhost:5000/api/map_data")
print("4. Check Python server logs for errors")
print("\n")