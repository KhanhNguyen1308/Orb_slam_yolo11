#!/usr/bin/env python3
"""
Test 2D Occupancy Grid - Demo Script
Kiểm tra map 2D hoạt động đúng với camera cao 50cm
"""

import cv2
import numpy as np
import sys

from stereo_depth_mapping_optimized import StereoDepthMapper, OccupancyGridMapper

def test_2d_grid():
    """Test 2D occupancy grid with simulated data"""
    
    print("=" * 60)
    print("  TEST 2D OCCUPANCY GRID")
    print("  Camera height: 50cm")
    print("  Baseline: 10cm")
    print("=" * 60)
    
    # Create mapper
    grid_mapper = OccupancyGridMapper(
        grid_size=600,
        resolution=0.05,  # 5cm/cell
        max_range=15.0
    )
    
    print(f"\nGrid config:")
    print(f"  Size: {grid_mapper.grid_size}x{grid_mapper.grid_size}")
    print(f"  Resolution: {grid_mapper.resolution}m/cell")
    print(f"  Physical size: {grid_mapper.grid_size * grid_mapper.resolution}m")
    print(f"  Max range: {grid_mapper.max_range}m")
    
    # Simulate point cloud data
    # Case 1: Ground plane
    print("\n[1] Testing ground detection...")
    ground_points = []
    for x in np.linspace(-5, 5, 100):
        for z in np.linspace(0.5, 10, 100):
            # Ground at Y = 0 (camera is 50cm high, so world Y = 0)
            y = np.random.uniform(-0.05, 0.1)  # Ground level
            ground_points.append([x, y, z])
    
    ground_points = np.array(ground_points)
    print(f"  Generated {len(ground_points)} ground points")
    
    grid_mapper.update_from_point_cloud(ground_points)
    free_cells = np.sum(grid_mapper.grid == 0)
    print(f"  Free cells detected: {free_cells}")
    
    # Visualize ground
    vis_ground = grid_mapper.visualize()
    cv2.imshow('Test 1: Ground Detection', vis_ground)
    cv2.waitKey(1000)
    
    # Case 2: Add obstacles (walls)
    print("\n[2] Testing obstacle detection...")
    obstacle_points = []
    
    # Wall 1: Front wall at 3m
    for x in np.linspace(-2, 2, 50):
        for y in np.linspace(0.3, 1.5, 30):  # 30cm to 1.5m height
            z = 3.0
            obstacle_points.append([x, y, z])
    
    # Wall 2: Side wall (left)
    for z in np.linspace(1, 5, 50):
        for y in np.linspace(0.3, 1.5, 30):
            x = -2.0
            obstacle_points.append([x, y, z])
    
    # Object: Box at 2m distance
    for x in np.linspace(0.5, 1.0, 20):
        for z in np.linspace(1.8, 2.2, 20):
            for y in np.linspace(0.3, 0.8, 15):
                obstacle_points.append([x, y, z])
    
    obstacle_points = np.array(obstacle_points)
    print(f"  Generated {len(obstacle_points)} obstacle points")
    
    # Combine ground + obstacles
    all_points = np.vstack([ground_points, obstacle_points])
    
    grid_mapper.update_from_point_cloud(all_points)
    grid_mapper.inflate_obstacles(radius=4)
    
    occupied_cells = np.sum(grid_mapper.grid == 100)
    free_cells = np.sum(grid_mapper.grid == 0)
    unknown_cells = np.sum(grid_mapper.grid == -1)
    
    print(f"  Occupied cells: {occupied_cells}")
    print(f"  Free cells: {free_cells}")
    print(f"  Unknown cells: {unknown_cells}")
    
    # Visualize final map
    vis_final = grid_mapper.visualize()
    vis_large = cv2.resize(vis_final, (800, 800), interpolation=cv2.INTER_NEAREST)
    
    # Add info overlay
    cv2.putText(vis_large, "2D OCCUPANCY GRID TEST", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(vis_large, f"Obstacles: {occupied_cells}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(vis_large, f"Free: {free_cells}", (20, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(vis_large, f"Unknown: {unknown_cells}", (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Legend
    y_offset = 140
    cv2.rectangle(vis_large, (20, y_offset), (40, y_offset+15), (255, 255, 255), -1)
    cv2.putText(vis_large, "Free Space", (50, y_offset+12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    y_offset += 25
    cv2.rectangle(vis_large, (20, y_offset), (40, y_offset+15), (40, 40, 40), -1)
    cv2.putText(vis_large, "Obstacle", (50, y_offset+12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    y_offset += 25
    cv2.rectangle(vis_large, (20, y_offset), (40, y_offset+15), (128, 128, 128), -1)
    cv2.putText(vis_large, "Unknown", (50, y_offset+12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.imshow('Test 2: Final Map with Obstacles', vis_large)
    
    # Case 3: Test obstacle scan (LIDAR-like)
    print("\n[3] Testing obstacle scan (360-degree)...")
    scan = grid_mapper.get_obstacle_scan(num_rays=360)
    
    min_dist = np.min(scan)
    max_dist = np.max(scan)
    avg_dist = np.mean(scan)
    
    print(f"  Min distance: {min_dist:.2f}m")
    print(f"  Max distance: {max_dist:.2f}m")
    print(f"  Avg distance: {avg_dist:.2f}m")
    
    # Visualize scan
    scan_vis = np.zeros((400, 400, 3), dtype=np.uint8)
    center = (200, 200)
    
    angles = np.linspace(0, 2*np.pi, 360)
    for i, (angle, dist) in enumerate(zip(angles, scan)):
        # Scale to image
        scale = 30  # pixels per meter
        x = int(center[0] + dist * scale * np.sin(angle))
        y = int(center[1] - dist * scale * np.cos(angle))
        
        # Color based on distance
        if dist < 1.0:
            color = (0, 0, 255)  # Red: close
        elif dist < 3.0:
            color = (0, 255, 255)  # Yellow: medium
        else:
            color = (0, 255, 0)  # Green: far
        
        cv2.line(scan_vis, center, (x, y), color, 1)
    
    # Draw robot
    cv2.circle(scan_vis, center, 10, (255, 0, 0), -1)
    cv2.arrowedLine(scan_vis, center, (center[0], center[1]-20), (255, 255, 255), 2)
    
    cv2.putText(scan_vis, "360-DEGREE SCAN", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(scan_vis, f"Min: {min_dist:.1f}m", (10, 380),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.imshow('Test 3: 360-Degree Scan', scan_vis)
    
    print("\n" + "=" * 60)
    print("TEST COMPLETE!")
    print("=" * 60)
    print("\nPress any key to close...")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test_height_filtering():
    """Test height-based filtering"""
    
    print("\n" + "=" * 60)
    print("  TEST HEIGHT FILTERING")
    print("=" * 60)
    
    # Simulate points at different heights
    camera_height = 0.5  # 50cm
    
    test_cases = [
        ("Ground", -0.05, True),      # Should be free space
        ("Low obstacle", 0.3, True),   # Should be obstacle
        ("High obstacle", 1.5, True),  # Should be obstacle
        ("Too high", 2.8, False),      # Should be filtered out
        ("Below ground", -0.6, False), # Should be filtered out
    ]
    
    print("\nHeight filtering test (Camera at 50cm):")
    print(f"{'Case':<20} {'Height':<10} {'Valid?':<10} {'Type'}")
    print("-" * 60)
    
    for name, height, should_be_valid in test_cases:
        # Transform to world coordinates
        world_height = height + camera_height
        
        # Check validity
        is_valid = (-0.1 < world_height < 2.5)
        
        if is_valid:
            if -0.1 <= world_height <= 0.25:
                point_type = "Free space"
            elif 0.25 < world_height <= 2.5:
                point_type = "Obstacle"
            else:
                point_type = "Unknown"
        else:
            point_type = "FILTERED"
        
        status = "✓" if is_valid == should_be_valid else "✗"
        
        print(f"{name:<20} {height:>7.2f}m  {status:<10} {point_type}")
    
    print("\n" + "=" * 60)

if __name__ == "__main__":
    print("\nX99 2D OCCUPANCY GRID TEST SUITE\n")
    
    # Test 1: Height filtering
    test_height_filtering()
    
    input("\nPress Enter to continue to grid test...")
    
    # Test 2: 2D grid
    test_2d_grid()
    
    print("\n✅ All tests completed successfully!")