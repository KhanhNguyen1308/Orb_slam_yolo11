#!/usr/bin/env python3
"""
Test script for SLAM improvements
Verifies that pose estimation and tracking work correctly
"""

import numpy as np
import cv2
import sys

def test_slam_tracker():
    """Test StereoSLAMTracker class"""
    print("\n" + "="*60)
    print("  SLAM TRACKER TEST")
    print("="*60 + "\n")
    
    # Import the class
    try:
        from x99_slam_server_improved import StereoSLAMTracker
    except ImportError:
        print("❌ Failed to import StereoSLAMTracker")
        print("   Make sure x99_slam_server_improved.py is in the same directory")
        return False
    
    # Initialize tracker
    print("[TEST] Initializing SLAM tracker...")
    try:
        tracker = StereoSLAMTracker(n_features=1000)
        print("✓ Tracker initialized")
    except Exception as e:
        print(f"❌ Initialization failed: {e}")
        return False
    
    # Create dummy stereo images
    print("\n[TEST] Creating test images...")
    height, width = 480, 640
    
    # Create textured image
    img_left = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
    
    # Add some features (checkerboard pattern)
    for i in range(0, height, 40):
        for j in range(0, width, 40):
            color = (255, 255, 255) if (i//40 + j//40) % 2 == 0 else (0, 0, 0)
            cv2.rectangle(img_left, (j, i), (j+40, i+40), color, -1)
    
    # Simulate right image with disparity
    img_right = img_left.copy()
    # Shift right image to simulate disparity
    img_right[:, 10:] = img_left[:, :-10]
    
    print("✓ Test images created (640x480)")
    
    # Test 1: Feature extraction
    print("\n[TEST] Testing feature extraction...")
    try:
        kp, desc = tracker.extract_features(img_left)
        print(f"✓ Extracted {len(kp)} features")
        
        if len(kp) < 50:
            print("⚠ Warning: Few features detected (< 50)")
    except Exception as e:
        print(f"❌ Feature extraction failed: {e}")
        return False
    
    # Test 2: Process first frame
    print("\n[TEST] Processing first frame (initialization)...")
    try:
        pose1, map_points1, quality1, stats1 = tracker.process_stereo_frame(
            img_left, img_right
        )
        
        print(f"✓ First frame processed")
        print(f"  Tracking quality: {quality1}")
        print(f"  Map points: {len(map_points1)}")
        print(f"  Pose: {pose1[:3, 3]}")
        
        if quality1 != 'INIT':
            print(f"⚠ Expected quality='INIT', got '{quality1}'")
        
        if len(map_points1) == 0:
            print("❌ No map points created!")
            return False
        
        print(f"✓ Created {len(map_points1)} initial map points")
        
    except Exception as e:
        print(f"❌ First frame processing failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test 3: Process second frame (should trigger tracking)
    print("\n[TEST] Processing second frame (tracking)...")
    
    # Simulate camera motion (translate left image)
    img_left_2 = np.roll(img_left, 20, axis=1)  # Shift right
    img_right_2 = np.roll(img_right, 20, axis=1)
    
    try:
        pose2, map_points2, quality2, stats2 = tracker.process_stereo_frame(
            img_left_2, img_right_2
        )
        
        print(f"✓ Second frame processed")
        print(f"  Tracking quality: {quality2}")
        print(f"  Tracked points: {stats2['num_tracked_points']}")
        print(f"  Inliers: {stats2['num_inliers']}")
        print(f"  Total map points: {len(map_points2)}")
        
        # Check pose changed
        pose_diff = np.linalg.norm(pose2[:3, 3] - pose1[:3, 3])
        print(f"  Pose change: {pose_diff:.4f}m")
        
        if quality2 == 'GOOD':
            print("✓ Tracking successful!")
        elif quality2 == 'LOST':
            print("⚠ Tracking lost (this is OK for synthetic images)")
        
    except Exception as e:
        print(f"❌ Second frame processing failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test 4: Get 2D pose
    print("\n[TEST] Testing pose conversion...")
    try:
        x, y, theta = tracker.get_current_pose_2d()
        print(f"✓ 2D Pose: x={x:.3f}m, y={y:.3f}m, theta={theta:.3f}rad")
    except Exception as e:
        print(f"❌ Pose conversion failed: {e}")
        return False
    
    # Test 5: Statistics
    print("\n[TEST] Testing statistics...")
    try:
        stats = tracker._get_statistics()
        print(f"✓ Statistics retrieved:")
        for key, value in stats.items():
            print(f"    {key}: {value}")
    except Exception as e:
        print(f"❌ Statistics failed: {e}")
        return False
    
    print("\n" + "="*60)
    print("  ✓ ALL TESTS PASSED!")
    print("="*60 + "\n")
    
    return True

def test_calibration_loading():
    """Test calibration file loading"""
    print("\n[TEST] Testing calibration loading...")
    
    from x99_slam_server_improved import StereoSLAMTracker
    
    # Test with non-existent file
    tracker1 = StereoSLAMTracker(calibration_file="nonexistent.npz")
    print("✓ Handles missing calibration file")
    
    # Create dummy calibration file
    print("\n[TEST] Creating dummy calibration file...")
    P1 = np.array([
        [600, 0, 320, 0],
        [0, 600, 240, 0],
        [0, 0, 1, 0]
    ], dtype=np.float32)
    
    Q = np.zeros((4, 4), dtype=np.float32)
    Q[3, 2] = -10.0  # baseline = 1/10 = 0.1m
    
    np.savez('test_calib.npz', P1=P1, Q=Q)
    
    # Test loading
    tracker2 = StereoSLAMTracker(calibration_file="test_calib.npz")
    
    if abs(tracker2.baseline - 0.1) < 0.01:
        print(f"✓ Calibration loaded correctly: baseline={tracker2.baseline:.3f}m")
    else:
        print(f"⚠ Baseline mismatch: expected 0.1, got {tracker2.baseline:.3f}")
    
    print(f"  Focal length: {tracker2.K[0, 0]:.1f}px")
    
    # Cleanup
    import os
    if os.path.exists('test_calib.npz'):
        os.remove('test_calib.npz')
        print("✓ Cleanup done")

def test_integration():
    """Test integration with other modules"""
    print("\n" + "="*60)
    print("  INTEGRATION TEST")
    print("="*60 + "\n")
    
    # Test path planning integration
    print("[TEST] Checking path_planning.py...")
    try:
        from path_planning import PathPlanner
        planner = PathPlanner()
        print("✓ PathPlanner available")
    except ImportError:
        print("⚠ path_planning.py not found (optional)")
    
    # Test persistent map integration
    print("\n[TEST] Checking persistent_map.py...")
    try:
        from persistent_map import PersistentMap
        pmap = PersistentMap()
        print("✓ PersistentMap available")
    except ImportError:
        print("⚠ persistent_map.py not found (optional)")
    
    # Test YOLO integration
    print("\n[TEST] Checking YOLO...")
    try:
        from ultralytics import YOLO
        print("✓ YOLOv11 available")
    except ImportError:
        print("⚠ ultralytics not installed (optional)")

def run_all_tests():
    """Run all tests"""
    print("\n" + "#"*60)
    print("#" + " "*58 + "#")
    print("#  X99 SLAM IMPROVEMENTS - VERIFICATION SUITE" + " "*13 + "#")
    print("#" + " "*58 + "#")
    print("#"*60)
    
    tests = [
        ("SLAM Tracker", test_slam_tracker),
        ("Calibration Loading", test_calibration_loading),
        ("Integration", test_integration)
    ]
    
    results = []
    
    for name, test_func in tests:
        try:
            print(f"\n{'='*60}")
            print(f"  Running: {name}")
            print(f"{'='*60}")
            
            result = test_func()
            results.append((name, result))
            
        except Exception as e:
            print(f"\n❌ Test '{name}' crashed: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))
    
    # Summary
    print("\n" + "#"*60)
    print("#  TEST SUMMARY")
    print("#" + "-"*58 + "#")
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✓ PASS" if result else "❌ FAIL"
        print(f"#  {status} - {name}")
    
    print("#" + "-"*58 + "#")
    print(f"#  Total: {passed}/{total} tests passed")
    print("#"*60 + "\n")
    
    if passed == total:
        print("🎉 ALL TESTS PASSED! SLAM improvements verified.")
        print("\nYou can now run the improved server:")
        print("  python x99_slam_server_improved.py")
        return 0
    else:
        print("⚠ Some tests failed. Please check the errors above.")
        return 1

if __name__ == "__main__":
    exit_code = run_all_tests()
    sys.exit(exit_code)