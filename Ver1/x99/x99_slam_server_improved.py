#!/usr/bin/env python3
"""
X99 Server - Receive camera streams from Jetson Nano and run SLAM
"""

import cv2
import numpy as np
import socket
import pickle
import struct
import threading
import time
import torch
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
import queue
import json

try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: ultralytics not installed")

# Import local modules
try:
    from path_planning import PathPlanner
    from persistent_map import PersistentMap
except ImportError:
    print("Warning: path_planning or persistent_map not found")
    PathPlanner = None
    PersistentMap = None

@dataclass
class ServerConfig:
    """Server configuration"""
    host: str = "0.0.0.0"  # Listen on all interfaces
    left_port: int = 9001
    right_port: int = 9002
    baseline: float = 0.10  # 10cm
    focal_length: float = 500

class CameraReceiver:
    """Receive camera stream over TCP"""
    
    def __init__(self, port: int, name: str):
        self.port = port
        self.name = name
        self.server_socket = None
        self.client_socket = None
        self.is_running = False
        self.frame_queue = queue.Queue(maxsize=10)
        
    def start_server(self):
        """Start TCP server and wait for connection"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("0.0.0.0", self.port))
            self.server_socket.listen(1)
            
            print(f"[{self.name}] Waiting for connection on port {self.port}...")
            self.client_socket, addr = self.server_socket.accept()
            
            print(f"[{self.name}] Connected from {addr}")
            return True
            
        except Exception as e:
            print(f"[{self.name}] Server start failed: {e}")
            return False
    
    def receive_frames(self):
        """Receive frames from client"""
        self.is_running = True
        frame_count = 0
        start_time = time.time()
        
        data = b""
        payload_size = struct.calcsize("!L")
        
        try:
            while self.is_running:
                # Receive message size
                while len(data) < payload_size:
                    packet = self.client_socket.recv(4096)
                    if not packet:
                        print(f"[{self.name}] Connection closed by client")
                        return
                    data += packet
                
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("!L", packed_msg_size)[0]
                
                # Receive frame data
                while len(data) < msg_size:
                    packet = self.client_socket.recv(4096)
                    if not packet:
                        print(f"[{self.name}] Connection lost")
                        return
                    data += packet
                
                frame_data = data[:msg_size]
                data = data[msg_size:]
                
                # Deserialize and decode frame
                encoded_frame = pickle.loads(frame_data)
                frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)
                
                # Add to queue (drop old frames if queue is full)
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                
                self.frame_queue.put(frame)
                frame_count += 1
                
                # Calculate FPS
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed
                    print(f"[{self.name}] Received {frame_count} frames, {fps:.1f} FPS")
                
        except Exception as e:
            print(f"[{self.name}] Receive error: {e}")
        
        finally:
            self.cleanup()
    
    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest frame from queue"""
        try:
            return self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            return None
    
    def cleanup(self):
        """Clean up resources"""
        self.is_running = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        print(f"[{self.name}] Receiver stopped")

class StereoSLAMTracker:
    """
    Complete Stereo SLAM with tracking and pose estimation
    Features:
    - ORB feature extraction and matching
    - Stereo triangulation for 3D reconstruction
    - PnP RANSAC for camera motion estimation
    - Keyframe management
    - Map point tracking
    """
    
    def __init__(self, n_features: int = 1500, calibration_file: str = None):
        """Initialize SLAM tracker"""
        # Feature extractor
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.orb = cv2.cuda.ORB_create(nfeatures=n_features)
                self.use_cuda = True
                print("✓ Using CUDA-accelerated ORB")
            else:
                self.orb = cv2.ORB_create(nfeatures=n_features)
                self.use_cuda = False
                print("ℹ Using CPU ORB")
        except:
            self.orb = cv2.ORB_create(nfeatures=n_features)
            self.use_cuda = False
        
        # Matcher
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Camera parameters (will be loaded from calibration)
        self.K = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        self.baseline = 0.10  # 10cm default
        self.dist_coeffs = None
        
        # Load calibration if available
        if calibration_file:
            self._load_calibration(calibration_file)
        
        # SLAM state
        self.current_pose = np.eye(4, dtype=np.float32)  # World to camera
        self.keyframes = []
        self.map_points = []
        
        # Previous frame data
        self.prev_kp = None
        self.prev_desc = None
        self.prev_points_3d = []
        self.prev_kp_to_3d = {}
        
        # Statistics
        self.frame_count = 0
        self.tracking_quality = 'INIT'
        self.num_tracked_points = 0
        self.num_inliers = 0
        
        print(f"[SLAM] Initialized - Baseline: {self.baseline:.3f}m, Focal: {self.K[0,0]:.1f}px")
    
    def _load_calibration(self, calib_file: str):
        """Load camera calibration from file"""
        try:
            import os
            if not os.path.exists(calib_file):
                print(f"[SLAM] Calibration file not found: {calib_file}")
                return
            
            data = np.load(calib_file)
            
            # Extract from projection matrix P1
            if 'P1' in data:
                P1 = data['P1']
                self.K = P1[:3, :3].astype(np.float32)
            
            # Extract baseline from Q matrix
            if 'Q' in data:
                Q = data['Q']
                if Q[3, 2] != 0:
                    self.baseline = 1.0 / abs(Q[3, 2])
            
            # Distortion coefficients
            if 'dist_l' in data:
                self.dist_coeffs = data['dist_l']
            
            print(f"[SLAM] Loaded calibration: f={self.K[0,0]:.1f}, baseline={self.baseline:.3f}m")
        
        except Exception as e:
            print(f"[SLAM] Failed to load calibration: {e}")
    
    def extract_features(self, image: np.ndarray):
        """Extract ORB features from image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if self.use_cuda:
            try:
                gpu_gray = cv2.cuda_GpuMat()
                gpu_gray.upload(gray)
                keypoints, descriptors = self.orb.detectAndCompute(gpu_gray, None)
                if descriptors is not None:
                    descriptors = descriptors.download()
            except:
                keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        else:
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        return keypoints, descriptors
    
    def match_features(self, desc1, desc2, ratio_thresh: float = 0.7):
        """Match features with ratio test"""
        if desc1 is None or desc2 is None:
            return []
        
        matches = self.bf_matcher.knnMatch(desc1, desc2, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < ratio_thresh * n.distance:
                    good_matches.append(m)
        
        return good_matches
    
    def triangulate_stereo_points(self, kp_left, desc_left, kp_right, desc_right):
        """
        Triangulate 3D points from stereo pair
        Returns: (points_3d, kp_indices, descriptors)
        """
        # Match left-right
        matches = self.match_features(desc_left, desc_right, ratio_thresh=0.8)
        
        if len(matches) < 10:
            return [], [], []
        
        points_3d = []
        kp_indices = []
        point_descriptors = []
        
        for match in matches:
            idx_left = match.queryIdx
            idx_right = match.trainIdx
            
            pt_left = kp_left[idx_left].pt
            pt_right = kp_right[idx_right].pt
            
            # Epipolar constraint check (same y after rectification)
            if abs(pt_left[1] - pt_right[1]) > 2.0:
                continue
            
            # Disparity
            disparity = pt_left[0] - pt_right[0]
            
            if disparity < 1.0:  # Invalid disparity
                continue
            
            # Compute depth: Z = (f * B) / d
            depth = (self.baseline * self.K[0, 0]) / disparity
            
            if depth < 0.1 or depth > 10.0:  # Filter unrealistic depths
                continue
            
            # Unproject to 3D camera frame
            x = (pt_left[0] - self.K[0, 2]) * depth / self.K[0, 0]
            y = (pt_left[1] - self.K[1, 2]) * depth / self.K[1, 1]
            z = depth
            
            point_3d = np.array([x, y, z], dtype=np.float32)
            points_3d.append(point_3d)
            kp_indices.append(idx_left)
            point_descriptors.append(desc_left[idx_left])
        
        return points_3d, kp_indices, point_descriptors
    
    def estimate_camera_motion(self, kp_current, desc_current):
        """
        Estimate camera motion using PnP RANSAC
        Returns: success (bool)
        """
        # Match with previous frame
        matches = self.match_features(self.prev_desc, desc_current)
        
        if len(matches) < 20:
            print(f"[SLAM] Not enough matches: {len(matches)}")
            return False
        
        # Build 3D-2D correspondences
        points_3d = []
        points_2d = []
        
        for match in matches:
            prev_idx = match.queryIdx
            curr_idx = match.trainIdx
            
            # Get 3D point from previous frame
            if prev_idx in self.prev_kp_to_3d:
                point_3d = self.prev_kp_to_3d[prev_idx]
                point_2d = kp_current[curr_idx].pt
                
                points_3d.append(point_3d)
                points_2d.append(point_2d)
        
        if len(points_3d) < 10:
            print(f"[SLAM] Not enough 3D-2D pairs: {len(points_3d)}")
            return False
        
        # Solve PnP RANSAC
        points_3d = np.array(points_3d, dtype=np.float32)
        points_2d = np.array(points_2d, dtype=np.float32)
        
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            objectPoints=points_3d,
            imagePoints=points_2d,
            cameraMatrix=self.K,
            distCoeffs=self.dist_coeffs,
            iterationsCount=100,
            reprojectionError=8.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success or inliers is None or len(inliers) < 15:
            print(f"[SLAM] PnP failed - inliers: {len(inliers) if inliers is not None else 0}")
            return False
        
        # Convert to transformation matrix
        R, _ = cv2.Rodrigues(rvec)
        T_current_to_prev = np.eye(4, dtype=np.float32)
        T_current_to_prev[:3, :3] = R
        T_current_to_prev[:3, 3] = tvec.flatten()
        
        # Update pose: T_world_to_current = T_world_to_prev * T_prev_to_current
        T_prev_to_current = np.linalg.inv(T_current_to_prev)
        self.current_pose = self.current_pose @ T_prev_to_current
        
        self.num_inliers = len(inliers)
        self.num_tracked_points = len(points_3d)
        
        return True
    
    def process_stereo_frame(self, img_left: np.ndarray, img_right: np.ndarray, 
                            timestamp: float = None):
        """
        Process stereo frame pair - Main SLAM function
        Returns: (current_pose, map_points, tracking_quality, statistics)
        """
        self.frame_count += 1
        
        if timestamp is None:
            timestamp = time.time()
        
        # Step 1: Extract ORB features
        kp_left, desc_left = self.extract_features(img_left)
        kp_right, desc_right = self.extract_features(img_right)
        
        if len(kp_left) < 50:
            print(f"[SLAM] Warning: Only {len(kp_left)} features detected")
            self.tracking_quality = 'POOR'
            return self.current_pose, self.map_points, self.tracking_quality, self._get_statistics()
        
        # Step 2: Track previous frame (if exists)
        if self.prev_desc is not None and len(self.prev_points_3d) > 0:
            tracking_success = self.estimate_camera_motion(kp_left, desc_left)
            
            if tracking_success:
                self.tracking_quality = 'GOOD'
                print(f"[SLAM] Frame {self.frame_count}: Tracked {self.num_inliers} inliers")
            else:
                self.tracking_quality = 'LOST'
                print(f"[SLAM] Frame {self.frame_count}: Tracking LOST")
        else:
            # First frame
            self.tracking_quality = 'INIT'
            print(f"[SLAM] Frame {self.frame_count}: Initializing")
        
        # Step 3: Triangulate 3D points from stereo
        points_3d_camera, kp_indices, point_descs = self.triangulate_stereo_points(
            kp_left, desc_left, kp_right, desc_right
        )
        
        print(f"[SLAM] Triangulated {len(points_3d_camera)} 3D points")
        
        # Step 4: Transform to world frame and update map
        if len(points_3d_camera) > 0:
            self._update_map(points_3d_camera, kp_left, kp_indices, point_descs)
        
        # Step 5: Keyframe decision
        if self._should_create_keyframe():
            self._add_keyframe(img_left, kp_left, desc_left, timestamp)
        
        # Step 6: Store for next frame
        self.prev_kp = kp_left
        self.prev_desc = desc_left
        self.prev_points_3d = points_3d_camera
        
        # Build mapping from keypoint index to 3D point
        self.prev_kp_to_3d = {}
        for i, kp_idx in enumerate(kp_indices):
            self.prev_kp_to_3d[kp_idx] = points_3d_camera[i]
        
        return self.current_pose, self.map_points, self.tracking_quality, self._get_statistics()
    
    def _update_map(self, points_3d_camera, kp_left, kp_indices, point_descs):
        """Transform points to world frame and add to map"""
        for i, point_camera in enumerate(points_3d_camera):
            # Transform to world frame
            point_camera_hom = np.append(point_camera, 1)
            point_world_hom = self.current_pose @ point_camera_hom
            point_world = point_world_hom[:3]
            
            # Add to map
            self.map_points.append({
                'position': point_world,
                'descriptor': point_descs[i],
                'observations': 1,
                'frame_id': self.frame_count,
                'color': [128, 128, 128]  # Default gray
            })
        
        # Limit map size
        if len(self.map_points) > 10000:
            self.map_points = self.map_points[-10000:]
    
    def _should_create_keyframe(self) -> bool:
        """Decide if current frame should be keyframe"""
        if len(self.keyframes) == 0:
            return True
        
        last_kf_pose = self.keyframes[-1]['pose']
        
        # Translation distance
        translation = np.linalg.norm(
            self.current_pose[:3, 3] - last_kf_pose[:3, 3]
        )
        
        # Rotation angle
        R_diff = last_kf_pose[:3, :3].T @ self.current_pose[:3, :3]
        angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
        
        return translation > 0.3 or angle > 0.2  # 30cm or ~11 degrees
    
    def _add_keyframe(self, image, keypoints, descriptors, timestamp):
        """Add keyframe to database"""
        keyframe = {
            'id': len(self.keyframes),
            'timestamp': timestamp,
            'pose': self.current_pose.copy(),
            'keypoints': keypoints,
            'descriptors': descriptors,
            'num_points': len(keypoints)
        }
        
        self.keyframes.append(keyframe)
        
        # Limit keyframe database
        if len(self.keyframes) > 100:
            self.keyframes = self.keyframes[-100:]
        
        print(f"[SLAM] Keyframe #{keyframe['id']} created")
    
    def get_current_pose_2d(self) -> Tuple[float, float, float]:
        """
        Get current pose as (x, y, theta) for 2D navigation
        """
        x = self.current_pose[0, 3]
        y = self.current_pose[2, 3]  # Z in camera = forward in world
        
        # Extract yaw
        theta = np.arctan2(self.current_pose[1, 0], self.current_pose[0, 0])
        
        return float(x), float(y), float(theta)
    
    def _get_statistics(self) -> Dict:
        """Get SLAM statistics"""
        return {
            'frame_count': self.frame_count,
            'num_keyframes': len(self.keyframes),
            'num_map_points': len(self.map_points),
            'tracking_quality': self.tracking_quality,
            'num_tracked_points': self.num_tracked_points,
            'num_inliers': self.num_inliers
        }

class YOLOSegmentator:
    """YOLOv11m-seg for semantic segmentation"""
    
    def __init__(self, model_path: str = "yolo11m-seg.pt"):
        try:
            # Auto-detect ROCm for AMD GPU
            if torch.cuda.is_available():
                device = 'cuda'
            else:
                device = 'cpu'
            
            print(f"[YOLO] Initializing on device: {device}")
            self.model = YOLO(model_path)
            self.device = device
        except Exception as e:
            print(f"[YOLO] Initialization failed: {e}")
            self.model = None
    
    def segment(self, image: np.ndarray, conf: float = 0.5):
        """Run segmentation on image"""
        if self.model is None:
            return None
        
        #results = self.model(image, conf=conf, device=self.device, verbose=False)
        try:
            results = self.model(
                image,
                conf=conf,
                device=self.device,  # CRITICAL: Always specify device
                verbose=False,
                half=True,  # FP16 for faster inference on MI50
                imgsz=640   # Fixed size for consistency
            )
            return results[0] if results else None
        except Exception as e:
            print(f" YOLO inference error: {e}")
            return None
    
    def draw_segments(self, image: np.ndarray, results) -> np.ndarray:
        """Draw segmentation masks on image"""
        if results is None or results.masks is None:
            return image
        
        return results.plot()

class X99SLAMServer:
    """X99 Server receiving streams and running SLAM with navigation"""
    
    def __init__(self, config: ServerConfig, use_yolo: bool = True, 
                 enable_planning: bool = True, calibration_file: str = None):
        self.config = config
        
        # Camera receivers
        self.left_receiver = CameraReceiver(config.left_port, "LEFT")
        self.right_receiver = CameraReceiver(config.right_port, "RIGHT")
        
        # SLAM tracker with full pose estimation
        self.slam_tracker = StereoSLAMTracker(
            n_features=1500, 
            calibration_file=calibration_file
        )
        
        # YOLO segmentation
        self.yolo = None
        if use_yolo:
            self.yolo = YOLOSegmentator()
        
        # Path planning
        self.path_planner = None
        self.enable_planning = enable_planning
        if enable_planning and PathPlanner is not None:
            self.path_planner = PathPlanner(grid_width=200, grid_height=200, resolution=0.05)
            print("[Server] Path planner enabled")
        
        # Persistent map
        self.persistent_map = None
        if PersistentMap is not None:
            self.persistent_map = PersistentMap(grid_size=800, resolution=0.02)
            print("[Server] Persistent map enabled")
        
        # Navigation command sender (to Jetson)
        self.nav_port = 9003
        self.nav_socket = None
        
        self.is_running = False
        
        # Statistics
        self.total_frames = 0
        self.start_time = None
    
    def _setup_navigation_sender(self):
        """Setup TCP connection to send commands to Jetson"""
        try:
            self.nav_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Note: In production, get Jetson IP from config
            # For now, assume localhost or same network
            # You'll need to configure this
            print(f"[Server] Navigation sender ready on port {self.nav_port}")
        except Exception as e:
            print(f"[Server] Navigation setup failed: {e}")
    
    def _send_pose_to_jetson(self, pose: Tuple[float, float, float]):
        """Send robot pose to Jetson for navigation"""
        if self.nav_socket is None:
            return
        
        try:
            message = {
                'type': 'pose',
                'pose': list(pose),
                'timestamp': time.time()
            }
            
            msg_data = json.dumps(message).encode()
            self.nav_socket.sendall(struct.pack('!I', len(msg_data)))
            self.nav_socket.sendall(msg_data)
        except Exception as e:
            print(f"[Server] Failed to send pose: {e}")
    
    def _send_path_to_jetson(self, path: List[Tuple[float, float]]):
        """Send planned path to Jetson"""
        if self.nav_socket is None:
            return
        
        try:
            message = {
                'type': 'path',
                'path': path,
                'timestamp': time.time()
            }
            
            msg_data = json.dumps(message).encode()
            self.nav_socket.sendall(struct.pack('!I', len(msg_data)))
            self.nav_socket.sendall(msg_data)
            
            print(f"[Server] Sent path with {len(path)} waypoints to Jetson")
        except Exception as e:
            print(f"[Server] Failed to send path: {e}")
    
    def start_receivers(self):
        """Start both camera receivers"""
        print("\n" + "=" * 60)
        print("  X99 Server - Advanced SLAM with Navigation")
        print("=" * 60)
        print(f"Listening on ports: {self.config.left_port}, {self.config.right_port}")
        print("=" * 60 + "\n")
        
        # Start left camera receiver
        left_thread = threading.Thread(
            target=lambda: (
                self.left_receiver.start_server() and 
                self.left_receiver.receive_frames()
            ),
            daemon=True
        )
        left_thread.start()
        
        # Start right camera receiver
        right_thread = threading.Thread(
            target=lambda: (
                self.right_receiver.start_server() and 
                self.right_receiver.receive_frames()
            ),
            daemon=True
        )
        right_thread.start()
        
        # Wait for both connections
        time.sleep(2)
        
        if not self.left_receiver.is_running or not self.right_receiver.is_running:
            print("\n[ERROR] Failed to establish connections")
            return False
        
        print("\n[OK] Both cameras connected!\n")
        return True
    
    def process_stereo_frames(self, frame_left: np.ndarray, frame_right: np.ndarray):
        """
        Process stereo frame pair with full SLAM pipeline
        """
        # Run SLAM tracking
        current_pose, map_points, tracking_quality, stats = \
            self.slam_tracker.process_stereo_frame(frame_left, frame_right)
        
        # Get 2D pose for navigation
        pose_2d = self.slam_tracker.get_current_pose_2d()
        
        # Send pose to Jetson for navigation
        self._send_pose_to_jetson(pose_2d)
        
        # YOLO segmentation
        yolo_results = None
        if self.yolo:
            yolo_results = self.yolo.segment(frame_left, conf=0.5)
        
        # Update persistent map
        if self.persistent_map and len(map_points) > 0:
            # Convert map points to format expected by persistent map
            points_array = np.array([mp['position'] for mp in map_points])
            colors_array = np.array([mp.get('color', [128, 128, 128]) for mp in map_points])
            robot_pose_3d = [pose_2d[0], 0.0, pose_2d[1]]  # x, y=0, z
            
            self.persistent_map.add_point_cloud(
                points_array, colors_array, robot_pose_3d
            )
        
        # Update path planner occupancy grid
        if self.path_planner and tracking_quality == 'GOOD':
            self.path_planner.update_map(map_points, current_pose)
        
        # Create visualization
        vis_frame = self._create_visualization(
            frame_left, map_points, yolo_results, stats, pose_2d
        )
        
        return vis_frame, current_pose, tracking_quality
    
    def _create_visualization(self, frame_left, map_points, yolo_results, stats, pose_2d):
        """Create visualization frame with all info"""
        # Start with YOLO visualization if available
        if self.yolo and yolo_results:
            vis_frame = self.yolo.draw_segments(frame_left, yolo_results)
        else:
            vis_frame = frame_left.copy()
        
        # Add text overlay with SLAM info
        info_lines = [
            f"Frame: {stats['frame_count']}",
            f"Tracking: {stats['tracking_quality']}",
            f"Keyframes: {stats['num_keyframes']}",
            f"Map Points: {stats['num_map_points']}",
            f"Tracked: {stats['num_inliers']} pts",
            f"Pose: ({pose_2d[0]:.2f}, {pose_2d[1]:.2f}, {pose_2d[2]:.2f})"
        ]
        
        y_offset = 30
        for line in info_lines:
            # Black background for text
            text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(vis_frame, (5, y_offset - 20), 
                         (15 + text_size[0], y_offset + 5), (0, 0, 0), -1)
            
            # Text color based on tracking quality
            if stats['tracking_quality'] == 'GOOD':
                color = (0, 255, 0)
            elif stats['tracking_quality'] == 'LOST':
                color = (0, 0, 255)
            else:
                color = (255, 255, 0)
            
            cv2.putText(vis_frame, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y_offset += 25
        
        return vis_frame
    
    def plan_path_to_goal(self, goal_x: float, goal_y: float):
        """Plan path to goal and send to Jetson"""
        if self.path_planner is None:
            print("[Server] Path planner not enabled")
            return False
        
        path = self.path_planner.plan_to_goal(goal_x, goal_y)
        
        if path is None:
            print("[Server] Path planning failed!")
            return False
        
        # Send to Jetson
        self._send_path_to_jetson(path)
        
        return True
    
    def run(self):
        """Main SLAM loop"""
        
        if not self.start_receivers():
            return
        
        # Setup navigation sender
        self._setup_navigation_sender()
        
        self.is_running = True
        self.start_time = time.time()
        
        print("[INFO] Starting SLAM processing...\n")
        print("[INFO] Commands:")
        print("  'q' - Quit")
        print("  'g' - Set goal and plan path")
        print("  's' - Save map")
        print("  'r' - Reset SLAM\n")
        
        last_display = time.time()
        fps_counter = 0
        fps = 0.0
        
        try:
            while self.is_running:
                # Get frames from both cameras
                frame_left = self.left_receiver.get_frame()
                frame_right = self.right_receiver.get_frame()
                
                if frame_left is None or frame_right is None:
                    continue
                
                # Process stereo pair
                vis_frame, current_pose, tracking_quality = \
                    self.process_stereo_frames(frame_left, frame_right)
                
                self.total_frames += 1
                fps_counter += 1
                
                # Calculate FPS
                if time.time() - last_display > 1.0:
                    fps = fps_counter / (time.time() - last_display)
                    fps_counter = 0
                    last_display = time.time()
                
                # Add FPS to display
                cv2.putText(vis_frame, f"FPS: {fps:.1f}", (10, frame_left.shape[0] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Display
                cv2.imshow('X99 SLAM - Tracking + Navigation', vis_frame)
                
                # Show path planning visualization if available
                if self.path_planner and self.path_planner.current_path:
                    path_vis = self.path_planner.visualize_path()
                    path_vis = cv2.resize(path_vis, (400, 400))
                    cv2.imshow('Path Planning', path_vis)
                
                # Show persistent map if available
                if self.persistent_map and self.total_frames % 10 == 0:
                    map_vis = self.persistent_map.visualize_2d(show_trajectory=True)
                    map_vis = cv2.resize(map_vis, (400, 400))
                    cv2.imshow('Persistent Map', map_vis)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('g'):
                    # Set goal for path planning
                    print("\n[INFO] Enter goal coordinates:")
                    try:
                        goal_x = float(input("  Goal X (meters): "))
                        goal_y = float(input("  Goal Y (meters): "))
                        print(f"[INFO] Planning path to ({goal_x}, {goal_y})...")
                        self.plan_path_to_goal(goal_x, goal_y)
                    except ValueError:
                        print("[ERROR] Invalid coordinates")
                elif key == ord('s'):
                    # Save map
                    if self.persistent_map:
                        filename = f"map_{int(time.time())}.npz"
                        self.persistent_map.save_map(filename)
                        print(f"[INFO] Map saved to {filename}")
                elif key == ord('r'):
                    # Reset SLAM
                    print("[INFO] Resetting SLAM...")
                    self.slam_tracker = StereoSLAMTracker(n_features=1500)
                    if self.persistent_map:
                        self.persistent_map.clear_map()
                
                # Status update
                if time.time() - last_display > 5.0:
                    elapsed = time.time() - self.start_time
                    print(f"[SLAM] Runtime: {elapsed:.1f}s, Frames: {self.total_frames}, "
                          f"FPS: {fps:.1f}, Quality: {tracking_quality}")
                
        except KeyboardInterrupt:
            print("\n[INFO] SLAM interrupted by user")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop SLAM server"""
        self.is_running = False
        self.left_receiver.is_running = False
        self.right_receiver.is_running = False
        
        if self.nav_socket:
            self.nav_socket.close()
        
        cv2.destroyAllWindows()
        
        # Final statistics
        if self.start_time:
            elapsed = time.time() - self.start_time
            avg_fps = self.total_frames / elapsed if elapsed > 0 else 0
            print(f"\n[INFO] Session statistics:")
            print(f"  Total frames: {self.total_frames}")
            print(f"  Runtime: {elapsed:.1f}s")
            print(f"  Average FPS: {avg_fps:.1f}")
            print(f"  Keyframes: {len(self.slam_tracker.keyframes)}")
            print(f"  Map points: {len(self.slam_tracker.map_points)}")
        
        print("\n[INFO] SLAM server stopped")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 SLAM Server with Navigation')
    parser.add_argument('--left-port', type=int, default=9002,
                       help='Left camera receive port')
    parser.add_argument('--right-port', type=int, default=9001,
                       help='Right camera receive port')
    parser.add_argument('--no-yolo', action='store_true',
                       help='Disable YOLO segmentation')
    parser.add_argument('--no-planning', action='store_true',
                       help='Disable path planning')
    parser.add_argument('--baseline', type=float, default=0.10,
                       help='Stereo baseline in meters')
    parser.add_argument('--calibration', type=str, default='calibration_params.npz',
                       help='Calibration file path')
    
    args = parser.parse_args()
    
    config = ServerConfig(
        left_port=args.left_port,
        right_port=args.right_port,
        baseline=args.baseline
    )
    
    server = X99SLAMServer(
        config, 
        use_yolo=not args.no_yolo,
        enable_planning=not args.no_planning,
        calibration_file=args.calibration
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        server.stop()

if __name__ == "__main__":
    main()