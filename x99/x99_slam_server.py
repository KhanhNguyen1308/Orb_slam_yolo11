#x99_slam_server.py
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
from typing import Optional, Tuple
import queue

try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: ultralytics not installed")

@dataclass
class ServerConfig:
    """Server configuration"""
    host: str = "0.0.0.0"  # Listen on all interfaces
    left_port: int = 9002
    right_port: int = 9001
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

class ORBFeatureExtractor:
    """ORB feature extraction and matching"""
    
    def __init__(self, n_features: int = 800):
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.orb = cv2.cuda.ORB_create(nfeatures=n_features)
                self.use_cuda = True
                print("✓ Using CUDA-accelerated ORB")
            else:
                self.orb = cv2.ORB_create(nfeatures=n_features)
                self.use_cuda = False
                print("ℹ Using CPU ORB (this is OK)")
        except:
            self.orb = cv2.ORB_create(nfeatures=n_features)
            self.use_cuda = False
        
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    def extract_features(self, image: np.ndarray):
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
    
    def match_features(self, desc1, desc2):
        """Match features between two frames"""
        if desc1 is None or desc2 is None:
            return []
        matches = self.bf_matcher.match(desc1, desc2)
        matches = sorted(matches, key=lambda x: x.distance)
        return matches[:100]

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
    """X99 Server receiving streams and running SLAM"""
    
    def __init__(self, config: ServerConfig, use_yolo: bool = True):
        self.config = config
        
        # Camera receivers
        self.left_receiver = CameraReceiver(config.left_port, "LEFT")
        self.right_receiver = CameraReceiver(config.right_port, "RIGHT")
        
        # SLAM components
        self.orb_extractor = ORBFeatureExtractor(n_features=800)
        
        self.yolo = None
        if use_yolo:
            self.yolo = YOLOSegmentator()
        
        self.is_running = False
        self.map_points = []
        
    def start_receivers(self):
        """Start both camera receivers"""
        print("\n" + "=" * 60)
        print("  X99 Server - SLAM Processing")
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
        """Process stereo frame pair"""
        
        # Extract ORB features
        kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
        kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
        
        # Match stereo features
        matches = self.orb_extractor.match_features(desc_left, desc_right)
        
        # Triangulate 3D points (simplified)
        num_3d_points = len(matches)
        
        # YOLO segmentation on left frame
        processed_frame = frame_left.copy()
        if self.yolo:
            yolo_results = self.yolo.segment(frame_left, conf=0.8)
            processed_frame = self.yolo.draw_segments(frame_left, yolo_results)
        
        # Draw ORB features
        frame_with_features = cv2.drawKeypoints(
            processed_frame, kp_left, None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        
        # Add info overlay
        info_text = [
            f"ORB Features: L={len(kp_left)}, R={len(kp_right)}",
            f"Stereo Matches: {len(matches)}",
            f"3D Points: {num_3d_points}",
        ]
        
        y_offset = 30
        for text in info_text:
            cv2.putText(frame_with_features, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
        
        return frame_with_features
    
    def run(self):
        """Main SLAM loop"""
        
        if not self.start_receivers():
            return
        
        self.is_running = True
        
        print("[INFO] Starting SLAM processing...\n")
        
        frame_count = 0
        last_display = time.time()
        
        try:
            while self.is_running:
                # Get frames from both cameras
                frame_left = self.left_receiver.get_frame()
                frame_right = self.right_receiver.get_frame()
                
                if frame_left is None or frame_right is None:
                    continue
                
                # Process stereo pair
                processed = self.process_stereo_frames(frame_left, frame_right)
                
                # Display
                cv2.imshow('X99 SLAM - Left Camera + YOLO', processed)
                cv2.imshow('X99 SLAM - Right Camera', frame_right)
                
                frame_count += 1
                
                # Status update
                if time.time() - last_display > 5.0:
                    print(f"[SLAM] Processing frames: {frame_count}")
                    last_display = time.time()
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
        except KeyboardInterrupt:
            print("\n[INFO] SLAM interrupted by user")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop SLAM server"""
        self.is_running = False
        self.left_receiver.is_running = False
        self.right_receiver.is_running = False
        
        cv2.destroyAllWindows()
        print("\n[INFO] SLAM server stopped")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 SLAM Server')
    parser.add_argument('--left-port', type=int, default=9002,
                       help='Left camera receive port')
    parser.add_argument('--right-port', type=int, default=9001,
                       help='Right camera receive port')
    parser.add_argument('--no-yolo', action='store_true',
                       help='Disable YOLO segmentation')
    parser.add_argument('--baseline', type=float, default=0.10,
                       help='Stereo baseline in meters')
    
    args = parser.parse_args()
    
    config = ServerConfig(
        left_port=args.left_port,
        right_port=args.right_port,
        baseline=args.baseline
    )
    
    server = X99SLAMServer(config, use_yolo=not args.no_yolo)
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        server.stop()

if __name__ == "__main__":
    main()
