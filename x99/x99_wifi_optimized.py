#!/usr/bin/env python3
"""
X99 Optimized Stream Receiver for WiFi
High-performance receiver with buffering and jitter handling
"""

import cv2
import numpy as np
import socket
import struct
import threading
import time
import queue
from collections import deque

class OptimizedCameraReceiver:
    """Optimized receiver for WiFi streaming"""
    
    def __init__(self, port: int, name: str, buffer_size: int = 65536):
        self.port = port
        self.name = name
        self.buffer_size = buffer_size
        
        self.server_socket = None
        self.client_socket = None
        self.is_running = False
        
        # Multi-level frame queuing
        self.decode_queue = queue.Queue(maxsize=10)
        self.frame_queue = queue.Queue(maxsize=5)
        
        # Statistics
        self.frame_count = 0
        self.bytes_received = 0
        self.start_time = None
        self.fps_history = deque(maxlen=30)
        
    def start_server(self):
        """Start TCP server with optimized settings"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
            
            self.server_socket.bind(("0.0.0.0", self.port))
            self.server_socket.listen(1)
            
            print(f"[{self.name}] Listening on port {self.port}...")
            self.client_socket, addr = self.server_socket.accept()
            
            # Set TCP options for connected socket
            self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
            
            print(f"[{self.name}] Connected from {addr[0]}:{addr[1]}")
            return True
            
        except Exception as e:
            print(f"[{self.name}] Server error: {e}")
            return False
    
    def decode_worker(self):
        """Async frame decoding thread"""
        while self.is_running:
            try:
                data = self.decode_queue.get(timeout=1.0)
                
                # Decode JPEG
                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # Add to frame queue (drop old if full)
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    
                    self.frame_queue.put(frame)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[{self.name}] Decode error: {e}")
    
    def receive_frames(self):
        """Optimized frame receiving with proper buffering"""
        self.is_running = True
        self.start_time = time.time()
        
        # Start decode thread
        decode_thread = threading.Thread(target=self.decode_worker, daemon=True)
        decode_thread.start()
        
        data = b""
        last_stats_time = time.time()
        
        print(f"[{self.name}] Receiving frames...")
        
        try:
            while self.is_running:
                # Receive frame size (4 bytes, unsigned int)
                while len(data) < 4:
                    chunk = self.client_socket.recv(4096)
                    if not chunk:
                        print(f"[{self.name}] Connection closed")
                        return
                    data += chunk
                
                # Unpack frame size
                frame_size = struct.unpack("!I", data[:4])[0]
                data = data[4:]
                
                # Sanity check frame size
                if frame_size > 5_000_000:  # Max 5MB per frame
                    print(f"[{self.name}] Invalid frame size: {frame_size}")
                    data = b""
                    continue
                
                # Receive frame data
                while len(data) < frame_size:
                    to_receive = min(4096, frame_size - len(data))
                    chunk = self.client_socket.recv(to_receive)
                    if not chunk:
                        print(f"[{self.name}] Connection lost")
                        return
                    data += chunk
                
                # Extract frame data
                frame_data = data[:frame_size]
                data = data[frame_size:]
                
                # Add to decode queue
                if not self.decode_queue.full():
                    self.decode_queue.put(frame_data)
                
                self.frame_count += 1
                self.bytes_received += frame_size + 4
                
                # Calculate and display statistics
                current_time = time.time()
                if current_time - last_stats_time >= 5.0:
                    elapsed = current_time - self.start_time
                    fps = self.frame_count / elapsed
                    mbps = (self.bytes_received * 8 / elapsed) / 1_000_000
                    avg_frame_kb = (self.bytes_received / self.frame_count) / 1024
                    
                    self.fps_history.append(fps)
                    avg_fps = sum(self.fps_history) / len(self.fps_history)
                    
                    print(f"[{self.name}] {self.frame_count} frames | "
                          f"FPS: {fps:.1f} (avg: {avg_fps:.1f}) | "
                          f"{mbps:.2f} Mbps | {avg_frame_kb:.1f} KB/frame")
                    
                    last_stats_time = current_time
                
        except Exception as e:
            print(f"[{self.name}] Receive error: {e}")
        
        finally:
            self.cleanup()
    
    def get_frame(self, timeout: float = 1.0):
        """Get latest frame"""
        try:
            return self.frame_queue.get(timeout=timeout)
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
        
        # Print final stats
        if self.start_time and self.frame_count > 0:
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed
            mbps = (self.bytes_received * 8 / elapsed) / 1_000_000
            print(f"[{self.name}] Final: {self.frame_count} frames, {fps:.1f} FPS, {mbps:.2f} Mbps")
        
        print(f"[{self.name}] Receiver stopped")

class X99OptimizedSLAMServer:
    """Optimized X99 SLAM server for WiFi streaming"""
    
    def __init__(self, left_port=9001, right_port=9002, buffer_size=65536):
        self.left_receiver = OptimizedCameraReceiver(left_port, "LEFT", buffer_size)
        self.right_receiver = OptimizedCameraReceiver(right_port, "RIGHT", buffer_size)
        
        # Import SLAM components
        try:
            from x99.x99_slam_server import ORBFeatureExtractor, YOLOSegmentator
            self.orb_extractor = ORBFeatureExtractor(n_features=3000)
            self.yolo = YOLOSegmentator()
        except ImportError:
            print("[WARNING] Cannot import SLAM components, will only display streams")
            self.orb_extractor = None
            self.yolo = None
        
        self.is_running = False
    
    def start_receivers(self):
        """Start both receivers"""
        print("\n" + "=" * 70)
        print("  X99 Optimized SLAM Server (WiFi)")
        print("=" * 70)
        print(f"Listening on ports: {self.left_receiver.port}, {self.right_receiver.port}")
        print("=" * 70 + "\n")
        
        # Start receivers in separate threads
        left_thread = threading.Thread(
            target=lambda: (
                self.left_receiver.start_server() and
                self.left_receiver.receive_frames()
            ),
            daemon=True
        )
        
        right_thread = threading.Thread(
            target=lambda: (
                self.right_receiver.start_server() and
                self.right_receiver.receive_frames()
            ),
            daemon=True
        )
        
        left_thread.start()
        right_thread.start()
        
        time.sleep(2)
        
        if not self.left_receiver.is_running or not self.right_receiver.is_running:
            print("\n[ERROR] Failed to start receivers")
            return False
        
        print("\n[OK] Both cameras connected via WiFi!\n")
        return True
    
    def process_frames(self, frame_left, frame_right):
        """Process stereo frames"""
        if self.orb_extractor is None:
            return frame_left  # Just return raw frame
        
        # Extract ORB features
        kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
        kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
        
        # Match features
        matches = self.orb_extractor.match_features(desc_left, desc_right)
        
        # YOLO segmentation
        processed = frame_left.copy()
        if self.yolo and self.yolo.model:
            results = self.yolo.segment(frame_left, conf=0.5)
            processed = self.yolo.draw_segments(frame_left, results)
        
        # Draw features
        frame_with_features = cv2.drawKeypoints(
            processed, kp_left, None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        
        # Overlay info
        info = [
            f"ORB: L={len(kp_left)}, R={len(kp_right)}",
            f"Matches: {len(matches)}",
            f"3D Points: {len(matches)}"
        ]
        
        y = 30
        for text in info:
            cv2.putText(frame_with_features, text, (10, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y += 25
        
        return frame_with_features
    
    def run(self):
        """Main processing loop"""
        if not self.start_receivers():
            return
        
        self.is_running = True
        
        print("[INFO] Starting SLAM processing...")
        print("[INFO] Press 'q' to quit\n")
        
        frame_count = 0
        
        try:
            while self.is_running:
                frame_left = self.left_receiver.get_frame(timeout=0.5)
                frame_right = self.right_receiver.get_frame(timeout=0.5)
                
                if frame_left is None or frame_right is None:
                    continue
                
                # Process frames
                if self.orb_extractor:
                    processed = self.process_frames(frame_left, frame_right)
                else:
                    processed = frame_left
                
                # Display
                cv2.imshow('X99 SLAM - Left + Processing', processed)
                cv2.imshow('X99 SLAM - Right Camera', frame_right)
                
                frame_count += 1
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop server"""
        self.is_running = False
        self.left_receiver.is_running = False
        self.right_receiver.is_running = False
        
        cv2.destroyAllWindows()
        print("\n[INFO] Server stopped")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 Optimized WiFi SLAM Server')
    parser.add_argument('--left-port', type=int, default=9001)
    parser.add_argument('--right-port', type=int, default=9002)
    parser.add_argument('--buffer-size', type=int, default=65536,
                       help='TCP receive buffer size')
    
    args = parser.parse_args()
    
    server = X99OptimizedSLAMServer(
        left_port=args.left_port,
        right_port=args.right_port,
        buffer_size=args.buffer_size
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        server.stop()

if __name__ == "__main__":
    main()