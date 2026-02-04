#x99_headless.py
#!/usr/bin/env python3
"""
X99 Optimized Stream Receiver for WiFi - Headless (No Display)
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
        
        # Latest frame for processing
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
    def start_server(self):
        """Start TCP server with optimized settings"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
            
            self.server_socket.bind(("0.0.0.0", self.port))
            self.server_socket.listen(1)
            
            print(f"[{self.name}] Listening on port {self.port}...")
            print(f"[{self.name}] Waiting for Jetson connection...")
            
            # Set timeout for accept
            self.server_socket.settimeout(30.0)
            
            try:
                self.client_socket, addr = self.server_socket.accept()
            except socket.timeout:
                print(f"[{self.name}] Timeout waiting for connection (30s)")
                return False
            
            # Remove timeout for data reception
            self.client_socket.settimeout(None)
            
            # Set TCP options
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
                    # Update latest frame
                    with self.frame_lock:
                        self.latest_frame = frame
                    
                    # Also add to queue
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
                # Receive frame size (4 bytes)
                while len(data) < 4:
                    chunk = self.client_socket.recv(4096)
                    if not chunk:
                        print(f"[{self.name}] Connection closed")
                        return
                    data += chunk
                
                frame_size = struct.unpack("!I", data[:4])[0]
                data = data[4:]
                
                # Sanity check
                if frame_size > 5_000_000:
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
                
                # Statistics
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
    
    def get_latest_frame(self):
        """Get latest decoded frame"""
        with self.frame_lock:
            return self.latest_frame
    
    def get_frame(self, timeout: float = 1.0):
        """Get frame from queue"""
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
        
        # Final stats
        if self.start_time and self.frame_count > 0:
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed
            mbps = (self.bytes_received * 8 / elapsed) / 1_000_000
            print(f"[{self.name}] Final: {self.frame_count} frames, {fps:.1f} FPS, {mbps:.2f} Mbps")
        
        print(f"[{self.name}] Receiver stopped")

class X99HeadlessSLAMServer:
    """Headless X99 SLAM server for WiFi streaming (no GUI)"""
    
    def __init__(self, left_port=9001, right_port=9002, buffer_size=65536, 
                 save_frames=False, save_interval=30):
        self.left_receiver = OptimizedCameraReceiver(left_port, "LEFT", buffer_size)
        self.right_receiver = OptimizedCameraReceiver(right_port, "RIGHT", buffer_size)
        
        # Import SLAM components
        try:
            import sys
            sys.path.append('/home/ndk/Projects/Slam_Yolo')
            from x99_slam_server import ORBFeatureExtractor, YOLOSegmentator
            self.orb_extractor = ORBFeatureExtractor(n_features=3000)
            self.yolo = YOLOSegmentator()
            print("[SLAM] Components loaded successfully")
        except ImportError as e:
            print(f"[WARNING] Cannot import SLAM components: {e}")
            self.orb_extractor = None
            self.yolo = None
        
        self.is_running = False
        self.save_frames = save_frames
        self.save_interval = save_interval
        
        # Statistics
        self.processed_frames = 0
        self.start_time = None
    
    def start_receivers(self):
        """Start both receivers"""
        print("\n" + "=" * 70)
        print("  X99 Headless SLAM Server (WiFi)")
        print("=" * 70)
        
        # Show network info
        import subprocess
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            ips = result.stdout.strip().split()
            print(f"X99 IP addresses: {', '.join(ips)}")
            if ips:
                print(f"→ Jetson should connect to one of these IPs")
        except:
            pass
        
        print(f"Listening on ports: {self.left_receiver.port}, {self.right_receiver.port}")
        print(f"Mode: Headless (no display)")
        print("=" * 70 + "\n")
        
        # Start receivers
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
        
        # Wait for connections
        print("\n[WAITING] For camera connections from Jetson Nano...")
        
        max_wait = 60
        for i in range(max_wait):
            time.sleep(1)
            if self.left_receiver.is_running and self.right_receiver.is_running:
                break
            
            if i % 5 == 0:
                status_left = "✓" if self.left_receiver.is_running else "⏳"
                status_right = "✓" if self.right_receiver.is_running else "⏳"
                print(f"  {i}s - Left: {status_left} Right: {status_right}")
        
        if not self.left_receiver.is_running or not self.right_receiver.is_running:
            print("\n[ERROR] Failed to start receivers")
            return False
        
        print("\n[OK] Both cameras connected via WiFi!\n")
        return True
    
    def process_frames(self, frame_left, frame_right):
        """Process stereo frames"""
        if self.orb_extractor is None:
            return  # Just receive, no processing
        
        # Extract ORB features
        kp_left, desc_left = self.orb_extractor.extract_features(frame_left)
        kp_right, desc_right = self.orb_extractor.extract_features(frame_right)
        
        # Match features
        matches = self.orb_extractor.match_features(desc_left, desc_right)
        
        # YOLO segmentation (optional, can be slow)
        # if self.yolo and self.yolo.model:
        #     results = self.yolo.segment(frame_left, conf=0.5)
        
        return len(kp_left), len(kp_right), len(matches)
    
    def run(self):
        """Main processing loop"""
        if not self.start_receivers():
            return
        
        self.is_running = True
        self.start_time = time.time()
        
        print("[INFO] SLAM processing started (headless mode)")
        print("[INFO] Press Ctrl+C to stop")
        print()
        
        last_stats_time = time.time()
        last_save_time = time.time()
        
        try:
            while self.is_running:
                # Get latest frames (non-blocking)
                frame_left = self.left_receiver.get_latest_frame()
                frame_right = self.right_receiver.get_latest_frame()
                
                if frame_left is not None and frame_right is not None:
                    # Process SLAM
                    if self.orb_extractor:
                        stats = self.process_frames(frame_left, frame_right)
                        self.processed_frames += 1
                    
                    # Save frames periodically
                    if self.save_frames:
                        current_time = time.time()
                        if current_time - last_save_time >= self.save_interval:
                            timestamp = int(current_time)
                            cv2.imwrite(f'/tmp/frame_left_{timestamp}.jpg', frame_left)
                            cv2.imwrite(f'/tmp/frame_right_{timestamp}.jpg', frame_right)
                            print(f"[SAVE] Saved frames at {timestamp}")
                            last_save_time = current_time
                
                # Print statistics
                current_time = time.time()
                if current_time - last_stats_time >= 10.0:
                    elapsed = current_time - self.start_time
                    process_fps = self.processed_frames / elapsed
                    
                    print(f"\n[STATS] Runtime: {elapsed:.0f}s")
                    print(f"  Processed: {self.processed_frames} frames ({process_fps:.1f} FPS)")
                    if self.orb_extractor:
                        print(f"  SLAM: Active")
                    print()
                    
                    last_stats_time = current_time
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop server"""
        self.is_running = False
        self.left_receiver.is_running = False
        self.right_receiver.is_running = False
        
        print("\n[INFO] Server stopped")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='X99 Headless WiFi SLAM Server')
    parser.add_argument('--left-port', type=int, default=9001)
    parser.add_argument('--right-port', type=int, default=9002)
    parser.add_argument('--buffer-size', type=int, default=65536)
    parser.add_argument('--save-frames', action='store_true',
                       help='Save frames periodically to /tmp')
    parser.add_argument('--save-interval', type=int, default=30,
                       help='Interval in seconds to save frames')
    
    args = parser.parse_args()
    
    server = X99HeadlessSLAMServer(
        left_port=args.left_port,
        right_port=args.right_port,
        buffer_size=args.buffer_size,
        save_frames=args.save_frames,
        save_interval=args.save_interval
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        server.stop()

if __name__ == "__main__":
    main()