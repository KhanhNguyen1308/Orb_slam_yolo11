#!/usr/bin/env python3
"""
Jetson Nano Optimized Camera Streaming Client
High-performance streaming over WiFi with compression
"""

import cv2
import numpy as np
import socket
import struct
import time
import threading
from dataclasses import dataclass
import argparse
import queue

@dataclass
class StreamConfig:
    """Optimized streaming configuration for WiFi"""
    server_ip: str = "192.168.2.10"
    left_port: int = 9001
    right_port: int = 9002
    left_camera_id: int = 0
    right_camera_id: int = 1
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 75  # Optimized for WiFi
    resize_before_encode: bool = True  # Resize before encoding
    use_fast_codec: bool = True  # Use faster codec settings
    buffer_size: int = 65536  # Socket buffer size
    tcp_nodelay: bool = True  # Disable Nagle's algorithm
    frame_skip: int = 0  # Skip frames if needed (0 = no skip)

class OptimizedCameraStreamer:
    """Optimized camera streamer for WiFi networks"""
    
    def __init__(self, camera_id: int, server_ip: str, port: int, 
                 width: int = 640, height: int = 480, 
                 quality: int = 75, config: StreamConfig = None):
        self.camera_id = camera_id
        self.server_ip = server_ip
        self.port = port
        self.width = width
        self.height = height
        self.quality = quality
        self.config = config or StreamConfig()
        
        self.cap = None
        self.sock = None
        self.is_running = False
        self.connected = False
        
        # Frame queue for async encoding
        self.frame_queue = queue.Queue(maxsize=2)
        self.encoded_queue = queue.Queue(maxsize=5)
        
        # Statistics
        self.frame_count = 0
        self.bytes_sent = 0
        self.start_time = None
        
    def initialize_camera(self) -> bool:
        """Initialize camera with optimized settings"""
        try:
            # Try GStreamer pipeline for better performance on Jetson
            gstreamer_pipeline = (
                f'v4l2src device=/dev/video{self.camera_id} ! '
                f'video/x-raw, width={self.width}, height={self.height}, '
                f'framerate={self.config.fps}/1 ! '
                f'videoconvert ! appsink'
            )
            
            # Try GStreamer first
            self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                # Fallback to standard OpenCV
                print(f"[INFO] GStreamer failed, using standard capture")
                self.cap = cv2.VideoCapture(self.camera_id)
                
                if not self.cap.isOpened():
                    print(f"[ERROR] Cannot open camera {self.camera_id}")
                    return False
                
                # Set properties for standard capture
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer lag
            
            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[OK] Camera {self.camera_id}: {actual_width}x{actual_height}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Camera {self.camera_id} init failed: {e}")
            return False
    
    def connect_to_server(self) -> bool:
        """Connect to server with optimized TCP settings"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
                # Optimize TCP settings for streaming
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.config.buffer_size)
                
                # Disable Nagle's algorithm for lower latency
                if self.config.tcp_nodelay:
                    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                # Set TCP keepalive
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                
                print(f"[INFO] Connecting to {self.server_ip}:{self.port} " +
                      f"(attempt {attempt+1}/{max_retries})")
                
                self.sock.connect((self.server_ip, self.port))
                
                self.connected = True
                print(f"[OK] Connected to server on port {self.port}")
                return True
                
            except Exception as e:
                print(f"[ERROR] Connection failed: {e}")
                if self.sock:
                    self.sock.close()
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
        
        return False
    
    def encode_frame_worker(self):
        """Async frame encoding thread"""
        # Optimized JPEG encoding parameters
        if self.config.use_fast_codec:
            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY), self.quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 0,  # Faster encoding
                int(cv2.IMWRITE_JPEG_PROGRESSIVE), 0
            ]
        else:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        
        while self.is_running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
                
                # Resize if needed (can reduce bandwidth significantly)
                if self.config.resize_before_encode and (
                    frame.shape[1] != self.width or frame.shape[0] != self.height
                ):
                    frame = cv2.resize(frame, (self.width, self.height), 
                                      interpolation=cv2.INTER_LINEAR)
                
                # Encode to JPEG
                result, encoded = cv2.imencode('.jpg', frame, encode_param)
                
                if result:
                    # Add to send queue
                    if not self.encoded_queue.full():
                        self.encoded_queue.put(encoded)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[ERROR] Encoding error: {e}")
    
    def stream(self):
        """Main streaming loop with optimizations"""
        if not self.initialize_camera():
            return
        
        if not self.connect_to_server():
            return
        
        self.is_running = True
        self.start_time = time.time()
        
        # Start encoding thread
        encode_thread = threading.Thread(target=self.encode_frame_worker, daemon=True)
        encode_thread.start()
        
        print(f"[INFO] Streaming camera {self.camera_id}")
        print(f"[INFO] Quality: {self.quality}, TCP_NODELAY: {self.config.tcp_nodelay}")
        
        frame_skip_counter = 0
        last_stats_time = time.time()
        
        try:
            while self.is_running:
                ret, frame = self.cap.read()
                
                if not ret:
                    print(f"[WARNING] Failed to capture frame")
                    continue
                
                # Frame skipping if configured
                if self.config.frame_skip > 0:
                    frame_skip_counter += 1
                    if frame_skip_counter <= self.config.frame_skip:
                        continue
                    frame_skip_counter = 0
                
                # Add to encoding queue (non-blocking)
                if not self.frame_queue.full():
                    self.frame_queue.put(frame)
                
                # Send encoded frames
                try:
                    encoded_frame = self.encoded_queue.get_nowait()
                    
                    # Send frame size + data
                    data = encoded_frame.tobytes()
                    size = len(data)
                    
                    # Pack size as 4 bytes
                    size_bytes = struct.pack("!I", size)
                    
                    # Send all data
                    self.sock.sendall(size_bytes + data)
                    
                    self.frame_count += 1
                    self.bytes_sent += size + 4
                    
                    # Print statistics every 5 seconds
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        elapsed = current_time - self.start_time
                        fps = self.frame_count / elapsed
                        mbps = (self.bytes_sent * 8 / elapsed) / 1_000_000
                        avg_frame_kb = (self.bytes_sent / self.frame_count) / 1024
                        
                        print(f"[CAM{self.camera_id}] {self.frame_count} frames | "
                              f"{fps:.1f} FPS | {mbps:.2f} Mbps | "
                              f"{avg_frame_kb:.1f} KB/frame")
                        
                        last_stats_time = current_time
                    
                except queue.Empty:
                    # No encoded frame ready yet
                    time.sleep(0.001)
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"[ERROR] Connection lost: {e}")
                    break
                
        except KeyboardInterrupt:
            print(f"\n[INFO] Stream interrupted")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.is_running = False
        
        if self.cap:
            self.cap.release()
        
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        
        # Print final statistics
        if self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > 0:
                fps = self.frame_count / elapsed
                mbps = (self.bytes_sent * 8 / elapsed) / 1_000_000
                print(f"[CAM{self.camera_id}] Final: {self.frame_count} frames, "
                      f"{fps:.1f} FPS, {mbps:.2f} Mbps")
        
        print(f"[INFO] Camera {self.camera_id} stopped")

class DualCameraStreamer:
    """Manage dual camera streaming"""
    
    def __init__(self, config: StreamConfig):
        self.config = config
        self.left_streamer = None
        self.right_streamer = None
        self.threads = []
        
    def print_config(self):
        """Print configuration"""
        print("=" * 70)
        print("  Jetson Nano - Optimized WiFi Dual Camera Streamer")
        print("=" * 70)
        print(f"Server IP:        {self.config.server_ip}")
        print(f"Left camera:      /dev/video{self.config.left_camera_id} → Port {self.config.left_port}")
        print(f"Right camera:     /dev/video{self.config.right_camera_id} → Port {self.config.right_port}")
        print(f"Resolution:       {self.config.width}x{self.config.height}")
        print(f"Target FPS:       {self.config.fps}")
        print(f"JPEG Quality:     {self.config.jpeg_quality}")
        print(f"TCP_NODELAY:      {self.config.tcp_nodelay}")
        print(f"Socket Buffer:    {self.config.buffer_size} bytes")
        print(f"Frame Skip:       {self.config.frame_skip}")
        print("=" * 70)
        
        # Calculate estimated bandwidth
        # Rough estimate: width * height * quality_factor * fps / 8
        quality_factor = self.config.jpeg_quality / 100 * 0.3  # Compression ratio estimate
        est_mbps = (self.config.width * self.config.height * quality_factor * 
                   self.config.fps * 2) / 1_000_000  # x2 for dual cameras
        print(f"Estimated BW:     ~{est_mbps:.1f} Mbps (both cameras)")
        print("=" * 70)
    
    def start(self):
        """Start streaming both cameras"""
        self.print_config()
        
        # Create streamers
        self.left_streamer = OptimizedCameraStreamer(
            camera_id=self.config.left_camera_id,
            server_ip=self.config.server_ip,
            port=self.config.left_port,
            width=self.config.width,
            height=self.config.height,
            quality=self.config.jpeg_quality,
            config=self.config
        )
        
        self.right_streamer = OptimizedCameraStreamer(
            camera_id=self.config.right_camera_id,
            server_ip=self.config.server_ip,
            port=self.config.right_port,
            width=self.config.width,
            height=self.config.height,
            quality=self.config.jpeg_quality,
            config=self.config
        )
        
        # Start streaming threads
        left_thread = threading.Thread(target=self.left_streamer.stream, daemon=False)
        right_thread = threading.Thread(target=self.right_streamer.stream, daemon=False)
        
        self.threads = [left_thread, right_thread]
        
        print("\n[INFO] Starting camera streams...\n")
        
        left_thread.start()
        time.sleep(0.5)
        right_thread.start()
        
        # Wait for threads
        try:
            for thread in self.threads:
                thread.join()
        except KeyboardInterrupt:
            print("\n[INFO] Shutting down...")
            self.stop()
    
    def stop(self):
        """Stop all streams"""
        if self.left_streamer:
            self.left_streamer.is_running = False
        if self.right_streamer:
            self.right_streamer.is_running = False

def detect_wifi_interface():
    """Detect WiFi interface and show signal strength"""
    try:
        import subprocess
        result = subprocess.run(['iwconfig'], capture_output=True, text=True)
        
        if 'ESSID' in result.stdout:
            lines = result.stdout.split('\n')
            for i, line in enumerate(lines):
                if 'wlan' in line or 'wlp' in line:
                    print(f"[WiFi] Interface: {line.split()[0]}")
                    if i+1 < len(lines):
                        quality_line = lines[i+1]
                        if 'Quality' in quality_line or 'Signal' in quality_line:
                            print(f"[WiFi] {quality_line.strip()}")
            return True
        else:
            print("[WiFi] No WiFi interface detected")
            return False
            
    except Exception as e:
        print(f"[WiFi] Cannot detect: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Jetson Nano Optimized WiFi Dual Camera Streamer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Low bandwidth (good for weak WiFi)
  python3 %(prog)s --server 192.168.1.100 --width 320 --height 240 --quality 60
  
  # Balanced (recommended)
  python3 %(prog)s --server 192.168.1.100 --width 640 --height 480 --quality 75
  
  # High quality (strong WiFi required)
  python3 %(prog)s --server 192.168.1.100 --width 1280 --height 720 --quality 85
        """
    )
    
    parser.add_argument('--server', type=str, required=True,
                       help='X99 server IP address')
    parser.add_argument('--left-port', type=int, default=9001)
    parser.add_argument('--right-port', type=int, default=9002)
    parser.add_argument('--left-camera', type=int, default=0)
    parser.add_argument('--right-camera', type=int, default=1)
    parser.add_argument('--width', type=int, default=640,
                       choices=[320, 640, 1280])
    parser.add_argument('--height', type=int, default=480,
                       choices=[240, 480, 720])
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--quality', type=int, default=75,
                       help='JPEG quality 1-100 (lower=faster, 60-80 recommended)')
    parser.add_argument('--buffer-size', type=int, default=65536,
                       help='TCP buffer size in bytes')
    parser.add_argument('--no-tcp-nodelay', action='store_true',
                       help='Disable TCP_NODELAY (may increase latency)')
    parser.add_argument('--frame-skip', type=int, default=0,
                       help='Skip N frames between sends (0=no skip)')
    
    args = parser.parse_args()
    
    # Show WiFi info
    print("\nChecking WiFi connection...\n")
    detect_wifi_interface()
    print()
    
    # Create config
    config = StreamConfig(
        server_ip=args.server,
        left_port=args.left_port,
        right_port=args.right_port,
        left_camera_id=args.left_camera,
        right_camera_id=args.right_camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        jpeg_quality=args.quality,
        buffer_size=args.buffer_size,
        tcp_nodelay=not args.no_tcp_nodelay,
        frame_skip=args.frame_skip
    )
    
    streamer = DualCameraStreamer(config)
    
    try:
        streamer.start()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
        streamer.stop()

if __name__ == "__main__":
    main()