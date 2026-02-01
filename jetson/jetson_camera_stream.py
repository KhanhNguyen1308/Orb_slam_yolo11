#!/usr/bin/env python3
"""
Jetson Nano Camera Streamer - FIXED EXPOSURE CONTROL
Fixes white washout / overexposure issues
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
    """Streaming configuration with exposure control"""
    server_ip: str = "192.168.2.10"
    left_port: int = 9001
    right_port: int = 9002
    left_camera_id: int = 0
    right_camera_id: int = 1
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 75
    resize_before_encode: bool = True
    use_fast_codec: bool = True
    buffer_size: int = 65536
    tcp_nodelay: bool = True
    frame_skip: int = 0
    
    # NEW: Exposure control settings
    auto_exposure: bool = False  # False = Manual exposure
    exposure_value: int = -5  # -7 to 0 (lower = darker, prevents washout)
    brightness: float = 0.4  # 0.0 to 1.0 (default 0.5)
    contrast: float = 0.5  # 0.0 to 1.0
    saturation: float = 0.5  # 0.0 to 1.0
    auto_white_balance: bool = False  # Disable for consistency
    wb_temperature: int = 4600  # Daylight white balance
    gain: int = 0  # Manual gain (0 = no gain)
    
    # Adaptive brightness correction
    enable_adaptive_correction: bool = True  # Auto-fix overexposure
    brightness_threshold: float = 180.0  # Trigger correction above this
    use_clahe: bool = True  # Use CLAHE for better contrast

class OptimizedCameraStreamer:
    """Camera streamer with exposure control"""
    
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
        
        # Frame queues
        self.frame_queue = queue.Queue(maxsize=2)
        self.encoded_queue = queue.Queue(maxsize=5)
        
        # CLAHE for adaptive correction
        if self.config.use_clahe:
            self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        
        # Statistics
        self.frame_count = 0
        self.bytes_sent = 0
        self.start_time = None
        self.brightness_stats = []  # Track brightness
        
    def initialize_camera(self) -> bool:
        """Initialize camera with MANUAL EXPOSURE settings"""
        try:
            # Try GStreamer pipeline
            gstreamer_pipeline = (
                f'nvarguscamerasrc sensor-id={self.camera_id} ! '
                f'video/x-raw(memory:NVMM), width={self.width}, height={self.height}, '
                f'format=NV12, framerate={self.config.fps}/1 ! '
                f'nvvidconv ! video/x-raw, format=BGRx ! '
                f'videoconvert ! video/x-raw, format=BGR ! appsink'
            )
            
            self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                print(f"[INFO] GStreamer failed, using standard capture")
                self.cap = cv2.VideoCapture(self.camera_id)
                
                if not self.cap.isOpened():
                    print(f"[ERROR] Cannot open camera {self.camera_id}")
                    return False
                
                # Standard capture settings
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.cap.set(cv2.CAP_PROP_FORMAT, cv2.CV_8UC3)
                self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            
            # ===== CRITICAL: SET MANUAL EXPOSURE =====
            print(f"[CAM{self.camera_id}] Configuring exposure control...")
            
            # 1. Set MANUAL exposure mode
            if self.config.auto_exposure:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto
                print(f"[CAM{self.camera_id}] Auto exposure: ON")
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual
                self.cap.set(cv2.CAP_PROP_EXPOSURE, self.config.exposure_value)
                print(f"[CAM{self.camera_id}] Manual exposure: {self.config.exposure_value}")
            
            # 2. Set brightness/contrast
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.config.brightness)
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.config.contrast)
            #self.cap.set(cv2.CAP_PROP_SATURATION, self.config.saturation)
            print(f"[CAM{self.camera_id}] Brightness: {self.config.brightness:.2f}")
            
            # 3. White balance
            if not self.config.auto_white_balance:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.config.wb_temperature)
                print(f"[CAM{self.camera_id}] WB: Manual ({self.config.wb_temperature}K)")
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
                print(f"[CAM{self.camera_id}] WB: Auto")
            
            # 4. Gain control
            try:
                self.cap.set(cv2.CAP_PROP_GAIN, self.config.gain)
                print(f"[CAM{self.camera_id}] Gain: {self.config.gain}")
            except:
                pass  # Not all cameras support this
            
            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            
            print(f"[CAM{self.camera_id}] Resolution: {actual_width}x{actual_height}")
            print(f"[CAM{self.camera_id}] Actual exposure: {actual_exposure}")
            print(f"[CAM{self.camera_id}] Adaptive correction: {self.config.enable_adaptive_correction}")
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Camera {self.camera_id} init failed: {e}")
            return False
    
    def fix_overexposure(self, frame):
        """
        Adaptive brightness correction
        Fixes white washout in real-time
        """
        if not self.config.enable_adaptive_correction:
            return frame
        
        # Check brightness
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        
        # Track statistics
        self.brightness_stats.append(mean_brightness)
        if len(self.brightness_stats) > 30:
            self.brightness_stats.pop(0)
        
        # If too bright, apply correction
        if mean_brightness > self.config.brightness_threshold:
            
            if self.config.use_clahe:
                # Method 1: CLAHE (best quality)
                lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
                l, a, b = cv2.split(lab)
                
                # Apply CLAHE only to L channel
                l = self.clahe.apply(l)
                
                # Darken the L channel
                l = np.clip(l * 0.85, 0, 255).astype(np.uint8)
                
                lab = cv2.merge([l, a, b])
                frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            else:
                # Method 2: Gamma correction (faster)
                gamma = 1.3  # Darken
                inv_gamma = 1.0 / gamma
                table = np.array([((i / 255.0) ** inv_gamma) * 255
                                 for i in np.arange(0, 256)]).astype("uint8")
                frame = cv2.LUT(frame, table)
        
        return frame
    
    def connect_to_server(self) -> bool:
        """Connect to server"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.config.buffer_size)
                
                if self.config.tcp_nodelay:
                    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                
                print(f"[CAM{self.camera_id}] Connecting to {self.server_ip}:{self.port} " +
                      f"(attempt {attempt+1}/{max_retries})")
                
                self.sock.connect((self.server_ip, self.port))
                
                self.connected = True
                print(f"[CAM{self.camera_id}] Connected!")
                return True
                
            except Exception as e:
                print(f"[ERROR] Connection failed: {e}")
                if self.sock:
                    self.sock.close()
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
        
        return False
    
    def encode_frame_worker(self):
        """Async frame encoding with preprocessing"""
        if self.config.use_fast_codec:
            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY), self.quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 0,
                int(cv2.IMWRITE_JPEG_PROGRESSIVE), 0
            ]
        else:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        
        while self.is_running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
                
                # Apply exposure correction
                frame = self.fix_overexposure(frame)
                
                # Resize if needed
                if self.config.resize_before_encode and (
                    frame.shape[1] != self.width or frame.shape[0] != self.height
                ):
                    frame = cv2.resize(frame, (self.width, self.height), 
                                      interpolation=cv2.INTER_LINEAR)
                
                # Encode to JPEG
                result, encoded = cv2.imencode('.jpg', frame, encode_param)
                
                if result:
                    if not self.encoded_queue.full():
                        self.encoded_queue.put(encoded)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[ERROR] Encoding error: {e}")
    
    def stream(self):
        """Main streaming loop"""
        if not self.initialize_camera():
            return
        
        if not self.connect_to_server():
            return
        
        self.is_running = True
        self.start_time = time.time()
        
        # Start encoding thread
        encode_thread = threading.Thread(target=self.encode_frame_worker, daemon=True)
        encode_thread.start()
        
        print(f"[CAM{self.camera_id}] Streaming started")
        
        frame_skip_counter = 0
        last_stats_time = time.time()
        
        try:
            while self.is_running:
                ret, frame = self.cap.read()
                
                if not ret:
                    print(f"[WARNING] Failed to capture frame")
                    continue
                
                # Frame skipping
                if self.config.frame_skip > 0:
                    frame_skip_counter += 1
                    if frame_skip_counter <= self.config.frame_skip:
                        continue
                    frame_skip_counter = 0
                
                # Add to encoding queue
                if not self.frame_queue.full():
                    self.frame_queue.put(frame)
                
                # Send encoded frames
                try:
                    encoded_frame = self.encoded_queue.get_nowait()
                    
                    data = encoded_frame.tobytes()
                    size = len(data)
                    size_bytes = struct.pack("!I", size)
                    
                    self.sock.sendall(size_bytes + data)
                    
                    self.frame_count += 1
                    self.bytes_sent += size + 4
                    
                    # Statistics
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        elapsed = current_time - self.start_time
                        fps = self.frame_count / elapsed
                        mbps = (self.bytes_sent * 8 / elapsed) / 1_000_000
                        avg_frame_kb = (self.bytes_sent / self.frame_count) / 1024
                        
                        avg_brightness = np.mean(self.brightness_stats) if self.brightness_stats else 0
                        
                        print(f"[CAM{self.camera_id}] {self.frame_count} frames | "
                              f"{fps:.1f} FPS | {mbps:.2f} Mbps | "
                              f"{avg_frame_kb:.1f} KB/frame | "
                              f"Brightness: {avg_brightness:.1f}")
                        
                        last_stats_time = current_time
                    
                except queue.Empty:
                    time.sleep(0.001)
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"[ERROR] Connection lost: {e}")
                    break
                
        except KeyboardInterrupt:
            print(f"\n[CAM{self.camera_id}] Stream interrupted")
        
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
        
        if self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > 0:
                fps = self.frame_count / elapsed
                mbps = (self.bytes_sent * 8 / elapsed) / 1_000_000
                print(f"[CAM{self.camera_id}] Final: {self.frame_count} frames, "
                      f"{fps:.1f} FPS, {mbps:.2f} Mbps")
        
        print(f"[CAM{self.camera_id}] Stopped")

class DualCameraStreamer:
    """Dual camera manager"""
    
    def __init__(self, config: StreamConfig):
        self.config = config
        self.left_streamer = None
        self.right_streamer = None
        self.threads = []
        
    def print_config(self):
        """Print configuration"""
        print("=" * 70)
        print("  Jetson Nano - FIXED EXPOSURE Camera Streamer")
        print("=" * 70)
        print(f"Server IP:        {self.config.server_ip}")
        print(f"Left camera:      /dev/video{self.config.left_camera_id} → Port {self.config.left_port}")
        print(f"Right camera:     /dev/video{self.config.right_camera_id} → Port {self.config.right_port}")
        print(f"Resolution:       {self.config.width}x{self.config.height}")
        print(f"Target FPS:       {self.config.fps}")
        print(f"JPEG Quality:     {self.config.jpeg_quality}")
        print("=" * 70)
        print(f"Exposure Control:")
        print(f"  Mode:           {'AUTO' if self.config.auto_exposure else 'MANUAL'}")
        if not self.config.auto_exposure:
            print(f"  Value:          {self.config.exposure_value}")
        print(f"  Brightness:     {self.config.brightness:.2f}")
        print(f"  Adaptive Fix:   {self.config.enable_adaptive_correction}")
        if self.config.enable_adaptive_correction:
            print(f"  Method:         {'CLAHE' if self.config.use_clahe else 'Gamma'}")
            print(f"  Threshold:      {self.config.brightness_threshold:.0f}")
        print("=" * 70)
    
    def start(self):
        """Start streaming"""
        self.print_config()
        
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
        
        left_thread = threading.Thread(target=self.left_streamer.stream, daemon=False)
        right_thread = threading.Thread(target=self.right_streamer.stream, daemon=False)
        
        self.threads = [left_thread, right_thread]
        
        print("\n[INFO] Starting camera streams...\n")
        
        left_thread.start()
        time.sleep(0.5)
        right_thread.start()
        
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

def main():
    parser = argparse.ArgumentParser(
        description='Jetson Nano Camera Streamer - FIXED EXPOSURE',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default (manual exposure -5, good for most cases)
  python3 %(prog)s --server 192.168.1.100
  
  # Very bright environment (lower exposure)
  python3 %(prog)s --server 192.168.1.100 --exposure -7
  
  # Dark environment (higher exposure)
  python3 %(prog)s --server 192.168.1.100 --exposure -3
  
  # Auto exposure (let camera decide)
  python3 %(prog)s --server 192.168.1.100 --auto-exposure
  
  # Disable adaptive correction (use only manual exposure)
  python3 %(prog)s --server 192.168.1.100 --no-adaptive
        """
    )
    
    parser.add_argument('--server', type=str, default='192.168.2.10')
    parser.add_argument('--left-port', type=int, default=9002)
    parser.add_argument('--right-port', type=int, default=9001)
    parser.add_argument('--left-camera', type=int, default=0)
    parser.add_argument('--right-camera', type=int, default=1)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--quality', type=int, default=75)
    
    # NEW: Exposure control arguments
    parser.add_argument('--auto-exposure', action='store_true',
                       help='Use auto exposure (default: manual)')
    parser.add_argument('--exposure', type=int, default=-5,
                       help='Manual exposure value -7 to 0 (default: -5)')
    parser.add_argument('--brightness', type=float, default=0.4,
                       help='Brightness 0.0-1.0 (default: 0.4)')
    parser.add_argument('--no-adaptive', action='store_true',
                       help='Disable adaptive brightness correction')
    parser.add_argument('--no-clahe', action='store_true',
                       help='Use gamma instead of CLAHE')
    parser.add_argument('--threshold', type=float, default=180.0,
                       help='Brightness threshold for correction (default: 180)')
    
    args = parser.parse_args()
    
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
        auto_exposure=args.auto_exposure,
        exposure_value=args.exposure,
        brightness=args.brightness,
        enable_adaptive_correction=not args.no_adaptive,
        use_clahe=not args.no_clahe,
        brightness_threshold=args.threshold
    )
    
    streamer = DualCameraStreamer(config)
    
    try:
        streamer.start()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
        streamer.stop()

if __name__ == "__main__":
    main()