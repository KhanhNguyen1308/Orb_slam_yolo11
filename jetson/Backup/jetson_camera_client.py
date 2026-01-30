#!/usr/bin/env python3
"""
Jetson Nano Camera Streaming Client
Streams dual OV9832 USB cameras to X99 server via network
"""

import cv2
import numpy as np
import socket
import pickle
import struct
import time
import threading
from dataclasses import dataclass
import argparse

@dataclass
class StreamConfig:
    """Streaming configuration"""
    server_ip: str = "192.168.2.10"
    left_port: int = 9001
    right_port: int = 9002
    left_camera_id: int = 0
    right_camera_id: int = 1
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 80  # Lower for Jetson Nano (60-80 recommended)

class CameraStreamer:
    """Stream camera over TCP socket"""
    
    def __init__(self, camera_id: int, server_ip: str, port: int, 
                 width: int = 640, height: int = 480, quality: int = 80):
        self.camera_id = camera_id
        self.server_ip = server_ip
        self.port = port
        self.width = width
        self.height = height
        self.quality = quality
        
        self.cap = None
        self.sock = None
        self.is_running = False
        self.connected = False
        
    def initialize_camera(self) -> bool:
        """Initialize camera"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                print(f"[ERROR] Cannot open camera {self.camera_id}")
                return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"[OK] Camera {self.camera_id} initialized: {actual_width}x{actual_height}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Camera {self.camera_id} initialization failed: {e}")
            return False
    
    def connect_to_server(self) -> bool:
        """Connect to X99 server"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                
                print(f"[INFO] Connecting to {self.server_ip}:{self.port} (attempt {attempt+1}/{max_retries})")
                self.sock.connect((self.server_ip, self.port))
                
                self.connected = True
                print(f"[OK] Connected to server on port {self.port}")
                return True
                
            except Exception as e:
                print(f"[ERROR] Connection failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    print(f"[ERROR] Failed to connect after {max_retries} attempts")
                    return False
        
        return False
    
    def stream(self):
        """Main streaming loop"""
        if not self.initialize_camera():
            return
        
        if not self.connect_to_server():
            return
        
        self.is_running = True
        frame_count = 0
        start_time = time.time()
        
        print(f"[INFO] Starting stream from camera {self.camera_id}")
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        
        try:
            while self.is_running:
                ret, frame = self.cap.read()
                
                if not ret:
                    print(f"[WARNING] Failed to capture frame from camera {self.camera_id}")
                    continue
                
                # Encode frame as JPEG
                result, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
                
                if not result:
                    continue
                
                # Serialize frame data
                data = pickle.dumps(encoded_frame, protocol=pickle.HIGHEST_PROTOCOL)
                message_size = struct.pack("!L", len(data))
                
                # Send frame size + frame data
                try:
                    self.sock.sendall(message_size + data)
                    frame_count += 1
                    
                    # Calculate FPS
                    if frame_count % 30 == 0:
                        elapsed = time.time() - start_time
                        fps = frame_count / elapsed
                        print(f"[INFO] Camera {self.camera_id}: {frame_count} frames, {fps:.1f} FPS")
                    
                except (BrokenPipeError, ConnectionResetError) as e:
                    print(f"[ERROR] Connection lost: {e}")
                    break
                
        except KeyboardInterrupt:
            print(f"\n[INFO] Stream interrupted by user")
        
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
        
        print(f"[INFO] Camera {self.camera_id} stream stopped")

class DualCameraStreamer:
    """Manage dual camera streaming"""
    
    def __init__(self, config: StreamConfig):
        self.config = config
        self.left_streamer = None
        self.right_streamer = None
        self.threads = []
        
    def start(self):
        """Start streaming both cameras"""
        print("=" * 60)
        print("  Jetson Nano - Dual Camera Streamer")
        print("=" * 60)
        print(f"Server: {self.config.server_ip}")
        print(f"Left camera (ID {self.config.left_camera_id}) -> Port {self.config.left_port}")
        print(f"Right camera (ID {self.config.right_camera_id}) -> Port {self.config.right_port}")
        print(f"Resolution: {self.config.width}x{self.config.height}")
        print(f"JPEG Quality: {self.config.jpeg_quality}")
        print("=" * 60)
        
        # Create streamers
        self.left_streamer = CameraStreamer(
            camera_id=self.config.left_camera_id,
            server_ip=self.config.server_ip,
            port=self.config.left_port,
            width=self.config.width,
            height=self.config.height,
            quality=self.config.jpeg_quality
        )
        
        self.right_streamer = CameraStreamer(
            camera_id=self.config.right_camera_id,
            server_ip=self.config.server_ip,
            port=self.config.right_port,
            width=self.config.width,
            height=self.config.height,
            quality=self.config.jpeg_quality
        )
        
        # Start streaming threads
        left_thread = threading.Thread(target=self.left_streamer.stream, daemon=False)
        right_thread = threading.Thread(target=self.right_streamer.stream, daemon=False)
        
        self.threads = [left_thread, right_thread]
        
        print("\n[INFO] Starting camera streams...\n")
        
        left_thread.start()
        time.sleep(0.5)  # Small delay between camera starts
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

def main():
    parser = argparse.ArgumentParser(description='Jetson Nano Dual Camera Streamer')
    parser.add_argument('--server', type=str, default='192.168.1.100',
                       help='X99 server IP address')
    parser.add_argument('--left-port', type=int, default=9001,
                       help='Left camera streaming port')
    parser.add_argument('--right-port', type=int, default=9002,
                       help='Right camera streaming port')
    parser.add_argument('--left-camera', type=int, default=0,
                       help='Left camera device ID')
    parser.add_argument('--right-camera', type=int, default=1,
                       help='Right camera device ID')
    parser.add_argument('--width', type=int, default=640,
                       help='Frame width')
    parser.add_argument('--height', type=int, default=480,
                       help='Frame height')
    parser.add_argument('--quality', type=int, default=80,
                       help='JPEG quality (1-100, lower = faster)')
    
    args = parser.parse_args()
    
    config = StreamConfig(
        server_ip=args.server,
        left_port=args.left_port,
        right_port=args.right_port,
        left_camera_id=args.left_camera,
        right_camera_id=args.right_camera,
        width=args.width,
        height=args.height,
        jpeg_quality=args.quality
    )
    
    streamer = DualCameraStreamer(config)
    
    try:
        streamer.start()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
        streamer.stop()

if __name__ == "__main__":
    main()