#!/usr/bin/env python3
"""
Jetson Nano Navigation Controller
Receives path from X99, controls robot via RP2040 Pico
"""

import serial
import json
import time
import numpy as np
import socket
import struct
from typing import List, Tuple, Optional
import threading
import queue

class PicoController:
    """Interface to RP2040 Pico motor controller via USB serial"""
    
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        
        # Command queue
        self.cmd_queue = queue.Queue()
        self.response_queue = queue.Queue()
        
    def connect(self) -> bool:
        """Connect to Pico"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            
            time.sleep(2)  # Wait for Pico to initialize
            
            # Test connection
            response = self.send_command({'cmd': 'status'}, wait_response=True)
            
            if response and response.get('status') == 'ok':
                self.connected = True
                print(f"[Pico] Connected on {self.port}")
                return True
            else:
                print(f"[Pico] Connection failed")
                return False
                
        except Exception as e:
            print(f"[Pico] Connection error: {e}")
            return False
    
    def send_command(self, cmd: dict, wait_response: bool = False, timeout: float = 1.0) -> Optional[dict]:
        """Send JSON command to Pico"""
        if not self.connected or not self.serial:
            return None
        
        try:
            # Send command
            cmd_str = json.dumps(cmd) + '\n'
            self.serial.write(cmd_str.encode())
            self.serial.flush()
            
            if wait_response:
                # Wait for response
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if self.serial.in_waiting > 0:
                        line = self.serial.readline().decode().strip()
                        try:
                            response = json.loads(line)
                            return response
                        except json.JSONDecodeError:
                            continue
                return None
            
            return {'status': 'sent'}
            
        except Exception as e:
            print(f"[Pico] Send error: {e}")
            return None
    
    def enable_motors(self) -> bool:
        """Enable motors"""
        response = self.send_command({'cmd': 'enable'}, wait_response=True)
        return response and response.get('status') == 'ok'
    
    def disable_motors(self) -> bool:
        """Disable motors"""
        response = self.send_command({'cmd': 'disable'}, wait_response=True)
        return response and response.get('status') == 'ok'
    
    def set_velocity(self, linear: float, angular: float):
        """Set robot velocity"""
        cmd = {
            'cmd': 'velocity',
            'linear': linear,
            'angular': angular
        }
        self.send_command(cmd, wait_response=False)
    
    def stop(self):
        """Emergency stop"""
        self.send_command({'cmd': 'stop'}, wait_response=False)
    
    def disconnect(self):
        """Disconnect from Pico"""
        if self.serial:
            self.stop()
            self.disable_motors()
            self.serial.close()
            self.connected = False
            print("[Pico] Disconnected")

class PurePursuitController:
    """Pure Pursuit path following controller"""
    
    def __init__(self, lookahead_distance: float = 0.3, max_linear_vel: float = 0.3,
                 max_angular_vel: float = 1.0):
        self.lookahead = lookahead_distance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Path
        self.path = []
        self.current_waypoint_idx = 0
    
    def update_pose(self, x: float, y: float, theta: float):
        """Update robot pose from SLAM"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
    
    def set_path(self, path: List[Tuple[float, float]]):
        """Set new path to follow"""
        self.path = path
        self.current_waypoint_idx = 0
        print(f"[PurePursuit] New path with {len(path)} waypoints")
    
    def get_lookahead_point(self) -> Optional[Tuple[float, float]]:
        """Find lookahead point on path"""
        if not self.path or self.current_waypoint_idx >= len(self.path):
            return None
        
        robot_pos = np.array([self.robot_x, self.robot_y])
        
        # Search for point at lookahead distance
        for i in range(self.current_waypoint_idx, len(self.path)):
            waypoint = np.array(self.path[i])
            dist = np.linalg.norm(waypoint - robot_pos)
            
            if dist >= self.lookahead:
                self.current_waypoint_idx = i
                return tuple(waypoint)
        
        # Return last point if all closer than lookahead
        return self.path[-1]
    
    def compute_control(self) -> Tuple[float, float]:
        """
        Compute control commands (linear_vel, angular_vel)
        Returns: (v, w) or (None, None) if no path
        """
        lookahead_point = self.get_lookahead_point()
        
        if lookahead_point is None:
            return 0.0, 0.0
        
        # Calculate steering angle
        dx = lookahead_point[0] - self.robot_x
        dy = lookahead_point[1] - self.robot_y
        
        # Angle to lookahead point
        alpha = np.arctan2(dy, dx) - self.robot_theta
        
        # Normalize angle to [-pi, pi]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        # Pure pursuit curvature
        ld = np.sqrt(dx**2 + dy**2)
        if ld < 0.01:  # Too close to waypoint
            return 0.0, 0.0
        
        curvature = 2 * np.sin(alpha) / ld
        
        # Compute velocities
        linear_vel = self.max_linear_vel
        angular_vel = curvature * linear_vel
        
        # Clamp angular velocity
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Reduce linear velocity when turning sharply
        if abs(angular_vel) > 0.5:
            linear_vel *= 0.5
        
        return linear_vel, angular_vel
    
    def is_goal_reached(self, tolerance: float = 0.1) -> bool:
        """Check if goal is reached"""
        if not self.path:
            return True
        
        goal = np.array(self.path[-1])
        robot_pos = np.array([self.robot_x, self.robot_y])
        dist = np.linalg.norm(goal - robot_pos)
        
        return dist < tolerance

class X99CommandReceiver:
    """Receive navigation commands from X99 server"""
    
    def __init__(self, port: int = 9003):
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.is_running = False
        
        self.path_queue = queue.Queue(maxsize=1)
        self.pose_queue = queue.Queue(maxsize=1)
    
    def start_server(self):
        """Start TCP server to receive commands"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("0.0.0.0", self.port))
            self.server_socket.listen(1)
            
            print(f"[NavReceiver] Listening on port {self.port}")
            self.client_socket, addr = self.server_socket.accept()
            
            print(f"[NavReceiver] Connected from {addr}")
            return True
            
        except Exception as e:
            print(f"[NavReceiver] Server error: {e}")
            return False
    
    def receive_loop(self):
        """Receive commands from X99"""
        self.is_running = True
        data = b""
        
        while self.is_running:
            try:
                # Receive data size
                while len(data) < 4:
                    chunk = self.client_socket.recv(4096)
                    if not chunk:
                        return
                    data += chunk
                
                msg_size = struct.unpack("!I", data[:4])[0]
                data = data[4:]
                
                # Receive message
                while len(data) < msg_size:
                    chunk = self.client_socket.recv(4096)
                    if not chunk:
                        return
                    data += chunk
                
                msg_data = data[:msg_size]
                data = data[msg_size:]
                
                # Parse JSON
                message = json.loads(msg_data.decode())
                
                msg_type = message.get('type')
                
                if msg_type == 'path':
                    # New path command
                    path = message.get('path', [])
                    if self.path_queue.full():
                        self.path_queue.get_nowait()
                    self.path_queue.put(path)
                    print(f"[NavReceiver] Received path with {len(path)} waypoints")
                
                elif msg_type == 'pose':
                    # Robot pose update
                    pose = message.get('pose', [0, 0, 0])
                    if self.pose_queue.full():
                        self.pose_queue.get_nowait()
                    self.pose_queue.put(pose)
                
            except Exception as e:
                print(f"[NavReceiver] Receive error: {e}")
                break
    
    def get_path(self, timeout: float = 0.1) -> Optional[List[Tuple[float, float]]]:
        """Get latest path"""
        try:
            return self.path_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_pose(self, timeout: float = 0.01) -> Optional[Tuple[float, float, float]]:
        """Get latest pose"""
        try:
            return self.pose_queue.get(timeout=timeout)
        except queue.Empty:
            return None

class NavigationController:
    """Main navigation controller on Jetson Nano"""
    
    def __init__(self, pico_port: str = "/dev/ttyACM0", nav_port: int = 9003):
        self.pico = PicoController(pico_port)
        self.controller = PurePursuitController(
            lookahead_distance=0.3,
            max_linear_vel=0.3,
            max_angular_vel=1.0
        )
        self.nav_receiver = X99CommandReceiver(nav_port)
        
        self.is_running = False
    
    def initialize(self) -> bool:
        """Initialize all components"""
        print("\n" + "="*60)
        print("  Jetson Nano Navigation Controller")
        print("="*60)
        
        # Connect to Pico
        if not self.pico.connect():
            print("[ERROR] Failed to connect to Pico")
            return False
        
        # Enable motors
        if not self.pico.enable_motors():
            print("[ERROR] Failed to enable motors")
            return False
        
        print("[OK] Motors enabled")
        
        # Start navigation receiver
        nav_thread = threading.Thread(
            target=lambda: (
                self.nav_receiver.start_server() and
                self.nav_receiver.receive_loop()
            ),
            daemon=True
        )
        nav_thread.start()
        
        time.sleep(1)
        
        if not self.nav_receiver.is_running:
            print("[ERROR] Failed to start navigation receiver")
            return False
        
        print("[OK] Navigation receiver started")
        print("="*60 + "\n")
        
        return True
    
    def run(self):
        """Main control loop"""
        if not self.initialize():
            return
        
        self.is_running = True
        
        print("[INFO] Navigation controller running")
        print("[INFO] Waiting for path from X99...")
        
        last_control_time = time.time()
        control_rate = 0.1  # 10 Hz
        
        try:
            while self.is_running:
                current_time = time.time()
                
                # Update robot pose from X99
                pose = self.nav_receiver.get_pose()
                if pose:
                    self.controller.update_pose(pose[0], pose[1], pose[2])
                
                # Check for new path
                new_path = self.nav_receiver.get_path()
                if new_path:
                    self.controller.set_path(new_path)
                
                # Control loop
                if current_time - last_control_time >= control_rate:
                    # Compute control
                    linear, angular = self.controller.compute_control()
                    
                    # Send to Pico
                    self.pico.set_velocity(linear, angular)
                    
                    # Check if goal reached
                    if self.controller.is_goal_reached():
                        print("[Navigation] Goal reached!")
                        self.pico.stop()
                        self.controller.set_path([])  # Clear path
                    
                    last_control_time = current_time
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n[INFO] Shutting down...")
        
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown navigation controller"""
        self.is_running = False
        self.nav_receiver.is_running = False
        
        if self.pico.connected:
            self.pico.stop()
            self.pico.disable_motors()
            self.pico.disconnect()
        
        print("[INFO] Navigation controller stopped")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Jetson Nano Navigation Controller')
    parser.add_argument('--pico-port', type=str, default='/dev/ttyACM0',
                       help='RP2040 Pico serial port')
    parser.add_argument('--nav-port', type=int, default=9003,
                       help='Port to receive navigation commands from X99')
    
    args = parser.parse_args()
    
    controller = NavigationController(
        pico_port=args.pico_port,
        nav_port=args.nav_port
    )
    
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")

if __name__ == "__main__":
    main()