#!/usr/bin/env python3
"""
Jetson Navigation Controller
Receives path from X99 server, sends motor commands to Pico via USB
"""

import serial
import json
import time
import requests
import numpy as np
from threading import Thread, Lock
import sys

# ===== CONFIGURATION =====
# X99 Server
X99_SERVER_URL = "http://192.168.1.100:5000"  # Change to your X99 IP

# Pico USB Serial
PICO_SERIAL_PORT = "/dev/ttyACM0"  # or /dev/ttyUSB0
PICO_BAUDRATE = 115200

# Robot specs (must match Pico config)
TRACK_WIDTH = 0.275  # meters
MAX_LINEAR_VEL = 0.5  # m/s
MAX_ANGULAR_VEL = 1.0  # rad/s

# Navigation params
WAYPOINT_REACHED_THRESHOLD = 0.15  # meters
LOOKAHEAD_DISTANCE = 0.3  # Pure pursuit lookahead
UPDATE_RATE = 10  # Hz

# Map resolution
MAP_RESOLUTION = 0.02  # 2cm per cell (from persistent_map.py)
MAP_CENTER = 400  # Grid center (800x800 / 2)

class PicoInterface:
    """Serial communication with Raspberry Pico"""
    
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.lock = Lock()
        self.connected = False
        
    def connect(self):
        """Connect to Pico via USB serial"""
        try:
            self.serial = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=0.1
            )
            time.sleep(2)  # Wait for Pico to reset
            
            # Enable motors
            self.send_command({'cmd': 'enable'})
            time.sleep(0.1)
            
            # Check status
            self.send_command({'cmd': 'status'})
            response = self.read_response()
            
            if response and response.get('status') == 'ok':
                self.connected = True
                print("[Pico] Connected successfully")
                return True
            else:
                print("[Pico] Connection failed - no response")
                return False
                
        except Exception as e:
            print(f"[Pico] Connection error: {e}")
            return False
    
    def send_command(self, cmd):
        """Send JSON command to Pico"""
        if not self.serial:
            return False
            
        with self.lock:
            try:
                cmd_str = json.dumps(cmd) + '\n'
                self.serial.write(cmd_str.encode())
                return True
            except Exception as e:
                print(f"[Pico] Send error: {e}")
                return False
    
    def read_response(self):
        """Read JSON response from Pico (non-blocking)"""
        if not self.serial:
            return None
            
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                if line:
                    return json.loads(line)
        except Exception as e:
            pass
        
        return None
    
    def set_velocity(self, linear, angular):
        """
        Set robot velocity
        linear: m/s (forward/backward)
        angular: rad/s (turn rate)
        """
        # Clamp values
        linear = np.clip(linear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        angular = np.clip(angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        cmd = {
            'cmd': 'velocity',
            'linear': linear,
            'angular': angular
        }
        
        return self.send_command(cmd)
    
    def stop(self):
        """Emergency stop"""
        return self.send_command({'cmd': 'stop'})
    
    def close(self):
        """Close serial connection"""
        if self.serial:
            self.stop()
            time.sleep(0.1)
            self.send_command({'cmd': 'disable'})
            self.serial.close()
            self.connected = False

class NavigationController:
    """Main navigation controller"""
    
    def __init__(self, x99_url, pico_port):
        self.x99_url = x99_url
        self.pico = PicoInterface(pico_port, PICO_BAUDRATE)
        
        self.current_path = []
        self.current_waypoint_idx = 0
        self.robot_pose = [0, 0, 0]  # x, y, theta
        self.goal_reached = False
        self.navigation_active = False
        
        self.lock = Lock()
        
    def start(self):
        """Initialize system"""
        print("=" * 60)
        print("  AUTONOMOUS NAVIGATION SYSTEM")
        print("=" * 60)
        
        # Connect to Pico
        print("\n[1/2] Connecting to Pico...")
        if not self.pico.connect():
            print("✗ Failed to connect to Pico")
            return False
        
        print("✓ Pico connected")
        
        # Check X99 server
        print("\n[2/2] Checking X99 server...")
        try:
            response = requests.get(f"{self.x99_url}/api/status", timeout=2)
            if response.status_code == 200:
                print(f"✓ X99 server online: {self.x99_url}")
            else:
                print("✗ X99 server not responding")
                return False
        except Exception as e:
            print(f"✗ Cannot reach X99 server: {e}")
            return False
        
        print("\n" + "=" * 60)
        print("  SYSTEM READY")
        print("=" * 60)
        return True
    
    def get_robot_pose(self):
        """Get current robot pose from X99"""
        try:
            response = requests.get(f"{self.x99_url}/api/map_2d", timeout=1)
            if response.status_code == 200:
                data = response.json()
                self.robot_pose = data.get('robot_pose', [0, 0, 0])
                return self.robot_pose
        except Exception as e:
            print(f"[Nav] Failed to get pose: {e}")
        
        return self.robot_pose
    
    def request_path(self, goal_x, goal_y):
        """
        Request path from X99 server
        goal_x, goal_y: grid coordinates
        """
        # Get current position in grid coordinates
        pose = self.get_robot_pose()
        start_x = int(MAP_CENTER + pose[0] / MAP_RESOLUTION)
        start_y = int(MAP_CENTER - pose[2] / MAP_RESOLUTION)  # Note: Z in world is Y in grid
        
        print(f"\n[Nav] Planning path:")
        print(f"  Start: ({start_x}, {start_y}) -> Goal: ({goal_x}, {goal_y})")
        
        try:
            response = requests.post(
                f"{self.x99_url}/api/plan_path",
                json={'start': [start_x, start_y], 'goal': [goal_x, goal_y]},
                timeout=5
            )
            
            if response.status_code == 200:
                data = response.json()
                path = data.get('path', [])
                
                if path:
                    # Convert grid path to world coordinates
                    world_path = []
                    for grid_x, grid_y in path:
                        world_x = (grid_x - MAP_CENTER) * MAP_RESOLUTION
                        world_y = -(grid_y - MAP_CENTER) * MAP_RESOLUTION
                        world_path.append([world_x, world_y])
                    
                    with self.lock:
                        self.current_path = world_path
                        self.current_waypoint_idx = 0
                        self.goal_reached = False
                    
                    print(f"✓ Path received: {len(world_path)} waypoints")
                    print(f"  Distance: {data.get('distance', 0):.2f}m")
                    return True
                else:
                    print("✗ No path found")
                    return False
            else:
                print(f"✗ Path planning failed: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"✗ Path request failed: {e}")
            return False
    
    def pure_pursuit_control(self):
        """
        Pure pursuit controller
        Returns: (linear_vel, angular_vel)
        """
        with self.lock:
            if not self.current_path or self.goal_reached:
                return 0, 0
            
            if self.current_waypoint_idx >= len(self.current_path):
                self.goal_reached = True
                return 0, 0
        
        # Get current pose
        x, y, theta = self.robot_pose
        
        # Find lookahead point
        lookahead_point = None
        
        with self.lock:
            for i in range(self.current_waypoint_idx, len(self.current_path)):
                wx, wy = self.current_path[i]
                dist = np.sqrt((wx - x)**2 + (wy - y)**2)
                
                # Check if waypoint is reached
                if i == self.current_waypoint_idx and dist < WAYPOINT_REACHED_THRESHOLD:
                    self.current_waypoint_idx += 1
                    print(f"[Nav] Waypoint {i}/{len(self.current_path)} reached")
                    continue
                
                # Find lookahead point
                if dist >= LOOKAHEAD_DISTANCE:
                    lookahead_point = [wx, wy]
                    break
            
            # Use last waypoint if no lookahead found
            if lookahead_point is None and self.current_waypoint_idx < len(self.current_path):
                lookahead_point = self.current_path[-1]
        
        if lookahead_point is None:
            self.goal_reached = True
            return 0, 0
        
        # Calculate control
        wx, wy = lookahead_point
        
        # Transform to robot frame
        dx = wx - x
        dy = wy - y
        
        # Angle to target
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - theta
        
        # Normalize angle to [-pi, pi]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        
        # Pure pursuit control law
        distance = np.sqrt(dx**2 + dy**2)
        
        # Linear velocity (proportional to distance)
        linear_vel = min(MAX_LINEAR_VEL, distance * 0.5)
        
        # Reduce speed when turning
        if abs(angle_diff) > np.pi / 6:  # 30 degrees
            linear_vel *= 0.5
        
        # Angular velocity (proportional to angle error)
        angular_vel = 2.0 * angle_diff
        angular_vel = np.clip(angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        return linear_vel, angular_vel
    
    def navigate_to_goal(self, goal_x, goal_y):
        """
        Navigate to goal (blocking)
        goal_x, goal_y: grid coordinates
        """
        # Request path
        if not self.request_path(goal_x, goal_y):
            print("✗ Cannot plan path")
            return False
        
        print("\n[Nav] Starting navigation...")
        self.navigation_active = True
        self.goal_reached = False
        
        rate = 1.0 / UPDATE_RATE
        
        try:
            while not self.goal_reached:
                start_time = time.time()
                
                # Update robot pose
                self.get_robot_pose()
                
                # Calculate control
                linear, angular = self.pure_pursuit_control()
                
                # Send to Pico
                if self.navigation_active:
                    self.pico.set_velocity(linear, angular)
                
                # Status update
                with self.lock:
                    progress = (self.current_waypoint_idx / len(self.current_path)) * 100 if self.current_path else 0
                
                if int(time.time()) % 2 == 0:  # Print every 2 seconds
                    print(f"[Nav] Progress: {progress:.1f}% | Vel: {linear:.2f}m/s | Turn: {angular:.2f}rad/s")
                
                # Sleep to maintain rate
                elapsed = time.time() - start_time
                if elapsed < rate:
                    time.sleep(rate - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[Nav] Navigation interrupted by user")
            self.pico.stop()
            return False
        
        # Goal reached
        self.pico.stop()
        print("\n✓ Goal reached!")
        self.navigation_active = False
        return True
    
    def stop_navigation(self):
        """Stop current navigation"""
        self.navigation_active = False
        self.pico.stop()
        with self.lock:
            self.current_path = []
            self.goal_reached = True
    
    def shutdown(self):
        """Cleanup"""
        print("\n[Nav] Shutting down...")
        self.stop_navigation()
        self.pico.close()
        print("✓ Shutdown complete")

def main():
    """Main entry point"""
    
    # Parse arguments
    if len(sys.argv) > 1:
        x99_url = sys.argv[1]
    else:
        x99_url = X99_SERVER_URL
    
    if len(sys.argv) > 2:
        pico_port = sys.argv[2]
    else:
        pico_port = PICO_SERIAL_PORT
    
    # Create controller
    nav = NavigationController(x99_url, pico_port)
    
    # Initialize
    if not nav.start():
        print("\n✗ Initialization failed")
        return
    
    try:
        # Interactive mode
        print("\n" + "=" * 60)
        print("  COMMANDS:")
        print("  g X Y  - Navigate to grid coordinates (X, Y)")
        print("  s      - Stop navigation")
        print("  q      - Quit")
        print("=" * 60)
        
        while True:
            cmd = input("\n> ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'q':
                break
            
            elif cmd[0] == 'g' and len(cmd) >= 3:
                try:
                    goal_x = int(cmd[1])
                    goal_y = int(cmd[2])
                    nav.navigate_to_goal(goal_x, goal_y)
                except ValueError:
                    print("✗ Invalid coordinates")
            
            elif cmd[0] == 's':
                nav.stop_navigation()
                print("✓ Navigation stopped")
            
            else:
                print("✗ Unknown command")
    
    except KeyboardInterrupt:
        print("\n\n[Nav] Interrupted")
    
    finally:
        nav.shutdown()

if __name__ == '__main__':
    main()