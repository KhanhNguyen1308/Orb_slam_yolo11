"""
RP2040 Pico Motor Controller for Tracked Robot
Controls 2x NEMA17 motors via A4988 drivers
Receives commands from Jetson Nano via USB serial
"""

from machine import Pin, PWM
import time
import sys
import json

# ===== ROBOT CONFIGURATION =====
# Mechanical
SPROCKET_TEETH = 20           # Main sprocket teeth
GEAR_RATIO = 20 / 68          # Transmission ratio
CHAIN_PITCH = 0.015           # Chain pitch in meters (15mm)
TRACK_WIDTH = 0.275           # Distance between tracks in meters (275mm)

# NEMA17 specifications
STEPS_PER_REV = 200           # Full steps per revolution (1.8° per step)
MICROSTEPS = 16               # A4988 microstepping (1, 2, 4, 8, 16)

# Calculated constants
STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS
SPROCKET_CIRCUMFERENCE = SPROCKET_TEETH * CHAIN_PITCH
STEPS_PER_METER = (STEPS_PER_MOTOR_REV / GEAR_RATIO) / SPROCKET_CIRCUMFERENCE

# Speed limits
MAX_SPEED_MPS = 0.5           # Maximum speed in m/s
MAX_ANGULAR_SPEED = 1.0       # Maximum angular speed in rad/s

# ===== PIN CONFIGURATION =====
# Left Motor (A4988)
LEFT_STEP_PIN = 15
LEFT_DIR_PIN = 14
LEFT_ENABLE_PIN = 4

# Right Motor (A4988)
RIGHT_STEP_PIN = 17
RIGHT_DIR_PIN = 16
RIGHT_ENABLE_PIN = 8

# Optional: Microstepping pins (MS1, MS2, MS3)
# For 1/16 microstepping: MS1=HIGH, MS2=HIGH, MS3=HIGH

class A4988Motor:
    """Control single NEMA17 motor via A4988 driver"""
    
    def __init__(self, step_pin: int, dir_pin: int, enable_pin: int, name: str):
        self.name = name
        
        # Setup pins
        self.step = Pin(step_pin, Pin.OUT)
        self.dir = Pin(dir_pin, Pin.OUT)
        self.enable = Pin(enable_pin, Pin.OUT)
        
        self.step.value(0)
        self.dir.value(0)
        self.enable.value(1)  # Active LOW (1 = disabled)
        
        self.enabled = False
        self.current_speed = 0  # steps per second
        
    def enable_motor(self):
        """Enable motor (A4988 enable is active LOW)"""
        self.enable.value(0)
        self.enabled = True
    
    def disable_motor(self):
        """Disable motor"""
        self.enable.value(1)
        self.enabled = False
    
    def set_direction(self, forward: bool):
        """Set rotation direction"""
        self.dir.value(1 if forward else 0)
    
    def step_once(self):
        """Execute single step"""
        self.step.value(1)
        time.sleep_us(2)  # Minimum HIGH pulse width
        self.step.value(0)
        time.sleep_us(2)  # Minimum LOW pulse width

class TrackedRobot:
    """Differential drive tracked robot controller"""
    
    def __init__(self):
        # Initialize motors
        self.left_motor = A4988Motor(LEFT_STEP_PIN, LEFT_DIR_PIN, 
                                     LEFT_ENABLE_PIN, "LEFT")
        self.right_motor = A4988Motor(RIGHT_STEP_PIN, RIGHT_DIR_PIN, 
                                      RIGHT_ENABLE_PIN, "RIGHT")
        
        # Robot state
        self.target_linear_vel = 0.0   # m/s
        self.target_angular_vel = 0.0  # rad/s
        
        self.left_speed_sps = 0        # Left motor speed (steps per second)
        self.right_speed_sps = 0       # Right motor speed
        
        self.is_running = False
        
        # Timing for step generation
        self.last_left_step = time.ticks_us()
        self.last_right_step = time.ticks_us()
        
        print(f"[ROBOT] Initialized")
        print(f"  Steps per meter: {STEPS_PER_METER:.1f}")
        print(f"  Track width: {TRACK_WIDTH}m")
        print(f"  Gear ratio: {GEAR_RATIO:.3f}")
    
    def enable(self):
        """Enable both motors"""
        self.left_motor.enable_motor()
        self.right_motor.enable_motor()
        self.is_running = True
        print("[ROBOT] Motors enabled")
    
    def disable(self):
        """Disable both motors"""
        self.left_motor.disable_motor()
        self.right_motor.disable_motor()
        self.is_running = False
        print("[ROBOT] Motors disabled")
    
    def set_velocity(self, linear: float, angular: float):
        """
        Set target velocity using differential drive kinematics
        linear: forward velocity in m/s (positive = forward)
        angular: angular velocity in rad/s (positive = counter-clockwise)
        """
        # Clamp velocities
        linear = max(-MAX_SPEED_MPS, min(MAX_SPEED_MPS, linear))
        angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular))
        
        self.target_linear_vel = linear
        self.target_angular_vel = angular
        
        # Differential drive kinematics
        # v_left = v - (w * L) / 2
        # v_right = v + (w * L) / 2
        v_left = linear - (angular * TRACK_WIDTH) / 2.0
        v_right = linear + (angular * TRACK_WIDTH) / 2.0
        
        # Convert m/s to steps/second
        self.left_speed_sps = int(v_left * STEPS_PER_METER)
        self.right_speed_sps = int(v_right * STEPS_PER_METER)
        
        # Set motor directions
        self.left_motor.set_direction(self.left_speed_sps >= 0)
        self.right_motor.set_direction(self.right_speed_sps >= 0)
        
        # Use absolute values for speed
        self.left_speed_sps = abs(self.left_speed_sps)
        self.right_speed_sps = abs(self.right_speed_sps)
    
    def stop(self):
        """Stop both motors"""
        self.set_velocity(0, 0)
    
    def update(self):
        """Update motor steps - call this in main loop"""
        if not self.is_running:
            return
        
        current_time = time.ticks_us()
        
        # Left motor
        if self.left_speed_sps > 0:
            step_interval = 1_000_000 // self.left_speed_sps  # microseconds
            if time.ticks_diff(current_time, self.last_left_step) >= step_interval:
                self.left_motor.step_once()
                self.last_left_step = current_time
        
        # Right motor
        if self.right_speed_sps > 0:
            step_interval = 1_000_000 // self.right_speed_sps
            if time.ticks_diff(current_time, self.last_right_step) >= step_interval:
                self.right_motor.step_once()
                self.last_right_step = current_time
    
    def get_status(self) -> dict:
        """Get robot status"""
        return {
            'enabled': self.is_running,
            'linear_vel': self.target_linear_vel,
            'angular_vel': self.target_angular_vel,
            'left_sps': self.left_speed_sps,
            'right_sps': self.right_speed_sps
        }

class SerialCommandHandler:
    """Handle commands from Jetson Nano via USB serial"""
    
    def __init__(self, robot: TrackedRobot):
        self.robot = robot
        self.command_buffer = ""
    
    def process_command(self, cmd: str):
        """Process JSON command"""
        try:
            data = json.loads(cmd)
            cmd_type = data.get('cmd')
            
            if cmd_type == 'enable':
                self.robot.enable()
                self.send_response({'status': 'ok', 'msg': 'enabled'})
                
            elif cmd_type == 'disable':
                self.robot.disable()
                self.send_response({'status': 'ok', 'msg': 'disabled'})
                
            elif cmd_type == 'velocity':
                linear = data.get('linear', 0.0)
                angular = data.get('angular', 0.0)
                self.robot.set_velocity(linear, angular)
                self.send_response({'status': 'ok', 'linear': linear, 'angular': angular})
                
            elif cmd_type == 'stop':
                self.robot.stop()
                self.send_response({'status': 'ok', 'msg': 'stopped'})
                
            elif cmd_type == 'status':
                status = self.robot.get_status()
                self.send_response({'status': 'ok', 'data': status})
                
            else:
                self.send_response({'status': 'error', 'msg': f'unknown command: {cmd_type}'})
                
        except Exception as e:
            self.send_response({'status': 'error', 'msg': str(e)})
    
    def send_response(self, data: dict):
        """Send JSON response"""
        response = json.dumps(data)
        print(response)
        sys.stdout.flush()
    
    def update(self):
        """Check for incoming commands"""
        # Check if data available on stdin
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            char = sys.stdin.read(1)
            
            if char == '\n':
                # Process complete command
                if self.command_buffer:
                    self.process_command(self.command_buffer)
                    self.command_buffer = ""
            else:
                self.command_buffer += char

def main():
    """Main control loop"""
    print("\n" + "="*50)
    print("  RP2040 Tracked Robot Controller")
    print("="*50)
    print(f"  NEMA17 Motors: 2x")
    print(f"  Driver: A4988")
    print(f"  Microstepping: 1/{MICROSTEPS}")
    print(f"  Sprocket: {SPROCKET_TEETH} teeth")
    print(f"  Gear ratio: {GEAR_RATIO:.3f}")
    print(f"  Track width: {TRACK_WIDTH*1000:.0f}mm")
    print("="*50)
    
    # Initialize robot
    robot = TrackedRobot()
    
    # Simple control without serial (for testing)
    # Comment this out when using with Jetson
    robot.enable()
    
    print("\n[INFO] Starting control loop")
    print("[INFO] Send commands via serial or modify code for testing")
    
    # Test sequence (comment out when using with Jetson)
    test_mode = False  # Set to True for standalone testing
    
    if test_mode:
        print("\n[TEST] Running test sequence...")
        
        # Forward
        print("[TEST] Forward 0.2 m/s for 2 seconds")
        robot.set_velocity(0.2, 0)
        for _ in range(2000):
            robot.update()
            time.sleep_ms(1)
        
        # Stop
        print("[TEST] Stop")
        robot.stop()
        time.sleep(1)
        
        # Turn right
        print("[TEST] Turn right for 2 seconds")
        robot.set_velocity(0, -0.5)
        for _ in range(2000):
            robot.update()
            time.sleep_ms(1)
        
        # Stop
        robot.stop()
        robot.disable()
        print("[TEST] Test complete")
    
    else:
        # Production mode: wait for serial commands
        try:
            while True:
                robot.update()
                time.sleep_us(10)  # Small delay
                
        except KeyboardInterrupt:
            print("\n[INFO] Shutting down...")
            robot.stop()
            robot.disable()

if __name__ == "__main__":
    main()