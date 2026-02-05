"""
RP2040 High-Performance Stepper Controller (PIO based)
Optimized for high-speed serial communication with Jetson Nano
"""
import sys
import json
import time
import select
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

# ===== CONFIGURATION =====
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

# Pins
LEFT_STEP = 15
LEFT_DIR = 14
LEFT_EN = 4
RIGHT_STEP = 17
RIGHT_DIR = 16
RIGHT_EN = 8

# Safety
WATCHDOG_TIMEOUT_MS = 500  # Stop if no command received in 500ms

# ===== PIO STEPPER DRIVER =====
# PIO program to generate pulses
@asm_pio(set_init=PIO.OUT_LOW)
def stepper_pio():
    pull(noblock) .side(0)      # Check for new speed data
    mov(x, osr)                 # Move OSR to X
    jmp(not_x, "wait")          # If X is 0, just wait (stop)
    
    label("step_loop")
    set(pins, 1) [1]            # Step HIGH (2 cycles)
    set(pins, 0)                # Step LOW
    mov(y, x)                   # Load delay counter
    
    label("delay")
    jmp(y_dec, "delay")         # Delay loop
    jmp("step_loop")            # Repeat
    
    label("wait")
    wait(1, irq, 4)             # Wait for interrupt (dummy wait)

class PIOMotor:
    def __init__(self, sm_id, step_pin, dir_pin, en_pin, reversed=False):
        self.step_pin = Pin(step_pin, Pin.OUT)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.en_pin = Pin(en_pin, Pin.OUT)
        self.reversed = reversed
        
        self.en_pin.value(1) # Disable initially
        
        # Initialize State Machine
        self.sm = StateMachine(sm_id, stepper_pio, freq=1_000_000, set_base=self.step_pin)
        self.sm.active(1)
        self.current_speed = 0

    def set_speed(self, sps):
        """Set speed in steps per second"""
        if sps == 0:
            self.sm.put(0)
            self.current_speed = 0
            return

        # Direction control
        direction = 1 if sps > 0 else 0
        if self.reversed: direction = 1 - direction
        self.dir_pin.value(direction)
        
        # Calculate PIO delay
        # PIO freq = 1MHz via clock divider logic could be complex
        # Simplified: Higher value = Slower speed
        # Formula depends on PIO loop cycles. This is a rough mapping:
        sps = abs(int(sps))
        if sps < 10: sps = 0
        
        # Mapping generic formula for this PIO loop
        # Tuning needed based on exact freq
        delay = max(100, int(1_000_000 / sps) - 5) 
        self.sm.put(delay)
        self.current_speed = sps

    def enable(self, state):
        self.en_pin.value(0 if state else 1)
        if not state:
            self.set_speed(0)

# ===== MAIN CONTROLLER =====
class RobotController:
    def __init__(self):
        # Initialize motors with PIO State Machines 0 and 1
        self.left_motor = PIOMotor(0, LEFT_STEP, LEFT_DIR, LEFT_EN)
        self.right_motor = PIOMotor(1, RIGHT_STEP, RIGHT_DIR, RIGHT_EN)
        self.last_cmd_time = time.ticks_ms()
        self.enabled = False

    def process_command(self, line):
        try:
            cmd = json.loads(line)
            cmd_type = cmd.get('cmd')
            
            if cmd_type == 'velocity':
                self.last_cmd_time = time.ticks_ms()
                v = cmd.get('linear', 0)
                w = cmd.get('angular', 0)
                
                # Kinematics
                v_l = v - (w * TRACK_WIDTH / 2)
                v_r = v + (w * TRACK_WIDTH / 2)
                
                # Convert to Steps/s
                sps_l = v_l * STEPS_PER_METER
                sps_r = v_r * STEPS_PER_METER
                
                self.left_motor.set_speed(sps_l)
                self.right_motor.set_speed(sps_r)
                
            elif cmd_type == 'enable':
                self.enabled = True
                self.left_motor.enable(True)
                self.right_motor.enable(True)
                print(json.dumps({"status": "ok"}))
                
            elif cmd_type == 'disable':
                self.enabled = False
                self.left_motor.enable(False)
                self.right_motor.enable(False)
                print(json.dumps({"status": "ok"}))
                
            elif cmd_type == 'stop':
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                
            elif cmd_type == 'status':
                print(json.dumps({"status": "ok", "enabled": self.enabled}))

        except ValueError:
            pass # Ignore malformed JSON

    def loop(self):
        # Input buffer
        buffer = ""
        
        while True:
            # 1. Non-blocking Serial Read
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1)
                if char == '\n':
                    self.process_command(buffer)
                    buffer = ""
                else:
                    buffer += char
            
            # 2. Watchdog Safety
            if self.enabled and time.ticks_diff(time.ticks_ms(), self.last_cmd_time) > WATCHDOG_TIMEOUT_MS:
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                # Optional: print debug log
                
            # 3. No Sleep needed! Loop runs as fast as possible
            
if __name__ == '__main__':
    robot = RobotController()
    robot.loop()