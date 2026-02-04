#!/usr/bin/env python3
"""
Test script for Tank Robot Control System
Run this from X99 server after both services are running
"""
import requests
import time
import sys

# Configuration
JETSON_URL = "http://192.168.2.13:5000"
X99_URL = "http://192.168.2.10:8080"

def test_color(text, color):
    colors = {
        'green': '\033[92m',
        'red': '\033[91m',
        'yellow': '\033[93m',
        'blue': '\033[94m',
        'end': '\033[0m'
    }
    return f"{colors.get(color, '')}{text}{colors['end']}"

def test_jetson_health():
    """Test Jetson Nano connection"""
    print("\n" + "="*60)
    print("TEST 1: Jetson Nano Health Check")
    print("="*60)
    try:
        response = requests.get(f"{JETSON_URL}/health", timeout=2)
        if response.status_code == 200:
            data = response.json()
            print(test_color("✓ Jetson connected", "green"))
            print(f"  Serial port: {data.get('serial_port')}")
            print(f"  Serial status: {data.get('serial_connected')}")
            return True
        else:
            print(test_color(f"✗ Unexpected status: {response.status_code}", "red"))
            return False
    except requests.RequestException as e:
        print(test_color(f"✗ Connection failed: {e}", "red"))
        return False

def test_motor_enable():
    """Test motor enable command"""
    print("\n" + "="*60)
    print("TEST 2: Motor Enable")
    print("="*60)
    try:
        response = requests.post(f"{JETSON_URL}/motor/enable", timeout=2)
        if response.status_code == 200:
            print(test_color("✓ Motors enabled successfully", "green"))
            return True
        else:
            print(test_color(f"✗ Enable failed: {response.status_code}", "red"))
            return False
    except requests.RequestException as e:
        print(test_color(f"✗ Request failed: {e}", "red"))
        return False

def test_velocity_command(linear, angular, duration=2):
    """Test velocity command"""
    print("\n" + "="*60)
    print(f"TEST 3: Velocity Command (Linear={linear}, Angular={angular})")
    print("="*60)
    try:
        response = requests.post(
            f"{JETSON_URL}/motor/velocity",
            json={'linear': linear, 'angular': angular},
            timeout=2
        )
        if response.status_code == 200:
            print(test_color(f"✓ Velocity command sent", "green"))
            print(f"  Running for {duration} seconds...")
            time.sleep(duration)
            return True
        else:
            print(test_color(f"✗ Command failed: {response.status_code}", "red"))
            return False
    except requests.RequestException as e:
        print(test_color(f"✗ Request failed: {e}", "red"))
        return False

def test_stop():
    """Test stop command"""
    print("\n" + "="*60)
    print("TEST 4: Stop Command")
    print("="*60)
    try:
        response = requests.post(f"{JETSON_URL}/motor/stop", timeout=2)
        if response.status_code == 200:
            print(test_color("✓ Stop command successful", "green"))
            return True
        else:
            print(test_color(f"✗ Stop failed: {response.status_code}", "red"))
            return False
    except requests.RequestException as e:
        print(test_color(f"✗ Request failed: {e}", "red"))
        return False

def test_web_interface():
    """Test X99 web interface"""
    print("\n" + "="*60)
    print("TEST 5: X99 Web Interface")
    print("="*60)
    try:
        response = requests.get(f"{X99_URL}/", timeout=2)
        if response.status_code == 200:
            print(test_color("✓ Web interface accessible", "green"))
            print(f"  URL: {X99_URL}")
            return True
        else:
            print(test_color(f"✗ Unexpected status: {response.status_code}", "red"))
            return False
    except requests.RequestException as e:
        print(test_color(f"✗ Connection failed: {e}", "red"))
        return False

def run_automated_test_sequence():
    """Run full test sequence"""
    print(test_color("\n" + "="*60, "blue"))
    print(test_color("TANK ROBOT AUTOMATED TEST SUITE", "blue"))
    print(test_color("="*60, "blue"))
    
    results = []
    
    # Test 1: Jetson Health
    results.append(("Jetson Health", test_jetson_health()))
    time.sleep(1)
    
    # Test 2: Enable Motors
    results.append(("Motor Enable", test_motor_enable()))
    time.sleep(1)
    
    # Test 3: Forward motion
    results.append(("Forward Motion", test_velocity_command(0.2, 0, 2)))
    
    # Test 4: Backward motion
    results.append(("Backward Motion", test_velocity_command(-0.2, 0, 2)))
    
    # Test 5: Left turn
    results.append(("Left Turn", test_velocity_command(0, -1.0, 2)))
    
    # Test 6: Right turn
    results.append(("Right Turn", test_velocity_command(0, 1.0, 2)))
    
    # Test 7: Stop
    results.append(("Stop Command", test_stop()))
    time.sleep(1)
    
    # Test 8: Web interface
    results.append(("Web Interface", test_web_interface()))
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    for test_name, result in results:
        status = test_color("PASS", "green") if result else test_color("FAIL", "red")
        print(f"{test_name:.<40} {status}")
    
    passed = sum(1 for _, r in results if r)
    total = len(results)
    print("="*60)
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print(test_color("\n🎉 All tests passed!", "green"))
    else:
        print(test_color(f"\n⚠️  {total - passed} test(s) failed", "yellow"))
    
    print("\nNext steps:")
    print(f"  1. Open browser: {X99_URL}")
    print("  2. Use joystick or WASD keys to control")
    print("  3. Monitor Jetson logs: journalctl -u jetson-serial-bridge -f")

def manual_control():
    """Manual control mode for testing"""
    print("\n" + "="*60)
    print("MANUAL CONTROL MODE")
    print("="*60)
    print("Commands:")
    print("  w/s - Forward/Backward (0.2 m/s)")
    print("  a/d - Left/Right turn (1.0 rad/s)")
    print("  e   - Enable motors")
    print("  x   - Stop")
    print("  q   - Quit")
    print("="*60)
    
    try:
        while True:
            cmd = input("\nCommand: ").lower().strip()
            
            if cmd == 'q':
                print("Exiting...")
                break
            elif cmd == 'e':
                test_motor_enable()
            elif cmd == 'w':
                requests.post(f"{JETSON_URL}/motor/velocity", 
                            json={'linear': 0.2, 'angular': 0})
                print("Moving forward...")
            elif cmd == 's':
                requests.post(f"{JETSON_URL}/motor/velocity", 
                            json={'linear': -0.2, 'angular': 0})
                print("Moving backward...")
            elif cmd == 'a':
                requests.post(f"{JETSON_URL}/motor/velocity", 
                            json={'linear': 0, 'angular': -1.0})
                print("Turning left...")
            elif cmd == 'd':
                requests.post(f"{JETSON_URL}/motor/velocity", 
                            json={'linear': 0, 'angular': 1.0})
                print("Turning right...")
            elif cmd == 'x':
                test_stop()
            else:
                print("Unknown command")
    except KeyboardInterrupt:
        print("\n\nStopping...")
        test_stop()

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'manual':
        manual_control()
    else:
        run_automated_test_sequence()