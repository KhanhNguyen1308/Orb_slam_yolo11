"""
=============================================================================
TEST UTILITIES - BALANCE ROBOT
=============================================================================
File này chứa các hàm test riêng lẻ cho từng module
Dùng để troubleshooting và kiểm tra phần cứng

Cách sử dụng:
1. Upload file này lên board
2. Import trong REPL:
   >>> import test_utils
   >>> test_utils.test_i2c()
   >>> test_utils.test_servo_sweep()
   >>> ...
=============================================================================
"""

from machine import I2C, Pin
import time
import math
import ustruct

# Import từ main code (nếu cần)
try:
    from balance_robot_v2 import Config, PCA9685, Servo, MPU6050, ADXL345
except:
    print("Warning: Could not import from main code. Some tests may not work.")
    # Định nghĩa Config mặc định
    class Config:
        I2C0_ID = 0
        I2C0_SDA_PIN = 20
        I2C0_SCL_PIN = 21
        I2C0_FREQ = 400000
        
        I2C1_ID = 1
        I2C1_SDA_PIN = 18
        I2C1_SCL_PIN = 19
        I2C1_FREQ = 400000
        
        ADDR_MPU6050 = 0x68
        ADDR_ADXL345 = 0x53
        ADDR_PCA9685 = 0x40
        SERVO_LEFT_CHANNEL = 0
        SERVO_RIGHT_CHANNEL = 1
        SERVO_CENTER = 90


# =============================================================================
# TEST 1: I2C BUS SCAN
# =============================================================================

def test_i2c():
    """
    Scan I2C bus và hiển thị các thiết bị tìm thấy
    """
    print("=" * 60)
    print("TEST 1: I2C BUS SCAN")
    print("=" * 60)
    
    try:
        # I2C Bus 0 (MPU6050 + PCA9685)
        print("\n--- I2C Bus 0 (MPU6050 + PCA9685) ---")
        i2c0 = I2C(
            Config.I2C0_ID,
            scl=Pin(Config.I2C0_SCL_PIN),
            sda=Pin(Config.I2C0_SDA_PIN),
            freq=Config.I2C0_FREQ
        )
        
        print(f"I2C Bus: {Config.I2C0_ID}")
        print(f"SDA Pin: {Config.I2C0_SDA_PIN}")
        print(f"SCL Pin: {Config.I2C0_SCL_PIN}")
        print(f"Frequency: {Config.I2C0_FREQ} Hz")
        print()
        
        devices0 = i2c0.scan()
        
        if not devices0:
            print("❌ No I2C devices found on Bus 0!")
        else:
            print(f"✓ Found {len(devices0)} device(s) on Bus 0:")
            
            device_names = {
                0x40: "PCA9685 (Servo Driver)",
                0x68: "MPU6050 (IMU - Platform)",
                0x69: "MPU6050 (IMU - Alt Address)",
            }
            
            for addr in devices0:
                name = device_names.get(addr, "Unknown Device")
                status = "✓" if addr in [0x40, 0x68] else "?"
                print(f"  {status} 0x{addr:02X} ({addr:3d}) - {name}")
        
        # I2C Bus 1 (ADXL345)
        print("\n--- I2C Bus 1 (ADXL345 - Terrain) ---")
        i2c1 = I2C(
            Config.I2C1_ID,
            scl=Pin(Config.I2C1_SCL_PIN),
            sda=Pin(Config.I2C1_SDA_PIN),
            freq=Config.I2C1_FREQ
        )
        
        print(f"I2C Bus: {Config.I2C1_ID}")
        print(f"SDA Pin: {Config.I2C1_SDA_PIN}")
        print(f"SCL Pin: {Config.I2C1_SCL_PIN}")
        print(f"Frequency: {Config.I2C1_FREQ} Hz")
        print()
        
        devices1 = i2c1.scan()
        
        if not devices1:
            print("❌ No I2C devices found on Bus 1!")
        else:
            print(f"✓ Found {len(devices1)} device(s) on Bus 1:")
            
            device_names = {
                0x53: "ADXL345 (Accelerometer - Terrain)",
                0x1D: "ADXL345 (Alt Address)"
            }
            
            for addr in devices1:
                name = device_names.get(addr, "Unknown Device")
                status = "✓" if addr == 0x53 else "?"
                print(f"  {status} 0x{addr:02X} ({addr:3d}) - {name}")
        
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        
        # Kiểm tra thiết bị bắt buộc
        missing = []
        if 0x68 not in devices0:
            missing.append("MPU6050 (Bus 0)")
        if 0x40 not in devices0:
            missing.append("PCA9685 (Bus 0)")
        
        if missing:
            print(f"❌ Missing required devices: {', '.join(missing)}")
        else:
            print("✓ All required devices found!")
        
        # Kiểm tra thiết bị tùy chọn
        if 0x53 not in devices1:
            print("⚠️  ADXL345 not found on Bus 1 (optional)")
        else:
            print("✓ ADXL345 found on Bus 1")
        
        print("\n📌 COORDINATE SYSTEM:")
        print("   ADXL345 = REFERENCE standard")
        print("   MPU6050 axes REMAPPED to match ADXL345:")
        print("   - ADXL X = MPU Y")
        print("   - ADXL Y = MPU X")
        
        return i2c0, i2c1
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import sys
        sys.print_exception(e)
        return None, None


# =============================================================================
# TEST 2: SERVO SWEEP TEST
# =============================================================================

def test_servo_sweep(i2c=None):
    """
    Test servo bằng cách quét từ 0° đến 180°
    """
    print("\n" + "=" * 60)
    print("TEST 2: SERVO SWEEP TEST")
    print("=" * 60)
    
    if i2c is None:
        i2c = I2C(Config.I2C0_ID, scl=Pin(Config.I2C0_SCL_PIN), 
                  sda=Pin(Config.I2C0_SDA_PIN), freq=Config.I2C0_FREQ)
    
    try:
        print("Initializing PCA9685...")
        pca = PCA9685(i2c, Config.ADDR_PCA9685)
        pca.set_pwm_freq(50)
        
        servo_left = Servo(pca, Config.SERVO_LEFT_CHANNEL)
        servo_right = Servo(pca, Config.SERVO_RIGHT_CHANNEL)
        
        print(f"Servo Left:  Channel {Config.SERVO_LEFT_CHANNEL}")
        print(f"Servo Right: Channel {Config.SERVO_RIGHT_CHANNEL}")
        print()
        
        # Test servo trái
        print("Testing LEFT servo...")
        for angle in [0, 45, 90, 135, 180, 90]:
            print(f"  Setting angle: {angle}°")
            servo_left.set_angle(angle)
            time.sleep(0.5)
        
        print()
        
        # Test servo phải
        print("Testing RIGHT servo...")
        for angle in [0, 45, 90, 135, 180, 90]:
            print(f"  Setting angle: {angle}°")
            servo_right.set_angle(angle)
            time.sleep(0.5)
        
        print()
        print("✓ Servo test completed")
        print("  Check:")
        print("  - Smooth movement?")
        print("  - Correct direction?")
        print("  - No stuttering?")
        
        return pca, servo_left, servo_right
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return None


# =============================================================================
# TEST 3: SERVO DIFFERENTIAL TEST
# =============================================================================

def test_servo_differential(i2c=None):
    """
    Test chế độ vi sai của servo (1 lên, 1 xuống)
    """
    print("\n" + "=" * 60)
    print("TEST 3: SERVO DIFFERENTIAL TEST")
    print("=" * 60)
    
    if i2c is None:
        i2c = I2C(Config.I2C0_ID, scl=Pin(Config.I2C0_SCL_PIN),
                  sda=Pin(Config.I2C0_SDA_PIN), freq=Config.I2C0_FREQ)
    
    try:
        pca = PCA9685(i2c, Config.ADDR_PCA9685)
        pca.set_pwm_freq(50)
        
        servo_left = Servo(pca, Config.SERVO_LEFT_CHANNEL)
        servo_right = Servo(pca, Config.SERVO_RIGHT_CHANNEL)
        
        center = Config.SERVO_CENTER
        
        print("This test simulates PITCH and ROLL movement")
        print()
        
        # Test Pitch (cùng chiều)
        print("1. PITCH Test (both servos same direction):")
        for delta in [0, 20, 0, -20, 0]:
            angle = center + delta
            print(f"   Both servos: {angle}°")
            servo_left.set_angle(angle)
            servo_right.set_angle(angle)
            time.sleep(1)
        
        print()
        
        # Test Roll (ngược chiều)
        print("2. ROLL Test (differential movement):")
        for delta in [0, 20, 0, -20, 0]:
            left_angle = center + delta
            right_angle = center - delta
            print(f"   Left: {left_angle}°, Right: {right_angle}°")
            servo_left.set_angle(left_angle)
            servo_right.set_angle(right_angle)
            time.sleep(1)
        
        print()
        print("✓ Differential test completed")
        print("  Expected behavior:")
        print("  - PITCH: Platform tilts forward/backward")
        print("  - ROLL: Platform tilts left/right")
        
    except Exception as e:
        print(f"❌ Error: {e}")


# =============================================================================
# TEST 4: MPU6050 READ TEST
# =============================================================================

def test_mpu6050(i2c=None, duration=10):
    """
    Đọc và hiển thị dữ liệu MPU6050 trong N giây
    """
    print("\n" + "=" * 60)
    print("TEST 4: MPU6050 READ TEST")
    print("=" * 60)
    
    if i2c is None:
        i2c = I2C(Config.I2C0_ID, scl=Pin(Config.I2C0_SCL_PIN),
                  sda=Pin(Config.I2C0_SDA_PIN), freq=Config.I2C0_FREQ)
    
    try:
        print("Initializing MPU6050...")
        print("NOTE: Axes are REMAPPED to match ADXL345 standard")
        print("      ADXL X = MPU Y")
        print("      ADXL Y = MPU X")
        mpu = MPU6050(i2c, Config.ADDR_MPU6050)
        time.sleep(0.5)
        
        print(f"Reading data for {duration} seconds...")
        print("Tilt the sensor to see changes")
        print()
        print("Time  | Pitch    | Roll     |")
        print("------|----------|----------|")
        
        start = time.time()
        while time.time() - start < duration:
            pitch, roll = mpu.update()
            elapsed = time.time() - start
            print(f"{elapsed:5.1f}s | {pitch:+7.2f}° | {roll:+7.2f}° |")
            time.sleep(0.1)
        
        print()
        print("✓ MPU6050 test completed")
        print("  Check:")
        print("  - Flat position: Pitch ≈ 0°, Roll ≈ 0°")
        print("  - Tilt forward: Pitch > 0°")
        print("  - Tilt left: Roll < 0° (or > 0° depending on axis)")
        
        return mpu
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return None


# =============================================================================
# TEST 5: ADXL345 READ TEST
# =============================================================================

def test_adxl345(i2c=None, duration=10):
    """
    Đọc và hiển thị dữ liệu ADXL345 trong N giây
    """
    print("\n" + "=" * 60)
    print("TEST 5: ADXL345 READ TEST")
    print("=" * 60)
    
    if i2c is None:
        i2c = I2C(Config.I2C1_ID, scl=Pin(Config.I2C1_SCL_PIN),
                  sda=Pin(Config.I2C1_SDA_PIN), freq=Config.I2C1_FREQ)
    
    try:
        print("Initializing ADXL345...")
        print("NOTE: ADXL345 is the REFERENCE coordinate system")
        adxl = ADXL345(i2c, Config.ADDR_ADXL345)
        time.sleep(0.5)
        
        print(f"Reading data for {duration} seconds...")
        print("Tilt the sensor to see changes")
        print()
        print("Time  | Pitch    | Roll     |")
        print("------|----------|----------|")
        
        start = time.time()
        while time.time() - start < duration:
            pitch, roll = adxl.get_angles()
            elapsed = time.time() - start
            print(f"{elapsed:5.1f}s | {pitch:+7.2f}° | {roll:+7.2f}° |")
            time.sleep(0.1)
        
        print()
        print("✓ ADXL345 test completed")
        
        return adxl
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return None


# =============================================================================
# TEST 6: IMU COMPARISON (MPU6050 vs ADXL345)
# =============================================================================

def test_imu_comparison(i2c0=None, i2c1=None, duration=10):
    """
    So sánh dữ liệu từ MPU6050 và ADXL345
    """
    print("\n" + "=" * 60)
    print("TEST 6: IMU COMPARISON TEST")
    print("=" * 60)
    
    if i2c0 is None:
        i2c0 = I2C(Config.I2C0_ID, scl=Pin(Config.I2C0_SCL_PIN),
                  sda=Pin(Config.I2C0_SDA_PIN), freq=Config.I2C0_FREQ)
    
    if i2c1 is None:
        i2c1 = I2C(Config.I2C1_ID, scl=Pin(Config.I2C1_SCL_PIN),
                  sda=Pin(Config.I2C1_SDA_PIN), freq=Config.I2C1_FREQ)
    
    try:
        print("Initializing sensors...")
        print("NOTE: MPU6050 axes have been REMAPPED to match ADXL345")
        mpu = MPU6050(i2c0, Config.ADDR_MPU6050)
        
        try:
            adxl = ADXL345(i2c1, Config.ADDR_ADXL345)
            has_adxl = True
        except:
            print("ADXL345 not available, skipping comparison")
            has_adxl = False
            return
        
        time.sleep(0.5)
        
        print(f"Reading data for {duration} seconds...")
        print()
        print("Time  |   MPU6050    |   ADXL345    | Difference")
        print("      | Pitch | Roll | Pitch | Roll | Pitch | Roll")
        print("------|-------|------|-------|------|-------|------")
        
        start = time.time()
        while time.time() - start < duration:
            mpu_p, mpu_r = mpu.update()
            adxl_p, adxl_r = adxl.get_angles()
            
            diff_p = mpu_p - adxl_p
            diff_r = mpu_r - adxl_r
            
            elapsed = time.time() - start
            print(f"{elapsed:5.1f}s | {mpu_p:+5.1f} | {mpu_r:+5.1f} | "
                  f"{adxl_p:+5.1f} | {adxl_r:+5.1f} | "
                  f"{diff_p:+5.1f} | {diff_r:+5.1f}")
            
            time.sleep(0.1)
        
        print()
        print("✓ Comparison test completed")
        print("  With axis remapping, MPU6050 and ADXL345 should now")
        print("  show VERY SIMILAR values (difference < 2-3°)")
        print("  Small differences are normal due to:")
        print("  - Different sensor characteristics")
        print("  - MPU6050 uses complementary filter (smoother)")
        print("  - Mounting position offset")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import sys
        sys.print_exception(e)


# =============================================================================
# TEST 7: MIXING ALGORITHM TEST
# =============================================================================

def test_mixing_algorithm():
    """
    Test thuật toán trộn kênh với các giá trị mẫu
    """
    print("\n" + "=" * 60)
    print("TEST 7: MIXING ALGORITHM TEST")
    print("=" * 60)
    
    try:
        from balance_robot_v2 import MixingController
        
        mixer = MixingController()
        
        print(f"Lever Arm Ratio: {mixer.mix_ratio:.3f}")
        print(f"Mount Compensation: {mixer.mount_compensation:.3f}")
        print()
        
        test_cases = [
            (0, 0, "Neutral"),
            (10, 0, "Pitch +10° only"),
            (-10, 0, "Pitch -10° only"),
            (0, 10, "Roll +10° only"),
            (0, -10, "Roll -10° only"),
            (10, 10, "Pitch +10°, Roll +10°"),
            (-10, -10, "Pitch -10°, Roll -10°"),
        ]
        
        print("Input (PID)           | Output (Servo Angles)")
        print("Pitch   Roll   Case   | Left      Right     Delta")
        print("------|------|--------|---------|---------|-------")
        
        for pitch, roll, case in test_cases:
            left, right = mixer.compute_servo_angles(pitch, roll)
            delta = abs(left - right)
            
            print(f"{pitch:+5.0f}° {roll:+5.0f}° {case:20s} | "
                  f"{left:6.2f}°  {right:6.2f}°  {delta:5.2f}°")
        
        print()
        print("✓ Mixing algorithm test completed")
        print("  Expected:")
        print("  - Pitch only: Both servos same angle")
        print("  - Roll only: Servos opposite direction")
        print("  - Combined: Different angles on each servo")
        
    except Exception as e:
        print(f"❌ Error: {e}")


# =============================================================================
# TEST 8: FULL SYSTEM MANUAL CONTROL
# =============================================================================

def test_manual_control(i2c=None):
    """
    Điều khiển thủ công hệ thống để test phản ứng
    """
    print("\n" + "=" * 60)
    print("TEST 8: MANUAL CONTROL TEST")
    print("=" * 60)
    
    if i2c is None:
        i2c = I2C(Config.I2C0_ID, scl=Pin(Config.I2C0_SCL_PIN),
                  sda=Pin(Config.I2C0_SDA_PIN), freq=Config.I2C0_FREQ)
    
    try:
        print("Initializing system...")
        mpu = MPU6050(i2c, Config.ADDR_MPU6050)
        pca = PCA9685(i2c, Config.ADDR_PCA9685)
        pca.set_pwm_freq(50)
        
        servo_left = Servo(pca, Config.SERVO_LEFT_CHANNEL)
        servo_right = Servo(pca, Config.SERVO_RIGHT_CHANNEL)
        
        from balance_robot_v2 import MixingController
        mixer = MixingController()
        
        print()
        print("Manual control active. Commands:")
        print("  w/s: Pitch forward/backward")
        print("  a/d: Roll left/right")
        print("  x: Reset to center")
        print("  q: Quit")
        print()
        
        pitch_cmd = 0.0
        roll_cmd = 0.0
        step = 5.0
        
        servo_left.set_angle(Config.SERVO_CENTER)
        servo_right.set_angle(Config.SERVO_CENTER)
        
        print("Type command and press Enter:")
        
        while True:
            # Đọc góc hiện tại
            mpu_p, mpu_r = mpu.update()
            
            # Tính servo angles
            left, right = mixer.compute_servo_angles(pitch_cmd, roll_cmd)
            servo_left.set_angle(left)
            servo_right.set_angle(right)
            
            # Hiển thị status
            print(f"\rCmd: P={pitch_cmd:+5.1f}° R={roll_cmd:+5.1f}° | "
                  f"MPU: P={mpu_p:+6.2f}° R={mpu_r:+6.2f}° | "
                  f"Servo: L={left:6.2f}° R={right:6.2f}°    ", end="")
            
            # Đọc input (non-blocking would be better, but not available)
            # Simplified version - just loop
            time.sleep(0.05)
            
            # Trong thực tế, cần implement non-blocking input
            # Hoặc sử dụng interrupt
            
    except KeyboardInterrupt:
        print("\n\n✓ Manual control stopped")
    except Exception as e:
        print(f"\n❌ Error: {e}")


# =============================================================================
# RUN ALL TESTS
# =============================================================================

def run_all_tests():
    """
    Chạy tất cả các test theo thứ tự
    """
    print("\n" + "=" * 60)
    print("RUNNING ALL TESTS")
    print("=" * 60)
    
    i2c0, i2c1 = test_i2c()
    
    if i2c0 is None:
        print("\n❌ I2C0 initialization failed. Stopping tests.")
        return
    
    input("\nPress Enter to continue to Servo Sweep Test...")
    test_servo_sweep(i2c0)
    
    input("\nPress Enter to continue to Servo Differential Test...")
    test_servo_differential(i2c0)
    
    input("\nPress Enter to continue to MPU6050 Test...")
    test_mpu6050(i2c0, duration=5)
    
    if i2c1 is not None:
        input("\nPress Enter to continue to ADXL345 Test...")
        test_adxl345(i2c1, duration=5)
        
        input("\nPress Enter to continue to IMU Comparison Test...")
        test_imu_comparison(i2c0, i2c1, duration=5)
    else:
        print("\n⚠️  Skipping ADXL345 tests (I2C1 not available)")
    
    input("\nPress Enter to continue to Mixing Algorithm Test...")
    test_mixing_algorithm()
    
    print("\n" + "=" * 60)
    print("ALL TESTS COMPLETED")
    print("=" * 60)


# =============================================================================
# QUICK DIAGNOSTIC
# =============================================================================

def quick_diagnostic():
    """
    Chẩn đoán nhanh toàn bộ hệ thống
    """
    print("\n" + "=" * 60)
    print("QUICK DIAGNOSTIC")
    print("=" * 60)
    
    results = {
        'i2c0': False,
        'i2c1': False,
        'mpu6050': False,
        'adxl345': False,
        'pca9685': False,
        'servo_left': False,
        'servo_right': False
    }
    
    # Test I2C Bus 0
    try:
        i2c0 = I2C(Config.I2C0_ID, scl=Pin(Config.I2C0_SCL_PIN),
                  sda=Pin(Config.I2C0_SDA_PIN), freq=Config.I2C0_FREQ)
        devices0 = i2c0.scan()
        results['i2c0'] = len(devices0) > 0
        
        results['mpu6050'] = Config.ADDR_MPU6050 in devices0
        results['pca9685'] = Config.ADDR_PCA9685 in devices0
        
    except:
        pass
    
    # Test I2C Bus 1
    try:
        i2c1 = I2C(Config.I2C1_ID, scl=Pin(Config.I2C1_SCL_PIN),
                  sda=Pin(Config.I2C1_SDA_PIN), freq=Config.I2C1_FREQ)
        devices1 = i2c1.scan()
        results['i2c1'] = len(devices1) > 0
        
        results['adxl345'] = Config.ADDR_ADXL345 in devices1
        
    except:
        pass
    
    # Test Servos (nếu PCA9685 có)
    if results['pca9685']:
        try:
            pca = PCA9685(i2c0, Config.ADDR_PCA9685)
            pca.set_pwm_freq(50)
            
            servo_left = Servo(pca, Config.SERVO_LEFT_CHANNEL)
            servo_right = Servo(pca, Config.SERVO_RIGHT_CHANNEL)
            
            servo_left.set_angle(90)
            servo_right.set_angle(90)
            
            results['servo_left'] = True
            results['servo_right'] = True
        except:
            pass
    
    # Hiển thị kết quả
    print("\nComponent Status:")
    print("-" * 40)
    
    status_icon = {True: "✓", False: "✗"}
    
    for component, status in results.items():
        icon = status_icon[status]
        component_name = component.upper().replace('_', ' ')
        print(f"  {icon} {component_name:15s} : {'OK' if status else 'FAIL'}")
    
    print()
    
    # Tổng kết
    critical = ['i2c0', 'mpu6050', 'pca9685', 'servo_left', 'servo_right']
    critical_ok = all(results[k] for k in critical)
    
    if critical_ok:
        print("✓ All critical components OK!")
        print("  System ready to run.")
    else:
        print("✗ Some critical components failed.")
        print("  Please check wiring and configuration.")
    
    if not results['i2c1']:
        print("\n⚠️  I2C1 bus not available")
    if not results['adxl345']:
        print("⚠️  ADXL345 not found (optional - system can run without it)")
    
    print("\n📌 COORDINATE SYSTEM:")
    print("   ADXL345 = REFERENCE standard")
    print("   MPU6050 axes REMAPPED to match ADXL345")
    
    return results


# =============================================================================
# HELP
# =============================================================================

def help():
    """
    Hiển thị danh sách các test có sẵn
    """
    print("""
╔════════════════════════════════════════════════════════════╗
║           BALANCE ROBOT - TEST UTILITIES                   ║
╚════════════════════════════════════════════════════════════╝

Available Tests:
────────────────────────────────────────────────────────────

  test_i2c()                  - Scan I2C bus
  test_servo_sweep()          - Test servo movement (0-180°)
  test_servo_differential()   - Test pitch/roll differential
  test_mpu6050(duration=10)   - Read MPU6050 data
  test_adxl345(duration=10)   - Read ADXL345 data
  test_imu_comparison()       - Compare MPU vs ADXL
  test_mixing_algorithm()     - Test mixing calculations
  test_manual_control()       - Manual servo control
  
  run_all_tests()             - Run all tests sequentially
  quick_diagnostic()          - Quick system check
  help()                      - Show this help

Usage Example:
──────────────
  >>> import test_utils
  >>> test_utils.quick_diagnostic()
  >>> test_utils.test_servo_sweep()
  >>> test_utils.test_mpu6050(duration=5)

For detailed testing, run:
  >>> test_utils.run_all_tests()

────────────────────────────────────────────────────────────
    """)


# Auto-run diagnostic on import (optional)
if __name__ == "__main__":
    help()