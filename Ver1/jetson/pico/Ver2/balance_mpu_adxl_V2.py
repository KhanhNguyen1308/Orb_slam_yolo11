"""
=============================================================================
ROBOT CÂN BẰNG 2 TẦNG - 2-AXIS GIMBAL STABILIZATION SYSTEM
=============================================================================
Hardware:
- MPU6050: Gắn trên THÂN TRÊN (Platform cần cân bằng)
- ADXL345: Gắn trên THÂN DƯỚI (Bánh xích - Đo địa hình)
- PCA9685: Điều khiển 2 servo
- 2x Servo: Lắp phía trước, cách nhau 115.5mm, góc nghiêng 60°

Cơ chế hoạt động:
- ADXL345 đo độ nghiêng địa hình (Feedforward - tùy chọn)
- MPU6050 đo độ nghiêng thân trên (Feedback)
- PID tính toán độ bù cần thiết
- Mixing Algorithm phân phối lực cho 2 servo

Author: Balance Robot Team
Version: 2.0
Date: 2025-01-30
=============================================================================
"""

from machine import I2C, Pin
import time
import math
import ustruct

# =============================================================================
# PHẦN 1: CẤU HÌNH HỆ THỐNG (USER CONFIGURABLE)
# =============================================================================

class Config:
    """
    Cấu hình tập trung - Chỉnh ở đây để điều chỉnh hệ thống
    """
    
    # --- I2C CONFIG ---
    # MPU6050 và PCA9685 dùng I2C bus 0
    I2C0_ID = 0
    I2C0_SDA_PIN = 20
    I2C0_SCL_PIN = 21
    I2C0_FREQ = 400000  # Hz
    
    # ADXL345 dùng I2C bus 1 riêng
    I2C1_ID = 1
    I2C1_SDA_PIN = 18
    I2C1_SCL_PIN = 19
    I2C1_FREQ = 400000  # Hz
    
    # --- I2C ADDRESSES ---
    ADDR_MPU6050 = 0x68   # IMU trên thân trên (I2C0)
    ADDR_ADXL345 = 0x53   # IMU trên thân dưới - địa hình (I2C1)
    ADDR_PCA9685 = 0x40   # Servo driver (I2C0)
    
    # --- MECHANICAL PARAMETERS (ĐƠN VỊ: mm) ---
    ARM_PITCH = 33.34     # Khoảng cách từ tâm quay đến servo (trục dọc)
    ARM_ROLL = 57.75      # Khoảng cách từ tâm quay đến servo (trục ngang) = 115.5/2
    MOUNT_ANGLE = 60      # Góc nghiêng thanh đẩy servo (độ)
    SERVO_SPACING = 115.5 # Khoảng cách giữa 2 servo (mm)
    
    # --- SERVO CONFIG ---
    SERVO_LEFT_CHANNEL = 0    # Kênh PCA9685 cho servo trái
    SERVO_RIGHT_CHANNEL = 1   # Kênh PCA9685 cho servo phải
    SERVO_CENTER = 90         # Góc trung tâm (độ)
    SERVO_MIN_ANGLE = 10      # Giới hạn góc tối thiểu (an toàn)
    SERVO_MAX_ANGLE = 170     # Giới hạn góc tối đa (an toàn)
    SERVO_PWM_FREQ = 50       # Tần số PWM (Hz)
    
    # Đảo chiều servo nếu cần
    SERVO_LEFT_INVERT = False   # True nếu servo trái quay ngược
    SERVO_RIGHT_INVERT = True   # True nếu servo phải quay ngược
    
    # --- PID PARAMETERS ---
    # Pitch (Nghiêng trước/sau)
    PID_PITCH_KP = 2.5    # Proportional gain
    PID_PITCH_KI = 0.03   # Integral gain
    PID_PITCH_KD = 1.2    # Derivative gain
    
    # Roll (Nghiêng trái/phải)
    PID_ROLL_KP = 2.5
    PID_ROLL_KI = 0.03
    PID_ROLL_KD = 1.2
    
    # Giới hạn Integral (chống windup)
    PID_INTEGRAL_LIMIT = 100
    
    # Giới hạn output PID (độ)
    PID_OUTPUT_LIMIT = 45  # Tối đa ±45° từ center
    
    # --- CONTROL MODE ---
    USE_ADXL_FEEDFORWARD = True   # Có dùng ADXL để bù trước không?
    FEEDFORWARD_GAIN = 0.5        # Hệ số bù trước (0.0 - 1.0)
    
    # --- LOOP TIMING ---
    LOOP_FREQUENCY = 100   # Hz (khuyến nghị: 50-100Hz)
    LOOP_PERIOD_MS = int(1000 / LOOP_FREQUENCY)
    
    # --- COMPLEMENTARY FILTER (MPU6050) ---
    MPU_ALPHA = 0.96  # Hệ số lọc (0.9-0.98: nhiều gyro, 0.5-0.8: nhiều accel)
    
    # --- DEBUG MODE ---
    DEBUG_PRINT = True          # In thông tin debug
    DEBUG_INTERVAL_MS = 500     # In mỗi 500ms
    
    # --- SAFETY ---
    STARTUP_DELAY_SEC = 2       # Đợi 2s trước khi bắt đầu
    ERROR_RETRY_LIMIT = 3       # Số lần thử lại khi lỗi


# =============================================================================
# PHẦN 2: DRIVER CLASSES (LOW-LEVEL HARDWARE)
# =============================================================================

class PCA9685:
    """Driver cho PCA9685 16-channel PWM controller"""
    
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.addr = address
        self.reset()
    
    def _write_reg(self, reg, value):
        """Ghi 1 byte vào register"""
        self.i2c.writeto_mem(self.addr, reg, bytearray([value]))
    
    def _read_reg(self, reg):
        """Đọc 1 byte từ register"""
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]
    
    def reset(self):
        """Reset chip về trạng thái mặc định"""
        self._write_reg(0x00, 0x00)  # MODE1 register
    
    def set_pwm_freq(self, freq_hz):
        """
        Đặt tần số PWM (thường là 50Hz cho servo)
        """
        prescale_val = int(25000000.0 / 4096.0 / freq_hz - 1)
        
        old_mode = self._read_reg(0x00)
        # Sleep mode để thay đổi prescaler
        self._write_reg(0x00, (old_mode & 0x7F) | 0x10)
        self._write_reg(0xFE, prescale_val)
        self._write_reg(0x00, old_mode)
        time.sleep(0.005)
        # Auto-increment mode
        self._write_reg(0x00, old_mode | 0xA1)
    
    def set_pwm(self, channel, on_time, off_time):
        """
        Đặt PWM cho 1 kênh
        channel: 0-15
        on_time: 0-4095 (thời điểm bật trong chu kỳ)
        off_time: 0-4095 (thời điểm tắt trong chu kỳ)
        """
        base_reg = 0x06 + 4 * channel
        self._write_reg(base_reg, on_time & 0xFF)
        self._write_reg(base_reg + 1, on_time >> 8)
        self._write_reg(base_reg + 2, off_time & 0xFF)
        self._write_reg(base_reg + 3, off_time >> 8)


class Servo:
    """
    Lớp điều khiển servo thông qua PCA9685
    """
    
    def __init__(self, pca, channel, invert=False):
        self.pca = pca
        self.channel = channel
        self.invert = invert
        self.current_angle = Config.SERVO_CENTER
    
    def set_angle(self, angle):
        """
        Đặt góc servo (0-180 độ)
        Tự động giới hạn trong phạm vi an toàn
        """
        # Đảo chiều nếu cần
        if self.invert:
            angle = 180 - angle
        
        # Giới hạn an toàn
        angle = max(Config.SERVO_MIN_ANGLE, 
                   min(Config.SERVO_MAX_ANGLE, angle))
        
        # Chuyển góc sang pulse width (500-2500 µs)
        pulse_us = 500 + (angle / 180.0) * 2000
        
        # Chuyển sang duty cycle (chu kỳ 20ms = 20000µs)
        duty = int((pulse_us / 20000.0) * 4096)
        
        self.pca.set_pwm(self.channel, 0, duty)
        self.current_angle = angle
    
    def get_angle(self):
        """Trả về góc hiện tại"""
        return self.current_angle


class MPU6050:
    """
    Driver cho MPU6050 6-axis IMU
    Sử dụng Complementary Filter để tính góc nghiêng
    
    LÀM CHUẨN THEO ADXL345:
    - X của ADXL345 = Y của MPU6050
    - Y của ADXL345 = X của MPU6050
    - Code đã hoán đổi trục để khớp với ADXL345
    """
    
    def __init__(self, i2c, addr=0x68, alpha=0.96):
        self.i2c = i2c
        self.addr = addr
        self.alpha = alpha  # Complementary filter coefficient
        
        # Wake up MPU6050 (thoát chế độ sleep)
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        time.sleep(0.1)
        
        # Khởi tạo góc
        self.angle_pitch = 0.0
        self.angle_roll = 0.0
        self.last_time = time.ticks_ms()
    
    def update(self):
        """
        Đọc dữ liệu và tính góc nghiêng
        Trả về: (pitch, roll) theo độ
        
        TRỤC ĐÃ ĐƯỢC CHUYỂN ĐỔI ĐỂ KHỚP VỚI ADXL345:
        - MPU X_raw → ADXL Y (Roll axis)
        - MPU Y_raw → ADXL X (Pitch axis)
        """
        # Đọc 14 bytes từ register 0x3B
        raw_data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        # Unpack dữ liệu: Accel XYZ, Temp, Gyro XYZ
        values = ustruct.unpack('>hhhhhhh', raw_data)
        
        # Accelerometer (raw) - Đọc theo trục MPU gốc
        ax_mpu, ay_mpu, az_mpu = values[0], values[1], values[2]
        
        # Gyroscope (chuyển sang độ/giây) - Đọc theo trục MPU gốc
        # Full scale = ±250°/s, sensitivity = 131 LSB/(°/s)
        gx_mpu = values[4] / 131.0  # Gyro X (pitch rate)
        gy_mpu = values[5] / 131.0  # Gyro Y (roll rate)
        gz_mpu = values[6] / 131.0  # Gyro Z (yaw rate)
        
        # HOÁN ĐỔI TRỤC ĐỂ KHỚP VỚI ADXL345:
        # ADXL X = MPU Y
        # ADXL Y = MPU X
        ax = ay_mpu  # ADXL X = MPU Y
        ay = ax_mpu  # ADXL Y = MPU X
        az = az_mpu  # Z giữ nguyên
        
        gx = gy_mpu  # Gyro cho Pitch (ADXL X axis)
        gy = gx_mpu  # Gyro cho Roll (ADXL Y axis)
        
        # Tính delta time
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000.0  # seconds
        self.last_time = now
        
        # Tính góc từ accelerometer (góc tĩnh)
        # Pitch: Xoay quanh trục X (ADXL standard)
        # Roll: Xoay quanh trục Y (ADXL standard)
        acc_pitch = math.atan2(ay, az) * 57.2958  # rad to deg
        acc_roll = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.2958
        
        # Complementary filter
        # angle = alpha * (angle + gyro*dt) + (1-alpha) * acc_angle
        self.angle_pitch = self.alpha * (self.angle_pitch + gx * dt) + \
                          (1 - self.alpha) * acc_pitch
        
        self.angle_roll = self.alpha * (self.angle_roll + gy * dt) + \
                         (1 - self.alpha) * acc_roll
        
        return self.angle_pitch, self.angle_roll


class ADXL345:
    """
    Driver cho ADXL345 3-axis Accelerometer
    Đơn giản hơn MPU6050, chỉ có accelerometer
    """
    
    def __init__(self, i2c, addr=0x53):
        self.i2c = i2c
        self.addr = addr
        
        # Power control: measurement mode
        self.i2c.writeto_mem(self.addr, 0x2D, b'\x08')
        
        # Data format: ±16g, full resolution
        self.i2c.writeto_mem(self.addr, 0x31, b'\x0B')
        time.sleep(0.1)
    
    def get_angles(self):
        """
        Đọc góc nghiêng từ accelerometer
        Trả về: (pitch, roll) theo độ
        """
        # Đọc 6 bytes từ register 0x32 (DATAX0)
        raw_data = self.i2c.readfrom_mem(self.addr, 0x32, 6)
        
        # Unpack little-endian signed shorts
        x, y, z = ustruct.unpack('<hhh', raw_data)
        
        # Tính góc (tương tự MPU6050)
        pitch = math.atan2(y, z) * 57.2958
        roll = math.atan2(-x, math.sqrt(y*y + z*z)) * 57.2958
        
        return pitch, roll


class PIDController:
    """
    PID Controller cơ bản với anti-windup
    """
    
    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        
        self.prev_error = 0.0
        self.integral = 0.0
    
    def compute(self, setpoint, measured_value):
        """
        Tính toán output PID
        setpoint: Giá trị mong muốn
        measured_value: Giá trị đo được
        """
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error
        if self.integral_limit:
            self.integral = max(-self.integral_limit, 
                              min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error)
        self.prev_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Limit output if specified
        if self.output_limit:
            output = max(-self.output_limit, 
                        min(self.output_limit, output))
        
        return output
    
    def reset(self):
        """Reset PID state"""
        self.prev_error = 0.0
        self.integral = 0.0


# =============================================================================
# PHẦN 3: MIXING ALGORITHM (CORE CONTROL LOGIC)
# =============================================================================

class MixingController:
    """
    Bộ điều khiển trộn kênh (Mixing Controller)
    Chuyển đổi lệnh Pitch/Roll thành góc servo cụ thể
    """
    
    def __init__(self):
        # Tính toán hệ số bù
        self.mix_ratio = Config.ARM_ROLL / Config.ARM_PITCH
        
        # Bù góc lắp đặt
        mount_angle_rad = math.radians(Config.MOUNT_ANGLE)
        self.sin_mount = math.sin(mount_angle_rad)
        self.cos_mount = math.cos(mount_angle_rad)
        
        # Hệ số bù mount angle
        self.mount_compensation = 1.0 / self.sin_mount
    
    def compute_servo_angles(self, pid_pitch, pid_roll):
        """
        Input: 
            pid_pitch: Output PID cho trục pitch (độ)
            pid_roll: Output PID cho trục roll (độ)
        
        Output:
            (left_angle, right_angle): Góc servo trái và phải (độ)
        
        Công thức:
            1. Bù góc lắp đặt (mount angle 60°)
            2. Bù tỷ lệ tay đòn (lever arm ratio)
            3. Mixing differential:
               - Pitch: Cả 2 servo cùng chiều
               - Roll: 2 servo ngược chiều
        """
        
        # Bù góc lắp đặt và tỷ lệ tay đòn
        pitch_component = pid_pitch * self.mount_compensation
        roll_component = (pid_roll * self.mix_ratio) * self.mount_compensation
        
        # Differential mixing
        # Servo trái: Pitch + Roll
        # Servo phải: Pitch - Roll
        delta_left = pitch_component + roll_component
        delta_right = pitch_component - roll_component
        
        # Tính góc servo cuối cùng
        left_angle = Config.SERVO_CENTER + delta_left
        right_angle = Config.SERVO_CENTER + delta_right
        
        return left_angle, right_angle


# =============================================================================
# PHẦN 4: MAIN CONTROL SYSTEM
# =============================================================================

class BalanceRobot:
    """
    Hệ thống điều khiển chính
    """
    
    def __init__(self):
        self.i2c0 = None  # I2C Bus 0 (MPU6050 + PCA9685)
        self.i2c1 = None  # I2C Bus 1 (ADXL345)
        self.mpu = None
        self.adxl = None
        self.pca = None
        self.servo_left = None
        self.servo_right = None
        self.pid_pitch = None
        self.pid_roll = None
        self.mixer = None
        
        self.adxl_available = False
        self.last_debug_time = 0
    
    def initialize(self):
        """
        Khởi tạo tất cả phần cứng
        """
        print("=" * 60)
        print("BALANCE ROBOT INITIALIZATION")
        print("=" * 60)
        
        # 1. Khởi tạo I2C Bus 0 (MPU6050 + PCA9685)
        print(f"[1/7] Initializing I2C Bus 0 (SDA={Config.I2C0_SDA_PIN}, SCL={Config.I2C0_SCL_PIN})...")
        self.i2c0 = I2C(
            Config.I2C0_ID,
            scl=Pin(Config.I2C0_SCL_PIN),
            sda=Pin(Config.I2C0_SDA_PIN),
            freq=Config.I2C0_FREQ
        )
        
        # Scan I2C bus 0
        devices0 = self.i2c0.scan()
        print(f"    I2C0 devices: {[hex(d) for d in devices0]}")
        
        # 2. Khởi tạo I2C Bus 1 (ADXL345)
        print(f"[2/7] Initializing I2C Bus 1 (SDA={Config.I2C1_SDA_PIN}, SCL={Config.I2C1_SCL_PIN})...")
        self.i2c1 = I2C(
            Config.I2C1_ID,
            scl=Pin(Config.I2C1_SCL_PIN),
            sda=Pin(Config.I2C1_SDA_PIN),
            freq=Config.I2C1_FREQ
        )
        
        # Scan I2C bus 1
        devices1 = self.i2c1.scan()
        print(f"    I2C1 devices: {[hex(d) for d in devices1]}")
        
        # 3. Khởi tạo MPU6050 (BẮT BUỘC) - I2C Bus 0
        print("[3/7] Initializing MPU6050 (Platform IMU) on I2C0...")
        if Config.ADDR_MPU6050 not in devices0:
            raise RuntimeError(f"MPU6050 not found at {hex(Config.ADDR_MPU6050)} on I2C0")
        self.mpu = MPU6050(self.i2c0, Config.ADDR_MPU6050, Config.MPU_ALPHA)
        print("    MPU6050 initialized successfully")
        print("    NOTE: MPU6050 axes remapped to match ADXL345 standard")
        
        # 4. Khởi tạo ADXL345 (TÙY CHỌN) - I2C Bus 1
        print("[4/7] Initializing ADXL345 (Terrain IMU) on I2C1...")
        if Config.ADDR_ADXL345 in devices1:
            try:
                self.adxl = ADXL345(self.i2c1, Config.ADDR_ADXL345)
                self.adxl_available = True
                print("    ADXL345 initialized successfully")
                print("    ADXL345 is the REFERENCE coordinate system")
            except Exception as e:
                print(f"    ADXL345 init failed: {e}")
                self.adxl_available = False
        else:
            print("    ADXL345 not found on I2C1 (running without terrain sensing)")
            self.adxl_available = False
        
        # 5. Khởi tạo PCA9685 - I2C Bus 0
        print("[5/7] Initializing PCA9685 (Servo Driver) on I2C0...")
        if Config.ADDR_PCA9685 not in devices0:
            raise RuntimeError(f"PCA9685 not found at {hex(Config.ADDR_PCA9685)} on I2C0")
        self.pca = PCA9685(self.i2c0, Config.ADDR_PCA9685)
        self.pca.set_pwm_freq(Config.SERVO_PWM_FREQ)
        print(f"    PCA9685 initialized (PWM freq: {Config.SERVO_PWM_FREQ}Hz)")
        
        # 6. Khởi tạo Servos
        print("[6/7] Initializing Servos...")
        self.servo_left = Servo(
            self.pca, 
            Config.SERVO_LEFT_CHANNEL,
            Config.SERVO_LEFT_INVERT
        )
        self.servo_right = Servo(
            self.pca,
            Config.SERVO_RIGHT_CHANNEL,
            Config.SERVO_RIGHT_INVERT
        )
        
        # Đặt servo về vị trí trung tâm
        self.servo_left.set_angle(Config.SERVO_CENTER)
        self.servo_right.set_angle(Config.SERVO_CENTER)
        print(f"    Servos centered at {Config.SERVO_CENTER}°")
        
        # 7. Khởi tạo PID Controllers
        print("[7/7] Initializing PID Controllers...")
        self.pid_pitch = PIDController(
            Config.PID_PITCH_KP,
            Config.PID_PITCH_KI,
            Config.PID_PITCH_KD,
            Config.PID_OUTPUT_LIMIT,
            Config.PID_INTEGRAL_LIMIT
        )
        self.pid_roll = PIDController(
            Config.PID_ROLL_KP,
            Config.PID_ROLL_KI,
            Config.PID_ROLL_KD,
            Config.PID_OUTPUT_LIMIT,
            Config.PID_INTEGRAL_LIMIT
        )
        print(f"    PID Pitch: Kp={Config.PID_PITCH_KP}, Ki={Config.PID_PITCH_KI}, Kd={Config.PID_PITCH_KD}")
        print(f"    PID Roll:  Kp={Config.PID_ROLL_KP}, Ki={Config.PID_ROLL_KI}, Kd={Config.PID_ROLL_KD}")
        
        # 8. Khởi tạo Mixing Controller
        self.mixer = MixingController()
        print(f"    Mixing ratio: {self.mixer.mix_ratio:.3f}")
        print(f"    Mount compensation: {self.mixer.mount_compensation:.3f}")
        
        print("\n✓ All systems initialized successfully!")
        print(f"  COORDINATE SYSTEM: ADXL345 is reference")
        print(f"  MPU6050 axes: REMAPPED to match ADXL345")
        print(f"  Waiting {Config.STARTUP_DELAY_SEC} seconds before start...\n")
        time.sleep(Config.STARTUP_DELAY_SEC)
    
    def shutdown(self):
        """
        Tắt hệ thống an toàn
        """
        print("\n[SHUTDOWN] Stopping servos...")
        if self.pca:
            self.pca.set_pwm(Config.SERVO_LEFT_CHANNEL, 0, 0)
            self.pca.set_pwm(Config.SERVO_RIGHT_CHANNEL, 0, 0)
        print("[SHUTDOWN] System halted.")
    
    def run(self):
        """
        Vòng lặp điều khiển chính
        """
        print("=" * 60)
        print("BALANCE ROBOT RUNNING")
        print("Press Ctrl+C to stop")
        print("=" * 60)
        
        loop_count = 0
        
        try:
            while True:
                loop_start = time.ticks_ms()
                
                # ===== BƯỚC 1: ĐỌC CẢM BIẾN =====
                
                # 1.1. Đọc MPU6050 (góc nghiêng thân trên)
                platform_pitch, platform_roll = self.mpu.update()
                
                # 1.2. Đọc ADXL345 (góc nghiêng địa hình) - nếu có
                terrain_pitch, terrain_roll = 0.0, 0.0
                if self.adxl_available and Config.USE_ADXL_FEEDFORWARD:
                    terrain_pitch, terrain_roll = self.adxl.get_angles()
                
                # ===== BƯỚC 2: TÍNH SETPOINT =====
                
                # Mode 1: Cân bằng tuyệt đối (luôn giữ thân trên ngang)
                setpoint_pitch = 0.0
                setpoint_roll = 0.0
                
                # Mode 2: Feedforward - Bù trước theo địa hình
                if Config.USE_ADXL_FEEDFORWARD and self.adxl_available:
                    # Setpoint = -terrain_angle * gain
                    # (Nếu địa hình nghiêng +10°, thân trên cần nghiêng -10° để ngang với mặt đất)
                    setpoint_pitch = -terrain_pitch * Config.FEEDFORWARD_GAIN
                    setpoint_roll = -terrain_roll * Config.FEEDFORWARD_GAIN
                
                # ===== BƯỚC 3: TÍNH PID =====
                
                pid_output_pitch = self.pid_pitch.compute(setpoint_pitch, platform_pitch)
                pid_output_roll = self.pid_roll.compute(setpoint_roll, platform_roll)
                
                # ===== BƯỚC 4: MIXING & ĐIỀU KHIỂN SERVO =====
                
                left_angle, right_angle = self.mixer.compute_servo_angles(
                    pid_output_pitch,
                    pid_output_roll
                )
                
                self.servo_left.set_angle(left_angle)
                self.servo_right.set_angle(right_angle)
                
                # ===== BƯỚC 5: DEBUG OUTPUT =====
                
                if Config.DEBUG_PRINT:
                    now = time.ticks_ms()
                    if time.ticks_diff(now, self.last_debug_time) >= Config.DEBUG_INTERVAL_MS:
                        self.last_debug_time = now
                        
                        print(f"[{loop_count:05d}] "
                              f"Platform: P={platform_pitch:+6.2f}° R={platform_roll:+6.2f}° | "
                              f"Terrain: P={terrain_pitch:+6.2f}° R={terrain_roll:+6.2f}° | "
                              f"PID: P={pid_output_pitch:+6.2f}° R={pid_output_roll:+6.2f}° | "
                              f"Servo: L={left_angle:6.2f}° R={right_angle:6.2f}°")
                
                loop_count += 1
                
                # ===== BƯỚC 6: TIMING CONTROL =====
                
                loop_time = time.ticks_diff(time.ticks_ms(), loop_start)
                sleep_time = Config.LOOP_PERIOD_MS - loop_time
                if sleep_time > 0:
                    time.sleep_ms(sleep_time)
                elif Config.DEBUG_PRINT and loop_count % 100 == 0:
                    print(f"[WARNING] Loop overrun: {loop_time}ms > {Config.LOOP_PERIOD_MS}ms")
        
        except KeyboardInterrupt:
            print("\n[INFO] KeyboardInterrupt received")
        except Exception as e:
            print(f"\n[ERROR] {e}")
            import sys
            sys.print_exception(e)
        finally:
            self.shutdown()


# =============================================================================
# PHẦN 5: ENTRY POINT
# =============================================================================

def main():
    """
    Hàm main - Entry point của chương trình
    """
    robot = BalanceRobot()
    
    try:
        robot.initialize()
        robot.run()
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        import sys
        sys.print_exception(e)
        robot.shutdown()


if __name__ == "__main__":
    main()