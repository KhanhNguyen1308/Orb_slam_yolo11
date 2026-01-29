from machine import I2C, Pin
import time
import math
import ustruct

# ==========================================
# 1. CẤU HÌNH HỆ THỐNG (USER CONFIG)
# ==========================================
# Cấu hình chân I2C cho Pi Pico
I2C_ID = 0
SDA_PIN = 20
SCL_PIN = 21

# Địa chỉ I2C
MPU_ADDR = 0x68
PCA_ADDR = 0x40

# Cấu hình kênh Servo trên PCA9685
CH_S1 = 0  # Trái Trước (Front Left)
CH_S2 = 1  # Phải Trước (Front Right)
CH_S3 = 2  # Phải Sau (Rear Right - Support S2)
CH_S4 = 3  # Trái Sau (Rear Left - Support S1)

# Giới hạn góc Servo (Bảo vệ cơ khí)
SERVO_MIN_ANGLE = 20
SERVO_MAX_ANGLE = 160
SERVO_CENTER = 90

# THAM SỐ PID (Cần tinh chỉnh thực tế)
# Kp: Độ nhạy | Ki: Cộng dồn lỗi | Kd: Chống rung
PID_PITCH_PARAMS = {'Kp': 2.0, 'Ki': 0.0, 'Kd': 0.5} 
PID_ROLL_PARAMS  = {'Kp': 2.0, 'Ki': 0.0, 'Kd': 0.5}

# Tần số vòng lặp (Hz)
LOOP_FREQ = 100 

# ==========================================
# 2. CLASS: PCA9685 (DRIVER SERVO)
# ==========================================
class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()
        
    def _write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytearray([value]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def reset(self):
        self._write(0x00, 0x00) # Mode1

    def set_pwm_freq(self, freq_hz):
        prescale_val = 25000000.0 / 4096.0 / float(freq_hz) - 1.0
        prescale = int(math.floor(prescale_val + 0.5))
        oldmode = self._read(0x00)
        newmode = (oldmode & 0x7F) | 0x10 # sleep
        self._write(0x00, newmode)
        self._write(0xFE, prescale)
        self._write(0x00, oldmode)
        time.sleep(0.005)
        self._write(0x00, oldmode | 0xa1)

    def set_pwm(self, channel, on, off):
        self._write(0x06 + 4 * channel, on & 0xFF)
        self._write(0x07 + 4 * channel, on >> 8)
        self._write(0x08 + 4 * channel, off & 0xFF)
        self._write(0x09 + 4 * channel, off >> 8)

class Servo:
    def __init__(self, pca, channel, min_us=500, max_us=2500, angle_range=180):
        self.pca = pca
        self.channel = channel
        self.min_us = min_us
        self.max_us = max_us
        self.angle_range = angle_range

    def set_angle(self, angle):
        # Kẹp giá trị an toàn
        angle = max(0, min(self.angle_range, angle))
        pulse_us = self.min_us + (angle / self.angle_range) * (self.max_us - self.min_us)
        duty = int((pulse_us / 20000) * 4096) # 20000us = 20ms (50Hz)
        self.pca.set_pwm(self.channel, 0, duty)

# ==========================================
# 3. CLASS: MPU6050 (DRIVER IMU)
# ==========================================
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00') # Wake up
        self.angle_pitch = 0.0
        self.angle_roll = 0.0
        self.last_time = time.ticks_ms()

    def update(self):
        # Đọc Accelerometer và Gyroscope
        raw = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        vals = ustruct.unpack('>hhhhhhh', raw)
        
        accX, accY, accZ = vals[0], vals[1], vals[2]
        gyroX, gyroY = vals[4] / 131.0, vals[5] / 131.0

        # Tính toán thời gian trôi qua (dt)
        curr_time = time.ticks_ms()
        dt = time.ticks_diff(curr_time, self.last_time) / 1000.0
        self.last_time = curr_time

        # Tính góc từ gia tốc (Accelerometer)
        acc_pitch = math.atan2(accY, accZ) * 57.2958
        acc_roll = math.atan2(-accX, math.sqrt(accY**2 + accZ**2)) * 57.2958

        # Bộ lọc bù (Complementary Filter)
        # Pitch: Xoay quanh trục Y (Gập bụng) -> dùng GyroY
        # Roll: Xoay quanh trục X (Lắc hông) -> dùng GyroX
        self.angle_pitch = 0.96 * (self.angle_pitch + gyroX * dt) + 0.04 * acc_pitch
        self.angle_roll  = 0.96 * (self.angle_roll + gyroY * dt)  + 0.04 * acc_roll

        return self.angle_pitch, self.angle_roll

# ==========================================
# 4.
CLASS: PID CONTROLLER
# ==========================================
class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        # Giới hạn integral để tránh windup (nếu cần)
        self.integral = max(-500, min(500, self.integral)) 
        
        derivative = error - self.prev_error
        self.prev_error = error
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

# ==========================================
# 5. CHƯƠNG TRÌNH CHÍNH (MAIN)
# ==========================================
def main():
    print("--- KHOI TAO ROBOT 4 SERVO ---")
    
    # 1. Setup I2C
    try:
        i2c = I2C(I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        print("I2C OK. Devices:", [hex(x) for x in i2c.scan()])
    except Exception as e:
        print("Loi I2C:", e)
        return

    # 2. Setup Modules
    mpu = MPU6050(i2c, MPU_ADDR)
    pca = PCA9685(i2c, PCA_ADDR)
    pca.set_pwm_freq(50) # Servo standard 50Hz

    # 3. Setup Servos
    s1 = Servo(pca, CH_S1) # Trước Trái
    s2 = Servo(pca, CH_S2) # Trước Phải
    s3 = Servo(pca, CH_S3) # Sau Phải
    s4 = Servo(pca, CH_S4) # Sau Trái

    # 4. Setup PID Controllers
    pid_pitch = PID(**PID_PITCH_PARAMS, setpoint=0) # Muốn giữ góc 0
    pid_roll  = PID(**PID_ROLL_PARAMS, setpoint=0)

    print("He thong san sang. Bat dau Loop...")
    
    # Biến tinh chỉnh Offset (Nếu servo không chuẩn 90 độ)
    OFFSET_TRIM = 0 

    try:
        while True:
            start_time = time.ticks_ms()

            # --- BƯỚC 1: Đọc Cảm Biến ---
            curr_pitch, curr_roll = mpu.update()

            # --- BƯỚC 2: Tính toán PID ---
            # Output PID chính là lượng góc cần bù vào
            out_pitch = pid_pitch.compute(curr_pitch)
            out_roll  = pid_roll.compute(curr_roll)

            # --- BƯỚC 3: Mixing (Trộn kênh) ---
            # S1 (Trái Trước): Offset + Pitch + Roll
            # S2 (Phải Trước): Offset - Pitch + Roll
            # Lưu ý: Dấu (+) hay (-) phụ thuộc vào cơ khí thực tế. 
            # Nếu robot phản ứng ngược, hãy đổi dấu ở đây.
            
            raw_s1 = SERVO_CENTER + out_pitch + out_roll + OFFSET_TRIM
            raw_s2 = SERVO_CENTER - out_pitch + out_roll + OFFSET_TRIM

            # Kẹp giá trị trong vùng an toàn trước khi tính toán servo sau
            val_s1 = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, raw_s1))
            val_s2 = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, raw_s2))

            # --- BƯỚC 4: Tính toán Servo Phụ (Support) ---
            # S4 ngược pha S1 | S3 ngược pha S2
            val_s4 = 180 - val_s1
            val_s3 = 180 - val_s2

            # --- BƯỚC 5: Điều khiển ---
            s1.set_angle(val_s1)
            s2.set_angle(val_s2)
            s3.set_angle(val_s3)
            s4.set_angle(val_s4)

            # Debug (In ra để theo dõi góc nghiêng và lệnh servo)
            # print(f"Pitch:{curr_pitch:.1f} Roll:{curr_roll:.1f} | S1:{int(val_s1)} S4:{int(val_s4)}")

            # Giữ tốc độ vòng lặp ổn định
            elapsed = time.ticks_diff(time.ticks_ms(), start_time)
            sleep_time = (1000 / LOOP_FREQ) - elapsed
            if sleep_time > 0:
                time.sleep_ms(int(sleep_time))

    except KeyboardInterrupt:
        print("\nDung chuong trinh!")
        # Tắt tất cả servo (về 0 hoặc thả lỏng) để an toàn
        pca.set_pwm(CH_S1, 0, 0)
        pca.set_pwm(CH_S2, 0, 0)
        pca.set_pwm(CH_S3, 0, 0)
        pca.set_pwm(CH_S4, 0, 0)

if name == "__main__":
    main()