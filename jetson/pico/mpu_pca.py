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
ADXL_ADDR = 0x53  # ADXL345 address
PCA_ADDR = 0x40

# Cấu hình kênh Servo trên PCA9685 (CHỈ DÙNG 2 SERVO)
CH_S1 = 0  # Servo 0
CH_S2 = 1  # Servo 1
CH_S3 = 2  # Không dùng
CH_S4 = 3  # Không dùng

# Góc lắp đặt servo (độ)
SERVO_MECHANICAL_ANGLE = 30  # Servo lắp nghiêng 30°

# Giới hạn góc Servo (Bảo vệ cơ khí)
SERVO_MIN_ANGLE = 20
SERVO_MAX_ANGLE = 160
SERVO_CENTER = 90

# THAM SỐ PID (Cần tinh chỉnh thực tế)
# Kp: Độ nhạy | Ki: Cộng dồn lỗi | Kd: Chống rung
PID_PITCH_PARAMS = {'kp': 2.0, 'ki': 0.0, 'kd': 0.5} 
PID_ROLL_PARAMS  = {'kp': 2.0, 'ki': 0.0, 'kd': 0.5}

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
# 3. CLASS: ADXL345 (DRIVER ACCELEROMETER)
# ==========================================
class ADXL345:
    def __init__(self, i2c, addr=0x53):
        self.i2c = i2c
        self.addr = addr
        
        # Khởi động ADXL345
        self.i2c.writeto_mem(self.addr, 0x2D, b'\x00')  # Reset
        time.sleep(0.005)
        self.i2c.writeto_mem(self.addr, 0x2D, b'\x08')  # Measure mode
        self.i2c.writeto_mem(self.addr, 0x31, b'\x0B')  # Full resolution, ±16g, 13-bit
        time.sleep(0.1)  # Đợi ổn định
        
        # Khởi tạo góc ban đầu từ accelerometer
        raw = self.i2c.readfrom_mem(self.addr, 0x32, 6)
        vals = ustruct.unpack('<hhh', raw)
        accX, accY, accZ = vals[0], vals[1], vals[2]
        
        # Tính góc ban đầu (chỉ dùng accelerometer)
        acc_pitch = math.atan2(accY, accZ) * 57.2958
        acc_roll = math.atan2(-accX, math.sqrt(accY**2 + accZ**2)) * 57.2958
        
        # Gán giá trị khởi tạo
        self.angle_pitch = acc_pitch
        self.angle_roll = -acc_roll  # Đảo chiều roll
        
        # ADXL345 không có gyroscope nên dùng low-pass filter
        self.alpha = 0.1  # Hệ số lọc (0.1 = lọc mạnh, 0.9 = nhạy)
        
    def update(self):
        # Đọc Accelerometer (ADXL345 chỉ có accelerometer)
        raw = self.i2c.readfrom_mem(self.addr, 0x32, 6)
        vals = ustruct.unpack('<hhh', raw)
        
        # ADXL345: 3.9mg/LSB ở ±16g, full resolution
        # Scale factor: 3.9mg = 0.0039g
        accX = vals[0] * 0.0039
        accY = vals[1] * 0.0039
        accZ = vals[2] * 0.0039

        # Tính góc từ gia tốc
        acc_pitch = math.atan2(accY, accZ) * 57.2958
        acc_roll = math.atan2(-accX, math.sqrt(accY**2 + accZ**2)) * 57.2958

        # Low-pass filter (vì không có gyroscope)
        # Công thức: angle_new = alpha * acc_angle + (1-alpha) * angle_old
        self.angle_pitch = self.alpha * acc_pitch + (1 - self.alpha) * self.angle_pitch
        self.angle_roll = self.alpha * (-acc_roll) + (1 - self.alpha) * self.angle_roll

        return self.angle_pitch, self.angle_roll

# ==========================================
# 4. CLASS: PID CONTROLLER
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
    print("--- KHOI TAO ROBOT 2 SERVO (ADXL345) ---")
    
    # 1. Setup I2C
    try:
        i2c = I2C(I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        print("I2C OK. Devices:", [hex(x) for x in i2c.scan()])
    except Exception as e:
        print("Loi I2C:", e)
        return

    # 2. Setup Modules
    adxl = ADXL345(i2c, ADXL_ADDR)
    pca = PCA9685(i2c, PCA_ADDR)
    pca.set_pwm_freq(50) # Servo standard 50Hz

    # 3. Setup Servos (CHỈ DÙNG 2 SERVO)
    s0 = Servo(pca, CH_S1) # Servo 0
    s1 = Servo(pca, CH_S2) # Servo 1
    s2 = Servo(pca, CH_S3) # Servo 0
    s3 = Servo(pca, CH_S4) # Servo 1
    # Servo lắp nghiêng 30 độ

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
            curr_pitch, curr_roll = adxl.update()

            # --- BƯỚC 2: Tính toán PID ---
            # Output PID chính là lượng góc cần bù vào
            out_pitch = pid_pitch.compute(curr_pitch)
            out_roll  = pid_roll.compute(curr_roll)

            # --- BƯỚC 3: Mixing (Trộn kênh) với bù góc lắp nghiêng 30° ---
            # Roll: S0 và S1 cùng chiều (95,100 hoặc 85,80)
            # Pitch: S0 và S1 ngược chiều (S0 tăng, S1 giảm)
            # 
            # Bù góc lắp nghiêng 30°:
            # - Để cân bằng Roll (cúi/ngẩng), cần bù thêm 30° vào output
            # - Công thức mixing có thể cần điều chỉnh hệ số tùy góc lắp
            
            MECHANICAL_ANGLE = 30  # Góc lắp servo (độ)
            
            # Hệ số bù cho góc nghiêng (có thể cần tinh chỉnh)
            # Với góc 30°: sin(30°)=0.5, cos(30°)≈0.866
            roll_factor = 1.0 / math.cos(math.radians(MECHANICAL_ANGLE))  # ≈1.15
            
            # Mixing với bù góc
            raw_s0 = SERVO_CENTER + (out_roll * roll_factor) - out_pitch + OFFSET_TRIM
            raw_s1 = SERVO_CENTER + (out_roll * roll_factor) + out_pitch + OFFSET_TRIM

            # Kẹp giá trị trong vùng an toàn
            val_s0 = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, raw_s0))
            val_s1 = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, raw_s1))

            # --- BƯỚC 4: Điều khiển chỉ 2 SERVO ---
            s0.set_angle(val_s0)
            s1.set_angle(val_s1)
            s3.set_angle(180-val_s1)
            s2.set_angle(180-val_s0)

            # Debug
            print(f"Pitch:{curr_pitch:.1f} Roll:{curr_roll:.1f} | S0:{int(val_s0)} S1:{int(val_s1)}")

            # Giữ tốc độ vòng lặp ổn định
            elapsed = time.ticks_diff(time.ticks_ms(), start_time)
            sleep_time = (1000 / LOOP_FREQ) - elapsed
            if sleep_time > 0:
                time.sleep_ms(int(sleep_time))

    except KeyboardInterrupt:
        print("\nDung chuong trinh!")
        # Tắt 2 servo (về 0 hoặc thả lỏng) để an toàn
        pca.set_pwm(CH_S1, 0, 0)
        pca.set_pwm(CH_S2, 0, 0)

# ✅ SỬA LẠI: Syntax đúng
if __name__ == "__main__":
    main()

