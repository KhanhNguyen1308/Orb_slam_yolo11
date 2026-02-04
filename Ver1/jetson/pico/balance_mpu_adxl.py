from machine import I2C, Pin
import time
import math
import ustruct

# ==========================================
# 1. CẤU HÌNH HỆ THỐNG & HÌNH HỌC (GEOMETRY)
# ==========================================
I2C_ID = 0
SDA_PIN = 20
SCL_PIN = 21
I2C_ID1 = 1
SDA_PIN1 = 18
SCL_PIN1 = 19

ADDR_ADXL = 0x53
ADDR_MPU  = 0x68
ADDR_PCA  = 0x40

# --- THÔNG SỐ CƠ KHÍ CHÍNH XÁC ---
# Khoảng cách từ tâm trục quay đến điểm đặt lực của Servo
ARM_PITCH = 33.34   # mm (Khoảng cách dọc - Forward)
ARM_ROLL  = 57.75   # mm (Khoảng cách ngang - Side: 115.5 / 2)
MOUNT_ANGLE = 60    # độ (Góc nghiêng thanh đẩy servo)

# Tỷ lệ bù hình học (Mixing Ratio)
# Vì tay đòn Roll dài hơn Pitch, servo cần hành trình lớn hơn để tạo cùng 1 góc nghiêng
MIX_RATIO = ARM_ROLL / ARM_PITCH  # ~1.73

# Servo Config
SERVO_LEFT_CH  = 0
SERVO_RIGHT_CH = 1
SERVO_CENTER   = 90

# PID Config (Kp cần giảm xuống vì hệ thống tay đòn dài nhạy hơn)
PID_PITCH = {'kp': 3.0, 'ki': 0.05, 'kd': 0.8}
PID_ROLL  = {'kp': 3.0, 'ki': 0.05, 'kd': 0.8}

# ==========================================
# 2. DRIVER CLASSES (Rút gọn)
# ==========================================
class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c; self.addr = address; self.reset()
    def _write(self, reg, val): self.i2c.writeto_mem(self.addr, reg, bytearray([val]))
    def _read(self, reg): return self.i2c.readfrom_mem(self.addr, reg, 1)[0]
    def reset(self): self._write(0x00, 0x00)
    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / 4096.0 / freq - 1)
        old = self._read(0x00); self._write(0x00, (old & 0x7F) | 0x10)
        self._write(0xFE, prescale); self._write(0x00, old); time.sleep(0.005); self._write(0x00, old | 0xa1)
    def set_pwm(self, ch, on, off):
        self._write(0x06 + 4*ch, on & 0xFF); self._write(0x07 + 4*ch, on >> 8)
        self._write(0x08 + 4*ch, off & 0xFF); self._write(0x09 + 4*ch, off >> 8)

class Servo:
    def __init__(self, pca, ch): self.pca = pca; self.ch = ch
    def set_angle(self, angle):
        angle = max(10, min(170, angle)) # Giới hạn góc an toàn
        us = 500 + (angle / 180) * 2000
        duty = int((us / 20000) * 4096)
        self.pca.set_pwm(self.ch, 0, duty)

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c; self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        self.angle_pitch = 0; self.angle_roll = 0
        self.last_time = time.ticks_ms(); self.alpha = 0.96
    def update(self):
        raw = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        vals = ustruct.unpack('>hhhhhhh', raw)
        ax, ay, az = vals[0], vals[1], vals[2]
        gx, gy, gz = vals[4]/131.0, vals[5]/131.0, vals[6]/131.0
        now = time.ticks_ms(); dt = time.ticks_diff(now, self.last_time)/1000.0; self.last_time = now
        
        acc_p = math.atan2(ay, az) * 57.296
        acc_r = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.296
        
        self.angle_pitch = self.alpha*(self.angle_pitch + gx*dt) + (1-self.alpha)*acc_p
        self.angle_roll  = self.alpha*(self.angle_roll  + gy*dt) + (1-self.alpha)*acc_r
        return self.angle_pitch, self.angle_roll

class ADXL345:
    def __init__(self, i2c, addr=0x53):
        self.i2c = i2c; self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x2D, b'\x08'); self.i2c.writeto_mem(self.addr, 0x31, b'\x0B')
    def get_angles(self):
        raw = self.i2c.readfrom_mem(self.addr, 0x32, 6)
        x, y, z = ustruct.unpack('<hhh', raw)
        pitch = math.atan2(y, z) * 57.296
        roll  = math.atan2(-x, math.sqrt(y*y + z*z)) * 57.296
        return pitch, roll

class PID:
    def __init__(self, kp, ki, kd):
        self.k = {'p': kp, 'i': ki, 'd': kd}
        self.prev_err = 0; self.integral = 0
    def compute(self, setpoint, measured):
        err = setpoint - measured
        self.integral = max(-100, min(100, self.integral + err))
        out = (self.k['p']*err) + (self.k['i']*self.integral) + (self.k['d']*(err - self.prev_err))
        self.prev_err = err
        return out

# ==========================================
# 3. LOGIC TRỘN KÊNH (MIXING) - QUAN TRỌNG
# ==========================================
def apply_mixing(pid_pitch, pid_roll):
    """
    Input: pid_pitch (độ bù trục dọc), pid_roll (độ bù trục ngang)
    Output: Góc servo Trái, Góc servo Phải
    """
    # 1. Bù góc lắp đặt (60 độ)
    # Lực đẩy thực tế = F * sin(60). Để đạt độ cao H, cần hành trình L = H / sin(60)
    sin_60 = 0.866
    mount_factor = 1.0 / sin_60 # ~1.15
    
    # 2. Bù tỷ lệ tay đòn (Lever Arm Ratio)
    # Roll Arm (57.75) dài hơn Pitch Arm (33.34) -> Cần bù nhiều hơn cho Roll
    roll_compensated = pid_roll * MIX_RATIO 
    
    # 3. Tính toán Differential (Vi sai)
    # Pitch: Cả 2 servo cùng lên/xuống -> Cộng pid_pitch
    # Roll: 1 lên, 1 xuống -> Cộng/Trừ pid_roll
    
    # Giả sử cả 2 servo đều lắp phía trước (Forward)
    # Servo Left: Cần gánh Pitch + Roll
    # Servo Right: Cần gánh Pitch - Roll
    
    delta_left  = (pid_pitch + roll_compensated) * mount_factor
    delta_right = (pid_pitch - roll_compensated) * mount_factor
    
    return delta_left, delta_right

# ==========================================
# 4. MAIN
# ==========================================
def main():
    i2c = I2C(I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
    i2c1 = I2C(I2C_ID1, scl=Pin(SCL_PIN1), sda=Pin(SDA_PIN1), freq=400000)
    # Khởi tạo modules
    try:
        mpu = MPU6050(i2c, ADDR_MPU)
        pca = PCA9685(i2c, ADDR_PCA); pca.set_pwm_freq(50)
        
        # Thử khởi tạo ADXL (Optional)
        adxl_present = False
        if ADDR_ADXL in i2c1.scan():
            adxl = ADXL345(i2c1, ADDR_ADXL)
            adxl_present = True
            print("ADXL345 detected.")
        else:
            print("ADXL345 not found.")

        s_left = Servo(pca, SERVO_LEFT_CH)
        s_right = Servo(pca, SERVO_RIGHT_CH)
        
        pid_p = PID(**PID_PITCH)
        pid_r = PID(**PID_ROLL)
        
        print("Robot start in 2 seconds...")
        s_left.set_angle(SERVO_CENTER)
        s_right.set_angle(SERVO_CENTER)
        time.sleep(2)
        
        while True:
            # 1. Đọc MPU6050 (Bàn cân bằng)
            plat_p, plat_r = mpu.update()
            
            # 2. Đọc ADXL345 (Địa hình) - Nếu có
            # Có thể dùng để thay đổi Setpoint: setpoint = -terrain_angle
            setpoint_p, setpoint_r = 0, 0
            if adxl_present:
                terr_p, terr_r = adxl.get_angles()
                # Debug địa hình
                # print(f"Terrain: P={terr_p:.1f} R={terr_r:.1f}")

            # 3. Tính toán PID
            # Setpoint = 0 nghĩa là luôn muốn bàn nằm ngang tuyệt đối
            out_p = pid_p.compute(setpoint_p, plat_p)
            out_r = pid_r.compute(setpoint_r, plat_r)
            
            # 4. Trộn kênh & Điều khiển
            d_left, d_right = apply_mixing(out_p, out_r)
            
            # Lưu ý chiều quay Servo:
            # Nếu servo TRÁI và PHẢI lắp đối xứng qua trục dọc,
            # một bên sẽ cần đảo ngược logic (+ thành -).
            # Ở đây giả sử lắp cùng chiều (cùng hướng trục quay).
            
            s_left.set_angle(SERVO_CENTER + d_left)
            s_right.set_angle(SERVO_CENTER + d_right) # Hoặc SERVO_CENTER - d_right nếu ngược
            
            time.sleep(0.01) # 100Hz Loop

    except Exception as e:
        print("Error:", e)
        # Dừng servo khi lỗi
        if 'pca' in locals():
            pca.set_pwm(SERVO_LEFT_CH, 0, 0)
            pca.set_pwm(SERVO_RIGHT_CH, 0, 0)

if __name__ == "__main__":
    main()