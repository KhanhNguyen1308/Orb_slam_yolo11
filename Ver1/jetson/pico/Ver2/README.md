# 🤖 ROBOT CÂN BẰNG 2 TẦNG - HƯỚNG DẪN SỬ DỤNG

## 📋 MỤC LỤC
1. [Tổng quan hệ thống](#tổng-quan-hệ-thống)
2. [Yêu cầu phần cứng](#yêu-cầu-phần-cứng)
3. [Cài đặt](#cài-đặt)
4. [Cấu hình chi tiết](#cấu-hình-chi-tiết)
5. [Quy trình kiểm tra](#quy-trình-kiểm-tra)
6. [Tinh chỉnh PID](#tinh-chỉnh-pid)
7. [Troubleshooting](#troubleshooting)

---

## 🎯 TỔNG QUAN HỆ THỐNG

### Nguyên lý hoạt động

```
┌─────────────────────────────────────┐
│      THÂN TRÊN (Platform)           │
│    ┌──────────────────┐             │
│    │    MPU6050       │ ← Đo góc    │
│    └──────────────────┘             │
│                                     │
│  ┌─────┐           ┌─────┐          │
│  │Servo│           │Servo│          │
│  │Left │           │Right│          │
│  └──┬──┘           └──┬──┘          │
├─────┼─────────────────┼─────────────┤
│     └─────┬───────┬───┘             │
│           │  60°  │                 │
│      THÂN DƯỚI (Chassis)            │
│    ┌──────────────────┐             │
│    │   ADXL345        │ ← Đo địa hình
│    └──────────────────┘             │
│         Bánh xích                   │
└─────────────────────────────────────┘
```

**Cơ chế:**
1. **ADXL345** (Tầng dưới) đo độ nghiêng địa hình
2. **MPU6050** (Tầng trên) đo độ nghiêng thực tế của platform
3. **PID Controller** tính toán độ bù cần thiết
4. **Mixing Algorithm** phân phối lực cho 2 servo
5. **Servo** điều chỉnh góc nghiêng của platform

---

## ⚙️ YÊU CẦU PHẦN CỨNG

### Danh sách linh kiện

| Linh kiện | Số lượng | Mô tả |
|-----------|----------|-------|
| MPU6050 | 1 | IMU 6-axis (gắn thân trên) |
| ADXL345 | 1 | Accelerometer 3-axis (gắn thân dưới) |
| PCA9685 | 1 | 16-channel PWM Servo Driver |
| Servo MG90S/SG90 | 2 | Servo góc 180° (hoặc tương tự) |
| Raspberry Pi Pico | 1 | Hoặc ESP32/ESP8266 với MicroPython |
| Nguồn 5V | 1 | Cho servo (ít nhất 2A) |

### Sơ đồ kết nối

```
Raspberry Pi Pico
┌─────────────────┐
│  GP20 (I2C SDA) ├────┬─── MPU6050 SDA
│  GP21 (I2C SCL) ├────┼─── MPU6050 SCL
│                 │    ├─── ADXL345 SDA
│                 │    └─── ADXL345 SCL
│                 │         PCA9685 SDA
│                 │         PCA9685 SCL
│                 │
│  3.3V           ├────── MPU6050 VCC
│  3.3V           ├────── ADXL345 VCC
│  GND            ├────── All GND
└─────────────────┘

PCA9685
┌─────────────────┐
│  V+             ├────── 5V Power (Servo)
│  GND            ├────── GND
│                 │
│  PWM 0          ├────── Servo Left (Signal)
│  PWM 1          ├────── Servo Right (Signal)
└─────────────────┘

QUAN TRỌNG: 
- I2C dùng chung 3.3V logic
- Servo dùng nguồn 5V riêng
- Nối GND chung tất cả
```

### Lắp đặt cơ khí

**Thông số quan trọng:**

```
                 ┌────── Platform ──────┐
                 │                       │
                 │     ARM_PITCH         │
                 │     = 33.34mm         │
    ┌───────┐    │                       │    ┌───────┐
    │Servo L├────┼───────┐   ┌───────────┼────┤Servo R│
    └───────┘    │       │   │           │    └───────┘
                 │       └───┘           │
                 │       60°             │
                 │  MOUNT_ANGLE          │
                 │                       │
                 │◄──────115.5mm────────►│
                 │   SERVO_SPACING       │
                 └───────────────────────┘
                          Chassis
```

**Lưu ý:**
- ARM_PITCH = 33.34mm: Khoảng cách từ tâm quay đến điểm đặt lực servo (chiều dọc)
- ARM_ROLL = 57.75mm: = SERVO_SPACING / 2 = 115.5 / 2
- MOUNT_ANGLE = 60°: Góc nghiêng thanh đẩy servo với trục ngang

---

## 📦 CÀI ĐẶT

### Bước 1: Upload code lên MicroPython

```bash
# Sử dụng Thonny IDE hoặc ampy
ampy --port /dev/ttyUSB0 put balance_robot_v2.py main.py

# Hoặc copy paste trực tiếp vào Thonny
```

### Bước 2: Chạy test

```python
# Trong REPL:
import main
main.main()
```

---

## 🔧 CẤU HÌNH CHI TIẾT

### 1. Cấu hình I2C (Config class)

```python
class Config:
    # --- I2C CONFIG ---
    I2C_ID = 0          # I2C bus number (0 hoặc 1)
    SDA_PIN = 20        # GPIO cho SDA
    SCL_PIN = 21        # GPIO cho SCL
    I2C_FREQ = 400000   # 400kHz (standard: 100kHz, fast: 400kHz)
```

**Lưu ý:**
- Pico: I2C0 (GP0,GP1 hoặc GP4,GP5, GP8,GP9, GP12,GP13, GP16,GP17, GP20,GP21)
- ESP32: Bất kỳ GPIO nào (software I2C)

### 2. Cấu hình Servo

```python
    # --- SERVO CONFIG ---
    SERVO_LEFT_CHANNEL = 0      # Kênh PCA9685 (0-15)
    SERVO_RIGHT_CHANNEL = 1     
    SERVO_CENTER = 90           # Góc trung tâm (độ)
    SERVO_MIN_ANGLE = 10        # Giới hạn tối thiểu
    SERVO_MAX_ANGLE = 170       # Giới hạn tối đa
    
    # ĐẢO CHIỀU SERVO (Quan trọng!)
    SERVO_LEFT_INVERT = False   # True nếu servo trái quay ngược
    SERVO_RIGHT_INVERT = True   # True nếu servo phải quay ngược
```

**Cách xác định chiều servo:**
1. Đặt cả 2 servo về 90°
2. Nghiêng robot về phía trước → Servo phải đẩy lên (ngược lại)
3. Nếu servo đẩy sai chiều → Đổi flag INVERT

### 3. Cấu hình PID

```python
    # --- PID PARAMETERS ---
    # PITCH (Nghiêng trước/sau)
    PID_PITCH_KP = 2.5    # Tăng để phản ứng nhanh hơn
    PID_PITCH_KI = 0.03   # Tăng để giảm lệch setpoint lâu dài
    PID_PITCH_KD = 1.2    # Tăng để giảm dao động
    
    # ROLL (Nghiêng trái/phải)
    PID_ROLL_KP = 2.5
    PID_ROLL_KI = 0.03
    PID_ROLL_KD = 1.2
```

**Nguyên tắc tinh chỉnh:** (Xem phần [Tinh chỉnh PID](#tinh-chỉnh-pid))

### 4. Chế độ điều khiển

```python
    # --- CONTROL MODE ---
    USE_ADXL_FEEDFORWARD = True   # Bật/tắt feedforward từ ADXL
    FEEDFORWARD_GAIN = 0.5        # Hệ số bù (0.0 - 1.0)
```

**Giải thích:**
- `False`: Thân trên luôn giữ nằm ngang tuyệt đối (0°)
- `True`: Thân trên bù trước theo địa hình
  - `FEEDFORWARD_GAIN = 0.5`: Bù 50% góc địa hình
  - Ví dụ: Địa hình +10° → Setpoint = -5°

### 5. Tần số vòng lặp

```python
    # --- LOOP TIMING ---
    LOOP_FREQUENCY = 100   # Hz (50-100 Hz khuyến nghị)
```

**Khuyến nghị:**
- 50Hz: Đủ cho hầu hết ứng dụng, tiết kiệm CPU
- 100Hz: Phản ứng nhanh hơn, cần CPU mạnh
- >100Hz: Không cần thiết (servo chỉ cập nhật 50Hz)

---

## 🧪 QUY TRÌNH KIỂM TRA

### Bước 1: Kiểm tra I2C

```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
devices = i2c.scan()
print([hex(d) for d in devices])

# Kết quả mong đợi:
# ['0x53', '0x68', '0x40']
# = ADXL345, MPU6050, PCA9685
```

### Bước 2: Test từng servo

```python
from machine import I2C, Pin
import time

# ... (khởi tạo PCA9685 như trong code)

# Test Servo Left
for angle in [60, 90, 120, 90]:
    servo_left.set_angle(angle)
    print(f"Left: {angle}°")
    time.sleep(1)

# Test Servo Right
for angle in [60, 90, 120, 90]:
    servo_right.set_angle(angle)
    print(f"Right: {angle}°")
    time.sleep(1)
```

**Kiểm tra:**
- Servo có quay êm không?
- Góc 90° có thực sự là vị trí trung tâm không?
- Chiều quay có đúng không?

### Bước 3: Test MPU6050

```python
# Đọc góc MPU6050
for i in range(50):
    pitch, roll = mpu.update()
    print(f"Pitch: {pitch:+6.2f}°  Roll: {roll:+6.2f}°")
    time.sleep(0.1)
```

**Kiểm tra:**
- Khi robot nằm ngang → Pitch ≈ 0°, Roll ≈ 0°
- Nghiêng về phía trước → Pitch dương
- Nghiêng sang trái → Roll âm (hoặc dương, tùy trục)

### Bước 4: Test ADXL345 (nếu có)

```python
for i in range(50):
    pitch, roll = adxl.get_angles()
    print(f"Terrain - Pitch: {pitch:+6.2f}°  Roll: {roll:+6.2f}°")
    time.sleep(0.1)
```

### Bước 5: Test toàn hệ thống

1. **Chạy code chính:**
   ```python
   import main
   main.main()
   ```

2. **Kiểm tra phản ứng:**
   - Nghiêng robot → Servo phải phản ứng ngược lại
   - Không dao động quá mức
   - Trở về vị trí cân bằng

---

## 📊 TINH CHỈNH PID

### Quy trình Ziegler-Nichols đơn giản

**Bước 1: Tắt I và D**
```python
PID_PITCH_KP = 1.0
PID_PITCH_KI = 0.0
PID_PITCH_KD = 0.0
```

**Bước 2: Tăng Kp dần**
- Bắt đầu từ Kp = 0.5
- Tăng từng bước 0.5: 1.0 → 1.5 → 2.0 → 2.5...
- Dừng khi hệ thống **bắt đầu dao động đều** (oscillation)
- Gọi giá trị này là **Ku** (Ultimate Gain)

**Bước 3: Đo chu kỳ dao động**
- Đo thời gian 1 chu kỳ dao động (giây)
- Gọi là **Tu** (Ultimate Period)

**Bước 4: Tính PID theo công thức**

| Kiểu điều khiển | Kp | Ki | Kd |
|-----------------|----|----|-----|
| P | 0.5 * Ku | 0 | 0 |
| PI | 0.45 * Ku | 1.2*Kp/Tu | 0 |
| PID | 0.6 * Ku | 2*Kp/Tu | Kp*Tu/8 |

**Ví dụ:**
- Ku = 4.0 (dao động tại Kp=4.0)
- Tu = 0.5s (dao động với chu kỳ 500ms)

→ PID:
```python
PID_PITCH_KP = 0.6 * 4.0 = 2.4
PID_PITCH_KI = 2 * 2.4 / 0.5 = 9.6  # (có thể giảm xuống)
PID_PITCH_KD = 2.4 * 0.5 / 8 = 0.15
```

### Tinh chỉnh thủ công (Manual Tuning)

**Nếu hệ thống quá chậm:**
- Tăng Kp (+0.5)

**Nếu dao động nhiều:**
- Giảm Kp (-0.5)
- Tăng Kd (+0.2)

**Nếu không về đúng vị trí:**
- Tăng Ki (+0.01)

**Nếu phản ứng quá mạnh:**
- Giảm Kp
- Giảm Kd

---

## 🐛 TROUBLESHOOTING

### Lỗi 1: "MPU6050 not found"

**Nguyên nhân:**
- Kết nối I2C lỏng
- Địa chỉ I2C sai
- Pull-up resistor thiếu

**Giải pháp:**
```python
# Kiểm tra scan I2C
i2c = I2C(0, scl=Pin(21), sda=Pin(20))
print(i2c.scan())

# Nếu trống → Kiểm tra dây
# Nếu có địa chỉ khác 0x68 → Thay đổi ADDR_MPU6050
```

### Lỗi 2: Servo không chuyển động

**Kiểm tra:**
1. Nguồn 5V có đủ dòng không? (ít nhất 1A)
2. PCA9685 có nhận được tín hiệu I2C không?
3. Kênh servo đúng chưa?

**Test:**
```python
# Set PWM trực tiếp
pca.set_pwm(0, 0, 300)  # Servo 0, duty ~300
time.sleep(1)
pca.set_pwm(0, 0, 400)  # Duty ~400
```

### Lỗi 3: Hệ thống dao động liên tục

**Nguyên nhân:**
- PID Kp quá cao
- Kd quá thấp
- Cơ khí lỏng lẻo

**Giải pháp:**
1. Giảm Kp xuống 50%
2. Tăng Kd lên gấp đôi
3. Kiểm tra ốc vít servo

### Lỗi 4: Servo quay ngược chiều

**Giải pháp:**
```python
# Đổi flag INVERT
SERVO_LEFT_INVERT = True   # Hoặc False
SERVO_RIGHT_INVERT = False # Hoặc True
```

### Lỗi 5: Góc đọc từ MPU6050 sai

**Kiểm tra hướng lắp đặt:**
```
MPU6050 Orientation:
     +Y
      │
      │
+X────┼────  (Chip face up)
      │
      │
     
Pitch: Xoay quanh trục X
Roll:  Xoay quanh trục Y
```

Nếu lắp ngược → Cần đổi dấu trong code:
```python
# Trong MPU6050.update():
acc_pitch = -math.atan2(ay, az) * 57.2958  # Đổi dấu
```

### Lỗi 6: "Loop overrun" warning

**Nguyên nhân:**
- Vòng lặp chạy quá chậm
- LOOP_FREQUENCY quá cao

**Giải pháp:**
```python
# Giảm tần số xuống
LOOP_FREQUENCY = 50  # Từ 100 xuống 50
```

---

## 📈 TỐI ƯU HÓA HIỆU NĂNG

### 1. Giảm độ trễ I2C

```python
# Tăng tốc độ I2C
I2C_FREQ = 400000  # 400kHz (tối đa cho MPU6050)
```

### 2. Tắt Debug khi chạy thực tế

```python
DEBUG_PRINT = False  # Tiết kiệm CPU
```

### 3. Điều chỉnh Complementary Filter

```python
# MPU6050 Alpha
MPU_ALPHA = 0.96  # Nhiều gyro (ít nhiễu accelerometer)
MPU_ALPHA = 0.85  # Nhiều accel (ít drift gyro)
```

---

## 📚 THAM KHẢO

### Công thức Mixing

```python
# Lever Arm Ratio
MIX_RATIO = ARM_ROLL / ARM_PITCH = 57.75 / 33.34 ≈ 1.73

# Mount Angle Compensation
MOUNT_COMP = 1 / sin(60°) = 1 / 0.866 ≈ 1.155

# Final Formula:
delta_left  = (pid_pitch + pid_roll * 1.73) * 1.155
delta_right = (pid_pitch - pid_roll * 1.73) * 1.155
```

### Giới hạn an toàn

- PID Output: ±45° (tránh servo quá giới hạn)
- Servo Range: 10° - 170° (tránh stall)
- Loop Frequency: 50-100 Hz (tối ưu)

---

## ✅ CHECKLIST TRƯỚC KHI CHẠY

- [ ] Tất cả dây đã kết nối chắc chắn
- [ ] Nguồn 5V đủ dòng (>1A)
- [ ] I2C scan thấy 3 thiết bị (0x40, 0x53, 0x68)
- [ ] Servo đã test riêng lẻ
- [ ] MPU6050 đọc góc chính xác
- [ ] Chiều servo đã kiểm tra
- [ ] PID đã tinh chỉnh sơ bộ
- [ ] Code đã upload lên board

---

## 📞 HỖ TRỢ

Nếu gặp vấn đề không giải quyết được, vui lòng cung cấp:
1. Log output đầy đủ
2. Giá trị Config đang dùng
3. Video hành vi của robot
4. Kết quả I2C scan

---

**Chúc bạn thành công! 🚀**