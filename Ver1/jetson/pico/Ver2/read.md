# 🔄 CẬP NHẬT QUAN TRỌNG - AXIS REMAPPING & DUAL I2C

## 📋 TÓM TẮT THAY ĐỔI

### 1. **Dual I2C Bus Configuration**

Robot hiện sử dụng **2 I2C bus riêng biệt** để tránh xung đột:

```
┌─────────────────────────────────────┐
│         I2C BUS 0 (Primary)         │
│  GPIO 20 (SDA) / GPIO 21 (SCL)      │
├─────────────────────────────────────┤
│  ✓ MPU6050  (0x68) - Platform IMU  │
│  ✓ PCA9685  (0x40) - Servo Driver  │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│       I2C BUS 1 (Secondary)         │
│  GPIO 18 (SDA) / GPIO 19 (SCL)      │
├─────────────────────────────────────┤
│  ✓ ADXL345  (0x53) - Terrain IMU   │
└─────────────────────────────────────┘
```

**Lợi ích:**
- Tránh xung đột địa chỉ I2C
- Tốc độ truyền thông tốt hơn
- Dễ debug từng bus riêng lẻ

---

### 2. **Axis Remapping - QUAN TRỌNG!**

**Vấn đề ban đầu:**
- MPU6050 và ADXL345 có định nghĩa trục khác nhau
- X của ADXL345 ≠ X của MPU6050
- Y của ADXL345 ≠ Y của MPU6050

**Giải pháp:**
- **ADXL345 được chọn làm hệ trục chuẩn** (reference)
- **MPU6050 được remap** để khớp với ADXL345

```
┌──────────────────────────────────────────────────┐
│         COORDINATE SYSTEM MAPPING                │
├──────────────────────────────────────────────────┤
│  ADXL345 (Reference)  →   MPU6050 (Hardware)     │
│                                                  │
│  ADXL X axis          =   MPU Y axis             │
│  ADXL Y axis          =   MPU X axis             │
│  ADXL Z axis          =   MPU Z axis (unchanged) │
└──────────────────────────────────────────────────┘
```

**Trong code MPU6050.update():**

```python
# Đọc raw data từ MPU6050
ax_mpu, ay_mpu, az_mpu = values[0], values[1], values[2]
gx_mpu, gy_mpu, gz_mpu = ...

# HOÁN ĐỔI TRỤC để khớp ADXL345:
ax = ay_mpu  # ADXL X = MPU Y
ay = ax_mpu  # ADXL Y = MPU X
az = az_mpu  # Z giữ nguyên

gx = gy_mpu  # Gyro Pitch
gy = gx_mpu  # Gyro Roll
```

**Kết quả:**
- Khi ADXL345 đọc Pitch = +10°, MPU6050 cũng đọc Pitch = +10°
- Khi ADXL345 đọc Roll = -5°, MPU6050 cũng đọc Roll = -5°
- **Hai sensor giờ đây đồng bộ hoàn toàn!**

---

## 🔌 WIRING DIAGRAM MỚI

```
Raspberry Pi Pico
┌─────────────────────────┐
│                         │
│  GP20 (I2C0 SDA) ───────┼─── MPU6050 SDA
│  GP21 (I2C0 SCL) ───────┼─── MPU6050 SCL
│                         │    PCA9685 SDA
│                         │    PCA9685 SCL
│                         │
│  GP18 (I2C1 SDA) ───────┼─── ADXL345 SDA
│  GP19 (I2C1 SCL) ───────┼─── ADXL345 SCL
│                         │
│  3.3V ──────────────────┼─── MPU6050 VCC
│  3.3V ──────────────────┼─── ADXL345 VCC
│  3.3V ──────────────────┼─── PCA9685 VCC
│  GND  ──────────────────┼─── All GND
└─────────────────────────┘

PCA9685
┌─────────────────────────┐
│  V+  ───────────────────┼─── 5V Power (Servo)
│  GND ───────────────────┼─── GND
│  PWM 0 ─────────────────┼─── Servo Left Signal
│  PWM 1 ─────────────────┼─── Servo Right Signal
└─────────────────────────┘

NOTES:
✓ Pull-up resistors (4.7kΩ) on both I2C buses
✓ Servo power supply: 5V, >1A
✓ Common ground for all devices
```

---

## 🧪 KIỂM TRA SAU KHI CẬP NHẬT

### Test 1: I2C Bus Scan

```python
import test_utils
test_utils.test_i2c()
```

**Kết quả mong đợi:**
```
--- I2C Bus 0 (MPU6050 + PCA9685) ---
✓ Found 2 device(s) on Bus 0:
  ✓ 0x40 (64) - PCA9685 (Servo Driver)
  ✓ 0x68 (104) - MPU6050 (IMU - Platform)

--- I2C Bus 1 (ADXL345 - Terrain) ---
✓ Found 1 device(s) on Bus 1:
  ✓ 0x53 (83) - ADXL345 (Accelerometer - Terrain)
```

### Test 2: IMU Comparison (Quan trọng!)

```python
test_utils.test_imu_comparison(duration=10)
```

**Kết quả mong đợi:**
- **Pitch và Roll của MPU6050 và ADXL345 giờ phải gần giống nhau!**
- Sai số < 2-3° là bình thường
- Nếu sai số > 10° → Có vấn đề với axis remapping

```
Time  |   MPU6050    |   ADXL345    | Difference
      | Pitch | Roll | Pitch | Roll | Pitch | Roll
------|-------|------|-------|------|-------|------
  0.1s |  +0.5 | -1.2 |  +0.8 | -1.5 |  -0.3 | +0.3  ← Tốt!
  0.2s |  +2.1 | -0.8 |  +2.3 | -1.0 |  -0.2 | +0.2  ← Tốt!
```

### Test 3: Quick Diagnostic

```python
test_utils.quick_diagnostic()
```

**Kết quả mong đợi:**
```
✓ I2C0           : OK
✓ I2C1           : OK
✓ MPU6050        : OK
✓ ADXL345        : OK
✓ PCA9685        : OK
✓ SERVO LEFT     : OK
✓ SERVO RIGHT    : OK

✓ All critical components OK!
  System ready to run.

📌 COORDINATE SYSTEM:
   ADXL345 = REFERENCE standard
   MPU6050 axes REMAPPED to match ADXL345
```

---

## 📂 FILES ĐÃ THAY ĐỔI

1. **`balance_robot_v2.py`**
   - ✅ Config: Thêm I2C0 và I2C1 config
   - ✅ MPU6050 class: Thêm axis remapping logic
   - ✅ BalanceRobot.initialize(): Khởi tạo 2 I2C bus riêng

2. **`test_utils.py`**
   - ✅ Tất cả test functions: Cập nhật để dùng đúng I2C bus
   - ✅ test_i2c(): Scan cả 2 bus
   - ✅ test_imu_comparison(): So sánh MPU vs ADXL sau khi remap

3. **`README_BALANCE_ROBOT.md`**
   - (Cần cập nhật manual - sẽ làm ở bước sau)

4. **`config_presets.py`**
   - (Không thay đổi - vẫn dùng được)

---

## ⚠️ LƯU Ý QUAN TRỌNG

### Nếu MPU6050 và ADXL345 vẫn không khớp sau khi remap:

**Kiểm tra hướng lắp đặt:**

```
Hướng chip (chip face up):

ADXL345:                    MPU6050:
     +Y                          +Y
      │                           │
      │                           │
+X────┼────                  +X───┼────
      │                           │
     
QUAN TRỌNG:
- Đặt CẢ 2 chip cùng mặt (face up)
- ADXL X phải trùng MPU Y
- ADXL Y phải trùng MPU X
```

**Nếu lắp ngược:**
- Thêm dấu `-` vào code MPU6050:
  ```python
  ax = -ay_mpu  # Hoặc
  ay = -ax_mpu
  ```

---

## 🚀 QUY TRÌNH KHỞI ĐỘNG MỚI

```bash
# 1. Upload code mới
ampy --port /dev/ttyUSB0 put balance_robot_v2.py main.py
ampy --port /dev/ttyUSB0 put test_utils.py

# 2. Kiểm tra I2C
>>> import test_utils
>>> test_utils.test_i2c()

# 3. Kiểm tra MPU6050
>>> test_utils.test_mpu6050(duration=5)

# 4. Kiểm tra ADXL345
>>> test_utils.test_adxl345(duration=5)

# 5. So sánh 2 IMU (QUAN TRỌNG!)
>>> test_utils.test_imu_comparison(duration=10)
# → Kiểm tra xem 2 sensor có đọc giống nhau không

# 6. Nếu OK, chạy robot
>>> import balance_robot_v2
>>> balance_robot_v2.main()
```

---

## ❓ TROUBLESHOOTING

### Lỗi: "MPU6050 not found at 0x68 on I2C0"

**Nguyên nhân:** Dây kết nối I2C0 lỏng hoặc sai

**Giải pháp:**
```python
from machine import I2C, Pin
i2c0 = I2C(0, scl=Pin(21), sda=Pin(20))
print(i2c0.scan())  # Phải thấy [0x40, 0x68]
```

### Lỗi: "ADXL345 not found on I2C1"

**Nguyên nhân:** Dây kết nối I2C1 lỏng hoặc sai

**Giải pháp:**
```python
from machine import I2C, Pin
i2c1 = I2C(1, scl=Pin(19), sda=Pin(18))
print(i2c1.scan())  # Phải thấy [0x53]
```

### MPU6050 và ADXL345 đọc khác nhau > 10°

**Nguyên nhân:** Axis remapping chưa đúng hoặc chip lắp ngược

**Giải pháp:**
1. Kiểm tra hướng lắp chip (face up, cùng hướng)
2. Test riêng từng sensor:
   ```python
   test_utils.test_mpu6050(duration=10)
   test_utils.test_adxl345(duration=10)
   ```
3. Nghiêng robot từ từ, xem sensor nào đọc đúng
4. Điều chỉnh dấu trong code MPU6050.update()

---

## ✅ CHECKLIST HOÀN THÀNH

- [ ] Cập nhật code lên board
- [ ] Test I2C bus scan (cả 2 bus)
- [ ] Test MPU6050 riêng lẻ
- [ ] Test ADXL345 riêng lẻ
- [ ] **Test IMU comparison (quan trọng nhất!)**
- [ ] Kiểm tra servo hoạt động
- [ ] Chạy full system
- [ ] Kiểm tra PID response

---

**Chúc may mắn với hệ thống mới! 🎉**

Với axis remapping đúng, robot sẽ hoạt động ổn định và chính xác hơn rất nhiều.