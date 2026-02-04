# =============================================================================
# BALANCE ROBOT - CONFIGURATION PRESETS
# =============================================================================
# File này chứa các cấu hình mẫu (presets) cho các tình huống khác nhau
# Copy phần cấu hình phù hợp vào class Config trong balance_robot_v2.py
# =============================================================================

# -----------------------------------------------------------------------------
# PRESET 1: CONSERVATIVE (An toàn - Khởi động đầu tiên)
# -----------------------------------------------------------------------------
# Dùng khi:
# - Lần đầu chạy robot
# - Chưa rõ hành vi của hệ thống
# - Muốn tránh rủi ro hư hỏng phần cứng
# -----------------------------------------------------------------------------

PRESET_CONSERVATIVE = {
    # PID Parameters - Rất nhẹ nhàng
    'PID_PITCH_KP': 1.0,    # Phản ứng chậm
    'PID_PITCH_KI': 0.01,   # Ít tích phân
    'PID_PITCH_KD': 0.5,    # Giảm dao động vừa phải
    
    'PID_ROLL_KP': 1.0,
    'PID_ROLL_KI': 0.01,
    'PID_ROLL_KD': 0.5,
    
    # Output Limits - Giới hạn chặt
    'PID_OUTPUT_LIMIT': 30,      # Chỉ cho phép ±30° từ center
    'PID_INTEGRAL_LIMIT': 50,
    
    # Control Mode
    'USE_ADXL_FEEDFORWARD': False,  # Tắt feedforward
    'FEEDFORWARD_GAIN': 0.0,
    
    # Loop Timing
    'LOOP_FREQUENCY': 50,      # 50Hz - Nhẹ nhàng
    
    # Safety
    'SERVO_MIN_ANGLE': 30,     # Giới hạn hẹp hơn
    'SERVO_MAX_ANGLE': 150,
}

# -----------------------------------------------------------------------------
# PRESET 2: BALANCED (Cân bằng - Khuyên dùng cho hầu hết trường hợp)
# -----------------------------------------------------------------------------
# Dùng khi:
# - Robot đã hoạt động ổn định với preset CONSERVATIVE
# - Muốn hiệu suất tốt hơn
# - Hệ thống cơ khí đã được kiểm tra kỹ
# -----------------------------------------------------------------------------

PRESET_BALANCED = {
    # PID Parameters - Cân bằng
    'PID_PITCH_KP': 2.5,
    'PID_PITCH_KI': 0.03,
    'PID_PITCH_KD': 1.2,
    
    'PID_ROLL_KP': 2.5,
    'PID_ROLL_KI': 0.03,
    'PID_ROLL_KD': 1.2,
    
    # Output Limits
    'PID_OUTPUT_LIMIT': 45,
    'PID_INTEGRAL_LIMIT': 100,
    
    # Control Mode
    'USE_ADXL_FEEDFORWARD': True,
    'FEEDFORWARD_GAIN': 0.5,    # Bù 50% góc địa hình
    
    # Loop Timing
    'LOOP_FREQUENCY': 100,      # 100Hz - Phản ứng nhanh
    
    # Safety
    'SERVO_MIN_ANGLE': 10,
    'SERVO_MAX_ANGLE': 170,
}

# -----------------------------------------------------------------------------
# PRESET 3: AGGRESSIVE (Tích cực - Cho robot nhanh nhạy)
# -----------------------------------------------------------------------------
# Dùng khi:
# - Cần phản ứng cực nhanh (racing, acrobatic)
# - Hệ thống đã được test kỹ
# - Có kinh nghiệm điều chỉnh PID
# CẢNH BÁO: Có thể gây dao động nếu cơ khí không chắc chắn
# -----------------------------------------------------------------------------

PRESET_AGGRESSIVE = {
    # PID Parameters - Mạnh mẽ
    'PID_PITCH_KP': 4.0,
    'PID_PITCH_KI': 0.05,
    'PID_PITCH_KD': 2.0,
    
    'PID_ROLL_KP': 4.0,
    'PID_ROLL_KI': 0.05,
    'PID_ROLL_KD': 2.0,
    
    # Output Limits
    'PID_OUTPUT_LIMIT': 60,      # Cho phép góc lớn hơn
    'PID_INTEGRAL_LIMIT': 150,
    
    # Control Mode
    'USE_ADXL_FEEDFORWARD': True,
    'FEEDFORWARD_GAIN': 0.7,     # Bù 70% góc địa hình
    
    # Loop Timing
    'LOOP_FREQUENCY': 100,
    
    # Safety
    'SERVO_MIN_ANGLE': 10,
    'SERVO_MAX_ANGLE': 170,
}

# -----------------------------------------------------------------------------
# PRESET 4: SMOOTH (Mượt mà - Cho camera gimbal, chống rung)
# -----------------------------------------------------------------------------
# Dùng khi:
# - Robot mang camera/payload nhạy cảm
# - Cần chuyển động mượt mà, ít giật
# - Không cần phản ứng nhanh
# -----------------------------------------------------------------------------

PRESET_SMOOTH = {
    # PID Parameters - Mượt mà
    'PID_PITCH_KP': 1.5,
    'PID_PITCH_KI': 0.02,
    'PID_PITCH_KD': 1.5,    # Kd cao để giảm dao động
    
    'PID_ROLL_KP': 1.5,
    'PID_ROLL_KI': 0.02,
    'PID_ROLL_KD': 1.5,
    
    # Output Limits
    'PID_OUTPUT_LIMIT': 40,
    'PID_INTEGRAL_LIMIT': 80,
    
    # Control Mode
    'USE_ADXL_FEEDFORWARD': True,
    'FEEDFORWARD_GAIN': 0.8,    # Bù nhiều để giảm shock
    
    # Loop Timing
    'LOOP_FREQUENCY': 100,
    
    # Safety
    'SERVO_MIN_ANGLE': 20,
    'SERVO_MAX_ANGLE': 160,
    
    # MPU Filter - Nhiều gyro hơn (mượt hơn)
    'MPU_ALPHA': 0.98,
}

# -----------------------------------------------------------------------------
# PRESET 5: OFFROAD (Địa hình xấu - Cho robot bánh xích)
# -----------------------------------------------------------------------------
# Dùng khi:
# - Robot chạy trên địa hình gồ ghề
# - Cần bù trước nhiều (feedforward cao)
# - Chấp nhận dao động nhỏ để theo kịp địa hình
# -----------------------------------------------------------------------------

PRESET_OFFROAD = {
    # PID Parameters
    'PID_PITCH_KP': 3.0,
    'PID_PITCH_KI': 0.04,
    'PID_PITCH_KD': 1.0,
    
    'PID_ROLL_KP': 3.0,
    'PID_ROLL_KI': 0.04,
    'PID_ROLL_KD': 1.0,
    
    # Output Limits - Cho phép góc lớn
    'PID_OUTPUT_LIMIT': 50,
    'PID_INTEGRAL_LIMIT': 120,
    
    # Control Mode - Feedforward mạnh
    'USE_ADXL_FEEDFORWARD': True,
    'FEEDFORWARD_GAIN': 0.9,    # Bù 90% góc địa hình
    
    # Loop Timing
    'LOOP_FREQUENCY': 100,
    
    # Safety
    'SERVO_MIN_ANGLE': 10,
    'SERVO_MAX_ANGLE': 170,
    
    # MPU Filter - Cân bằng
    'MPU_ALPHA': 0.92,
}

# -----------------------------------------------------------------------------
# PRESET 6: DEBUG (Chế độ debug - Không dùng servo, chỉ in log)
# -----------------------------------------------------------------------------
# Dùng khi:
# - Muốn kiểm tra sensor mà không chạy servo
# - Debug thuật toán
# - Phân tích dữ liệu
# -----------------------------------------------------------------------------

PRESET_DEBUG = {
    # PID Parameters - Không quan trọng
    'PID_PITCH_KP': 0.0,
    'PID_PITCH_KI': 0.0,
    'PID_PITCH_KD': 0.0,
    
    'PID_ROLL_KP': 0.0,
    'PID_ROLL_KI': 0.0,
    'PID_ROLL_KD': 0.0,
    
    # Output Limits
    'PID_OUTPUT_LIMIT': 0,
    'PID_INTEGRAL_LIMIT': 0,
    
    # Control Mode
    'USE_ADXL_FEEDFORWARD': True,
    'FEEDFORWARD_GAIN': 0.5,
    
    # Loop Timing - Chậm để dễ đọc
    'LOOP_FREQUENCY': 10,
    
    # Debug
    'DEBUG_PRINT': True,
    'DEBUG_INTERVAL_MS': 100,    # In mỗi 100ms
}

# =============================================================================
# HARDWARE SPECIFIC PRESETS
# =============================================================================

# -----------------------------------------------------------------------------
# MG90S Servo (Phổ biến, giá rẻ)
# -----------------------------------------------------------------------------

SERVO_MG90S = {
    'SERVO_MIN_ANGLE': 0,
    'SERVO_MAX_ANGLE': 180,
    'SERVO_CENTER': 90,
    'SERVO_PWM_FREQ': 50,
}

# -----------------------------------------------------------------------------
# SG90 Servo (Micro servo, nhẹ)
# -----------------------------------------------------------------------------

SERVO_SG90 = {
    'SERVO_MIN_ANGLE': 0,
    'SERVO_MAX_ANGLE': 180,
    'SERVO_CENTER': 90,
    'SERVO_PWM_FREQ': 50,
}

# -----------------------------------------------------------------------------
# MG996R Servo (High torque, kim loại)
# -----------------------------------------------------------------------------

SERVO_MG996R = {
    'SERVO_MIN_ANGLE': 0,
    'SERVO_MAX_ANGLE': 180,
    'SERVO_CENTER': 90,
    'SERVO_PWM_FREQ': 50,
}

# =============================================================================
# MECHANICAL CONFIGURATIONS
# =============================================================================

# -----------------------------------------------------------------------------
# Small Robot (Robot nhỏ - Servo spacing < 80mm)
# -----------------------------------------------------------------------------

MECH_SMALL = {
    'ARM_PITCH': 20.0,      # mm
    'ARM_ROLL': 30.0,       # mm
    'MOUNT_ANGLE': 60,      # độ
    'SERVO_SPACING': 60.0,  # mm
}

# -----------------------------------------------------------------------------
# Medium Robot (Robot trung bình - Default)
# -----------------------------------------------------------------------------

MECH_MEDIUM = {
    'ARM_PITCH': 33.34,     # mm
    'ARM_ROLL': 57.75,      # mm
    'MOUNT_ANGLE': 60,      # độ
    'SERVO_SPACING': 115.5, # mm
}

# -----------------------------------------------------------------------------
# Large Robot (Robot lớn - Servo spacing > 150mm)
# -----------------------------------------------------------------------------

MECH_LARGE = {
    'ARM_PITCH': 50.0,      # mm
    'ARM_ROLL': 87.5,       # mm
    'MOUNT_ANGLE': 60,      # độ
    'SERVO_SPACING': 175.0, # mm
}

# =============================================================================
# HOW TO USE THESE PRESETS
# =============================================================================

"""
CÁCH SỬ DỤNG:

1. Mở file balance_robot_v2.py

2. Tìm class Config

3. Copy preset phù hợp vào Config. Ví dụ:

   class Config:
       # ... (giữ nguyên phần I2C và Addresses)
       
       # PRESET: BALANCED
       PID_PITCH_KP = 2.5
       PID_PITCH_KI = 0.03
       PID_PITCH_KD = 1.2
       # ... (copy các giá trị khác)

4. Hoặc dùng update dictionary (advanced):

   import config_presets
   
   # Merge preset vào Config
   for key, value in config_presets.PRESET_BALANCED.items():
       setattr(Config, key, value)

5. Test và tinh chỉnh theo nhu cầu cụ thể

LƯU Ý:
- Luôn bắt đầu với PRESET_CONSERVATIVE
- Tăng dần lên PRESET_BALANCED
- Chỉ dùng PRESET_AGGRESSIVE khi cần thiết
- Mỗi robot có đặc tính riêng, preset chỉ là điểm khởi đầu
"""

# =============================================================================
# TUNING CHECKLIST
# =============================================================================

"""
CHECKLIST TINH CHỈNH:

□ Bước 1: Hardware
  □ Kiểm tra kết nối I2C
  □ Test từng servo riêng lẻ
  □ Kiểm tra MPU6050 đọc đúng góc
  □ Kiểm tra ADXL345 (nếu có)
  □ Xác nhận chiều quay servo

□ Bước 2: Mechanical
  □ Đo chính xác ARM_PITCH
  □ Đo chính xác ARM_ROLL (= SERVO_SPACING / 2)
  □ Đo chính xác MOUNT_ANGLE
  □ Kiểm tra ốc vít chắc chắn
  □ Kiểm tra không có lỏng lẻo

□ Bước 3: Software
  □ Chọn preset phù hợp
  □ Xác nhận SERVO_INVERT flags
  □ Chạy test_utils.quick_diagnostic()
  □ Test mixing algorithm

□ Bước 4: PID Tuning
  □ Bắt đầu với PRESET_CONSERVATIVE
  □ Tăng Kp từ từ cho đến khi dao động
  □ Giảm Kp xuống 80% giá trị dao động
  □ Tăng Kd để giảm overshoot
  □ Tăng Ki nếu không về đúng setpoint
  □ Test với địa hình thực tế

□ Bước 5: Final
  □ Test độ bền (chạy 30 phút liên tục)
  □ Test nhiệt độ servo
  □ Test với nhiều góc nghiêng khác nhau
  □ Tối ưu FEEDFORWARD_GAIN (nếu dùng ADXL)
  □ Backup cấu hình hoạt động tốt nhất
"""