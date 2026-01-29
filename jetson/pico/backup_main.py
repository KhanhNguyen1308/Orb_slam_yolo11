# main.py trên RP2040
import machine
import _thread
import time
import sys
import uselect

# --- CẤU HÌNH PHẦN CỨNG ---
# Chân Driver A4988 (Ngài kiểm tra lại chân thực tế)
DIR_R = machine.Pin(14, machine.Pin.OUT)
STEP_R = machine.Pin(15, machine.Pin.OUT)
DIR_L = machine.Pin(16, machine.Pin.OUT)
STEP_L = machine.Pin(17, machine.Pin.OUT)

# --- BIẾN CHIA SẺ (SHARED MEMORY) ---
# Lock để tránh xung đột dữ liệu giữa 2 lõi
data_lock = _thread.allocate_lock()

# Tốc độ mục tiêu (Steps/second)
target_speed_L = 0.0
target_speed_R = 0.0

# Bộ đếm bước (Odometry) - Gửi lên Jetson để tính toán vị trí
counter_L = 0
counter_R = 0

def motor_core_task():
    """
    NGHI THỨC LÕI 1: TẠO XUNG ĐỘNG CƠ
    Chạy vòng lặp vô tận, không bị ngắt quãng bởi USB.
    """
    global counter_L, counter_R
    
    # Biến cục bộ để tăng tốc độ truy cập
    next_step_time_L = time.ticks_us()
    next_step_time_R = time.ticks_us()
    step_state_L = 0
    step_state_R = 0
    
    while True:
        # 1. Đọc dữ liệu an toàn từ biến toàn cục
        with data_lock:
            spd_L = target_speed_L
            spd_R = target_speed_R
        
        current_time = time.ticks_us()

        # --- ĐỘNG CƠ TRÁI ---
        if abs(spd_L) > 5.0: # Deadzone nhỏ
            # Xác định chiều (1=Tiến, 0=Lùi - Tuỳ cách đấu dây)
            DIR_L.value(1 if spd_L > 0 else 0)
            
            # Tính delay: 1 giây / số step. Toggle cần 1/2 chu kỳ.
            # delay_us = 500000 / abs(spd_L)
            if current_time >= next_step_time_L:
                step_state_L = 1 - step_state_L
                STEP_L.value(step_state_L)
                # Chỉ đếm khi sườn dương (0->1)
                if step_state_L == 1:
                    with data_lock:
                        counter_L += (1 if spd_L > 0 else -1)
                
                # Cập nhật thời gian cho xung tiếp theo
                delay = int(500000 / abs(spd_L))
                next_step_time_L = time.ticks_add(current_time, delay)

        # --- ĐỘNG CƠ PHẢI ---
        if abs(spd_R) > 5.0:
            DIR_R.value(0 if spd_R > 0 else 1) # Đảo chiều nếu motor đối xứng
            
            if current_time >= next_step_time_R:
                step_state_R = 1 - step_state_R
                STEP_R.value(step_state_R)
                if step_state_R == 1:
                    with data_lock:
                        counter_R += (1 if spd_R > 0 else -1)
                
                delay = int(500000 / abs(spd_R))
                next_step_time_R = time.ticks_add(current_time, delay)
                
        # Nhường 1 chút CPU (rất nhỏ) để hệ thống ổn định
        # time.sleep_us(1)

def main():
    """
    NGHI THỨC LÕI 0: GIAO TIẾP VỚI JETSON
    """
    global target_speed_L, target_speed_R
    
    # Khởi động Lõi 1
    _thread.start_new_thread(motor_core_task, ())
    
    # Buffer Serial
    poll_obj = uselect.poll()
    poll_obj.register(sys.stdin, uselect.POLLIN)
    input_buf = ""
    
    last_telemetry_time = time.ticks_ms()
    
    while True:
        # 1. Gửi Telemetry lên Jetson (20Hz - 50ms/lần)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_telemetry_time) > 50:
            with data_lock:
                cL = counter_L
                cR = counter_R
            # Format: "O:left_steps,right_steps\n"
            print(f"O:{cL},{cR}")
            last_telemetry_time = now

        # 2. Đọc Lệnh từ Jetson (Non-blocking)
        if poll_obj.poll(0):
            char = sys.stdin.read(1)
            if char == '\n':
                try:
                    # Parse lệnh: "S:1000,-1000" (S:SpeedL,SpeedR)
                    if input_buf.startswith("S:"):
                        vals = input_buf[2:].split(',')
                        vL = float(vals[0])
                        vR = float(vals[1])
                        with data_lock:
                            target_speed_L = vL
                            target_speed_R = vR
                except:
                    pass
                input_buf = ""
            else:
                input_buf += char
        
        time.sleep_ms(1)

if __name__ == "__main__":
    main()
