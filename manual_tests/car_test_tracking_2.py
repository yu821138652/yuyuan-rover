import serial
import time
import json
from gpiozero import DigitalInputDevice

# ==========================================
# 1. Configuration
# ==========================================
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

BASE_SPEED = 0.35
INNER = 0.05
OUTER = 0.5
PIVOT_SPEED = 0.35 

# 传感器引脚 (L=5, R=6)
sensor_ll = DigitalInputDevice(12) 
sensor_l  = DigitalInputDevice(5)  
sensor_r  = DigitalInputDevice(6)  
sensor_rr = DigitalInputDevice(13) 

ser = None

def init_serial():
    global ser
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
        ser.setRTS(False)
        ser.setDTR(False)
        return True
    except: return False

def set_motors(left, right):
    if ser and ser.is_open:
        data = {"T": 1, "L": left, "R": right}
        ser.write((json.dumps(data) + '\n').encode('utf-8'))

# ==========================================
# 2. 动作函数: 原地旋转
# ==========================================

def execute_pivot(direction):
    print(f"Action: Pivot {direction}")
    

    if direction == "LEFT":
        set_motors(-PIVOT_SPEED, PIVOT_SPEED)
    else:
        set_motors(PIVOT_SPEED, -PIVOT_SPEED)
    
    # 第一阶段：强制转离当前横线
    time.sleep(0.35)
    
    # 第二阶段：直到中间找回新线
    while True:
        if sensor_l.value == 1 or sensor_r.value == 1:
            break
        time.sleep(0.005)

# ==========================================
# 3. 核心寻迹逻辑 (十字路口 vs T型路口)
# ==========================================

def start_tracking():
    print("Logic running: Cross-junction Immune Mode")
    try:
        while True:
            LL, L, R, RR = sensor_ll.value, sensor_l.value, sensor_r.value, sensor_rr.value

            # --- 优先级 1：路口特征识别 (LL 或 RR 触发时) ---
            if LL == 1 or RR == 1:
                # 关键：短时间确认，消除左右传感器触发的时间差
                time.sleep(0.015) 
                # 重新读取外侧状态
                LL_now, RR_now = sensor_ll.value, sensor_rr.value
                
                # A. 如果两边都亮了 -> 是十字路口
                if LL_now == 1 and RR_now == 1:
                    print("--- Cross Junction: Ignore and Pass ---")
                    set_motors(BASE_SPEED, BASE_SPEED)
                    time.sleep(0.15) # 强制直行一小段，冲过十字路口
                    continue
                
                # B. 如果只有左边亮 -> 是左转 T 型路口
                elif LL_now == 1:
                    execute_pivot("LEFT")
                    continue

                # C. 如果只有右边亮 -> 是右转 T 型路口
                elif RR_now == 1:
                    execute_pivot("RIGHT")
                    continue

            # --- 优先级 2：常规寻迹 (中间感应器控制) ---
            if L == 1 or R == 1:
                if L == 1 and R == 1:
                    set_motors(BASE_SPEED, BASE_SPEED)
                elif L == 1:
                    set_motors(INNER, OUTER) # 左微调
                else:
                    set_motors(OUTER, INNER) # 右微调
            
            # --- 默认：直行 ---
            else:
                set_motors(BASE_SPEED, BASE_SPEED)

            time.sleep(0.005)

    except KeyboardInterrupt:
        set_motors(0, 0)

if __name__ == "__main__":
    if init_serial():
        time.sleep(1)
        start_tracking()