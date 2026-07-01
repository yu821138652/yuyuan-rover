import serial
import time
import json
from gpiozero import DistanceSensor, DigitalInputDevice

# ==========================================
# 1. Configuration (参数完全对齐你的最新测试值)
# ==========================================
SERIAL_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 115200

# 避障速度
SPEED_STRAIGHT = 0.25      
SPEED_NAV_FAST = 0.3      
SPEED_NAV_SLOW = 0.1      
SPEED_EMERGENCY_FAST = 0.4 
SPEED_EMERGENCY_SLOW = 0.08
SPEED_PIVOT = 0.35         

# --- 避障阈值 ---
THRESH_FRONT = 10.0        # 前方探测距离
THRESH_MIN = 4.0           # 侧边紧急避让 (由于传感器前移，这个值很关键)
THRESH_DIFF = 2.0          # 空间探测灵敏度
STABILIZE_TIME = 0.3       

# 初始化传感器
ultra_f = DistanceSensor(echo=24, trigger=23, queue_len=2)
ultra_l = DistanceSensor(echo=27, trigger=17, queue_len=2)
ultra_r = DistanceSensor(echo=25, trigger=22, queue_len=2)

sensor_l = DigitalInputDevice(5)
sensor_r = DigitalInputDevice(6)

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
        data = {"T": 1, "L": round(left, 2), "R": round(right, 2)}
        ser.write((json.dumps(data) + '\n').encode('utf-8'))

def perform_pivot(direction):
    """强制 90 度自旋动作"""
    # 缓冲动作
    set_motors(-0.2, -0.2)
    time.sleep(0.3)
    set_motors(0, 0)
    time.sleep(0.1)

    print(f"Executing PIVOT to {direction}")
    if direction == "LEFT":
        set_motors(-SPEED_PIVOT, SPEED_PIVOT)
    else:
        set_motors(SPEED_PIVOT, -SPEED_PIVOT)
    
    time.sleep(0.6) # 旋转时间
    set_motors(0, 0)
    time.sleep(STABILIZE_TIME)

# ==========================================
# 2. Logic Loop (适配车头两侧传感器)
# ==========================================

def start_avoidance():
    first_pivot_completed = False
    
    # 存储上一次有效值
    last_l = 15.0
    last_r = 15.0
    
    print("Obstacle Avoidance Active (Front-Side Layout). Monitoring...")
    
    try:
        while True:
            # 实时获取距离
            raw_f = ultra_f.distance * 100
            raw_l = ultra_l.distance * 100
            raw_r = ultra_r.distance * 100

            # 20cm 无效值过滤
            if raw_l <= 20.0: last_l = raw_l
            if raw_r <= 20.0: last_r = raw_r

            dist_f = raw_f
            dist_l = round(last_l, 1)
            dist_r = round(last_r, 1)

            # --- 优先级 1: 前方探测 ---
            if dist_f < THRESH_FRONT:
                print(f"!!! FRONT WALL: {dist_f:.1f}cm !!!")
                
                # 决定方向
                if dist_l > dist_r:
                    perform_pivot("LEFT")
                else:
                    perform_pivot("RIGHT")
                
                # 第一次自旋后的强制直行 1.5s
                if not first_pivot_completed:
                    print(f"First turn done. Boost straight 1.5s. (L:{dist_l}, R:{dist_r})")
                    set_motors(SPEED_STRAIGHT, SPEED_STRAIGHT)
                    time.sleep(1.5)
                    first_pivot_completed = True
                
                continue

            # --- 优先级 2: 寻路逻辑 (仅在第一转后激活) ---
            if first_pivot_completed:
                # 紧急避让 (<= 4cm)
                # 因为传感器现在在最前面，一旦小于4cm说明车头即将撞侧墙，必须大力回正
                if dist_r <= THRESH_MIN:
                    print(f"EMERGENCY_left (R:{dist_r}cm)")
                    set_motors(SPEED_EMERGENCY_SLOW, SPEED_EMERGENCY_FAST)
                elif dist_l <= THRESH_MIN:
                    print(f"EMERGENCY_right (L:{dist_l}cm)")
                    set_motors(SPEED_EMERGENCY_FAST, SPEED_EMERGENCY_SLOW)

                # 空间寻路 (哪边宽往哪边靠)
                elif dist_l > dist_r + THRESH_DIFF:
                    print(f"nav_left (L:{dist_l}, R:{dist_r})")
                    set_motors(SPEED_NAV_SLOW, SPEED_NAV_FAST)
                elif dist_r > dist_l + THRESH_DIFF:
                    print(f"nav_right (L:{dist_l}, R:{dist_r})")
                    set_motors(SPEED_NAV_FAST, SPEED_NAV_SLOW)
                else:
                    set_motors(SPEED_STRAIGHT, SPEED_STRAIGHT)
            
            else:
                # 第一堵墙出现前的探测直行
                set_motors(SPEED_STRAIGHT, SPEED_STRAIGHT)

            # 退出检测 (寻迹传感器亮起)

            time.sleep(0.005)

    except KeyboardInterrupt:
        set_motors(0, 0)

if __name__ == "__main__":
    if init_serial():
        time.sleep(1)
        start_avoidance()