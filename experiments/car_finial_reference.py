import serial
import time
import json
import os
import cv2
import apriltag
import lgpio
from collections import deque
from gpiozero import DigitalInputDevice, DistanceSensor
from picamera2 import Picamera2

# ==========================================
# 1. 硬件与引脚配置 (保留你调试好的所有参数)
# ==========================================
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
STATE_FILE = "mission_state.json"

# 电机速度参数
BASE_SPEED = 0.3      
VALLEY_SPEED = 0.3    # 河谷提速
INNER = 0.05
OUTER = 0.5
PIVOT_SPEED = 0.35 
AVOID_STRAIGHT = 0.15 

# 避障阈值参数
THRESH_FRONT = 11.0
THRESH_EMG = 12.0
THRESH_DIFF = 3.0     # 寻路差值阈值
MAX_VALID = 40.0

# 十字路口检测去抖参数
CROSS_DETECT_THRESHOLD = 3  # 连续检测次数

# 传感器与执行器
sensor_ll = DigitalInputDevice(12)
sensor_l  = DigitalInputDevice(5)
sensor_r  = DigitalInputDevice(6)
sensor_rr = DigitalInputDevice(13)
SERVO_PIN = 8

# ==========================================
# 2. 状态持久化函数 (完整版)
# ==========================================
def save_state(startpoint, target_id, current_flag=None, current_cross=None):
    """Save mission state to file"""
    state_dict = {
        "startpoint": startpoint,
        "target_id": target_id,
        "flag": current_flag,
        "num_cross": current_cross,
        "timestamp": time.time()
    }
    try:
        with open(STATE_FILE, 'w') as f:
            json.dump(state_dict, f, indent=2)
    except Exception as e:
        print(f"[ERROR] State save failed: {e}")

def load_state():
    """Load mission state from file"""
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, 'r') as f:
                state = json.load(f)
            return state
        except Exception as e:
            print(f"[ERROR] State load failed: {e}")
    
    default_state = {
        "startpoint": "S1",
        "target_id": None,
        "flag": 1,
        "num_cross": 0,
        "timestamp": time.time()
    }
    return default_state

def reset_to_s1():
    """Reset to S1 (for debugging)"""
    if os.path.exists(STATE_FILE):
        try:
            os.remove(STATE_FILE)
        except Exception as e:
            print(f"[ERROR] Reset failed: {e}")

# ==========================================
# 3. 硬件初始化与基础驱动模块
# ==========================================
ser = None
gpio_handle = None

def init_hardware():
    """Initialize all hardware"""
    global ser, gpio_handle
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.setRTS(False)
        ser.setDTR(False)
    except Exception as e:
        print(f"[ERROR] Serial init failed: {e}")
        return False
    
    try:
        gpio_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(gpio_handle, SERVO_PIN)
    except Exception as e:
        print(f"[ERROR] GPIO init failed: {e}")
        return False
    
    return True

def cleanup_hardware():
    """Cleanup hardware resources"""
    global ser, gpio_handle
    try:
        if ser and ser.is_open:
            set_motors(0, 0)
            time.sleep(0.1)
            ser.close()
    except:
        pass
    
    try:
        if gpio_handle:
            lgpio.gpiochip_close(gpio_handle)
    except:
        pass

def set_motors(left, right):
    """设置左右电机速度"""
    if ser and ser.is_open:
        l = max(-0.5, min(0.5, left))
        r = max(-0.5, min(0.5, right))
        data = {"T": 1, "L": round(l, 2), "R": round(r, 2)}
        try:
            ser.write((json.dumps(data) + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[ERROR] Motor control failed: {e}")

def angle_to_duty(angle):
    """将角度转换为PWM占空比"""
    angle = max(-90, min(90, angle))
    pulse_us = 1500 + (angle / 90) * 1000
    return (pulse_us / 20000) * 100

def servo_move(target_angle):
    """Move servo to target angle"""
    if not gpio_handle:
        print("[ERROR] GPIO not initialized")
        return
    try:
        duty = angle_to_duty(target_angle)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, 50, duty)
        time.sleep(0.5)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, 50, 0)
    except Exception as e:
        print(f"[ERROR] Servo move failed: {e}")

class RobustUltra:
    """Ultrasonic sensor (with filtering)"""
    def __init__(self, t, e, name=""):
        self.sensor = DistanceSensor(echo=e, trigger=t, queue_len=1)
        self.history = deque(maxlen=3)
        self.name = name
    
    def get(self):
        try:
            val = self.sensor.distance * 100
            if val < 2.1:
                return 999.0
            self.history.append(val)
        except:
            return 999.0
        
        if self.history:
            avg = sum(self.history) / len(self.history)
            return avg
        return 999.0

# ==========================================
# 4. 任务核心模块
# ==========================================
def perform_scan():
    """Scan target code at S1"""
    try:
        picam2 = Picamera2()
        picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
        picam2.start()
        detector = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9"))
        
        target = None
        timeout_start = time.time()
        
        while target is None:
            if time.time() - timeout_start > 30:
                target = 0
                break
            
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            tags = detector.detect(gray)
            
            for tag in tags:
                if tag.tag_id in [0, 1, 2]:
                    target = tag.tag_id
                    tag_name = chr(ord('A') + target)
                    print(f"[TAG] Target: {tag_name}")
                    break
        
        picam2.stop()
        cv2.destroyAllWindows()
        return target
    
    except Exception as e:
        print(f"[ERROR] Scan failed: {e}")
        return 0

def read_sensors():
    """实时读取所有传感器"""
    return (
        sensor_ll.value,
        sensor_l.value,
        sensor_r.value,
        sensor_rr.value
    )

def detect_cross_junction(current_state):
    """检测十字路口（带去抖）"""
    LL, L, R, RR = current_state
    # 十字路口特征：LL=0, L=0, R=0, RR=0
    return (LL == 0 and L == 0 and R == 0 and RR == 0)

def tracking_logic(speed_val):
    """通用寻迹动作"""
    LL, L, R, RR = read_sensors()
    
    # 直线行驶
    if LL == RR and L == 1 and R == 1:
        set_motors(speed_val, speed_val)
    # 左偏
    elif (LL == 1 and L == 1 and R == 0 and RR == 1) or (LL == 1 and R == 1 and L == 1 and RR == 0):
        set_motors(OUTER, INNER)
    # 右偏
    elif (LL == 1 and L == 0 and R == 1 and RR == 1) or (LL == 0 and R == 1 and L == 1 and RR == 1):
        set_motors(INNER, OUTER)
    # 其他情况：继续直行
    else:
        set_motors(speed_val, speed_val)

def handle_t_junction(flag):
    """Handle T-junction turn (only in Flag 2)"""
    if flag != 2:
        return False
    
    LL, L, R, RR = read_sensors()
    
    if LL == 0 and L == 0 and R == 0 and RR == 1:
        time.sleep(0.03) # 这里已经按照你的要求从0.06改为了0.03
        LL_now = sensor_ll.value
        RR_now = sensor_rr.value
        
        if LL_now == 0 and RR_now == 1:
            print("[TURN] T-junction left")
            set_motors(0, 0)
            time.sleep(0.2)
            set_motors(-PIVOT_SPEED, PIVOT_SPEED)
            time.sleep(0.6)
            set_motors(0, 0)
            time.sleep(0.1)
            return True
    
    return False

def initialize_from_startpoint(startpoint, target_id):
    """Initialize flag and num_cross from startpoint"""
    if startpoint == "S1":
        flag = 1
        num_cross = 0
        need_scan = (target_id is None)
    elif startpoint == "S2":
        flag = 2
        num_cross = 6
        need_scan = False
    elif startpoint == "S3":
        flag = 3 # 修正：S3起点应对应Flag 4(河谷)，但在你的逻辑里S3是避障结束后的路口
        num_cross = 8
        need_scan = False
    else:
        print(f"[ERROR] Unknown startpoint: {startpoint}")
        flag = 1
        num_cross = 0
        need_scan = True
    
    return flag, num_cross, need_scan

# ==========================================
# 5. 主程序状态机
# ==========================================
def main():
    """Main mission loop"""
    
    if not init_hardware():
        print("[ERROR] Hardware init failed")
        return
    
    state = load_state()
    startpoint = state.get("startpoint", "S1")
    target_id = state.get("target_id")
    
    flag, num_cross, need_scan = initialize_from_startpoint(startpoint, target_id)
    
    if need_scan:
        target_id = perform_scan()
        save_state(startpoint, target_id, flag, num_cross)
    
    set_motors(0, 0)
    time.sleep(0.5)
    
    u_f = RobustUltra(23, 24, "Front")
    u_l = RobustUltra(17, 27, "Left")
    u_r = RobustUltra(22, 25, "Right")
    
    cross_detected_counter = 0
    junction_locked = False # 新增锁定变量：防止在大黑框内重复计数
    first_avoid_pivot_done = False
    last_valid_l = 15.0
    last_valid_r = 15.0
    
    try:
        while True:
            LL, L, R, RR = read_sensors()
            
            # ============================================================
            # Cross junction detection (修复计数逻辑：增加边缘触发锁定)
            # ============================================================
            if detect_cross_junction((LL, L, R, RR)):
                if not junction_locked: # 只有未锁定时才尝试计数
                    cross_detected_counter += 1
                    if cross_detected_counter >= CROSS_DETECT_THRESHOLD:
                        num_cross += 1
                        junction_locked = True # 计完数立即锁定
                        cross_detected_counter = 0
                        print(f"[CROSS] #{num_cross} Flag:{flag} (Locked)")
                        
                        if num_cross == 6:
                            save_state("S2", target_id, flag, num_cross)
                        elif num_cross == 8:
                            save_state("S3", target_id, flag, num_cross)
            else:
                # 只要传感器看到白地(不再全黑)，就解锁下一次计数
                if junction_locked:
                    print("[CROSS] Sensor cleared white ground (Unlocked)")
                junction_locked = False
                cross_detected_counter = 0
            
            # ============================================================
            # FLAG 1: Initial tracking
            # ============================================================
            if flag == 1:
                tracking_logic(BASE_SPEED)
                
                if num_cross >= 6:
                    print(f"[FLAG] 1->2")
                    flag = 2
            
            # ============================================================
            # FLAG 2: Mid-section tracking
            # ============================================================
            elif flag == 2:
                if not handle_t_junction(flag):
                    tracking_logic(BASE_SPEED)
                
                if u_f.get() < THRESH_FRONT:
                    print(f"[FLAG] 2->3 obs:{u_f.get():.1f}")
                    flag = 3
                    first_avoid_pivot_done = False
            
            # ============================================================
            # FLAG 3: Avoidance
            # ============================================================
            elif flag == 3:
                df = u_f.get()
                rl = u_l.get()
                rr = u_r.get()
                
                if 2.1 <= rl <= MAX_VALID:
                    last_valid_l = rl
                if 2.1 <= rr <= MAX_VALID:
                    last_valid_r = rr
                
                dl = round(last_valid_l, 1)
                dr = round(last_valid_r, 1)
                
                if first_avoid_pivot_done and (L == 0 or R == 0):
                    print("[FLAG] 3->4")
                    set_motors(0, 0)
                    time.sleep(0.2)
                    set_motors(-PIVOT_SPEED, PIVOT_SPEED)
                    time.sleep(0.6)
                    set_motors(0, 0)
                    time.sleep(0.1)
                    flag = 4
                    continue
                
                if df < THRESH_FRONT:
                    if not first_avoid_pivot_done:
                        print("[AVOID] pivot")
                        set_motors(-0.2, -0.2)
                        time.sleep(0.1)
                        set_motors(0, 0)
                        time.sleep(0.05)
                        
                        set_motors(-PIVOT_SPEED, PIVOT_SPEED)
                        time.sleep(0.65)
                        set_motors(0, 0)
                        time.sleep(0.1)
                        
                        set_motors(AVOID_STRAIGHT, AVOID_STRAIGHT)
                        time.sleep(0.8)
                        
                        first_avoid_pivot_done = True
                    else:
                        if dl > dr:
                            set_motors(-PIVOT_SPEED, PIVOT_SPEED)
                        else:
                            set_motors(PIVOT_SPEED, -PIVOT_SPEED)
                        time.sleep(0.5)
                else:
                    if first_avoid_pivot_done:
                        if dr <= THRESH_EMG:
                            set_motors(-0.1, 0.4)
                        elif dl <= THRESH_EMG:
                            set_motors(0.4, -0.1)
                        elif dl > dr + THRESH_DIFF:
                            set_motors(AVOID_STRAIGHT, AVOID_STRAIGHT * 1.2)
                        elif dr > dl + THRESH_DIFF:
                            set_motors(AVOID_STRAIGHT * 1.2, AVOID_STRAIGHT)
                        else:
                            set_motors(AVOID_STRAIGHT, AVOID_STRAIGHT)
                    else:
                        set_motors(AVOID_STRAIGHT, AVOID_STRAIGHT)
            
            # ============================================================
            # FLAG 4: Valley section
            # ============================================================
            elif flag == 4:
                tracking_logic(VALLEY_SPEED)
                
                if num_cross >= 10:
                    print(f"[FLAG] 4->5")
                    set_motors(0, 0)
                    time.sleep(0.2)
                    flag = 5
            
            # ============================================================
            # FLAG 5: Delivery
            # ============================================================
            elif flag == 5:
                set_motors(0, 0)
                time.sleep(0.3)
                
                target_name = chr(ord('A') + target_id)
                print(f"[DELIVERY] Target:{target_name}")
                
                if target_id == 0:
                    set_motors(-PIVOT_SPEED, PIVOT_SPEED)
                    time.sleep(0.6)
                elif target_id == 2:
                    set_motors(PIVOT_SPEED, -PIVOT_SPEED)
                    time.sleep(0.6)
                
                set_motors(0, 0)
                time.sleep(0.2)
                
                # 特别修复：这里的内部循环也需要锁定逻辑
                stop_cross_counter = 0
                stop_locked = False
                while num_cross < 11:
                    tracking_logic(BASE_SPEED)
                    
                    LL, L, R, RR = read_sensors()
                    if detect_cross_junction((LL, L, R, RR)):
                        if not stop_locked:
                            stop_cross_counter += 1
                            if stop_cross_counter >= CROSS_DETECT_THRESHOLD:
                                num_cross += 1
                                stop_locked = True
                                print(f"[DELIVERY] Cross detected, count: {num_cross}")
                    else:
                        stop_cross_counter = 0
                        stop_locked = False
                    
                    time.sleep(0.005)
                
                set_motors(0, 0)
                time.sleep(0.5)
                
                print("[DONE]")
                servo_move(-35)
                time.sleep(2.0)
                servo_move(-12)
                time.sleep(0.5)
                
                break
            
            time.sleep(0.005)
    
    except KeyboardInterrupt:
        print("\n[STOP]")
        set_motors(0, 0)
    
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
        set_motors(0, 0)
    
    finally:
        set_motors(0, 0)
        time.sleep(0.2)
        cleanup_hardware()

if __name__ == "__main__":
    main()