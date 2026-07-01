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

"""
YuYuan Rover final mission controller.

整体流程：
1. 读取/恢复任务状态，必要时在 S1 扫描 AprilTag 目标。
2. 用红外传感器做循迹和十字路口计数。
3. 在指定阶段结合超声波避障、河谷段柔和循迹、终点卸货。
4. 程序退出或异常时停止电机并释放串口/GPIO。
"""

# ==========================================
# 1. 全局配置：硬件端口、速度参数、传感器阈值
# ==========================================
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
STATE_FILE = "mission_state.json"

# Motor speed parameters
BASE_SPEED = 0.3
VALLEY_SPEED = 0.3
INNER = 0.05
OUTER = 0.5
PIVOT_SPEED = 0.35 
AVOID_STRAIGHT = 0.15 

THRESH_FRONT = 11.0
THRESH_EMG = 15.0
THRESH_DIFF = 3
MAX_VALID = 40.0

# 十字路口去抖：连续检测到多次全黑才算真正经过一个路口。
CROSS_DETECT_THRESHOLD = 3

# 四路红外循迹传感器：0 通常表示压在线/黑色区域，1 表示白底。
sensor_ll = DigitalInputDevice(12)
sensor_l  = DigitalInputDevice(5)
sensor_r  = DigitalInputDevice(6)
sensor_rr = DigitalInputDevice(13)
SERVO_PIN = 8

# ==========================================
# 2. 任务状态持久化：支持从 S1/S2/S3 检查点恢复
# ==========================================
def save_state(startpoint, target_id, current_flag=None, current_cross=None):
    """保存当前检查点、目标编号、状态阶段和十字路口计数。"""
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
    """读取上次任务状态；没有状态文件时默认从 S1 开始。"""
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
    """调试用：删除状态文件，下次启动将从 S1 重新开始。"""
    if os.path.exists(STATE_FILE):
        try:
            os.remove(STATE_FILE)
        except Exception as e:
            print(f"[ERROR] Reset failed: {e}")

# ==========================================
# 3. 硬件抽象层：串口底盘、舵机、超声波封装
# ==========================================
ser = None
gpio_handle = None

def init_hardware():
    """初始化串口底盘控制和舵机 GPIO。"""
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
    """安全停止电机并释放硬件资源。"""
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
    """设置左右轮速度，并限制在底盘安全范围内。"""
    if ser and ser.is_open:
        l = max(-0.5, min(0.5, left))
        r = max(-0.5, min(0.5, right))
        data = {"T": 1, "L": round(l, 2), "R": round(r, 2)}
        try:
            ser.write((json.dumps(data) + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[ERROR] Motor control failed: {e}")

def angle_to_duty(angle):
    """把舵机角度转换成 50 Hz PWM 占空比。"""
    angle = max(-90, min(90, angle))
    pulse_us = 1500 + (angle / 90) * 1000
    return (pulse_us / 20000) * 100

def servo_move(target_angle):
    """移动卸货舵机到指定角度，随后关闭 PWM 保持安静。"""
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
    """带简单滑动平均的超声波测距封装。"""
    def __init__(self, t, e, name=""):
        self.sensor = DistanceSensor(echo=e, trigger=t, queue_len=1)
        self.history = deque(maxlen=3)
        self.name = name
    
    def get(self):
        """返回厘米单位距离；异常或过近读数用 999 表示无效。"""
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
# 4. 感知与基础动作：视觉识别、循迹、路口判断
# ==========================================
def perform_scan():
    """在 S1 使用摄像头扫描 AprilTag，返回目标编号 0/1/2。"""
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
    """读取四路红外循迹传感器，顺序为 LL、L、R、RR。"""
    return (
        sensor_ll.value,
        sensor_l.value,
        sensor_r.value,
        sensor_rr.value
    )

def detect_cross_junction(current_state):
    """四路传感器全为 0 时判定为十字路口候选。"""
    LL, L, R, RR = current_state
    return (LL == 0 and L == 0 and R == 0 and RR == 0)

def tracking_logic(speed_val):
    """通用循迹控制：根据左右偏差调整内外轮速度。"""
    LL, L, R, RR = read_sensors()
    
    if LL == RR and L == 1 and R == 1:
        set_motors(speed_val, speed_val)
    elif (LL == 1 and L == 1 and R == 0 and RR == 1) or (LL == 1 and R == 1 and L == 1 and RR == 0):
        set_motors(OUTER, INNER)
    elif (LL == 1 and L == 0 and R == 1 and RR == 1) or (LL == 0 and R == 1 and L == 1 and RR == 1):
        set_motors(INNER, OUTER)
    # 传感器状态不明确时保守直行，避免原地抖动。
    else:
        set_motors(speed_val, speed_val)

def handle_t_junction(flag):
    """Flag 2 专用：检测 T 字路口后执行一次左转。"""
    if flag != 2:
        return False
    
    LL, L, R, RR = read_sensors()
    
    if LL == 0 and L == 0 and R == 0 and RR == 1:
        time.sleep(0.04)
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
    """根据恢复点把状态机阶段和路口计数对齐。"""
    if startpoint == "S1":
        flag = 1
        num_cross = 0
        need_scan = (target_id is None)
    elif startpoint == "S2":
        flag = 2
        num_cross = 8
        need_scan = False
    elif startpoint == "S3":
        flag = 4  # S3 起步直接进入河谷段。
        num_cross = 11
        need_scan = False
    else:
        print(f"[ERROR] Unknown startpoint: {startpoint}")
        flag = 1
        num_cross = 0
        need_scan = True
    
    return flag, num_cross, need_scan

# ==========================================
# 5. 主状态机：按赛道分段执行任务
# ==========================================
def main():
    """主任务循环：调度扫描、循迹、避障、河谷和卸货。"""
    
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
    junction_locked = False
    first_avoid_pivot_done = False
    # 避障阶段用最近一次有效侧向距离兜底，减少超声波瞬时异常的影响。
    last_valid_l = 15.0
    last_valid_r = 15.0
    
    try:
        while True:
            LL, L, R, RR = read_sensors()
            
            # ============================================================
            # 全局十字路口计数：锁定机制避免在同一个黑框内重复计数。
            # ============================================================
            if detect_cross_junction((LL, L, R, RR)):
                if not junction_locked:
                    cross_detected_counter += 1
                    if cross_detected_counter >= CROSS_DETECT_THRESHOLD:
                        num_cross += 1
                        junction_locked = True
                        cross_detected_counter = 0
                        print(f"[CROSS] #{num_cross} Flag:{flag} (Locked)")
                        
                        if num_cross == 8:
                            save_state("S2", target_id, flag, num_cross)
                        elif num_cross == 11:
                            save_state("S3", target_id, flag, num_cross)
            else:
                if junction_locked:
                    print("[CROSS] Sensor cleared white ground (Unlocked)")
                junction_locked = False
                cross_detected_counter = 0
            
            # ============================================================
            # FLAG 1: 初始循迹，直到累计到第 8 个十字路口。
            # ============================================================
            if flag == 1:
                tracking_logic(BASE_SPEED)
                
                if num_cross >= 8:
                    print(f"[FLAG] 1->2")
                    flag = 2
            
            # ============================================================
            # FLAG 2: 中段循迹，处理 T 字路口，并等待前方障碍触发避障。
            # ============================================================
            elif flag == 2:
                if not handle_t_junction(flag):
                    tracking_logic(BASE_SPEED)
                
                if u_f.get() < THRESH_FRONT:
                    print(f"[FLAG] 2->3 obs:{u_f.get():.1f}")
                    flag = 3
                    first_avoid_pivot_done = False
            
            # ============================================================
            # FLAG 3: 避障段。第一次遇障执行固定绕行脚本，随后寻找线路接回。
            # ============================================================
            elif flag == 3:
                df = u_f.get()
                rl, rr = u_l.get(), u_r.get()
                
                # 只接受有效范围内的侧向测距，避免 999 这类无效值干扰决策。
                if 2.1 <= rl <= MAX_VALID: last_valid_l = rl
                if 2.1 <= rr <= MAX_VALID: last_valid_r = rr
                dl, dr = round(last_valid_l, 1), round(last_valid_r, 1)

                if df < THRESH_FRONT:
                    if not first_avoid_pivot_done:
                        print("[AVOID] Executing scripted maneuver...")
                        set_motors(-0.2, -0.2); time.sleep(0.5); set_motors(0, 0); time.sleep(0.05)
                        set_motors(-PIVOT_SPEED, PIVOT_SPEED); time.sleep(0.4); set_motors(0, 0); time.sleep(0.1)
                        set_motors(AVOID_STRAIGHT*1.5 + 0.1, AVOID_STRAIGHT*1.5); time.sleep(2.7)
                        set_motors(PIVOT_SPEED, -PIVOT_SPEED); time.sleep(0.8)
                        set_motors(AVOID_STRAIGHT*1.5, AVOID_STRAIGHT*1.5); time.sleep(1.2)
                        set_motors(PIVOT_SPEED, -PIVOT_SPEED); time.sleep(0.2)
                        set_motors(AVOID_STRAIGHT*1.5, AVOID_STRAIGHT*1.5); time.sleep(1.5)
                        
                        first_avoid_pivot_done = True
                    else:
                        # 前方仍有障碍时，朝侧向空间更大的方向微调。
                        if dl > dr: set_motors(INNER, OUTER)
                        else: set_motors(OUTER, INNER)
                else:
                    if first_avoid_pivot_done:
                        tracking_logic(BASE_SPEED)
                        # 避障后重新看到十字路口，认为已经接回 S3 并进入河谷段。
                        if LL == 0 and L == 0 and R == 0 and RR == 0:
                            if not junction_locked:
                                num_cross += 1
                                junction_locked = True
                                print(f"[FLAGs] 3->4 S3 Junction Detected (#{num_cross})")
                                save_state("S3", target_id, 4, num_cross)
                                flag = 4
                    else:
                        set_motors(AVOID_STRAIGHT, AVOID_STRAIGHT)
               
            
            # ============================================================
            # FLAG 4: 河谷段。使用更柔和的速度差，避免在窄路中过度修正。
            # ============================================================
            elif flag == 4:
                LL, L, R, RR = read_sensors()
                
                # 河谷段专用速度表：内外轮差值比普通循迹更小。
                V_BASE = VALLEY_SPEED      # 0.3
                V_SOFT_OUT = V_BASE + 0.08
                V_SOFT_IN  = V_BASE - 0.08
                V_HARD_OUT = V_BASE + 0.15
                V_HARD_IN  = V_BASE - 0.15

                if (L == 0 and R == 0):
                    set_motors(V_BASE, V_BASE)
                
                elif LL == 0:
                    set_motors(V_HARD_IN, V_HARD_OUT)
                elif RR == 0:
                    set_motors(V_HARD_OUT, V_HARD_IN)
                
                elif L == 0:
                    set_motors(V_SOFT_IN, V_SOFT_OUT)
                elif R == 0:
                    set_motors(V_SOFT_OUT, V_SOFT_IN)
                
                else:
                    set_motors(V_BASE, V_BASE)

                if detect_cross_junction((LL, L, R, RR)):
                    if not junction_locked:
                        cross_detected_counter += 1
                        if cross_detected_counter >= CROSS_DETECT_THRESHOLD:
                            num_cross += 1
                            junction_locked = True
                            print(f"[CROSS] #{num_cross} in Valley")
                            # 到达卸货路口后切换到最终投放阶段。
                            if num_cross >= 13:
                                print(f"[FLAG] 4->5 Reached Unloading Junction")
                                set_motors(0, 0); time.sleep(0.2)
                                flag = 5
                else:
                    junction_locked = False
                    cross_detected_counter = 0
            # ============================================================
            # FLAG 5: 根据目标编号调整朝向，继续循迹到停止线后卸货。
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
                
                # 卸货阶段内部也使用独立锁定，避免停止线前重复计数。
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
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n[STOP]")
        set_motors(0, 0)
    
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
        set_motors(0, 0)
    
    finally:
        # 无论正常结束、Ctrl+C 还是异常，都优先让车停住。
        set_motors(0, 0)
        time.sleep(0.2)
        cleanup_hardware()

if __name__ == "__main__":
    main()
