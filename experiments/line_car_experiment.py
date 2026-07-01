# ===================== 全量依赖库导入 =====================
from picamera2 import Picamera2
import cv2
import apriltag
import serial
import time
import json
import os
import lgpio
from collections import deque
from gpiozero import DigitalInputDevice, DistanceSensor

# 树莓派GPIO驱动配置
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

# ===================== 1. 全局统一配置【标准化命名】 =====================
# --- 摄像头配置 ---
TAG_FAMILY = "tag25h9"
CAM_SIZE = (640, 480)

# --- 串口&电机通用配置 ---
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

# --- 巡线传感器硬件配置 ---
sensor_ll = DigitalInputDevice(12)
sensor_l  = DigitalInputDevice(5)
sensor_r  = DigitalInputDevice(6)
sensor_rr = DigitalInputDevice(13)

# --- 基础巡线参数（状态2/4/6共用） ---
TRACK_BASE_SPEED = 0.2
TRACK_INNER_SPEED = 0.05
TRACK_OUTER_SPEED = 0.5
TRACK_PIVOT_SPEED = 0.4

# --- 避障专用参数（状态3 独立） ---
AVOID_FRONT_THRESH = 11.0
AVOID_SIDE_EMG_THRESH = 12.0
AVOID_OFFSET_THRESH = 3
AVOID_MAX_VALID = 40.0
AVOID_STRAIGHT_SPEED = 0.15
AVOID_NAV_FAST = 0.32
AVOID_NAV_SLOW = 0.04
AVOID_EMG_FAST = 0.4
AVOID_EMG_REVERSE = -0.10
AVOID_PIVOT_SPEED = 0.35

# --- 状态5专属参数（左右等距行驶） ---
WALL_EQUAL_TOLERANCE = 3.0
WALL_CORRECTION_STEP = 0.04
WALL_MAIN_SPEED = 0.20
WALL_MAX_CORRECTION = 0.08
WALL_ULTRA_MAX_FILTER = 80.0

# --- 舵机配置（状态6卸货专用） ---
SERVO_PIN = 8
SERVO_CHIP = 0
SERVO_FREQ = 50

# ===================== 2. 全局核心变量 =====================
ser = None                      # 全局串口
gpio_handle = None             # 舵机控制句柄
now_angle = -12                # 舵机初始角度
cross_count = 0                # 全局十字路口计数器
checkpoint = "S1"              # 存档点：S1→≥6=S2→≥8=S3
target_unload_id = None        # 状态1识别的ABC结果(0=A,1=B,2=C)

# 状态6内部专用变量
state6_cross_count = 0
state6_cross_detected_counter = 0
need_second_turn = False
second_turn_dir = ""
# 状态机常量
STATE_TRACKING = 'TRACKING'
STATE_TURNING = 'TURNING'
STATE_ARRIVED = 'ARRIVED'
STATE_UNLOADING = 'UNLOADING'
STATE_FINISHED = 'FINISHED'

# ===================== 3. 通用工具函数（全局复用） =====================
def init_serial():
    """全局串口初始化"""
    global ser
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
        ser.setRTS(False)
        ser.setDTR(False)
        print("✅ Serial initialized successfully")
        return True
    except:
        print("❌ Serial initialization failed")
        return False

def init_servo():
    """全局舵机初始化（状态6专用）"""
    global gpio_handle, now_angle
    try:
        gpio_handle = lgpio.gpiochip_open(SERVO_CHIP)
        lgpio.gpio_claim_output(gpio_handle, SERVO_PIN)
        duty = angle_to_duty(now_angle)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, SERVO_FREQ, duty)
        time.sleep(0.3)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, SERVO_FREQ, 0)
        print("✅ Servo initialized successfully")
        return True
    except Exception as e:
        print(f"❌ Servo initialization failed: {e}")
        return False

def set_motors(left, right):
    """全局电机控制（所有状态共用）"""
    if ser and ser.is_open:
        left = max(-0.5, min(0.5, left))
        right = max(-0.5, min(0.5, right))
        data = {"T": 1, "L": round(left, 2), "R": round(right, 2)}
        ser.write((json.dumps(data) + '\n').encode('utf-8'))

def angle_to_duty(angle):
    """舵机角度转换"""
    angle = max(-90, min(90, angle))
    pulse_us = 1500 + (angle / 90) * 1000
    duty = (pulse_us / 20000) * 100
    return duty

def slow_move(target_angle, step=1, delay=0.015):
    """舵机缓慢移动（卸货）"""
    global now_angle
    try:
        while now_angle < target_angle:
            now_angle += step
            duty = angle_to_duty(now_angle)
            lgpio.tx_pwm(gpio_handle, SERVO_PIN, SERVO_FREQ, duty)
            time.sleep(delay)
        while now_angle > target_angle:
            now_angle -= step
            duty = angle_to_duty(now_angle)
            lgpio.tx_pwm(gpio_handle, SERVO_PIN, SERVO_FREQ, duty)
            time.sleep(delay)
        time.sleep(0.3)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, SERVO_FREQ, 0)
    except:
        pass

def turn_90_degree(direction):
    """90度原地转向（状态6专用）"""
    set_motors(0, 0)
    time.sleep(0.1)
    if direction == 'left':
        set_motors(-TRACK_PIVOT_SPEED, TRACK_PIVOT_SPEED)
    elif direction == 'right':
        set_motors(TRACK_PIVOT_SPEED, -TRACK_PIVOT_SPEED)
    time.sleep(0.5)
    set_motors(0, 0)
    time.sleep(0.1)
    print(f"✅ {direction} turn completed")

# ===================== 4. 超声波传感器类 =====================
class RobustUltra:
    """抗干扰超声波传感器"""
    def __init__(self, t, e):
        self.sensor = DistanceSensor(echo=e, trigger=t, queue_len=1)
        self.history = deque(maxlen=3)
    def get(self):
        try:
            val = self.sensor.distance * 100
            if val < 2.1: return 999.0
            self.history.append(val)
        except: return 999.0
        return sum(self.history)/len(self.history) if self.history else 999.0

# 初始化超声波传感器
ultra_f = RobustUltra(23, 24)
ultra_l = RobustUltra(17, 27)
ultra_r = RobustUltra(22, 25)

# ===================== 【状态机标准化命名】 =====================
# 状态0-系统初始化-硬件启动
# 状态1-单次图像识别-无窗口执行(获取ABC)
# 状态2-带转向巡线-四路丢线退出
# 状态3-超声波避障-循迹回归退出
# 状态4-纯巡线行驶-左右丢线退出
# 状态5-左右等距直道行驶-检测黑线终止
# 状态6-最终巡线卸货-第一个路口ABC选择

# ===================== 状态函数 =====================
def state0_system_init():
    """状态0-系统初始化-硬件启动"""
    print("\n===== State 0: System Initialization =====")
    init_serial()
    init_servo()
    time.sleep(1)
    return 1

def state1_image_detect():
    """状态1-单次图像识别-无窗口执行(保存ABC结果)"""
    global target_unload_id
    print("\n===== State 1: Image Detection =====")
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": CAM_SIZE}))
    picam2.start()
    detector = apriltag.Detector(apriltag.DetectorOptions(families=TAG_FAMILY))
    gray = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2GRAY)
    tags = detector.detect(gray)
    
    # 识别ABC标签
    for tag in tags:
        if tag.tag_id == 0:
            target_unload_id = 0
            print("✅ Tag detected: A")
        elif tag.tag_id == 1:
            target_unload_id = 1
            print("✅ Tag detected: B")
        elif tag.tag_id == 2:
            target_unload_id = 2
            print("✅ Tag detected: C")
    
    picam2.stop()
    print("✅ State 1 completed")
    return 2

def state2_track_with_turn():
    """状态2-带转向巡线-四路丢线退出"""
    global cross_count, checkpoint
    print("\n===== State 2: Line Tracking with Turn =====")
    try:
        while True:
            LL, L, R, RR = sensor_ll.value, sensor_l.value, sensor_r.value, sensor_rr.value
            if LL == 1 and L == 1 and R == 1 and RR == 1:
                print("\n⚠️ All sensors lost -> Enter State 3")
                set_motors(0, 0)
                time.sleep(0.5)
                return 3
            if LL == 0 and L == 0 and R == 0 and RR == 0:
                cross_count +=1
                print(f"🚦 Cross count: {cross_count} | Checkpoint: {checkpoint}")
                if cross_count >=6 and checkpoint == "S1":
                    checkpoint = "S2"
                    print(f"✅ Checkpoint updated: S2")
                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                time.sleep(0.06)
                continue
            if LL == 0 and L == 0 and R == 0 and RR == 1:
                time.sleep(0.06)
                if sensor_ll.value==0 and sensor_rr.value==1:
                    set_motors(0,0)
                    time.sleep(0.1)
                    set_motors(-TRACK_PIVOT_SPEED, TRACK_PIVOT_SPEED)
                    time.sleep(0.6)
                    continue
            if LL == RR and L == 1 and R == 1:
                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
            elif LL ==1 and L==1 and R==0 and RR==1:
                set_motors(TRACK_OUTER_SPEED, TRACK_INNER_SPEED)
            elif LL ==1 and L==0 and R==1 and RR==1:
                set_motors(TRACK_INNER_SPEED, TRACK_OUTER_SPEED)
            elif LL ==0 and L==1 and R==1 and RR==1:
                set_motors(TRACK_INNER_SPEED, TRACK_OUTER_SPEED)
            elif LL ==1 and L==1 and R==1 and RR==0:
                set_motors(TRACK_OUTER_SPEED, TRACK_INNER_SPEED)
            else:
                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
            time.sleep(0.005)
    except KeyboardInterrupt:
        set_motors(0,0)
        return -1

def state3_ultrasonic_avoid():
    """状态3-超声波避障-循迹回归退出"""
    print("\n===== State 3: Ultrasonic Obstacle Avoidance =====")
    first_pivot_done = False
    last_l, last_r = 15.0,15.0
    try:
        while True:
            df = ultra_f.get()
            raw_l, raw_r = ultra_l.get(), ultra_r.get()
            if 2.1<=raw_l<=AVOID_MAX_VALID: last_l=raw_l
            if 2.1<=raw_r<=AVOID_MAX_VALID: last_r=raw_r
            dl, dr = round(last_l,1), round(last_r,1)
            if df < AVOID_FRONT_THRESH:
                if not first_pivot_done:
                    set_motors(-0.2,-0.2);time.sleep(0.05)
                    set_motors(0,0);time.sleep(0.1)
                    set_motors(-AVOID_PIVOT_SPEED, AVOID_PIVOT_SPEED);time.sleep(0.6)
                    set_motors(0,0);time.sleep(0.3)
                    set_motors(AVOID_STRAIGHT_SPEED, AVOID_STRAIGHT_SPEED);time.sleep(0.8)
                    first_pivot_done=True
                else:
                    set_motors(-0.2,-0.2);time.sleep(0.05)
                    set_motors(0,0);time.sleep(0.1)
                    set_motors(-AVOID_PIVOT_SPEED, AVOID_PIVOT_SPEED) if dl>dr else set_motors(AVOID_PIVOT_SPEED, -AVOID_PIVOT_SPEED)
                    time.sleep(0.6)
                    set_motors(0,0);time.sleep(0.3)
                continue
            if first_pivot_done:
                if dr <= AVOID_SIDE_EMG_THRESH:
                    set_motors(AVOID_EMG_REVERSE, AVOID_EMG_FAST)
                elif dl <= AVOID_SIDE_EMG_THRESH:
                    set_motors(AVOID_EMG_FAST, AVOID_EMG_REVERSE)
                elif dl > dr + AVOID_OFFSET_THRESH:
                    set_motors(AVOID_NAV_SLOW, AVOID_NAV_FAST)
                elif dr > dl + AVOID_OFFSET_THRESH:
                    set_motors(AVOID_NAV_FAST, AVOID_NAV_SLOW)
                else:
                    set_motors(AVOID_STRAIGHT_SPEED, AVOID_STRAIGHT_SPEED)
            else:
                set_motors(AVOID_STRAIGHT_SPEED, AVOID_STRAIGHT_SPEED)
            if (sensor_l.value ==0 or sensor_r.value ==0) and first_pivot_done:
                set_motors(0,0);time.sleep(0.2)
                set_motors(-AVOID_PIVOT_SPEED, AVOID_PIVOT_SPEED);time.sleep(0.6)
                set_motors(0,0)
                print("✅ Obstacle avoided -> Enter State 4")
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        set_motors(0,0)
        return -1
    return 4

def state4_pure_track():
    """状态4-纯巡线行驶-左右丢线退出"""
    global cross_count, checkpoint
    print("\n===== State 4: Pure Line Tracking =====")
    try:
        while True:
            LL, L, R, RR = sensor_ll.value, sensor_l.value, sensor_r.value, sensor_rr.value
            if L == 1 and R == 1:
                print("\n⚠️ Left/Right sensors lost -> Enter State 5")
                set_motors(0, 0)
                time.sleep(0.5)
                return 5
            if LL == 0 and L == 0 and R == 0 and RR == 0:
                cross_count +=1
                print(f"🚦 Cross count: {cross_count} | Checkpoint: {checkpoint}")
                if cross_count >=8 and checkpoint == "S2":
                    checkpoint = "S3"
                    print(f"✅ Checkpoint updated: S3")
                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                time.sleep(0.06)
                continue
            if LL == RR and L == 1 and R == 1:
                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
            elif LL ==1 and L==1 and R==0 and RR==1:
                set_motors(TRACK_OUTER_SPEED, TRACK_INNER_SPEED)
            elif LL ==1 and L==0 and R==1 and RR==1:
                set_motors(TRACK_INNER_SPEED, TRACK_OUTER_SPEED)
            elif LL ==0 and L==1 and R==1 and RR==1:
                set_motors(TRACK_INNER_SPEED, TRACK_OUTER_SPEED)
            elif LL ==1 and L==1 and R==1 and RR==0:
                set_motors(TRACK_OUTER_SPEED, TRACK_INNER_SPEED)
            else:
                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
            time.sleep(0.005)
    except KeyboardInterrupt:
        set_motors(0,0)
        return -1

def state5_wall_equal_straight():
    """状态5-左右等距直道行驶-检测黑线终止→进入状态6"""
    global cross_count
    print("\n===== State 5: Wall Equal Distance Straight Driving =====")
    try:
        while True:
            left_raw = ultra_l.get()
            right_raw = ultra_r.get()
            left_dist = left_raw if left_raw <= WALL_ULTRA_MAX_FILTER else 30.0
            right_dist = right_raw if right_raw <= WALL_ULTRA_MAX_FILTER else 30.0

            # 退出条件：检测到黑线 → 进入状态6
            if (sensor_ll.value == 0 or sensor_l.value == 0 or
                sensor_r.value == 0 or sensor_rr.value == 0):
                print("\n✅ Black line detected -> Enter State 6")
                cross_count += 1
                set_motors(0, 0)
                time.sleep(0.5)
                return 6  # 关键：跳转到状态6

            dist_diff = left_dist - right_dist
            if abs(dist_diff) <= WALL_EQUAL_TOLERANCE:
                spd_l = WALL_MAIN_SPEED
                spd_r = WALL_MAIN_SPEED
            elif dist_diff > WALL_EQUAL_TOLERANCE:
                adjust_val = min(WALL_CORRECTION_STEP, WALL_MAX_CORRECTION)
                spd_l, spd_r = WALL_MAIN_SPEED, WALL_MAIN_SPEED - adjust_val
            else:
                adjust_val = min(WALL_CORRECTION_STEP, WALL_MAX_CORRECTION)
                spd_r, spd_l = WALL_MAIN_SPEED, WALL_MAIN_SPEED - adjust_val

            set_motors(max(0.05, spd_l), max(0.05, spd_r))
            time.sleep(0.025)
    except KeyboardInterrupt:
        set_motors(0, 0)
        return -1

def state6_final_track_unload():
    """
    状态6-最终巡线卸货-第一个路口ABC选择
    核心修正：第一个路口直接执行选择逻辑
    """
    global state6_cross_count, state6_cross_detected_counter
    global need_second_turn, second_turn_dir
    print("\n===== State 6: Final Tracking & Unloading =====")
    print(f"✅ Executing target selection: {chr(ord('A')+target_unload_id)}")
    
    # 重置状态6内部计数
    state6_cross_count = 0
    state6_cross_detected_counter = 0
    need_second_turn = False
    second_turn_dir = ""
    current_state = STATE_TRACKING

    try:
        while True:
            if current_state == STATE_FINISHED:
                break
            LL, L, R, RR = sensor_ll.value, sensor_l.value, sensor_r.value, sensor_rr.value
            black_count = sum([1 for val in [LL, L, R, RR] if val == 0])

            if current_state == STATE_TRACKING:
                # 二次转向逻辑
                if need_second_turn and black_count == 3:
                    set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                    time.sleep(0.3)
                    turn_90_degree(second_turn_dir)
                    need_second_turn = False
                    continue

                # 十字路口检测（核心修正：第一个路口就选择）
                if LL == 0 and L == 0 and R == 0 and RR == 0:
                    state6_cross_detected_counter += 1
                    if state6_cross_detected_counter >= 2:
                        state6_cross_count += 1
                        print(f"🚦 State6 cross count: {state6_cross_count}")
                        state6_cross_detected_counter = 0
                        time.sleep(0.03)

                        # =====================
                        # 核心：第一个路口直接ABC选择
                        # =====================
                        if state6_cross_count == 1:
                            print("✅ 1st cross: Execute ABC route selection")
                            if target_unload_id == 0:
                                # A路线：左转+二次右转
                                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                                time.sleep(0.3)
                                turn_90_degree('left')
                                need_second_turn = True
                                second_turn_dir = "right"
                            elif target_unload_id == 1:
                                # B路线：直行
                                print("✅ Route B: Go straight")
                                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                                time.sleep(0.2)
                            elif target_unload_id == 2:
                                # C路线：右转+二次左转
                                set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                                time.sleep(0.3)
                                turn_90_degree('right')
                                need_second_turn = True
                                second_turn_dir = "left"
                        
                        # 第二个路口：直行
                        elif state6_cross_count == 2:
                            print("✅ 2nd cross: Go straight")
                            set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                            time.sleep(0.2)
                        
                        # 第三个路口：到达卸货点
                        elif state6_cross_count == 3:
                            set_motors(0, 0)
                            time.sleep(0.1)
                            set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                            time.sleep(1.2)
                            set_motors(0, 0)
                            time.sleep(0.1)
                            current_state = STATE_ARRIVED
                else:
                    state6_cross_detected_counter = 0
                    # 基础巡线
                    if LL == RR and L == 1 and R == 1:
                        set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)
                    elif LL == 1 and L == 1 and R == 0 and RR == 1:
                        set_motors(TRACK_OUTER_SPEED, TRACK_INNER_SPEED)
                    elif LL == 1 and L == 0 and R == 1 and RR == 1:
                        set_motors(TRACK_INNER_SPEED, TRACK_OUTER_SPEED)
                    elif LL == 0 and L == 1 and R == 1 and RR == 1:
                        set_motors(TRACK_INNER_SPEED, TRACK_OUTER_SPEED)
                    elif LL == 1 and L == 1 and R == 1 and RR == 0:
                        set_motors(TRACK_OUTER_SPEED, TRACK_INNER_SPEED)
                    else:
                        set_motors(TRACK_BASE_SPEED, TRACK_BASE_SPEED)

            # 卸货流程
            elif current_state == STATE_ARRIVED:
                print("✅ Arrived at unloading point, start unloading")
                slow_move(-35)
                time.sleep(2)
                slow_move(-12)
                print("✅ Unloading completed! All tasks finished")
                current_state = STATE_FINISHED

            time.sleep(0.005)

    except KeyboardInterrupt:
        set_motors(0, 0)
    return -1

# ===================== 状态机引擎 =====================
def state_machine():
    current_state = 0
    while current_state != -1:
        if current_state == 0: current_state = state0_system_init()
        elif current_state == 1: current_state = state1_image_detect()
        elif current_state == 2: current_state = state2_track_with_turn()
        elif current_state == 3: current_state = state3_ultrasonic_avoid()
        elif current_state == 4: current_state = state4_pure_track()
        elif current_state == 5: current_state = state5_wall_equal_straight()
        elif current_state == 6: current_state = state6_final_track_unload()
        else: current_state = 0

# ===================== 启动程序 =====================
if __name__ == "__main__":
    print("========= 6-State Automatic Process Started =========")
    state_machine()
    # 收尾释放硬件
    set_motors(0, 0)
    if ser and ser.is_open: ser.close()
    if gpio_handle: lgpio.gpiochip_close(gpio_handle)
    print("\n========= All tasks completed! =========")