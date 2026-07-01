import serial
import time
import json
from gpiozero import DigitalInputDevice
from picamera2 import Picamera2
import cv2
import apriltag
import lgpio

# ==========================================
# 1. 全锟斤拷硬锟斤拷锟斤拷锟斤拷锟斤拷锟矫ｏ拷锟斤拷全锟斤拷锟斤拷锟斤拷锟皆硷拷锟斤拷锟斤拷锟?# ==========================================
SERIAL_PORT = '/dev/ttyAMA0' 
BAUD_RATE = 115200
BASE_SPEED = 0.2
INNER = 0.05
OUTER = 0.5
PIVOT_SPEED = 0.4

sensor_ll = DigitalInputDevice(12)
sensor_l  = DigitalInputDevice(5)
sensor_r  = DigitalInputDevice(6)
sensor_rr = DigitalInputDevice(13)

SERVO_PIN = 8
CHIP = 0
FREQ = 50

TAG_FAMILY = "tag25h9"
CAM_WIDTH = 640
CAM_HEIGHT = 480
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600

# ==========================================
# 2. 全锟斤拷状态锟斤拷锟斤拷
# ==========================================
ser = None                
gpio_handle = None        
now_angle = -12           
target_unload_id = None   
cross_count = 0           
cross_detected_counter = 0
# A/C锟斤拷锟斤拷诙锟斤拷锟阶拷锟斤拷锟?need_second_turn = False  
second_turn_dir = ""      

# 状态锟斤拷
STATE_IDLE = 'IDLE'
STATE_DETECT_TAG = 'DETECT_TAG'
STATE_TRACKING = 'TRACKING'
STATE_TURNING = 'TURNING'
STATE_ARRIVED = 'ARRIVED'
STATE_UNLOADING = 'UNLOADING'
STATE_FINISHED = 'FINISHED'
current_state = STATE_IDLE

# ==========================================
# 3. 硬锟斤拷锟斤拷始锟斤拷锟斤拷锟斤拷全锟斤拷锟斤拷锟斤拷
# ==========================================
def init_serial():
    global ser
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
        ser.setRTS(False)
        ser.setDTR(False)
        print("Serial init success")
        return True
    except Exception as e:
        print(f"Serial init failed: {e}")
        return False
        
def init_servo():
    global gpio_handle

    try:
        gpio_handle = lgpio.gpiochip_open(CHIP)
        lgpio.gpio_claim_output(gpio_handle, SERVO_PIN)
        print("Servo init success")
        duty = angle_to_duty(now_angle)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, duty)
        time.sleep(0.3)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, 0)
        return True
    except Exception as e:
        print(f"Servo init failed: {e}")
        return False

def init_camera():
    try:
        picam2 = Picamera2()
        video_config = picam2.create_video_configuration(main={"size": (CAM_WIDTH, CAM_HEIGHT)})
        picam2.configure(video_config)
        picam2.start()
        options = apriltag.DetectorOptions(families=TAG_FAMILY)
        detector = apriltag.Detector(options)
        cv2.namedWindow("AprilTag Tag25h9 Detector", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("AprilTag Tag25h9 Detector", DISPLAY_WIDTH, DISPLAY_HEIGHT)
        print("Camera and AprilTag detector init success")
        return picam2, detector
    except Exception as e:
        print(f"Camera init failed: {e}")
        return None, None

# ==========================================
# 4. 锟斤拷锟侥癸拷锟杰猴拷锟斤拷锟斤拷锟斤拷全锟斤拷锟斤拷锟斤拷
# ==========================================
def set_motors(left, right):
    if ser and ser.is_open:
        data = {"T": 1, "L": round(left, 2), "R": round(right, 2)}
        ser.write((json.dumps(data) + '\n').encode('utf-8'))

def angle_to_duty(angle):
    angle = max(-90, min(90, angle))
    pulse_us = 1500 + (angle / 90) * 1000
    duty = (pulse_us / 20000) * 100
    return duty

def slow_move(target_angle, step=1, delay=0.015):
    global now_angle
    try:
        while now_angle < target_angle:
            now_angle += step
            duty = angle_to_duty(now_angle)
            lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, duty)
            time.sleep(delay)
            
        while now_angle > target_angle:
            now_angle -= step
            duty = angle_to_duty(now_angle)
            lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, duty)
            time.sleep(delay)
        
        time.sleep(0.3)
        lgpio.tx_pwm(gpio_handle, SERVO_PIN, FREQ, 0)
        print(f"Servo moved to {target_angle} degrees")
    except Exception as e:
        print(f"Servo move failed: {e}")

def detect_apriltag(picam2, detector):
    global target_unload_id
    print("Start AprilTag detection, press Q to quit detection")
    try:
        while True:
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray)
            
            valid_tag = None
            for tag in tags:
                corners = tag.corners.astype(int)
                cv2.polylines(frame_bgr, [corners], True, (0, 255, 0), 2)
                cX = int(tag.center[0])
                cY = int(tag.center[1])
                cv2.circle(frame_bgr, (cX, cY), 4, (0, 0, 255), -1)
                
                if tag.tag_id == 0:
                    display_text = "ID: A"
                    valid_tag = 0
                elif tag.tag_id == 1:
                    display_text = "ID: B"
                    valid_tag = 1
                elif tag.tag_id == 2:
                    display_text = "ID: C"
                    valid_tag = 2
                else:
                    display_text = f"ID: {tag.tag_id}"
                
                cv2.putText(frame_bgr, display_text, (cX - 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            cv2.imshow("AprilTag Tag25h9 Detector", frame_bgr)
            
            if valid_tag is not None:
                target_unload_id = valid_tag
                tag_name = chr(ord('A') + valid_tag)
                print(f"Detected valid target tag: {tag_name} (ID: {valid_tag})")
                picam2.stop()
                cv2.destroyAllWindows()
                print("Camera closed, start line tracking mission")
                return True
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                picam2.stop()
                cv2.destroyAllWindows()
                print("Detection quit by user")
                return False
    except Exception as e:
        print(f"AprilTag detection error: {e}")
        picam2.stop()
        cv2.destroyAllWindows()
        return False

def turn_90_degree(direction):
    global current_state
    current_state = STATE_TURNING
    set_motors(0, 0)
    time.sleep(0.1)
    LL = sensor_ll.value
    L = sensor_l.value
    R = sensor_r.value
    RR = sensor_rr.value
    
    if direction == 'left':
        print(f"Executing 90-degree left turn, current IR values: LL={LL}, L={L}, R={R}, RR={RR}")
        set_motors(-PIVOT_SPEED, PIVOT_SPEED)
        time.sleep(0.5)
    elif direction == 'right':
        print(f"Executing 90-degree right turn, current IR values: LL={LL}, L={L}, R={R}, RR={RR}")
        set_motors(PIVOT_SPEED, -PIVOT_SPEED)
        time.sleep(0.5)
    
    set_motors(0, 0)
    time.sleep(0.1)
    print(f"90-degree {direction} turn completed")
    current_state = STATE_TRACKING

# ==========================================
# 5. 锟斤拷循锟斤拷锟竭硷拷锟斤拷? 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟节讹拷锟斤拷转锟斤拷=3锟斤拷1锟斤拷直锟斤拷路锟节ｏ拷
# ==========================================
def start_tracking_mission():
    global current_state, cross_count, cross_detected_counter
    global need_second_turn, second_turn_dir
    current_state = STATE_TRACKING
    cross_count = 0
    cross_detected_counter = 0
    need_second_turn = False
    second_turn_dir = ""
    print("Line tracking mission started")
    
    try:
        while True:
            if current_state == STATE_FINISHED:
                break
            
            # 实时锟斤拷取锟斤拷路锟斤拷锟斤拷
            LL = sensor_ll.value
            L = sensor_l.value
            R = sensor_r.value
            RR = sensor_rr.value
            # 锟斤拷锟斤拷锟解到锟斤拷锟竭碉拷锟斤拷锟斤拷锟斤拷0=锟斤拷獾斤拷锟?=未锟斤拷獾斤拷锟?            black_count = sum([1 for val in [LL, L, R, RR] if val == 0])
            
            if current_state == STATE_TRACKING:
                # ======================
                # ? 锟斤拷锟侥ｏ拷A/C锟节讹拷锟斤拷转锟戒触锟斤拷锟斤拷3锟斤拷1锟斤拷=直锟斤拷路锟节ｏ拷
                # ======================
                if need_second_turn and black_count == 3:
                    print(f"Right-angle junction detected (3 black 1 white), start second turn! IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                    set_motors(BASE_SPEED, BASE_SPEED)
                    time.sleep(0.3)
                    turn_90_degree(second_turn_dir)
                    need_second_turn = False
                    continue  # 转锟斤拷锟斤拷桑锟斤拷锟斤拷锟斤拷锟角把拷锟?                
                # 原锟斤拷十锟斤拷路锟节硷拷猓拷锟斤拷锟斤拷诘锟揭伙拷锟阶拷洌?                if LL == 0 and L == 0 and R == 0 and RR == 0:
                    cross_detected_counter += 1
                    if cross_detected_counter >= 2:
                        cross_count += 1
                        print(f"Valid cross junction detected, total cross count: {cross_count}, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        cross_detected_counter = 0
                        time.sleep(0.03)

                        # 锟斤拷一锟轿凤拷锟斤拷选锟今（第讹拷锟斤拷全锟斤拷十锟斤拷路锟节ｏ拷
                        if cross_count == 1:
                            print("1st cross junction: Entered unloading tracking area, go straight")
                            set_motors(BASE_SPEED, BASE_SPEED)
                            time.sleep(0.2)

                        elif cross_count == 2:
                            print("2nd cross junction: Execute first direction selection")
                            if target_unload_id == 0:
                                # A锟斤拷锟斤拷锟斤拷一锟斤拷锟斤拷转 + 锟斤拷锟斤拷锟揭拷诙锟斤拷锟街憋拷锟阶拷锟?                                set_motors(BASE_SPEED, BASE_SPEED)
                                time.sleep(0.3)
                                turn_90_degree('left')
                                need_second_turn = True
                                second_turn_dir = "right"
                            elif target_unload_id == 1:
                                # B锟斤拷锟斤拷直锟叫ｏ拷锟睫第讹拷锟斤拷转锟斤拷
                                print("2nd cross junction: Target B, go straight")
                                set_motors(BASE_SPEED, BASE_SPEED)
                                time.sleep(0.2)
                            elif target_unload_id == 2:
                                set_motors(BASE_SPEED, BASE_SPEED)
                                time.sleep(0.3)
                                # C锟斤拷锟斤拷锟斤拷一锟斤拷锟斤拷转 + 锟斤拷锟斤拷锟揭拷诙锟斤拷锟街憋拷锟阶拷锟?                                turn_90_degree('right')
                                need_second_turn = True
                                second_turn_dir = "left"
                            elif cross_count == 3:
                                print("3rd cross junction: Arrived at unloading area front line, keep going")
                                set_motors(BASE_SPEED, BASE_SPEED)
                                time.sleep(0.2)

                        elif cross_count == 3:
                            print("3th cross junction: Arrived at unloading area back line")
                            set_motors(0, 0)
                            time.sleep(0.1)
                            print("Move forward to adjust unloading position")
                            set_motors(BASE_SPEED, BASE_SPEED)
                            time.sleep(1.2)
                            set_motors(0, 0)
                            time.sleep(0.1)
                            current_state = STATE_ARRIVED
                else:
                    cross_detected_counter = 0
                    # 锟斤拷锟斤拷循锟斤拷
                    if LL == RR and L == 1 and R == 1:
                        print(f"Line tracking: Go straight, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        set_motors(BASE_SPEED, BASE_SPEED)
                    elif LL == 1  and L == 1 and R == 0 and RR == 1:
                        print(f"Line tracking: Correct left, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        set_motors(OUTER, INNER)
                    elif LL == 1  and L == 0 and R == 1 and RR == 1:
                        print(f"Line tracking: Correct right, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        set_motors(INNER, OUTER)
                    elif LL == 0 and R == 1 and L == 1 and RR == 1:
                        print(f"Line tracking: Big correct left, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        set_motors(INNER, OUTER) 
                    elif LL == 1 and R == 1 and L == 1 and RR == 0:
                        print(f"Line tracking: Big correct right, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        set_motors(OUTER, INNER) 
                    else:
                        print(f"Line tracking: Line lost, keep straight, IR values: LL={LL}, L={L}, R={R}, RR={RR}")
                        set_motors(BASE_SPEED, BASE_SPEED)
            
            # 卸锟斤拷锟斤拷锟斤拷
            elif current_state == STATE_ARRIVED:
                current_state = STATE_UNLOADING
                print("Start unloading process")
                slow_move(-35)
                time.sleep(2)
                slow_move(-12)
                print("Unloading process completed")
                current_state = STATE_FINISHED
            
            time.sleep(0.005)
    
    except KeyboardInterrupt:
        set_motors(0, 0)
        print("\nProgram stopped by user, motors off")
        current_state = STATE_FINISHED

# ==========================================
# 6. 锟斤拷锟斤拷锟斤拷锟斤拷锟?# ==========================================
if __name__ == "__main__":
    print("========== Unloading Line Tracking System Start ==========")
    serial_ok = init_serial()
    servo_ok = init_servo()
    if not serial_ok or not servo_ok:
        print("Hardware init failed, program exit")
        if ser and ser.is_open: ser.close()
        if gpio_handle: lgpio.gpiochip_close(gpio_handle)
        exit(1)
    
    set_motors(0, 0)
    time.sleep(1)
    
    picam2, detector = init_camera()
    if not picam2 or not detector:
        print("Camera init failed, program exit")
        set_motors(0, 0)
        ser.close()
        lgpio.gpiochip_close(gpio_handle)
        exit(1)
    
    tag_detected = detect_apriltag(picam2, detector)
    if not tag_detected or target_unload_id is None:
        print("No valid tag detected, program exit")
        set_motors(0, 0)
        ser.close()
        lgpio.gpiochip_close(gpio_handle)
        exit(1)
    
    time.sleep(0.5)
    start_tracking_mission()
    
    set_motors(0, 0)
    ser.close()
    lgpio.gpiochip_close(gpio_handle)
    print("========== Mission Completed, Program Exit ==========")