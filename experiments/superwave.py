import serial
import time
import json
import os
from collections import deque
from gpiozero import DistanceSensor, DigitalInputDevice

# 锟斤拷锟斤拷锟斤拷莓锟斤拷5
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

# ==========================================
# 1. 锟斤拷锟侥诧拷锟斤拷 (锟斤拷锟斤拷锟狡碉拷械锟秸拷锟斤拷呕锟?
# ==========================================
SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

# 锟劫度硷拷 - 锟斤拷锟斤拷锟劫讹拷锟斤拷锟斤拷弑锟斤拷锟酵拷锟斤拷锟?SPEED_STRAIGHT = 0.15      # 锟斤拷锟斤拷探锟斤拷
SPEED_NAV_FAST = 0.32      # 锟秸硷拷寻路锟斤拷锟斤拷 (锟斤拷一锟斤拷锟街顾ν诽拷锟?
SPEED_NAV_SLOW = 0.04      # 锟秸硷拷寻路锟斤拷锟斤拷
SPEED_EMG_FAST = 0.4      # 锟斤拷锟斤拷锟斤拷锟秸匡拷锟斤拷
SPEED_EMG_SLOW = -0.10     # 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 (锟斤拷转强锟斤拷锟斤拷锟斤拷)
SPEED_PIVOT = 0.35         

# 锟斤拷值锟斤拷 (cm) - 45锟饺诧拷锟斤拷专锟斤拷
THRESH_FRONT = 11.0        # 前锟斤拷锟斤拷前预锟斤拷
THRESH_EMG = 12.0           # 锟斤拷呓锟斤拷锟斤拷锟斤拷锟?(45锟饺讹拷锟斤拷)
THRESH_DIFF = 3          # 寻路锟斤拷锟斤拷锟斤拷 (锟斤拷锟斤拷锟街蛊碉拷锟脚わ拷锟?
MAX_VALID = 40.0           # 锟斤拷野锟斤拷围

# ==========================================
# 2. 硬锟斤拷锟洁定锟斤拷
# ==========================================

class RobustUltra:
    """锟斤拷锟斤拷锟斤拷锟剿诧拷锟斤拷盲锟斤拷锟斤拷锟斤拷锟侥筹拷锟斤拷锟斤拷锟斤拷"""
    def __init__(self, t, e):
        self.sensor = DistanceSensor(echo=e, trigger=t, queue_len=1)
        self.history = deque(maxlen=3) # 锟斤拷锟斤拷锟斤拷锟斤拷为3

    def get(self):
        try:
            val = self.sensor.distance * 100
            # --- 锟睫革拷锟斤拷锟斤拷 ---
            # 锟斤拷锟斤拷锟斤拷锟教★拷锟斤拷锟斤拷锟叫★拷锟?2cm锟斤拷锟斤拷说锟斤拷锟斤拷锟斤拷锟斤拷盲锟斤拷锟斤拷锟杰碉拷锟剿革拷锟斤拷
            # 锟斤拷锟斤拷直锟接凤拷锟斤拷一锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷呒锟斤拷筒锟斤拷锟斤拷锟斤拷锟斤拷锟缴★拷锟较帮拷锟斤”
            if val < 2.1: 
                return 999.0 
            
            self.history.append(val)
        except:
            return 999.0
        return sum(self.history)/len(self.history) if self.history else 999.0

# 锟斤拷始锟斤拷
ultra_f = RobustUltra(23, 24)
ultra_l = RobustUltra(17, 27)
ultra_r = RobustUltra(22, 25)
sensor_l = DigitalInputDevice(5)
sensor_r = DigitalInputDevice(6)

ser = None
def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        ser.setRTS(False); ser.setDTR(False)
        return True
    except: return False

def set_motors(l, r):
    if ser and ser.is_open:
        l, r = max(-0.5, min(0.5, l)), max(-0.5, min(0.5, r))
        data = {"T": 1, "L": round(l, 2), "R": round(r, 2)}
        ser.write((json.dumps(data) + '\n').encode('utf-8'))

def perform_pivot(direction):
    """强锟斤拷原锟斤拷锟斤拷转锟斤拷锟斤拷"""
    set_motors(-0.2, -0.2) # 锟斤拷锟斤拷刹锟斤拷
    time.sleep(0.05)
    set_motors(0, 0); time.sleep(0.1)
    
    print(f"[ACTION] PIVOT 90 to {direction}")
    if direction == "LEFT": set_motors(-SPEED_PIVOT, SPEED_PIVOT)
    else: set_motors(SPEED_PIVOT, -SPEED_PIVOT)
    
    time.sleep(0.6) # 锟斤拷校锟斤拷锟绞憋拷锟街憋拷锟斤拷锟阶?0锟斤拷
    set_motors(0, 0); time.sleep(0.3)

# ==========================================
# 3. 锟斤拷锟斤拷锟斤拷锟竭硷拷
# ==========================================

def start_avoidance():
    first_pivot_done = False
    last_l, last_r = 15.0, 15.0
    
    print("[RUN] Safe Mode Activated.")
    
    try:
        while True:
            # 1. 锟缴硷拷锟斤拷锟斤拷锟斤拷
            df = ultra_f.get()
            raw_l = ultra_l.get()
            raw_r = ultra_r.get()

            # 22cm 锟斤拷围锟斤拷锟斤拷
            if 2.1 <= raw_l <= MAX_VALID: 
                last_l = raw_l
            
            if 2.1 <= raw_r <= MAX_VALID: 
                last_r = raw_r
            
            dl, dr = round(last_l, 1), round(last_r, 1)

            # 锟斤拷印锟斤拷志锟斤拷锟节观诧拷盲锟斤拷
            print(f"F:{df:.1f} | L:{dl} | R:{dr}")

            # --- 锟竭硷拷 1: 前锟斤拷预锟斤拷 (锟斤拷锟斤拷锟斤拷燃锟? ---
            if df < THRESH_FRONT:
                print(f"[TRIGGER] FRONT BLOCK: {df:.1f}")
                if not first_pivot_done:
                    perform_pivot("LEFT")
                    # 突锟斤拷时锟斤拷锟斤拷锟教碉拷 0.8s锟斤拷锟斤拷止锟斤拷锟斤拷锟斤拷
                    set_motors(SPEED_STRAIGHT, SPEED_STRAIGHT)
                    time.sleep(0.8)
                    first_pivot_done = True
                else:
                    if dl > dr: perform_pivot("LEFT")
                    else: perform_pivot("RIGHT")
                continue

            # --- 锟竭硷拷 2: 锟斤拷弑锟斤拷锟?(锟斤拷一转锟斤拷珊蠹せ锟? ---
            if first_pivot_done:
                # A. 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 (<= 7cm 45锟饺诧拷锟?
                if dr <= THRESH_EMG:
                    print(f"[EMG] TOO CLOSE RIGHT: {dr}")
                    set_motors(SPEED_EMG_SLOW, SPEED_EMG_FAST)
                elif dl <= THRESH_EMG:
                    print(f"[EMG] TOO CLOSE LEFT: {dl}")
                    set_motors(SPEED_EMG_FAST, SPEED_EMG_SLOW)

                # B. 寻路锟斤拷 (锟饺斤拷锟斤拷锟揭诧拷值)
                elif dl > dr + THRESH_DIFF:
                    print(f"[NAV] Pushing LEFT")
                    set_motors(SPEED_NAV_SLOW, SPEED_NAV_FAST)
                elif dr > dl + THRESH_DIFF:
                    print(f"[NAV] Pushing RIGHT")
                    set_motors(SPEED_NAV_FAST, SPEED_NAV_SLOW)
                
                # C. 锟斤拷全直锟斤拷
                else:
                    set_motors(SPEED_STRAIGHT, SPEED_STRAIGHT)
            
            else:
                print()
                set_motors(SPEED_STRAIGHT, SPEED_STRAIGHT)

            # --- 锟剿筹拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟竭诧拷执锟斤拷 90锟斤拷锟斤拷转锟斤拷尾 ---
            if (sensor_l.value == 0 or sensor_r.value == 0) and first_pivot_done==True:
                print("[DONE] Line captured! Executing final turn...\n")
                print(sensor_l.value,sensor_r.value)
                # 1. 锟斤拷锟斤拷全停锟斤拷
                set_motors(0, 0)
                time.sleep(0.2)
                
                # 2. 执锟斤拷原锟斤拷锟斤拷转 90锟斤拷 (使锟斤拷锟斤拷之前锟借定锟斤拷锟劫度猴拷时锟斤拷)
                print("[ACTION] Final 90锟斤拷 LEFT turn to align with line")
                set_motors(-SPEED_PIVOT, SPEED_PIVOT)
                time.sleep(0.6) # 锟斤拷锟?90锟斤拷转锟斤拷准锟斤拷锟斤拷锟斤拷微锟斤拷锟斤拷锟?0.6
                
                # 3. 锟斤拷锟斤拷停止
                set_motors(0, 0)
                print("[FINISH] Car stopped. Ready for tracking mode.")
                break
            time.sleep(0.02)


    except KeyboardInterrupt:
        set_motors(0, 0)

if __name__ == "__main__":
    if init_serial():
        time.sleep(1)
        start_avoidance()