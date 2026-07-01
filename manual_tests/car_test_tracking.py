import serial
import time
import json
from gpiozero import DigitalInputDevice

# ==========================================
# 1. Configuration
# ==========================================
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Motor speed (0.0 to 1.0). 
# Start with low speed for the first test!
BASE_SPEED = 0.3
TURN_SPEED = 0.1

# Initialize sensors
# LEFT = GPIO 6, RIGHT = GPIO 5
sensor_l = DigitalInputDevice(6)
sensor_r = DigitalInputDevice(5)

ser = None

# ==========================================
# 2. Communication Functions
# ==========================================

def init_serial():
    global ser
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
        # Prevents unintended reset
        ser.setRTS(False)
        ser.setDTR(False)
        print(f"Serial initialized on {SERIAL_PORT}")
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        return False

def set_motors(left, right):
    """Sends JSON speed control command"""
    if ser and ser.is_open:
        data = {"T": 1, "L": left, "R": right}
        msg = json.dumps(data) + '\n'
        ser.write(msg.encode('utf-8'))

def stop():
    set_motors(0, 0)

# ==========================================
# 3. Main Logic Loop
# ==========================================

def start_tracking():
    print("Logic started. Use Ctrl+C to stop.")
    try:
        while True:
            # Get current sensor states
            # value = 1 (Black line), value = 0 (White ground)
            L = sensor_l.value
            R = sensor_r.value

            # Condition 1: Straight ahead
            if L == 0 and R == 0:
                set_motors(BASE_SPEED, BASE_SPEED)
            
            # Condition 2: Adjusting left
            elif L == 1 and R == 0:
                set_motors(0.5*TURN_SPEED, 4*TURN_SPEED)
            
            # Condition 3: Adjusting right
            elif L == 0 and R == 1:
                set_motors(4*TURN_SPEED, 0.5*TURN_SPEED)
            
            # Condition 4: Stop at crossing or end
            elif L == 1 and R == 1:
                set_motors(BASE_SPEED, BASE_SPEED)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
        stop()

if __name__ == "__main__":
    if init_serial():
        time.sleep(1)
        start_tracking()
#python3 test_tracking.py