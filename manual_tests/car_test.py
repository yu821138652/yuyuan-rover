import serial
import threading
import time
import json

# Global serial port object
ser = None

def read_serial():
    """Read and print feedback data from the lower computer"""
    while True:
        if ser and ser.is_open:
            try:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    print(f"[Feedback] {data}")
            except Exception as e:
                print(f"Serial read error: {e}")
                break
        else:
            break

def send_json_command(cmd_dict):
    """Send JSON format command to the lower computer"""
    if ser and ser.is_open:
        cmd_str = json.dumps(cmd_dict) + '\n'
        ser.write(cmd_str.encode('utf-8'))
        print(f"[Sent] {cmd_str.strip()}")

# ---------------------- Chassis Control Functions ----------------------
def move_forward(speed=0.3):
    """Move forward"""
    send_json_command({"T": 1, "L": speed, "R": speed})

def move_backward(speed=0.3):
    """Move backward"""
    send_json_command({"T": 1, "L": -speed, "R": -speed})

def turn_left_in_place(speed=0.3):
    """Spin left in place"""
    send_json_command({"T": 1, "L": -speed, "R": speed})

def turn_right_in_place(speed=0.3):
    """Spin right in place"""
    send_json_command({"T": 1, "L": speed, "R": -speed})

def stop_chassis():
    """Emergency stop"""
    send_json_command({"T": 1, "L": 0, "R": 0})

# ---------------------- Automatic Test Process ----------------------
def auto_test_chassis():
    """Automatic chassis movement test"""
    print("=== Raspberry Pi 5 USB Car Automatic Test Started ===")
    try:
        # Move forward for 2 seconds
        print("\n1. Move forward 2s")
        move_forward(0.3)
        time.sleep(2)
        stop_chassis()
        time.sleep(1)

        # Move backward for 2 seconds
        print("\n2. Move backward 2s")
        move_backward(0.3)
        time.sleep(2)
        stop_chassis()
        time.sleep(1)

        # Spin left for 1 second
        print("\n3. Spin left 1s")
        turn_left_in_place(0.3)
        time.sleep(1)
        stop_chassis()
        time.sleep(1)

        # Spin right for 1 second
        print("\n4. Spin right 1s")
        turn_right_in_place(0.3)
        time.sleep(1)
        stop_chassis()

        print("\n=== Automatic Test Completed ===")

    except KeyboardInterrupt:
        print("\nTest interrupted, car stopped!")
        stop_chassis()
    except Exception as e:
        print(f"Error: {e}")
        stop_chassis()

def main():
    global ser
    # Fixed USB serial port for your device /dev/ttyUSB0
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 115200  # Standard baud rate

    try:
        # Initialize USB serial port, prevent lower computer reset
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=1,
            dsrdtr=None
        )
        ser.setRTS(False)
        ser.setDTR(False)

        print(f"✅ USB Serial Connected: {SERIAL_PORT}")

        # Start serial receiving thread
        threading.Thread(target=read_serial, daemon=True).start()

        # Run automatic test
        auto_test_chassis()

        # Enter manual control mode
        print("\nEnter manual mode, type commands to control (type exit to quit)")
        while True:
            cmd = input("> ")
            if cmd.lower() == "exit":
                break
            ser.write((cmd + "\n").encode("utf-8"))

    except Exception as e:
        print(f"❌ Connection failed: {e}")
    finally:
        # Safe exit: stop car and close serial port
        if ser and ser.is_open:
            stop_chassis()
            ser.close()
            print("✅ Car stopped, serial port closed")

if __name__ == "__main__":
    main()