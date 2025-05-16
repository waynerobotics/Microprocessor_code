from roboclaw_3 import Roboclaw
import time

# Define serial port for Linux (typically /dev/ttyACM0 or /dev/ttyS*)
SERIAL_PORT = "/dev/ttyACM2"  # Common Linux serial port for USB devices
BAUD_RATE = 38400
ADDRESS = 0x80  # Default Roboclaw address

# Initialize Roboclaw instance
roboclaw = Roboclaw(SERIAL_PORT, BAUD_RATE)

# Open serial connection
if roboclaw.Open():
    print("Connected to Roboclaw successfully!")
else:
    print("Failed to connect to Roboclaw")
    exit()

def read_motor_data():
    """Reads and displays motor encoder values and speed."""
    enc1 = roboclaw.ReadEncM1(ADDRESS)
    enc2 = roboclaw.ReadEncM2(ADDRESS)
    speed1 = roboclaw.ReadSpeedM1(ADDRESS)
    speed2 = roboclaw.ReadSpeedM2(ADDRESS)

    if enc1[0] and enc2[0] and speed1[0] and speed2[0]:
        print(f"Motor 1 - Encoder: {enc1[1]}, Speed: {speed1[1]}")
        print(f"Motor 2 - Encoder: {enc2[1]}, Speed: {speed2[1]}")
        print("-----------------------------------------")
    else:
        print("Failed to read motor data")

try:
    while True:
        read_motor_data()
        time.sleep(0.1)  # 100ms delay between readings
except KeyboardInterrupt:
    print("\nExiting program")
finally:
    roboclaw.Close()