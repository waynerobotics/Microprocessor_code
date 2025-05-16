# This script reads data from an ESP32 microcontroller via serial communication.
# It parses the data to extract RPM, direction, and displacement values for two encoders.
import serial
import time

def parse_serial_data(line):
    data = {}
    parts = line.strip().split(',')
    for part in parts:
        key, value = part.split(':')
        if key in ['E1_RPM', 'E2_RPM']:
            data[key] = float(value)
        elif key in ['E1_DISP', 'E2_DISP']:
            data[key] = float(value)
        elif key in ['E1_DIR', 'E2_DIR']:
            data[key] = value
    return data

# Find your ESP32's serial port (e.g., /dev/ttyUSB0 or /dev/ttyACM0)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            try:
                data = parse_serial_data(line)
                print(f"Encoder 1 - RPM: {data['E1_RPM']:.2f}, Direction: {data['E1_DIR']}, Displacement: {data['E1_DISP']:.3f} m")
                print(f"Encoder 2 - RPM: {data['E2_RPM']:.2f}, Direction: {data['E2_DIR']}, Displacement: {data['E2_DISP']:.3f} m\n")
            except (KeyError, ValueError) as e:
                print(f"Error parsing data: {e} | Raw line: {line}")

        time.sleep(0.01)

except KeyboardInterrupt:
    ser.close()
    print("\nSerial connection closed.")