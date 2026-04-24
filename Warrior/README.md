🛡️ Warrior

Arduino firmware for the Wayne State Warrior1 robot competing in the
Intelligent Ground Vehicle Competition (IGVC).

🚀 Overview

This repository contains all microcontroller code used on the Warrior1 robot.
It is structured to support:

Modular hardware development
Incremental testing on Arduino devices
Clean separation between input, control, and output systems
Simulation and testing from a PC using Python (no ROS required)


🧠 System Architecture

The firmware is organized into reusable libraries and thin Arduino sketches.

Core Layers
RadioLink (Input)
      ↓
SerialProtocol (Communication)
      ↓
Control (PID, logic)
      ↓
MotorControl / LEDControl / FanControl (Outputs)

📁 Repository Structure

Warrior/
├── README.md
│
├── libraries/           # Reusable Arduino libraries (core logic)
│   ├── RadioLink/       # PPM input from RC controller
│   ├── SerialProtocol/  # Serial messaging system
│   ├── MotorControl/    # Motor drivers (SparkMax, Flipsky, etc.)
│   ├── PID/             # Control loops
│   ├── LEDControl/      # LED control
│   └── FanControl/      # Fan / thermal control
│
├── sketches/            # Arduino sketches (entry points)
│   ├── 00_base/         # Main base controller Arduino
│   ├── 01_swerve/       # Swerve module controller (flashed to each module)
│   └── tests/           # Isolated test sketches
│
└── Python/              # PC-side testing tools (no ROS required)
🔧 Hardware Overview
Controller: RadioLink T8S
Receiver: RadioLink R8EF (PPM mode)
MCUs: Arduino Nano ESP32 (or equivalent)
Motor Controllers:
Flipsky (drive motors)
REV Spark MAX (steering motors)


🧪 Test

Every subsystem is testable independently.

Arduino Tests

Located in:

sketches/tests/

Examples:

02_ppm_input_test → verify RC input
03_serial_send_test → verify outgoing messages
04_serial_receive_test → verify incoming commands
05_motor_output_test → verify motor control
Python Tests

Located in:

Python/

Used to simulate:

Controller input
Motor feedback
Serial communication

Example:

python serial_send_mock.py


🔌 Serial Protocol

Communication between systems uses a simple framed ASCII protocol:

<MSGTYPE,data1,data2,...>

Examples:

<CTRL,12,-4,87,0,-100,0,100,35>
<VEL,50,-30>
<POS,90>
<LED,1,0,1>
<FBK,48,-29,91>
< = start
> = end
Comma-separated values