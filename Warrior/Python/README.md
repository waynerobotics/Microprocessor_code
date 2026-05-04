# Warrior Serial Bridge

Python-side support for the Warrior microcontroller stack. Today this is a
single-process bridge that forwards control messages between Arduinos over USB
serial. Tomorrow it will be a set of ROS 2 nodes — see
[ROS 2 conversion prompt](#ros-2-conversion-prompt) at the bottom.

---

## Architecture

Two Arduinos, one PC bridge.

```
┌──────────────────┐    USB     ┌─────────────────┐    USB    ┌──────────────────┐
│ 00_base          │  <MOT,…>   │ serial_bridge   │  <MOT,…>  │ 02_swerve        │
│ Mega + LCD/keypad│ ─────────> │  (this code)    │ ────────> │ Nano ESP32       │
│ 09_LCD_Keypad    │            │                 │           │ 04_set_pwm       │
│ generates motor  │            │                 │           │ maps to PWM,     │
│ cmds @20 Hz      │            │                 │           │ outputs on D6/D7 │
└──────────────────┘            └─────────────────┘           └──────────────────┘
```

`00_base` is the input device (LCD keypad shield, eventually an SBUS receiver,
joystick, etc.). `02_swerve` is the output device — it receives PWM commands
and drives ESCs/servos via the ESP32Servo library. The Python bridge is the
glue.

The bridge auto-discovers both Arduinos by **device name** (not COM port), so
USB enumeration order doesn't matter.

---

## Wire protocol

Every message is ASCII, framed by `<` and `>`, terminated with `\n`:

```
<TYPE,field1,field2,...>
```

Defined in `Warrior/libraries/SerialProtocol/`. Both directions use the same
format. Known types:

| Direction        | Message                                  | Meaning                                       |
| ---------------- | ---------------------------------------- | --------------------------------------------- |
| host → device    | `<WHO>`                                  | Ask device to identify itself                 |
| host → device    | `<PING>`                                 | Liveness probe                                |
| host → 02_swerve | `<MOT,spark,flipsky>`                    | Set motor command, normalized -100..100       |
| device → host    | `<NAME,name>`                            | Reply to WHO                                  |
| device → host    | `<ACK,type>`                             | Generic ack (e.g. `<ACK,PONG>` for PING)      |
| device → host    | `<ERR,reason>`                           | Error                                         |
| 00_base → host   | `<MOT,spark,flipsky>`                    | Generated motor command (forwarded to 02_swerve)|
| reserved         | `<CTRL,ail,ele,thr,rud,swA,btn,swB,knob>`| Controller state                              |
| reserved         | `<FBK,driveVel,steerVel,steerPos>`       | Motor feedback                                |

Devices identify themselves with stable string names:

- `00_base` — LCD keypad / input device
- `02_swerve` — ESP32 PWM output device

Names are constants at the top of each `.ino` (`const char* DEVICE_NAME = …;`).

Safety: `02_swerve` returns motors to neutral (1500 µs) if no `<MOT,…>` is
received for 500 ms. Whatever produces motor commands MUST send at ≥ 2 Hz.

---

## Files

| File                    | Purpose                                                        |
| ----------------------- | -------------------------------------------------------------- |
| `serial_protocol.py`    | `WarriorSerial` class + `query_device_name` helper             |
| `serial_bridge.py`      | The forwarding daemon. Auto-discovers, reconnects on drop      |
| `find_arduino_ports.py` | Standalone port-scanning utility                               |
| `test_serial_protocol.py` | Unit tests for the framing/parsing                           |
| `requirements.txt`      | `pyserial`                                                     |

---

## Running

### Windows

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python .\serial_bridge.py
```

### Ubuntu 22.04 (x64 or ARM)

One-time setup — add yourself to `dialout` so you can open `/dev/ttyACM*`:

```bash
sudo usermod -aG dialout $USER
# log out, log back in (or reboot)
```

Then:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python serial_bridge.py
```

### CLI options

```
python serial_bridge.py                      # auto-discover both ports
python serial_bridge.py --base COM5 --swerve COM7
python serial_bridge.py --base /dev/ttyUSB0 --swerve /dev/ttyACM0
python serial_bridge.py --scan               # list every port and its DEVICE_NAME
```

The bridge runs forever — closing/unplugging Arduinos triggers re-discovery,
not an exit. Stop with Ctrl+C.

---

## Cross-platform notes

- pyserial abstracts over Win/Linux/macOS port naming. The same Python code
  runs everywhere.
- Both Arduinos auto-reset when the host opens the serial port (DTR toggle).
  `WarriorSerial.open()` sleeps 2 s after open to wait for that reset to
  complete. Don't shorten this without testing.
- Both `00_base` and `02_swerve` may stream messages continuously. The
  `query_device_name` helper sends `<WHO>` and then **filters** for `<NAME,…>`
  — it ignores other messages until the deadline. Don't go back to "trust the
  first byte" or you'll mistake a stray `<MOT,…>` for the device name.
- On Linux the `dialout` group requirement is the #1 footgun.

---

## Known caveats

1. Discovery sends `<WHO>` to every COM port, including non-Arduino devices.
   Most just time out, but a misbehaving device that responds to arbitrary
   bytes could confuse things. Mitigation: only ports that send a properly
   framed `<NAME,…>` reply are accepted.
2. The `00_base` and `02_swerve` Megas auto-reset on every serial open. If
   you reopen mid-session (e.g. to update firmware) you'll briefly lose PWM
   output. The 500 ms safety watchdog on `02_swerve` covers this — motors
   return to 1500 µs.
3. There is no flow control. If `02_swerve` is overwhelmed, messages are
   dropped silently. The 20 Hz send rate is conservative enough that this
   hasn't been observed.
4. The bridge does not parse `<NAME>`/`<ACK>`/`<ERR>` from `02_swerve` —
   responses fill its TX buffer indefinitely (see comment in
   `04_set_pwm.ino:71`). If you start consuming them, drain the buffer too.

---

## ROS 2 conversion prompt

> Paste this entire section into the new repo's Claude Code / Cursor session.
> It is self-contained — assume the reader has not seen the `Warrior/Python`
> code.

### Goal

Convert the existing single-process Python serial bridge described below into
a set of ROS 2 (Humble) Python nodes. Target Ubuntu 22.04 on x64 and ARM.
Maintain compatibility with the existing Arduino firmware — DO NOT change the
wire protocol.

### Hardware context

Two Arduino-family microcontrollers connected via USB to a Linux host:

1. **`00_base`** — Arduino Mega 2560 with an LCD Keypad Shield. Generates
   motor commands at 20 Hz and streams them as `<MOT,spark,flipsky>` over USB
   serial. Spark and flipsky are normalized integer commands in -100..100,
   where 0 = neutral.
2. **`02_swerve`** — Arduino Nano ESP32. Reads `<MOT,spark,flipsky>` over
   serial (values normalized -100..100, where 0 = neutral). Maps to RC PWM
   1000–2000 µs and outputs on GPIO 9 (D6, "spark") and GPIO 10 (D7,
   "flipsky") via the ESP32Servo library. Has a 500 ms safety watchdog —
   returns to 1500 µs neutral if no message arrives in time.

Both devices use the same line-based ASCII framing. Every message is
`<TYPE,field1,field2,...>` followed by `\n`.

### Wire protocol (cannot change)

| Direction        | Message                          | Meaning                          |
| ---------------- | -------------------------------- | -------------------------------- |
| host → device    | `<WHO>`                          | Ask device to identify itself    |
| host → device    | `<PING>`                         | Liveness probe                   |
| host → 02_swerve | `<MOT,spark,flipsky>`            | Motor command, normalized -100..100 |
| device → host    | `<NAME,name>`                    | Reply to WHO                     |
| device → host    | `<ACK,type>`                     | Acknowledgement                  |
| device → host    | `<ERR,reason>`                   | Error                            |
| 00_base → host   | `<MOT,spark,flipsky>`            | Outgoing motor command stream    |

Discovery: send `<WHO>`, read messages until `<NAME,xxx>` appears (filtering
out other traffic), match against expected name. Both `00_base` and
`02_swerve` are valid responses.

### Constraints

- **Auto-reset**: opening the serial port toggles DTR and resets the Arduino.
  Wait 2 s after open before sending. This is non-negotiable.
- **Linux permissions**: include setup docs for `sudo usermod -aG dialout
  $USER`.
- **Continuous streams**: `00_base` streams `<MOT,…>` at 20 Hz. The discovery
  helper must skip these and look only for `<NAME,…>` replies.
- **Watchdog**: any node publishing motor commands must send at ≥ 2 Hz, or
  the ESP32 will fail-safe to 1500 µs. This is a feature — preserve it.
- **Reconnect**: USB drops should not crash nodes. Re-discover and reconnect.
- **No backwards compat shims**: this is a fresh ROS 2 package. Don't carry
  over the old `serial_bridge.py` structure.

### Suggested architecture

Two driver nodes; the Python "bridge" concept goes away:

```
┌─────────────────────────┐                ┌──────────────────────────┐
│ warrior_base_driver     │  topic:        │ warrior_swerve_driver    │
│ - opens 00_base by name │  /motor_cmd    │ - opens 02_swerve by name│
│ - parses incoming       │  ───────────>  │ - subscribes /motor_cmd  │
│   <MOT,…> from device   │  warrior_msgs/ │ - sends <MOT,…> to device│
│ - publishes /motor_cmd  │  MotorCommand  │                          │
└─────────────────────────┘                └──────────────────────────┘
```

Decoupling via the `/motor_cmd` topic means future input sources (SBUS, Xbox
controller, autonomy stack) can publish to the same topic without touching
the swerve driver.

Custom message recommendation:

```
# warrior_msgs/msg/MotorCommand.msg
int8 spark      # -100..100, 0 = neutral
int8 flipsky    # -100..100, 0 = neutral
```

The microsecond mapping (`-100..100 -> 1000..2000 µs`) lives in the firmware
on `02_swerve`, not in the ROS 2 layer. Keep it that way — it lets you
re-calibrate ESC ranges in firmware without touching the control stack.

### ROS 2 parameters per node

- `device_name` (string, required) — e.g. `"00_base"` or `"02_swerve"`
- `baud_rate` (int, default 115200)
- `discovery_retry_period_s` (double, default 2.0)
- `read_timeout_s` (double, default 0.1)

### Implementation notes

- Each node's main work happens in a timer callback (not a blocking thread).
  Use a small read timeout (0.05–0.1 s) inside a periodic spin so callbacks
  stay responsive.
- Wrap port discovery in a state machine: `DISCONNECTED → DISCOVERING →
  CONNECTED → DISCONNECTED`. Re-enter discovery on any read/write exception.
- Use `rclpy`'s logging (`self.get_logger().info(...)`), not `print`.
- Provide a `launch/warrior_drivers.launch.py` that brings up both drivers.
- Include a `package.xml` with `rclpy`, `pyserial`, and `warrior_msgs` deps.

### Testing

- `pytest` for the framing/parser logic — port over the existing
  `test_serial_protocol.py` style.
- A "loopback" test that runs both nodes against `pyserial`'s loopback or a
  null-modem pipe, asserting that publishing on `/motor_cmd` produces the
  expected bytes on the swerve serial port.
- Manual integration test with real hardware: `ros2 topic pub /motor_cmd
  warrior_msgs/msg/MotorCommand "{spark: 0, flipsky: 0}"` and observe motor
  behaviour.

### Reference implementation in the legacy repo

The existing single-process Python at `Warrior/Python/` is a working
reference for the wire protocol and discovery logic. Read these in order:

1. `serial_protocol.py` — `WarriorSerial` class, `query_device_name` helper
2. `serial_bridge.py` — discovery + reconnect state machine
3. `Warrior/libraries/SerialProtocol/src/SerialProtocol.{h,cpp}` —
   firmware-side framing
4. `Warrior/sketches/tests/09_LCD_Keypad_Shield/09_LCD_Keypad_Shield.ino` —
   `00_base` reference
5. `Warrior/sketches/tests/04_set_pwm/04_set_pwm.ino` — `02_swerve` reference

Do not vendor the legacy Python — re-implement cleanly inside the ROS 2
package layout. Reuse only the framing/parsing helpers, ported to a
`warrior_serial` Python module under the new package.
