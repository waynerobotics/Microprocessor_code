"""
serial_bridge.py

Reads <PWM,spark,flipsky> messages from the 00_base Arduino (LCD Keypad Shield)
and forwards them to the 02_swerve Arduino (PWM output).

Usage:
    python serial_bridge.py                        # auto-discover both ports
    python serial_bridge.py --base COM5 --swerve COM7   # manual override
    python serial_bridge.py --scan                 # just list what's on each port
"""

import argparse
import time
from serial.tools import list_ports
from serial_protocol import WarriorSerial, query_device_name


BAUDRATE = 115200
QUERY_TIMEOUT = 3.0
READ_TIMEOUT  = 0.1


def scan_ports() -> None:
    """Print every COM port and what device name it reports."""
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print(f"Found {len(ports)} port(s):")
    for p in ports:
        print(f"  {p.device}  ({p.description})", end="  →  ")
        try:
            name = query_device_name(p.device, BAUDRATE, timeout=QUERY_TIMEOUT)
            print(name if name else "<no response>")
        except Exception as exc:
            print(f"<error: {exc}>")


def find_port(device_name: str) -> str:
    """Auto-discover a port by querying every available COM port."""
    for p in list_ports.comports():
        print(f"  Querying {p.device} ({p.description})...", end=" ", flush=True)
        try:
            name = query_device_name(p.device, BAUDRATE, timeout=QUERY_TIMEOUT)
            print(name if name else "<no response>")
            if name and name.strip() == device_name:
                return p.device
        except Exception as exc:
            print(f"<error: {exc}>")
    raise RuntimeError(f"Arduino '{device_name}' not found on any port")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base",   help="COM port for 00_base (e.g. COM5)")
    parser.add_argument("--swerve", help="COM port for 02_swerve (e.g. COM7)")
    parser.add_argument("--scan",   action="store_true", help="Scan all ports and exit")
    args = parser.parse_args()

    if args.scan:
        scan_ports()
        return

    if args.base and args.swerve:
        base_port   = args.base
        swerve_port = args.swerve
        print(f"Using manual ports: base={base_port}  swerve={swerve_port}")
    else:
        print("Auto-discovering Arduino devices...")
        base_port   = args.base   or find_port("00_base")
        swerve_port = args.swerve or find_port("02_swerve")

    print(f"  00_base   → {base_port}")
    print(f"  02_swerve → {swerve_port}")

    forwarded = 0
    last_status_time = time.time()

    base   = WarriorSerial(base_port,   BAUDRATE, timeout=READ_TIMEOUT)
    swerve = WarriorSerial(swerve_port, BAUDRATE, timeout=READ_TIMEOUT)

    def open_both() -> None:
        for label, conn, port in (("00_base", base, base_port), ("02_swerve", swerve, swerve_port)):
            while True:
                try:
                    conn.open()
                    print(f"  Connected: {label} on {port}")
                    break
                except Exception as exc:
                    print(f"  Waiting for {label} on {port}: {exc}")
                    time.sleep(2.0)

    open_both()
    print(f"\nBridge running: {base_port} → {swerve_port}")
    print("Press Ctrl+C to stop.\n")

    while True:
        try:
            msg = base.read_message(timeout=READ_TIMEOUT)
        except Exception as exc:
            print(f"\n[DISCONNECT] {exc}")
            try:
                base.close()
            except Exception:
                pass
            try:
                swerve.close()
            except Exception:
                pass
            print("Reconnecting...")
            open_both()
            print("Reconnected.\n")
            continue

        if msg is None:
            continue

        parts = msg.split(",")

        if parts[0] != "PWM" or len(parts) != 3:
            continue

        try:
            swerve.send_message(parts[0], parts[1], parts[2])
        except Exception as exc:
            print(f"\n[SWERVE ERROR] {exc}")
            continue

        forwarded += 1

        now = time.time()
        if now - last_status_time >= 1.0:
            last_status_time = now
            print(f"[{forwarded:6d}] spark={parts[1]} us  flipsky={parts[2]} us")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBridge stopped.")
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
