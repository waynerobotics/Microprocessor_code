"""
serial_bridge.py

Reads <PWM,spark,flipsky> messages from the 00_base Arduino (LCD Keypad Shield)
and forwards them to the 02_swerve Arduino (PWM output).

Runs forever — if either Arduino is unplugged or rebooted, the bridge
re-discovers and reconnects automatically. Press Ctrl+C to stop.

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
RETRY_DELAY_S = 2.0


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


def find_port_once(device_name: str, verbose: bool) -> str | None:
    """Scan every COM port once. Return the matching port name, or None."""
    for p in list_ports.comports():
        if verbose:
            print(f"  Querying {p.device} ({p.description})...", end=" ", flush=True)
        try:
            name = query_device_name(p.device, BAUDRATE, timeout=QUERY_TIMEOUT)
            if verbose:
                print(name if name else "<no response>")
            if name and name.strip() == device_name:
                return p.device
        except Exception as exc:
            if verbose:
                print(f"<error: {exc}>")
    return None


def find_port_blocking(device_name: str) -> str:
    """Scan ports repeatedly until the named device is found."""
    print(f"Looking for DEVICE_NAME=\"{device_name}\"...")
    verbose = True
    while True:
        port = find_port_once(device_name, verbose=verbose)
        if port is not None:
            print(f"  FOUND DEVICE_NAME=\"{device_name}\" on {port}")
            return port
        if verbose:
            print(f"  {device_name} not found; will keep retrying every {RETRY_DELAY_S:.0f}s "
                  "(close any open Serial Monitors)")
            verbose = False
        time.sleep(RETRY_DELAY_S)


def open_blocking(conn: WarriorSerial, label: str, port: str) -> None:
    """Open a serial port, retrying forever on permission/access errors."""
    first = True
    while True:
        try:
            conn.open()
            print(f"  Connected: {label} on {port}")
            return
        except Exception as exc:
            if first:
                print(f"  Waiting for {label} on {port}: {exc}")
                first = False
            time.sleep(RETRY_DELAY_S)


def discover_and_open(base_port_arg: str | None,
                      swerve_port_arg: str | None) -> tuple[WarriorSerial, WarriorSerial, str, str]:
    """Resolve both ports (auto-discover or manual) and open them. Blocks until ready."""
    base_port   = base_port_arg   or find_port_blocking("00_base")
    swerve_port = swerve_port_arg or find_port_blocking("02_swerve")

    base   = WarriorSerial(base_port,   BAUDRATE, timeout=READ_TIMEOUT)
    swerve = WarriorSerial(swerve_port, BAUDRATE, timeout=READ_TIMEOUT)

    open_blocking(base,   "00_base",   base_port)
    open_blocking(swerve, "02_swerve", swerve_port)
    return base, swerve, base_port, swerve_port


def safe_close(conn: WarriorSerial) -> None:
    try:
        conn.close()
    except Exception:
        pass


def run_bridge(base: WarriorSerial, swerve: WarriorSerial) -> None:
    """Forward PWM messages from base to swerve. Raises on disconnect."""
    forwarded = 0
    last_log_time = 0.0
    LOG_INTERVAL_S = 2.0

    while True:
        msg = base.read_message(timeout=READ_TIMEOUT)
        if msg is None:
            continue

        parts = msg.split(",")
        if parts[0] != "PWM" or len(parts) != 3:
            print(f"  [skip] non-PWM message from 00_base: <{msg}>")
            continue

        swerve.send_message(parts[0], parts[1], parts[2])
        forwarded += 1

        now = time.time()
        if now - last_log_time >= LOG_INTERVAL_S:
            last_log_time = now
            print(f"[{forwarded:6d}] 00_base -> 02_swerve: <{msg}>")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base",   help="COM port for 00_base (e.g. COM5)")
    parser.add_argument("--swerve", help="COM port for 02_swerve (e.g. COM7)")
    parser.add_argument("--scan",   action="store_true", help="Scan all ports and exit")
    args = parser.parse_args()

    if args.scan:
        scan_ports()
        return

    print("Press Ctrl+C to stop.\n")

    while True:
        base, swerve, base_port, swerve_port = discover_and_open(args.base, args.swerve)
        print(f"\nBridge running: {base_port} → {swerve_port}\n")
        try:
            run_bridge(base, swerve)
        except Exception as exc:
            print(f"\n[DISCONNECT] {exc}")
            safe_close(base)
            safe_close(swerve)
            print("Re-discovering...\n")
            time.sleep(RETRY_DELAY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBridge stopped.")
