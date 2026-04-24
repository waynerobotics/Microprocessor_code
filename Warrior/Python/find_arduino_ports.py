# Python/find_serial_ports.py

from dataclasses import dataclass
from typing import Dict, List, Optional
from serial.tools import list_ports

# This function should come from another Python file later.
# For example:
# from serial_protocol import query_device_name
#
# Expected behavior:
#   query_device_name(port: str, baudrate: int, timeout: float) -> Optional[str]
#
# It should return names like:
#   "00_base"
#   "02_swerve"
#   "03_swerve"
#   "04_swerve"
from serial_protocol import query_device_name


EXPECTED_DEVICE_NAMES = {
    "00_base",
    "02_swerve",
    "03_swerve",
    "04_swerve",
}


DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT_SECONDS = 1.0


@dataclass
class ArduinoDevice:
    name: str
    port: str
    description: str
    hwid: str


def list_usb_serial_ports() -> List:
    """
    Return all visible serial ports.

    On Windows these are usually COM ports:
      COM3, COM4, COM5, etc.
    """
    return list(list_ports.comports())


def find_warrior_arduinos(
    baudrate: int = DEFAULT_BAUDRATE,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
) -> Dict[str, ArduinoDevice]:
    """
    Search all serial ports, ask each Arduino for its flashed device name,
    and return a mapping:

      {
        "00_base": ArduinoDevice(...),
        "02_swerve": ArduinoDevice(...),
        ...
      }
    """

    discovered: Dict[str, ArduinoDevice] = {}

    for port_info in list_usb_serial_ports():
        port = port_info.device

        try:
            device_name = query_device_name(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
            )
        except Exception as exc:
            print(f"[WARN] Could not query {port}: {exc}")
            continue

        if device_name is None:
            continue

        device_name = device_name.strip()

        if device_name not in EXPECTED_DEVICE_NAMES:
            print(f"[WARN] Unknown device name '{device_name}' on {port}")
            continue

        if device_name in discovered:
            previous = discovered[device_name]
            print(
                f"[WARN] Duplicate device name '{device_name}' found on "
                f"{previous.port} and {port}"
            )

        discovered[device_name] = ArduinoDevice(
            name=device_name,
            port=port,
            description=port_info.description,
            hwid=port_info.hwid,
        )

    return discovered


def get_port_map(
    baudrate: int = DEFAULT_BAUDRATE,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
) -> Dict[str, str]:
    """
    Simpler mapping for other scripts:

      {
        "00_base": "COM3",
        "02_swerve": "COM4",
        ...
      }
    """

    devices = find_warrior_arduinos(
        baudrate=baudrate,
        timeout=timeout,
    )

    return {
        name: device.port
        for name, device in devices.items()
    }


def require_device(
    device_name: str,
    baudrate: int = DEFAULT_BAUDRATE,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
) -> str:
    """
    Return the COM port for a required Arduino.
    Raise an error if it is missing.
    """

    port_map = get_port_map(
        baudrate=baudrate,
        timeout=timeout,
    )

    if device_name not in port_map:
        raise RuntimeError(f"Required Arduino '{device_name}' not found")

    return port_map[device_name]


def print_discovered_devices(devices: Dict[str, ArduinoDevice]) -> None:
    if not devices:
        print("No Warrior Arduino devices found.")
        return

    print("Discovered Warrior Arduino devices:")

    for name, device in sorted(devices.items()):
        print(f"  {name}: {device.port}")
        print(f"    Description: {device.description}")
        print(f"    HWID: {device.hwid}")


if __name__ == "__main__":
    devices = find_warrior_arduinos()
    print_discovered_devices(devices)