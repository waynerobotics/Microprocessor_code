from serial_protocol import WarriorSerial
from find_serial_ports import get_port_map


def test_device(name: str, port: str) -> None:
    print(f"\nTesting {name} on {port}")

    with WarriorSerial(port) as device:
        response = device.request("WHO")
        print("WHO ->", response)

        response = device.request("PING")
        print("PING ->", response)

        if name == "00_base":
            device.send_message("LED", "r")
            print("Sent LED red")

            device.send_message("LED", "g")
            print("Sent LED green")

            device.send_message("LED", "b")
            print("Sent LED blue")

            device.send_message("LED", "o")
            print("Sent LED off")

        if name == "09_lcd_keypad":
            device.send_message("LCD", 0, "Warrior Serial")
            device.send_message("LCD", 1, "Python Test OK")
            print("Sent LCD text")


def main() -> None:
    ports = get_port_map()

    if not ports:
        print("No Warrior devices found.")
        return

    print("Found devices:")
    for name, port in ports.items():
        print(f"  {name}: {port}")

    for name, port in ports.items():
        test_device(name, port)


if __name__ == "__main__":
    main()