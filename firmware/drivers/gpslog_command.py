import argparse
import time

import serial


def parse_args():
    parser = argparse.ArgumentParser(description="Send GPSLOG command to ESP32.")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("command", choices=["status", "arm", "clear"])
    parser.add_argument("--seconds", type=int, default=540)
    parser.add_argument("--verbose", action="store_true", help="Print all serial lines while waiting.")
    return parser.parse_args()


def show_line(line, verbose=False):
    if verbose or line.startswith("GPSLOG_") or line.startswith("ESP_ERROR"):
        print(line)


def main():
    args = parse_args()
    if args.command == "status":
        cmd = "GPSLOG_STATUS\n"
        expected_prefix = "GPSLOG_STATUS"
    elif args.command == "clear":
        cmd = "GPSLOG_CLEAR\n"
        expected_prefix = "GPSLOG_CLEAR"
    else:
        cmd = f"GPSLOG_ARM {args.seconds}\n"
        expected_prefix = "GPSLOG_ARMED"

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        # Opening the serial port resets many ESP32 boards. Wait until firmware
        # has finished booting and is polling UART0 commands; otherwise ARM can
        # be transmitted too early and silently lost.
        boot_deadline = time.time() + 8.0
        while time.time() < boot_deadline:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            show_line(line, args.verbose)
            if line.startswith("SENSOR_HEADER,") or line.startswith("GPSLOG_STATUS,") or line.startswith("GPSLOG_CAPTURE_PENDING,"):
                break

        ser.write(cmd.encode("ascii"))
        deadline = time.time() + 5.0
        matched = False
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            show_line(line, args.verbose)
            if line.startswith(expected_prefix):
                matched = True
                break
        if not matched:
            raise SystemExit(f"No {expected_prefix} response received.")


if __name__ == "__main__":
    main()
