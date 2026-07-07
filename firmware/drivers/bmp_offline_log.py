import argparse
import csv
import re
import time
from datetime import datetime
from pathlib import Path

import serial


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def clean_line(raw):
    text = raw.decode("utf-8", errors="ignore").strip()
    return ANSI_RE.sub("", text)


def open_serial(port, baud):
    ser = serial.Serial(port, baud, timeout=0.5)
    time.sleep(1.0)
    ser.reset_input_buffer()
    return ser


def send_command(ser, command):
    ser.write((command + "\n").encode("ascii"))
    ser.flush()


def read_until(ser, prefixes, timeout_s):
    deadline = time.time() + timeout_s
    lines = []
    while time.time() < deadline:
        line = clean_line(ser.readline())
        if not line:
            continue
        lines.append(line)
        if any(line.startswith(prefix) for prefix in prefixes):
            return line, lines
    return "", lines


def cmd_arm(args):
    with open_serial(args.port, args.baud) as ser:
        send_command(ser, f"BMPLOG_ARM,{args.seconds}")
        hit, lines = read_until(ser, ["BMPLOG_ARMED"], 5.0)
    for line in lines:
        print(line)
    if not hit or "ok=0" in hit:
        raise SystemExit("Failed to arm offline BMP logger.")


def cmd_status(args):
    with open_serial(args.port, args.baud) as ser:
        send_command(ser, "BMPLOG_STATUS")
        _, lines = read_until(ser, ["BMPLOG_STATUS"], 5.0)
    for line in lines:
        print(line)


def cmd_clear(args):
    with open_serial(args.port, args.baud) as ser:
        send_command(ser, "BMPLOG_CLEAR")
        hit, lines = read_until(ser, ["BMPLOG_CLEAR"], 5.0)
    for line in lines:
        print(line)
    if not hit or "ok=1" not in hit:
        raise SystemExit("Failed to clear offline BMP log.")


def cmd_dump(args):
    out_dir = Path(__file__).resolve().parents[1] / "data"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.output:
        out_path = Path(args.output)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_dir / f"bmp_offline_{stamp}.csv"

    rows = []
    meta = []
    in_csv = False
    header = []
    end_seen = False

    with open_serial(args.port, args.baud) as ser:
        send_command(ser, "BMPLOG_DUMP")
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            line = clean_line(ser.readline())
            if not line:
                continue
            print(line)
            if line.startswith("BMPLOG_DUMP,ok=0"):
                raise SystemExit("No stored offline BMP log found.")
            if line.startswith("BMPLOG_META,"):
                meta = line.split(",")[1:]
            elif line.startswith("BMPLOG_CSV_HEADER,"):
                header = line.split(",")[1:]
                in_csv = True
            elif line.startswith("BMPLOG,") and in_csv:
                rows.append(line.split(",")[1:])
            elif line.startswith("BMPLOG_END"):
                end_seen = True
                break

    if not end_seen:
        raise SystemExit("Timed out before BMPLOG_END.")
    if not header or not rows:
        raise SystemExit("Dump did not contain CSV rows.")

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Saved CSV: {out_path}")
    if meta:
        meta_path = out_path.with_suffix(".txt")
        meta_path.write_text(
            "BMP280 offline log metadata\n\n"
            f"sequence: {meta[0]}\n"
            f"count: {meta[1]}\n"
            f"interval_ms: {meta[2]}\n"
            f"duration_ms: {meta[3]}\n"
            f"p0_pa: {meta[4]}\n",
            encoding="utf-8",
        )
        print(f"Saved metadata: {meta_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="Arm, dump, or clear ESP32 offline BMP280 height logs.")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="command", required=True)

    arm = sub.add_parser("arm", help="Arm one offline capture on next boot.")
    arm.add_argument("--seconds", type=int, default=180)
    arm.set_defaults(func=cmd_arm)

    dump = sub.add_parser("dump", help="Dump the stored offline capture to CSV.")
    dump.add_argument("--output", default="")
    dump.add_argument("--timeout", type=float, default=20.0)
    dump.set_defaults(func=cmd_dump)

    clear = sub.add_parser("clear", help="Clear the stored offline capture.")
    clear.set_defaults(func=cmd_clear)

    status = sub.add_parser("status", help="Show logger status.")
    status.set_defaults(func=cmd_status)

    return parser.parse_args()


def main():
    args = parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
