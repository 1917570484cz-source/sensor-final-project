import argparse
import csv
import re
import time
from datetime import datetime
from pathlib import Path

import serial


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def clean_line(raw):
    return ANSI_RE.sub("", raw.decode("utf-8", errors="ignore").strip())


def parse_args():
    parser = argparse.ArgumentParser(description="Dump offline GPSLOG rows from ESP32 NVS to CSV.")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = Path(__file__).resolve().parents[1] / "data"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.output) if args.output else out_dir / f"gpslog_dump_{stamp}.csv"

    header = None
    rows = []
    meta = {}
    seen = []
    deadline = time.time() + args.timeout

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b"GPSLOG_STATUS\n")
        time.sleep(0.2)
        ser.write(b"GPSLOG_DUMP\n")
        while time.time() < deadline:
            line = clean_line(ser.readline())
            if not line:
                continue
            if line.startswith("GPSLOG_") or line.startswith("ESP_ERROR"):
                seen.append(line)
            if line.startswith("GPSLOG_CSV_HEADER,"):
                header = line.split(",")[1:]
            elif line.startswith("GPSLOG_META,"):
                parts = line.split(",")
                if len(parts) >= 4:
                    meta = {"sequence": parts[1], "count": parts[2], "duration_ms": parts[3]}
            elif line.startswith("GPSLOG,"):
                rows.append(line.split(",")[1:])
            elif line.startswith("GPSLOG_END"):
                break

    if not header:
        if seen:
            print("Board response:")
            for line in seen:
                print(line)
        raise SystemExit("No GPSLOG_CSV_HEADER received. GPSLOG probably has no stored fix rows.")

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Saved {len(rows)} GPSLOG rows to {out_path}")
    if meta:
        print(f"Meta: {meta}")


if __name__ == "__main__":
    main()
