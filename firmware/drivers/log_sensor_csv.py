import argparse
import csv
import re
import time
from datetime import datetime
from pathlib import Path

import serial


DEFAULT_HEADER = [
    "t_s",
    "ax_raw_g",
    "ay_raw_g",
    "az_raw_g",
    "gx_raw_dps",
    "gy_raw_dps",
    "gz_raw_dps",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "mpu_temp_c",
    "roll_deg",
    "pitch_deg",
    "bx_uT",
    "by_uT",
    "bz_uT",
    "mag_uT",
    "bx_cal_uT",
    "by_cal_uT",
    "bz_cal_uT",
    "mag_cal_uT",
    "yaw_flat_deg",
    "yaw_tilt_deg",
    "mahony_roll_deg",
    "mahony_pitch_deg",
    "mahony_yaw_deg",
    "bmp_temp_c",
    "pressure_pa",
    "altitude_m",
    "imu_dt_us",
    "mag_dt_us",
    "bmp_dt_us",
    "imu_mag_dt_us",
    "imu_bmp_dt_us",
]


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def clean_line(raw: bytes) -> str:
    text = raw.decode("utf-8", errors="ignore").strip()
    return ANSI_RE.sub("", text)


def parse_args():
    parser = argparse.ArgumentParser(description="Log SENSOR CSV rows from ESP32-S3 monitor serial output.")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--output", default="")
    parser.add_argument("--gps-raw-output", default="")
    parser.add_argument("--gps-fix-output", default="")
    parser.add_argument("--gps-parsed-output", default="")
    parser.add_argument("--gps-pps-output", default="")
    parser.add_argument("--gps-nmea-ts-output", default="")
    parser.add_argument("--sync-sample-output", default="")
    parser.add_argument("--prefix", default="sensor_static")
    return parser.parse_args()


def nmea_latlon(value: str, hemi: str):
    if not value or not hemi:
        return ""
    dot = value.find(".")
    deg_digits = 2 if dot == 4 else 3
    try:
        degrees = float(value[:deg_digits])
        minutes = float(value[deg_digits:])
    except ValueError:
        return ""
    coord = degrees + minutes / 60.0
    if hemi in ("S", "W"):
        coord = -coord
    return f"{coord:.8f}"


def parse_gps_fix(nmea: str, host_t_s: float):
    parts = nmea.split("*", 1)[0].split(",")
    if not parts:
        return None
    msg = parts[0][-3:]
    if msg == "GGA" and len(parts) >= 10:
        return [
            f"{host_t_s:.3f}",
            parts[0][1:],
            parts[1],
            parts[6],
            nmea_latlon(parts[2], parts[3]),
            nmea_latlon(parts[4], parts[5]),
            parts[9],
            "",
            "",
            "",
            parts[7],
            parts[8],
        ]
    if msg == "RMC" and len(parts) >= 10:
        return [
            f"{host_t_s:.3f}",
            parts[0][1:],
            parts[1],
            parts[2],
            nmea_latlon(parts[3], parts[4]),
            nmea_latlon(parts[5], parts[6]),
            "",
            parts[7],
            parts[8],
            parts[9],
            "",
            "",
        ]
    return None


def main():
    args = parse_args()
    out_dir = Path(__file__).resolve().parents[1] / "data"
    out_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.output:
        out_path = Path(args.output)
    else:
        out_path = out_dir / f"{args.prefix}_{stamp}.csv"
    gps_raw_path = Path(args.gps_raw_output) if args.gps_raw_output else out_dir / f"{args.prefix}_gps_raw_{stamp}.txt"
    gps_fix_path = Path(args.gps_fix_output) if args.gps_fix_output else out_dir / f"{args.prefix}_gps_fix_{stamp}.csv"
    gps_parsed_path = (
        Path(args.gps_parsed_output)
        if args.gps_parsed_output
        else out_dir / f"{args.prefix}_gps_parsed_{stamp}.csv"
    )
    gps_pps_path = Path(args.gps_pps_output) if args.gps_pps_output else out_dir / f"{args.prefix}_gps_pps_{stamp}.csv"
    gps_nmea_ts_path = (
        Path(args.gps_nmea_ts_output)
        if args.gps_nmea_ts_output
        else out_dir / f"{args.prefix}_gps_nmea_ts_{stamp}.csv"
    )
    sync_sample_path = (
        Path(args.sync_sample_output)
        if args.sync_sample_output
        else out_dir / f"{args.prefix}_sync_sample_{stamp}.csv"
    )

    header = DEFAULT_HEADER
    count = 0
    gps_raw_count = 0
    gps_fix_count = 0
    gps_parsed_count = 0
    gps_pps_count = 0
    gps_nmea_ts_count = 0
    sync_sample_count = 0
    deadline = time.time() + args.seconds

    print(f"Opening {args.port} at {args.baud} baud")
    print(f"Logging SENSOR rows for {args.seconds:.1f}s -> {out_path}")
    print(f"Logging GPS raw NMEA -> {gps_raw_path}")
    print(f"Logging GPS fixes -> {gps_fix_path}")
    print(f"Logging firmware GPS parsed rows -> {gps_parsed_path}")
    print(f"Logging GPS PPS timestamps -> {gps_pps_path}")
    print(f"Logging GPS NMEA timestamps -> {gps_nmea_ts_path}")
    print(f"Logging PPS-triggered sensor samples -> {sync_sample_path}")

    with serial.Serial(args.port, args.baud, timeout=1) as ser, \
            out_path.open("w", newline="", encoding="utf-8") as f, \
            gps_raw_path.open("w", newline="", encoding="utf-8") as gps_raw_f, \
            gps_fix_path.open("w", newline="", encoding="utf-8") as gps_fix_f, \
            gps_parsed_path.open("w", newline="", encoding="utf-8") as gps_parsed_f, \
            gps_pps_path.open("w", newline="", encoding="utf-8") as gps_pps_f, \
            gps_nmea_ts_path.open("w", newline="", encoding="utf-8") as gps_nmea_ts_f, \
            sync_sample_path.open("w", newline="", encoding="utf-8") as sync_sample_f:
        writer = csv.writer(f)
        writer.writerow(header)
        gps_pps_writer = csv.writer(gps_pps_f)
        gps_pps_writer.writerow(["pps_count", "esp_t_us", "interval_us"])
        gps_nmea_ts_writer = csv.writer(gps_nmea_ts_f)
        gps_nmea_ts_writer.writerow(["esp_t_us", "nmea"])
        sync_sample_writer = csv.writer(sync_sample_f)
        sync_sample_header = [
            "pps_count",
            "pps_us",
            "imu_start_us",
            "imu_done_us",
            "imu_start_delta_us",
            "imu_ok",
            "mag_start_us",
            "mag_done_us",
            "mag_start_delta_us",
            "mag_ok",
        ]
        sync_sample_writer.writerow(sync_sample_header)
        gps_fix_writer = csv.writer(gps_fix_f)
        gps_fix_writer.writerow([
            "host_t_s",
            "sentence",
            "utc",
            "status_or_fix",
            "lat_deg",
            "lon_deg",
            "altitude_m",
            "speed_kn",
            "course_deg",
            "date_ddmmyy",
            "satellites",
            "hdop",
        ])
        gps_parsed_writer = csv.writer(gps_parsed_f)
        gps_parsed_writer.writerow([
            "kind",
            "esp_t_us",
            "sentence",
            "utc",
            "status_or_quality",
            "lat_deg",
            "lon_deg",
            "altitude_m",
            "speed_kn",
            "course_deg",
            "date_ddmmyy",
            "satellites",
            "hdop",
            "checksum_ok",
        ])

        while time.time() < deadline:
            line = clean_line(ser.readline())
            if not line:
                continue

            if line.startswith("SENSOR_HEADER,"):
                header = line.split(",")[1:]
                f.seek(0)
                f.truncate()
                writer = csv.writer(f)
                writer.writerow(header)
                continue

            if line.startswith("GPS_RAW,"):
                nmea = line[len("GPS_RAW,"):]
                gps_raw_f.write(nmea + "\n")
                gps_raw_count += 1
                parsed = parse_gps_fix(nmea, time.time())
                if parsed is not None:
                    gps_fix_writer.writerow(parsed)
                    gps_fix_count += 1
                if gps_raw_count % 20 == 0:
                    print(f"Saved {gps_raw_count} GPS NMEA lines")
                continue

            if line.startswith("GPS_PPS,"):
                row = line.split(",")[1:]
                if len(row) == 3:
                    gps_pps_writer.writerow(row)
                    gps_pps_count += 1
                    if gps_pps_count % 10 == 0:
                        print(f"Saved {gps_pps_count} GPS PPS events")
                continue

            if line.startswith("GPS_NMEA_TS,"):
                row = line.split(",", 2)[1:]
                if len(row) == 2:
                    gps_nmea_ts_writer.writerow(row)
                    gps_nmea_ts_count += 1
                continue

            if line.startswith("GPS_GGA,"):
                row = line.split(",")[1:]
                if len(row) == 10:
                    gps_parsed_writer.writerow([
                        "GGA",
                        row[0],
                        row[1],
                        row[2],
                        row[3],
                        row[4],
                        row[5],
                        row[6],
                        "",
                        "",
                        "",
                        row[7],
                        row[8],
                        row[9],
                    ])
                    gps_parsed_count += 1
                continue

            if line.startswith("GPS_RMC,"):
                row = line.split(",")[1:]
                if len(row) == 10:
                    gps_parsed_writer.writerow([
                        "RMC",
                        row[0],
                        row[1],
                        row[2],
                        row[3],
                        row[4],
                        row[5],
                        "",
                        row[6],
                        row[7],
                        row[8],
                        "",
                        "",
                        row[9],
                    ])
                    gps_parsed_count += 1
                continue

            if line.startswith("SYNC_HEADER,"):
                sync_sample_header = line.split(",")[1:]
                sync_sample_f.seek(0)
                sync_sample_f.truncate()
                sync_sample_writer = csv.writer(sync_sample_f)
                sync_sample_writer.writerow(sync_sample_header)
                continue

            if line.startswith("SYNC_SAMPLE,"):
                row = line.split(",")[1:]
                if len(row) == len(sync_sample_header):
                    sync_sample_writer.writerow(row)
                    sync_sample_count += 1
                    if sync_sample_count % 10 == 0:
                        print(f"Saved {sync_sample_count} PPS-triggered sensor samples")
                else:
                    print(f"Skip malformed SYNC_SAMPLE row: got {len(row)} fields, expected {len(sync_sample_header)}")
                continue

            if not line.startswith("SENSOR,"):
                continue

            row = line.split(",")[1:]
            if len(row) != len(header):
                print(f"Skip malformed SENSOR row: got {len(row)} fields, expected {len(header)}")
                continue

            writer.writerow(row)
            count += 1
            if count % 20 == 0:
                print(f"Saved {count} rows")

    print(f"Done. Saved {count} rows to {out_path}")
    print(f"Done. Saved {gps_raw_count} GPS NMEA lines to {gps_raw_path}")
    print(f"Done. Saved {gps_fix_count} parsed GPS fix rows to {gps_fix_path}")
    print(f"Done. Saved {gps_parsed_count} firmware GPS parsed rows to {gps_parsed_path}")
    print(f"Done. Saved {gps_pps_count} GPS PPS events to {gps_pps_path}")
    print(f"Done. Saved {gps_nmea_ts_count} GPS NMEA timestamp rows to {gps_nmea_ts_path}")
    print(f"Done. Saved {sync_sample_count} PPS-triggered sensor samples to {sync_sample_path}")


if __name__ == "__main__":
    main()
