import argparse
import csv
import math
import re
import time
from datetime import datetime
from pathlib import Path

import serial

try:
    import numpy as np
except ModuleNotFoundError as exc:
    raise SystemExit(
        "This script needs numpy for ellipsoid fitting.\n"
        "In this workspace, run it with:\n"
        'D:\\Anaconda\\python.exe analysis\\calibrate_mag_ellipsoid.py --port COM7 --seconds 90'
    ) from exc


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
]

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def parse_args():
    parser = argparse.ArgumentParser(description="Collect and fit HMC5883L ellipsoid calibration.")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--input", default="", help="Fit an existing CSV instead of reading serial.")
    parser.add_argument("--output", default="")
    parser.add_argument("--target-uT", type=float, default=50.0, help="Calibrated magnetic field magnitude.")
    return parser.parse_args()


def clean_line(raw):
    text = raw.decode("utf-8", errors="ignore").strip()
    return ANSI_RE.sub("", text)


def parse_sensor_line(line, header):
    if line.startswith("SENSOR_HEADER,"):
        return line.split(",")[1:], None
    if not line.startswith("SENSOR,"):
        return header, None

    values = line.split(",")[1:]
    if len(values) != len(header):
        return header, None

    row = {}
    for key, value in zip(header, values):
        try:
            row[key] = float(value)
        except ValueError:
            row[key] = math.nan
    return header, row


def read_csv_points(path):
    points = []
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                point = [float(row["bx_uT"]), float(row["by_uT"]), float(row["bz_uT"])]
            except (KeyError, ValueError):
                continue
            if all(math.isfinite(v) for v in point):
                points.append(point)
    return np.asarray(points, dtype=float)


def collect_serial_points(port, baud, seconds, output):
    header = DEFAULT_HEADER
    rows = []
    points = []
    deadline = time.time() + seconds
    last_print = 0.0

    print("Stop ESP-IDF Monitor before running this script, otherwise COM7 will be busy.")
    print("Rotate the board slowly through as many 3D orientations as possible.")
    print("Avoid magnets, motors, speakers, steel tools, and laptop edges during collection.")
    print(f"Opening {port} at {baud} baud for {seconds:.1f}s")

    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            ser.reset_input_buffer()
            while time.time() < deadline:
                line = clean_line(ser.readline())
                if not line:
                    continue
                header, row = parse_sensor_line(line, header)
                if row is None:
                    continue

                point = [row.get("bx_uT", math.nan), row.get("by_uT", math.nan), row.get("bz_uT", math.nan)]
                if not all(math.isfinite(v) for v in point):
                    continue

                rows.append([row.get(key, math.nan) for key in header])
                points.append(point)

                now = time.time()
                if now - last_print >= 2.0 and points:
                    arr = np.asarray(points, dtype=float)
                    span = arr.max(axis=0) - arr.min(axis=0)
                    remaining = max(0.0, deadline - now)
                    print(
                        f"rows={len(points):4d} remaining={remaining:5.1f}s "
                        f"span=({span[0]:5.1f},{span[1]:5.1f},{span[2]:5.1f}) uT"
                    )
                    last_print = now
    except serial.SerialException as exc:
        raise SystemExit(f"Cannot open {port}: {exc}\nStop ESP-IDF Monitor and try again.")

    out_path = Path(output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

    return np.asarray(points, dtype=float), out_path


def ellipsoid_fit(points, target_uT):
    if len(points) < 80:
        raise SystemExit(f"Need at least 80 magnetic samples, got {len(points)}")

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    design = np.column_stack(
        [
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            np.ones_like(x),
        ]
    )

    _, _, vh = np.linalg.svd(design, full_matrices=False)
    coeff = vh[-1, :]
    a = np.array(
        [
            [coeff[0], coeff[3], coeff[4]],
            [coeff[3], coeff[1], coeff[5]],
            [coeff[4], coeff[5], coeff[2]],
        ],
        dtype=float,
    )
    d = np.array([coeff[6], coeff[7], coeff[8]], dtype=float)
    c = float(coeff[9])

    if np.linalg.det(a) < 0:
        a = -a
        d = -d
        c = -c

    center = -0.5 * np.linalg.solve(a, d)
    k = float(center @ a @ center - c)
    if k <= 0:
        a = -a
        d = -d
        c = -c
        center = -0.5 * np.linalg.solve(a, d)
        k = float(center @ a @ center - c)
    if k <= 0:
        raise SystemExit("Ellipsoid fit failed: non-positive scale. Recollect data with better 3D coverage.")

    shape = a / k
    eigvals, eigvecs = np.linalg.eigh(shape)
    if np.any(eigvals <= 0):
        raise SystemExit("Ellipsoid fit failed: shape matrix is not positive definite.")

    w_unit = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
    mag_w = target_uT * w_unit
    corrected = (mag_w @ (points - center).T).T
    norms = np.linalg.norm(corrected, axis=1)
    raw_norms = np.linalg.norm(points, axis=1)
    return center, mag_w, corrected, raw_norms, norms


def format_c_array(name, values):
    if values.ndim == 2:
        rows = []
        for row in values:
            rows.append("    {" + ", ".join(f"{v:.9f}f" for v in row) + "},")
        return f"static const float {name}[3][3] = {{\n" + "\n".join(rows) + "\n};"
    return (
        f"static const float {name}[3] = {{\n"
        + "\n".join(f"    {v:.9f}f," for v in values)
        + "\n};"
    )


def write_report(points, source_path, center, mag_w, raw_norms, cal_norms, target_uT):
    root = Path(__file__).resolve().parents[1]
    data_dir = root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = data_dir / f"mag_calibration_{stamp}.txt"

    span = points.max(axis=0) - points.min(axis=0)
    with report_path.open("w", encoding="utf-8") as f:
        f.write("HMC5883L ellipsoid calibration\n\n")
        f.write(f"source: {source_path}\n")
        f.write(f"samples: {len(points)}\n")
        f.write(f"target_uT: {target_uT:.3f}\n")
        f.write(f"span_uT: {span[0]:.3f}, {span[1]:.3f}, {span[2]:.3f}\n\n")
        f.write(format_c_array("MAG_C", center))
        f.write("\n\n")
        f.write(format_c_array("MAG_W", mag_w))
        f.write("\n\n")
        f.write("Norm summary:\n")
        f.write(f"raw_mean_uT: {raw_norms.mean():.3f}\n")
        f.write(f"raw_std_uT: {raw_norms.std(ddof=1):.3f}\n")
        f.write(f"cal_mean_uT: {cal_norms.mean():.3f}\n")
        f.write(f"cal_std_uT: {cal_norms.std(ddof=1):.3f}\n")
        f.write(f"cal_min_uT: {cal_norms.min():.3f}\n")
        f.write(f"cal_max_uT: {cal_norms.max():.3f}\n")
        f.write(
            "\nResult: "
            + ("PASS" if cal_norms.std(ddof=1) < raw_norms.std(ddof=1) * 0.35 else "CHECK")
            + "\n"
        )
    return report_path


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    data_dir = root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    if args.input:
        source_path = Path(args.input)
        points = read_csv_points(source_path)
    else:
        if args.output:
            source_path = Path(args.output)
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            source_path = data_dir / f"mag_rotate_{stamp}.csv"
        points, source_path = collect_serial_points(args.port, args.baud, args.seconds, source_path)

    center, mag_w, corrected, raw_norms, cal_norms = ellipsoid_fit(points, args.target_uT)
    report_path = write_report(points, source_path, center, mag_w, raw_norms, cal_norms, args.target_uT)

    print("\nCalibration constants for main.c:")
    print(format_c_array("MAG_C", center))
    print()
    print(format_c_array("MAG_W", mag_w))
    print("\nNorm summary:")
    print(f"raw mean/std: {raw_norms.mean():.3f} / {raw_norms.std(ddof=1):.3f} uT")
    print(f"cal mean/std: {cal_norms.mean():.3f} / {cal_norms.std(ddof=1):.3f} uT")
    print(f"cal min/max:  {cal_norms.min():.3f} / {cal_norms.max():.3f} uT")
    print(f"Saved source: {source_path}")
    print(f"Saved report: {report_path}")


if __name__ == "__main__":
    main()
