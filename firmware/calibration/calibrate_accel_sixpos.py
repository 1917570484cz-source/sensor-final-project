import argparse
import csv
import math
import re
import os
import time
from datetime import datetime
from pathlib import Path

import serial

if os.name == "nt":
    import msvcrt


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

POSITION_TARGETS = {
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
}

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect six-position raw accelerometer data and solve affine calibration."
    )
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--min-rows", type=int, default=20)
    parser.add_argument("--cycles", type=int, default=1, help="Repeat the six positions N times for better precision.")
    parser.add_argument("--target-mg", type=float, default=2.0, help="Residual error target in mg.")
    parser.add_argument(
        "--keep-ratio",
        type=float,
        default=0.80,
        help="Keep the most stable fraction of samples in each position.",
    )
    parser.add_argument(
        "--positions",
        nargs="+",
        default=["+Z", "-Z", "+X", "-X", "+Y", "-Y"],
        choices=list(POSITION_TARGETS),
    )
    parser.add_argument(
        "--positions-csv",
        default="",
        help='Comma-separated positions, for shells that treat "-Z" as an option. Example: "+Z,-Z"',
    )
    return parser.parse_args()


def clean_line(raw):
    text = raw.decode("utf-8", errors="ignore").strip()
    return ANSI_RE.sub("", text)


def median(values):
    ordered = sorted(values)
    count = len(ordered)
    mid = count // 2
    if count % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def select_stable_rows(rows, keep_ratio):
    if not rows:
        return []
    keep_ratio = max(0.20, min(1.0, keep_ratio))
    center = tuple(median([row[i] for row in rows]) for i in range(3))
    scored = []
    for row in rows:
        score = norm(vec_sub(row, center))
        scored.append((score, row))
    keep_count = max(1, int(round(len(rows) * keep_ratio)))
    scored.sort(key=lambda item: item[0])
    return [row for _, row in scored[:keep_count]]


def parse_sensor_row(line, header):
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


def closest_position(raw):
    labels = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    vectors = {
        "+X": (1.0, 0.0, 0.0),
        "-X": (-1.0, 0.0, 0.0),
        "+Y": (0.0, 1.0, 0.0),
        "-Y": (0.0, -1.0, 0.0),
        "+Z": (0.0, 0.0, 1.0),
        "-Z": (0.0, 0.0, -1.0),
    }
    raw_norm = norm(raw)
    if raw_norm <= 1e-9:
        return "UNKNOWN", 180.0

    unit = tuple(v / raw_norm for v in raw)
    best = max(labels, key=lambda label: sum(unit[i] * vectors[label][i] for i in range(3)))
    dot = max(-1.0, min(1.0, sum(unit[i] * vectors[best][i] for i in range(3))))
    angle_deg = math.degrees(math.acos(dot))
    return best, angle_deg


def wait_for_enter_with_preview(ser, expected_position):
    print(f"\nPlace board at {expected_position}. Watch preview; press Enter when it shows OK.")
    if os.name != "nt":
        input("Press Enter to start sampling...")
        return DEFAULT_HEADER

    header = DEFAULT_HEADER
    last_print = 0.0
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getwch()
            if key in ("\r", "\n"):
                print()
                return header

        line = clean_line(ser.readline())
        if not line:
            continue
        header, row = parse_sensor_row(line, header)
        if row is None:
            continue

        raw = (row.get("ax_raw_g", math.nan), row.get("ay_raw_g", math.nan), row.get("az_raw_g", math.nan))
        if not all(math.isfinite(v) for v in raw):
            continue

        now = time.time()
        if now - last_print < 0.25:
            continue
        last_print = now

        inferred, angle_deg = closest_position(raw)
        status = "OK" if inferred == expected_position else "MOVE"
        raw_norm = norm(raw)
        print(
            f"\rExpected {expected_position:>2s} | current {inferred:>2s} | {status:4s} | "
            f"raw=({raw[0]: .4f},{raw[1]: .4f},{raw[2]: .4f})g | "
            f"norm={raw_norm:.4f}g | angle={angle_deg:5.2f}deg   ",
            end="",
            flush=True,
        )


def read_position(ser, position, seconds, settle, min_rows, keep_ratio):
    header = wait_for_enter_with_preview(ser, position)
    if settle > 0:
        print(f"Settling {settle:.1f}s")
        time.sleep(settle)

    rows = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        line = clean_line(ser.readline())
        if not line:
            continue
        header, row = parse_sensor_row(line, header)
        if row is None:
            continue
        if not {"ax_raw_g", "ay_raw_g", "az_raw_g"}.issubset(row):
            continue
        raw = (row["ax_raw_g"], row["ay_raw_g"], row["az_raw_g"])
        if all(math.isfinite(v) for v in raw):
            rows.append(raw)

    if len(rows) < min_rows:
        raise SystemExit(f"{position}: only got {len(rows)} valid rows, need at least {min_rows}")

    stable_rows = select_stable_rows(rows, keep_ratio)
    if len(stable_rows) < min_rows:
        raise SystemExit(
            f"{position}: only kept {len(stable_rows)} stable rows, need at least {min_rows}. "
            "Increase --seconds or --keep-ratio."
        )

    mean = tuple(sum(r[i] for r in stable_rows) / len(stable_rows) for i in range(3))
    std = tuple(
        math.sqrt(sum((r[i] - mean[i]) ** 2 for r in stable_rows) / max(1, len(stable_rows) - 1))
        for i in range(3)
    )
    rejected = len(rows) - len(stable_rows)
    print(
        f"{position}: rows={len(rows)} kept={len(stable_rows)} rejected={rejected} "
        f"mean=({mean[0]:.6f}, {mean[1]:.6f}, {mean[2]:.6f}) g "
        f"std=({std[0] * 1000.0:.2f}, {std[1] * 1000.0:.2f}, {std[2] * 1000.0:.2f}) mg"
    )
    return rows, mean


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def matmul(a, b):
    rows = len(a)
    cols = len(b[0])
    inner = len(b)
    out = [[0.0 for _ in range(cols)] for _ in range(rows)]
    for r in range(rows):
        for c in range(cols):
            out[r][c] = sum(a[r][k] * b[k][c] for k in range(inner))
    return out


def solve_square(a, b):
    n = len(a)
    m = len(b[0])
    aug = [a[i][:] + b[i][:] for i in range(n)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise SystemExit("Calibration matrix is singular; repeat the six positions with clearer orientations.")
        aug[col], aug[pivot] = aug[pivot], aug[col]

        scale = aug[col][col]
        for c in range(col, n + m):
            aug[col][c] /= scale

        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            for c in range(col, n + m):
                aug[r][c] -= factor * aug[col][c]

    return [row[n:] for row in aug]


def invert_3x3(a):
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return solve_square([row[:] for row in a], identity)


def vec_sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def mat_vec_mul(a, v):
    return tuple(sum(a[r][c] * v[c] for c in range(3)) for r in range(3))


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def solve_calibration(samples):
    design = []
    observed = []
    for position, mean in samples:
        tx, ty, tz = POSITION_TARGETS[position]
        design.append([tx, ty, tz, 1.0])
        observed.append([mean[0], mean[1], mean[2]])

    bt = transpose(design)
    normal = matmul(bt, design)
    rhs = matmul(bt, observed)
    coeff = solve_square(normal, rhs)

    a_matrix = [
        [coeff[0][0], coeff[1][0], coeff[2][0]],
        [coeff[0][1], coeff[1][1], coeff[2][1]],
        [coeff[0][2], coeff[1][2], coeff[2][2]],
    ]
    c_bias = [coeff[3][0], coeff[3][1], coeff[3][2]]
    a_inv = invert_3x3(a_matrix)
    return a_matrix, a_inv, c_bias


def format_c_array(name, values):
    if isinstance(values[0], list):
        rows = []
        for row in values:
            rows.append("    {" + ", ".join(f"{v:.9f}f" for v in row) + "},")
        return f"static const float {name}[3][3] = {{\n" + "\n".join(rows) + "\n};"
    return (
        f"static const float {name}[3] = {{\n"
        + "\n".join(f"    {v:.9f}f," for v in values)
        + "\n};"
    )


def evaluate_errors(samples, a_inv, c_bias):
    before = []
    after = []
    corrected_rows = []
    for index, (position, mean) in enumerate(samples, start=1):
        target = POSITION_TARGETS[position]
        corrected = mat_vec_mul(a_inv, vec_sub(mean, c_bias))
        before_err = abs(norm(mean) - 1.0)
        after_err = norm(vec_sub(corrected, target))
        before.append(before_err)
        after.append(after_err)
        corrected_rows.append((index, position, before_err, after_err, corrected))
    return before, after, corrected_rows


def write_outputs(samples, all_rows, a_matrix, a_inv, c_bias, target_mg):
    root = Path(__file__).resolve().parents[1]
    data_dir = root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    raw_path = data_dir / f"accel_6pos_{stamp}.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["cycle", "position", "ax_raw_g", "ay_raw_g", "az_raw_g"])
        for cycle, position, rows in all_rows:
            for row in rows:
                writer.writerow([cycle, position, *row])

    report_path = data_dir / f"accel_calibration_{stamp}.txt"
    with report_path.open("w", encoding="utf-8") as f:
        f.write("Six-position accelerometer calibration\n\n")
        f.write("Position means:\n")
        for index, (position, mean) in enumerate(samples, start=1):
            f.write(f"{index:02d} {position}: {mean[0]:.9f}, {mean[1]:.9f}, {mean[2]:.9f}\n")
        f.write("\nModel: raw = A * true + C_BIAS; corrected = A_INV * (raw - C_BIAS)\n\n")
        f.write(format_c_array("A_INV", a_inv))
        f.write("\n\n")
        f.write(format_c_array("C_BIAS", c_bias))
        f.write("\n\nA matrix:\n")
        for row in a_matrix:
            f.write(", ".join(f"{v:.9f}" for v in row) + "\n")

        f.write("\nErrors:\n")
        before, after, corrected_rows = evaluate_errors(samples, a_inv, c_bias)
        for index, position, before_err, after_err, corrected in corrected_rows:
            f.write(
                f"{index:02d} {position}: before_norm_err={before_err * 1000.0:.3f} mg, "
                f"after_vector_err={after_err * 1000.0:.3f} mg, "
                f"corrected=({corrected[0]:.6f}, {corrected[1]:.6f}, {corrected[2]:.6f})\n"
            )
        mean_after_mg = sum(after) / len(after) * 1000.0
        max_after_mg = max(after) * 1000.0
        f.write(
            f"\nMean before norm error: {sum(before) / len(before) * 1000.0:.3f} mg\n"
            f"Mean after vector error: {mean_after_mg:.3f} mg\n"
            f"Max after vector error: {max_after_mg:.3f} mg\n"
            f"Target: {target_mg:.3f} mg\n"
            f"Result: {'PASS' if mean_after_mg <= target_mg and max_after_mg <= target_mg * 2.0 else 'CHECK'}\n"
        )

    return raw_path, report_path, mean_after_mg, max_after_mg


def main():
    args = parse_args()
    if args.positions_csv:
        args.positions = [item.strip() for item in args.positions_csv.split(",") if item.strip()]
        bad_positions = [item for item in args.positions if item not in POSITION_TARGETS]
        if bad_positions:
            raise SystemExit(f"Invalid --positions-csv values: {bad_positions}")
    if args.cycles < 1:
        raise SystemExit("--cycles must be >= 1")
    if not 0.20 <= args.keep_ratio <= 1.0:
        raise SystemExit("--keep-ratio must be between 0.20 and 1.0")

    all_rows = []
    samples = []

    print("Stop ESP-IDF Monitor before running this script, otherwise COM7 will be busy.")
    print("Use the board's physical orientation. Example: +Z means chip/board top side faces up.")
    print(
        f"Plan: {args.cycles} cycle(s), {len(args.positions)} positions per cycle, "
        f"{args.seconds:.1f}s per position, keep {args.keep_ratio * 100.0:.0f}% most stable rows."
    )
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            time.sleep(0.5)
            ser.reset_input_buffer()
            for cycle in range(1, args.cycles + 1):
                print(f"\n=== Cycle {cycle}/{args.cycles} ===")
                for position in args.positions:
                    rows, mean = read_position(
                        ser, position, args.seconds, args.settle, args.min_rows, args.keep_ratio
                    )
                    all_rows.append((cycle, position, rows))
                    samples.append((position, mean))
    except serial.SerialException as exc:
        raise SystemExit(f"Cannot open {args.port}: {exc}\nStop ESP-IDF Monitor and try again.")

    if len(samples) < 6:
        print("\nCheck-only run: fewer than six positions were collected, so calibration constants were not solved.")
        print("Collected means:")
        for position, mean in samples:
            target_axis = {"X": 0, "Y": 1, "Z": 2}[position[1]]
            target_value = 1.0 if position[0] == "+" else -1.0
            axis_error_mg = (mean[target_axis] - target_value) * 1000.0
            print(
                f"{position}: mean=({mean[0]:.6f}, {mean[1]:.6f}, {mean[2]:.6f}) g, "
                f"main_axis_error={axis_error_mg:.1f} mg"
            )
        print("Run all six positions to solve A_INV and C_BIAS.")
        return

    a_matrix, a_inv, c_bias = solve_calibration(samples)
    raw_path, report_path, mean_after_mg, max_after_mg = write_outputs(
        samples, all_rows, a_matrix, a_inv, c_bias, args.target_mg
    )

    print("\nCalibration constants for main.c:")
    print(format_c_array("A_INV", a_inv))
    print()
    print(format_c_array("C_BIAS", c_bias))
    print("\nPrecision check:")
    print(f"Mean after vector error: {mean_after_mg:.3f} mg")
    print(f"Max after vector error:  {max_after_mg:.3f} mg")
    print(
        "Result: "
        + ("PASS" if mean_after_mg <= args.target_mg and max_after_mg <= args.target_mg * 2.0 else "CHECK")
        + f"  (target mean <= {args.target_mg:.3f} mg)"
    )
    print(f"\nSaved raw data: {raw_path}")
    print(f"Saved report:   {report_path}")
    print("If Result is CHECK, repeat the positions with the largest after_vector_err in the report.")


if __name__ == "__main__":
    main()
