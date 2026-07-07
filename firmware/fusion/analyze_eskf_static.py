import argparse
import csv
import math
from datetime import datetime
from pathlib import Path

import numpy as np


G_WORLD = np.array([0.0, 0.0, 1.0], dtype=float)


def parse_args():
    parser = argparse.ArgumentParser(description="Run an offline SO(3) ESKF static attitude experiment.")
    parser.add_argument("csv", help="Static SENSOR CSV file.")
    parser.add_argument("--settle", type=float, default=5.0, help="Seconds to discard at the beginning.")
    parser.add_argument("--target-deg", type=float, default=0.2, help="Static precision target in degrees.")
    parser.add_argument(
        "--expected",
        default="+z",
        choices=["+z", "-z", "+x", "-x", "+y", "-y"],
        help="Expected gravity direction in the board/body frame.",
    )
    parser.add_argument("--output", default="", help="Report path. Defaults beside the CSV file.")
    return parser.parse_args()


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_rows(path):
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            rows.append({key: to_float(value) for key, value in row.items()})
        return rows


def valid(values):
    return np.array([v for v in values if math.isfinite(v)], dtype=float)


def mean(values):
    data = valid(values)
    return float(np.mean(data)) if data.size else math.nan


def std(values):
    data = valid(values)
    return float(np.std(data, ddof=1)) if data.size > 1 else math.nan


def rms(values):
    data = valid(values)
    return float(np.sqrt(np.mean(data * data))) if data.size else math.nan


def max_abs(values):
    data = valid(values)
    return float(np.max(np.abs(data))) if data.size else math.nan


def q_normalize(q):
    n = np.linalg.norm(q)
    if n <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def q_from_rotvec(rv):
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return q_normalize(np.array([1.0, 0.5 * rv[0], 0.5 * rv[1], 0.5 * rv[2]], dtype=float))
    axis = rv / angle
    half = 0.5 * angle
    return np.array([math.cos(half), *(math.sin(half) * axis)], dtype=float)


def q_from_euler(roll_deg, pitch_deg, yaw_deg=0.0):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r / 2.0), math.sin(r / 2.0)
    cp, sp = math.cos(p / 2.0), math.sin(p / 2.0)
    cy, sy = math.cos(y / 2.0), math.sin(y / 2.0)
    return q_normalize(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=float,
        )
    )


def q_to_rotmat(q):
    w, x, y, z = q_normalize(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def q_to_euler(q):
    w, x, y, z = q_normalize(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def skew(v):
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=float,
    )


def predict_acc_body(q):
    return q_to_rotmat(q).T @ G_WORLD


def numeric_h(q):
    eps = 1e-6
    base = predict_acc_body(q)
    h = np.zeros((3, 6), dtype=float)
    for i in range(3):
        d = np.zeros(3, dtype=float)
        d[i] = eps
        qp = q_mul(q, q_from_rotvec(d))
        h[:, i] = (predict_acc_body(qp) - base) / eps
    return h


def accel_to_roll_pitch(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def expected_gravity(axis):
    mapping = {
        "+z": np.array([0.0, 0.0, 1.0]),
        "-z": np.array([0.0, 0.0, -1.0]),
        "+x": np.array([1.0, 0.0, 0.0]),
        "-x": np.array([-1.0, 0.0, 0.0]),
        "+y": np.array([0.0, 1.0, 0.0]),
        "-y": np.array([0.0, -1.0, 0.0]),
    }
    return mapping[axis]


def angle_between(a, b):
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na <= 0.0 or nb <= 0.0:
        return math.nan
    c = float(np.dot(a, b) / (na * nb))
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def run_eskf(rows):
    first = rows[0]
    roll0, pitch0 = accel_to_roll_pitch(first["ax_g"], first["ay_g"], first["az_g"])
    q = q_from_euler(roll0, pitch0, 0.0)
    bias = np.zeros(3, dtype=float)

    p = np.diag([math.radians(2.0) ** 2] * 3 + [math.radians(0.5) ** 2] * 3)
    gyro_noise = math.radians(0.08)
    bias_noise = math.radians(0.002)
    accel_noise = math.radians(0.8)
    q_base = np.diag([gyro_noise * gyro_noise] * 3 + [bias_noise * bias_noise] * 3)
    r = np.eye(3) * (accel_noise * accel_noise)
    i6 = np.eye(6)

    out = []
    last_t = first["t_s"]
    for row in rows:
        t = row["t_s"]
        if not math.isfinite(t):
            continue
        dt = max(0.001, min(0.2, t - last_t if math.isfinite(last_t) else 0.01))
        last_t = t

        gyro = np.radians(np.array([row["gx_dps"], row["gy_dps"], row["gz_dps"]], dtype=float))
        acc = np.array([row["ax_g"], row["ay_g"], row["az_g"]], dtype=float)
        if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(acc)):
            continue

        omega = gyro - bias
        q = q_normalize(q_mul(q, q_from_rotvec(omega * dt)))

        f = np.eye(6)
        f[0:3, 0:3] -= skew(omega) * dt
        f[0:3, 3:6] = -np.eye(3) * dt
        p = f @ p @ f.T + q_base * dt

        acc_norm = np.linalg.norm(acc)
        if 0.5 < acc_norm < 1.5:
            z = acc / acc_norm
            h = predict_acc_body(q)
            residual = z - h
            h_mat = numeric_h(q)
            s = h_mat @ p @ h_mat.T + r
            k = p @ h_mat.T @ np.linalg.inv(s)
            dx = k @ residual
            q = q_normalize(q_mul(q, q_from_rotvec(dx[0:3])))
            bias += dx[3:6]
            p = (i6 - k @ h_mat) @ p @ (i6 - k @ h_mat).T + k @ r @ k.T

        roll, pitch, yaw = q_to_euler(q)
        out.append(
            {
                "t_s": t,
                "eskf_roll_deg": roll,
                "eskf_pitch_deg": pitch,
                "eskf_yaw_deg": yaw,
                "eskf_bgx_dps": math.degrees(bias[0]),
                "eskf_bgy_dps": math.degrees(bias[1]),
                "eskf_bgz_dps": math.degrees(bias[2]),
                "eskf_gx_body": predict_acc_body(q)[0],
                "eskf_gy_body": predict_acc_body(q)[1],
                "eskf_gz_body": predict_acc_body(q)[2],
            }
        )
    return out


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    source = Path(args.csv)
    rows = read_rows(source)
    if not rows:
        raise SystemExit("CSV has no rows")

    t0 = rows[0].get("t_s", math.nan)
    filtered = [row for row in rows if math.isfinite(row.get("t_s", math.nan)) and row["t_s"] - t0 >= args.settle]
    if len(filtered) < 20:
        raise SystemExit("Not enough rows after settle time")

    eskf_rows = run_eskf(filtered)
    if not eskf_rows:
        raise SystemExit("ESKF produced no rows")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = source.with_name(f"{source.stem}_eskf_static_{stamp}.csv")
    report_path = Path(args.output) if args.output else source.with_name(f"{source.stem}_eskf_static_{stamp}.txt")
    write_csv(out_csv, eskf_rows)

    roll = [row["eskf_roll_deg"] for row in eskf_rows]
    pitch = [row["eskf_pitch_deg"] for row in eskf_rows]
    roll_dev = [v - mean(roll) for v in roll]
    pitch_dev = [v - mean(pitch) for v in pitch]

    expected_g = expected_gravity(args.expected)
    angle_err = [
        angle_between(np.array([row["eskf_gx_body"], row["eskf_gy_body"], row["eskf_gz_body"]]), expected_g)
        for row in eskf_rows
    ]

    comp_roll = [row.get("roll_deg", math.nan) for row in filtered]
    comp_pitch = [row.get("pitch_deg", math.nan) for row in filtered]
    mahony_roll = [row.get("mahony_roll_deg", math.nan) for row in filtered]
    mahony_pitch = [row.get("mahony_pitch_deg", math.nan) for row in filtered]

    lines = [
        "ESKF static attitude experiment",
        "",
        f"source: {source}",
        f"rows used: {len(filtered)}",
        f"settle_s: {args.settle:.3f}",
        f"expected gravity axis: {args.expected}",
        "",
        "ESKF roll/pitch:",
        f"roll mean/std/max_dev_from_mean: {mean(roll):.6f} / {std(roll):.6f} / {max_abs(roll_dev):.6f} deg",
        f"pitch mean/std/max_dev_from_mean: {mean(pitch):.6f} / {std(pitch):.6f} / {max_abs(pitch_dev):.6f} deg",
        f"stability_rms: {math.sqrt(rms(roll_dev) ** 2 + rms(pitch_dev) ** 2):.6f} deg",
        "",
        "Reference check against expected gravity direction:",
        f"gravity_angle_error mean/std/max: {mean(angle_err):.6f} / {std(angle_err):.6f} / {max_abs(angle_err):.6f} deg",
        "",
        "Comparison with firmware outputs:",
        f"complementary roll std / pitch std: {std(comp_roll):.6f} / {std(comp_pitch):.6f} deg",
        f"mahony roll std / pitch std: {std(mahony_roll):.6f} / {std(mahony_pitch):.6f} deg",
        f"eskf roll std / pitch std: {std(roll):.6f} / {std(pitch):.6f} deg",
        "",
    ]

    sigma_pass = max(std(roll), std(pitch)) <= args.target_deg
    stable_pass = max(max_abs(roll_dev), max_abs(pitch_dev)) <= args.target_deg
    reference_pass = max_abs(angle_err) <= args.target_deg
    lines.append(f"1-sigma static precision: {'PASS' if sigma_pass else 'CHECK'} (target std <= {args.target_deg:.3f} deg)")
    lines.append(f"Max-deviation stability: {'PASS' if stable_pass else 'CHECK'} (target max deviation <= {args.target_deg:.3f} deg)")
    lines.append(
        f"Reference result: {'PASS' if reference_pass else 'CHECK'} "
        f"(target gravity angle max <= {args.target_deg:.3f} deg; depends on fixture/table accuracy)"
    )
    lines.append("")
    lines.append(f"Saved ESKF CSV: {out_csv}")

    report = "\n".join(lines) + "\n"
    report_path.write_text(report, encoding="utf-8")
    print(report)
    print(f"Saved report: {report_path}")


if __name__ == "__main__":
    main()
