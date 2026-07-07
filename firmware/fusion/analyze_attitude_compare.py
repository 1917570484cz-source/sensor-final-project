import argparse
import csv
import math
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Compare complementary filter and Mahony attitude outputs.")
    parser.add_argument("csv", help="Path to a dynamic SENSOR CSV recording.")
    return parser.parse_args()


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_rows(path):
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return [{key: to_float(value) for key, value in row.items()} for row in reader]


def valid(values):
    return [v for v in values if math.isfinite(v)]


def mean(values):
    values = valid(values)
    if not values:
        return math.nan
    return sum(values) / len(values)


def std(values):
    values = valid(values)
    if len(values) < 2:
        return math.nan
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def span(values):
    values = valid(values)
    if not values:
        return math.nan
    return max(values) - min(values)


def circular_error(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def unwrap_degrees(values):
    out = []
    last = math.nan
    offset = 0.0
    for value in values:
        if not math.isfinite(value):
            out.append(math.nan)
            continue
        if math.isfinite(last):
            delta = value + offset - last
            if delta > 180.0:
                offset -= 360.0
            elif delta < -180.0:
                offset += 360.0
        unwrapped = value + offset
        out.append(unwrapped)
        last = unwrapped
    return out


def series(rows, col):
    return [row.get(col, math.nan) for row in rows]


def diff_series(a, b, circular=False):
    out = []
    for x, y in zip(a, b):
        if not math.isfinite(x) or not math.isfinite(y):
            out.append(math.nan)
        elif circular:
            out.append(circular_error(x, y))
        else:
            out.append(x - y)
    return out


def print_metric(name, values, unit="deg"):
    print(f"{name:24s} mean={mean(values):10.4f}  std={std(values):9.4f}  span={span(values):9.4f} {unit}")


def write_report(path, rows, metrics):
    report_path = path.with_name(f"{path.stem}_attitude_compare.txt")
    with report_path.open("w", encoding="utf-8") as f:
        f.write("Attitude algorithm comparison\n\n")
        f.write(f"source: {path}\n")
        f.write(f"rows: {len(rows)}\n")
        if rows and "t_s" in rows[0]:
            f.write(f"duration_s: {rows[-1].get('t_s', math.nan) - rows[0].get('t_s', math.nan):.3f}\n")
        f.write("\n")
        for name, values, unit in metrics:
            f.write(
                f"{name}: mean={mean(values):.6f}, std={std(values):.6f}, "
                f"span={span(values):.6f} {unit}\n"
            )
    return report_path


def main():
    args = parse_args()
    path = Path(args.csv)
    rows = read_rows(path)
    if not rows:
        raise SystemExit("CSV has no rows")

    comp_roll = unwrap_degrees(series(rows, "roll_deg"))
    comp_pitch = unwrap_degrees(series(rows, "pitch_deg"))
    yaw_tilt = unwrap_degrees(series(rows, "yaw_tilt_deg"))
    mahony_roll = unwrap_degrees(series(rows, "mahony_roll_deg"))
    mahony_pitch = unwrap_degrees(series(rows, "mahony_pitch_deg"))
    mahony_yaw = unwrap_degrees(series(rows, "mahony_yaw_deg"))

    roll_diff = diff_series(mahony_roll, comp_roll)
    pitch_diff = diff_series(mahony_pitch, comp_pitch)
    yaw_diff = diff_series(mahony_yaw, yaw_tilt, circular=True)

    t0 = rows[0].get("t_s", math.nan)
    t1 = rows[-1].get("t_s", math.nan)
    duration = t1 - t0 if math.isfinite(t0) and math.isfinite(t1) else math.nan

    print(f"file: {path}")
    print(f"rows: {len(rows)}  duration_s: {duration:.3f}")
    print("\nalgorithm outputs:")
    metrics = [
        ("comp_roll", comp_roll, "deg"),
        ("mahony_roll", mahony_roll, "deg"),
        ("mahony_minus_comp_roll", roll_diff, "deg"),
        ("comp_pitch", comp_pitch, "deg"),
        ("mahony_pitch", mahony_pitch, "deg"),
        ("mahony_minus_comp_pitch", pitch_diff, "deg"),
        ("yaw_tilt", yaw_tilt, "deg"),
        ("mahony_yaw", mahony_yaw, "deg"),
        ("mahony_minus_yaw_tilt", yaw_diff, "deg"),
    ]
    for name, values, unit in metrics:
        print_metric(name, values, unit)

    print("\nquick judgement:")
    print(f"- roll dynamic range:  {span(comp_roll):.2f} deg")
    print(f"- pitch dynamic range: {span(comp_pitch):.2f} deg")
    print(f"- roll algorithm std difference:  {std(roll_diff):.3f} deg")
    print(f"- pitch algorithm std difference: {std(pitch_diff):.3f} deg")
    print("- Mahony yaw quality depends strongly on magnetic calibration and heading residual compensation.")

    report_path = write_report(path, rows, metrics)
    print(f"\nSaved report: {report_path}")


if __name__ == "__main__":
    main()
