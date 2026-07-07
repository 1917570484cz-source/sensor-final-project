import argparse
import csv
import math
from pathlib import Path


GYRO_RAW_COLS = ["gx_raw_dps", "gy_raw_dps", "gz_raw_dps"]
GYRO_COMP_COLS = ["gx_dps", "gy_dps", "gz_dps"]
GYRO_PAIRS = [
    ("gx", "gx_raw_dps", "gx_dps"),
    ("gy", "gy_raw_dps", "gy_dps"),
    ("gz", "gz_raw_dps", "gz_dps"),
]


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze gyro Allan deviation from a static SENSOR CSV.")
    parser.add_argument("csv", help="Path to gyro_allan_*.csv or long sensor_static_*.csv")
    parser.add_argument("--min-tau", type=float, default=0.2)
    parser.add_argument("--max-tau", type=float, default=600.0)
    parser.add_argument("--points", type=int, default=40)
    return parser.parse_args()


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_csv(path):
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [{key: to_float(value) for key, value in row.items()} for row in reader]
    rows = [row for row in rows if math.isfinite(row.get("t_s", math.nan))]
    return rows


def median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return math.nan
    mid = n // 2
    if n % 2:
        return values[mid]
    return 0.5 * (values[mid - 1] + values[mid])


def mean(values):
    valid = [v for v in values if math.isfinite(v)]
    if not valid:
        return math.nan
    return sum(valid) / len(valid)


def std(values):
    valid = [v for v in values if math.isfinite(v)]
    if len(valid) < 2:
        return math.nan
    m = mean(valid)
    return math.sqrt(sum((v - m) ** 2 for v in valid) / (len(valid) - 1))


def logspace_int(min_m, max_m, count):
    if max_m < min_m:
        return []
    if min_m == max_m:
        return [min_m]
    values = set()
    log_min = math.log10(min_m)
    log_max = math.log10(max_m)
    for i in range(count):
        x = log_min + (log_max - log_min) * i / max(1, count - 1)
        values.add(max(min_m, min(max_m, int(round(10 ** x)))))
    return sorted(values)


def allan_deviation(values, dt_s, min_tau, max_tau, points):
    valid = [v for v in values if math.isfinite(v)]
    n = len(valid)
    if n < 20:
        return []

    min_m = max(1, int(math.ceil(min_tau / dt_s)))
    max_m = min(int(max_tau / dt_s), n // 3)
    taus = []
    for m in logspace_int(min_m, max_m, points):
        clusters = n // m
        if clusters < 3:
            continue
        averages = []
        for k in range(clusters):
            segment = valid[k * m : (k + 1) * m]
            averages.append(sum(segment) / m)
        diffs = [(averages[i + 1] - averages[i]) for i in range(len(averages) - 1)]
        adev = math.sqrt(0.5 * sum(d * d for d in diffs) / len(diffs))
        taus.append((m * dt_s, adev, clusters))
    return taus


def summarize_axis(series_name, adev_rows):
    if not adev_rows:
        return {}

    arw_candidates = []
    for tau, adev, _ in adev_rows:
        if 0.5 <= tau <= 5.0:
            arw_candidates.append(adev * math.sqrt(tau))
    if not arw_candidates:
        arw_candidates = [adev_rows[0][1] * math.sqrt(adev_rows[0][0])]

    min_tau, min_adev, _ = min(adev_rows, key=lambda row: row[1])
    arw_deg_per_sqrt_s = median(arw_candidates)
    return {
        "series": series_name,
        "arw_deg_per_sqrt_s": arw_deg_per_sqrt_s,
        "arw_deg_per_sqrt_hr": arw_deg_per_sqrt_s * 60.0,
        "bias_instability_dps": min_adev / 0.664,
        "bias_instability_dph": min_adev / 0.664 * 3600.0,
        "min_adev_dps": min_adev,
        "min_adev_tau_s": min_tau,
    }


def main():
    args = parse_args()
    path = Path(args.csv)
    rows = read_csv(path)
    if len(rows) < 20:
        raise SystemExit("Not enough rows for Allan analysis")

    t = [row["t_s"] for row in rows]
    dts = [t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] > t[i]]
    dt_s = median(dts)
    duration_s = t[-1] - t[0]

    out_dir = path.parent
    stem = path.stem
    allan_csv = out_dir / f"{stem}_allan.csv"
    report_path = out_dir / f"{stem}_allan.txt"

    all_adev = {}
    summaries = []
    with allan_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["series", "tau_s", "adev_dps", "clusters"])
        for axis in GYRO_RAW_COLS + GYRO_COMP_COLS:
            if axis not in rows[0]:
                continue
            values = [row.get(axis, math.nan) for row in rows]
            adev_rows = allan_deviation(values, dt_s, args.min_tau, args.max_tau, args.points)
            all_adev[axis] = adev_rows
            summaries.append(summarize_axis(axis, adev_rows))
            for tau, adev, clusters in adev_rows:
                writer.writerow([axis, f"{tau:.6f}", f"{adev:.9f}", clusters])

    with report_path.open("w", encoding="utf-8") as f:
        f.write("Gyro Allan deviation analysis\n\n")
        f.write(f"source: {path}\n")
        f.write(f"rows: {len(rows)}\n")
        f.write(f"duration_s: {duration_s:.3f}\n")
        f.write(f"sample_dt_s: {dt_s:.6f}\n")
        f.write(f"sample_rate_hz: {1.0 / dt_s:.3f}\n\n")
        f.write("static gyro summary:\n")
        for _, raw_col, comp_col in GYRO_PAIRS:
            if raw_col in rows[0]:
                values = [row.get(raw_col, math.nan) for row in rows]
                f.write(f"{raw_col}: mean={mean(values):.9f} dps, std={std(values):.9f} dps\n")
            if comp_col in rows[0]:
                values = [row.get(comp_col, math.nan) for row in rows]
                f.write(f"{comp_col}: mean={mean(values):.9f} dps, std={std(values):.9f} dps\n")
        f.write("\nAllan summary:\n")
        for item in summaries:
            f.write(
                f"{item['series']}: ARW={item['arw_deg_per_sqrt_hr']:.6f} deg/sqrt(hr), "
                f"bias_instability={item['bias_instability_dph']:.6f} deg/hr, "
                f"min_adev={item['min_adev_dps']:.9f} dps at tau={item['min_adev_tau_s']:.3f}s\n"
            )
        f.write("\nCompensation improvement:\n")
        summary_by_name = {item["series"]: item for item in summaries}
        for axis_name, raw_col, comp_col in GYRO_PAIRS:
            raw_item = summary_by_name.get(raw_col)
            comp_item = summary_by_name.get(comp_col)
            if not raw_item or not comp_item:
                continue
            raw_bias = raw_item["bias_instability_dph"]
            comp_bias = comp_item["bias_instability_dph"]
            raw_arw = raw_item["arw_deg_per_sqrt_hr"]
            comp_arw = comp_item["arw_deg_per_sqrt_hr"]
            f.write(
                f"{axis_name}: bias_improvement={raw_bias / comp_bias:.3f}x, "
                f"arw_improvement={raw_arw / comp_arw:.3f}x\n"
            )

    print(f"file: {path}")
    print(f"rows: {len(rows)}  duration_s: {duration_s:.3f}  sample_rate_hz: {1.0 / dt_s:.3f}")
    print("\nstatic gyro summary:")
    for _, raw_col, comp_col in GYRO_PAIRS:
        if raw_col in rows[0]:
            values = [row.get(raw_col, math.nan) for row in rows]
            print(f"{raw_col:11s} mean={mean(values): .6f} dps  std={std(values): .6f} dps")
        if comp_col in rows[0]:
            values = [row.get(comp_col, math.nan) for row in rows]
            print(f"{comp_col:11s} mean={mean(values): .6f} dps  std={std(values): .6f} dps")
    print("\nAllan summary:")
    for item in summaries:
        print(
            f"{item['series']:11s} ARW={item['arw_deg_per_sqrt_hr']:.4f} deg/sqrt(hr)  "
            f"bias_instability={item['bias_instability_dph']:.3f} deg/hr  "
            f"min_tau={item['min_adev_tau_s']:.2f}s"
        )
    print("\nCompensation improvement:")
    summary_by_name = {item["series"]: item for item in summaries}
    for axis_name, raw_col, comp_col in GYRO_PAIRS:
        raw_item = summary_by_name.get(raw_col)
        comp_item = summary_by_name.get(comp_col)
        if not raw_item or not comp_item:
            continue
        raw_bias = raw_item["bias_instability_dph"]
        comp_bias = comp_item["bias_instability_dph"]
        raw_arw = raw_item["arw_deg_per_sqrt_hr"]
        comp_arw = comp_item["arw_deg_per_sqrt_hr"]
        print(
            f"{axis_name}: bias {raw_bias:.3f}->{comp_bias:.3f} deg/hr "
            f"({raw_bias / comp_bias:.2f}x), "
            f"ARW {raw_arw:.4f}->{comp_arw:.4f} deg/sqrt(hr) ({raw_arw / comp_arw:.2f}x)"
        )
    print(f"\nSaved Allan CSV: {allan_csv}")
    print(f"Saved report:    {report_path}")


if __name__ == "__main__":
    main()
