import argparse
import csv
import math
from datetime import datetime
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Cross-validate heading residual compensation.")
    parser.add_argument(
        "inputs",
        nargs="*",
        help="heading_12dir_*.csv files. If omitted, use all full 12-direction CSV files in data/.",
    )
    parser.add_argument("--measurement", default="yaw_flat_deg", choices=["yaw_flat_deg", "yaw_tilt_deg", "mahony_yaw_deg"])
    parser.add_argument("--order", type=int, default=2, help="Harmonic order, usually 2 or 3.")
    parser.add_argument("--train-count", type=int, default=0, help="Number of input files used for training; default is all but last.")
    parser.add_argument("--min-directions", type=int, default=12)
    return parser.parse_args()


def circular_error(measured, reference):
    return (measured - reference + 180.0) % 360.0 - 180.0


def wrap360(angle):
    return angle % 360.0


def rmse(errors):
    if not errors:
        return math.nan
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def abs_mean(errors):
    if not errors:
        return math.nan
    return sum(abs(e) for e in errors) / len(errors)


def read_group(path, measurement):
    rows = []
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                reference = float(row["reference_deg"])
                measured = float(row[measurement])
            except (KeyError, ValueError):
                continue
            if math.isfinite(reference) and math.isfinite(measured):
                rows.append({"reference": reference, "measured": measured, "source": str(path)})
    return rows


def feature_vector(measured_deg, order):
    rad = math.radians(measured_deg)
    features = [1.0]
    for k in range(1, order + 1):
        features.append(math.sin(k * rad))
        features.append(math.cos(k * rad))
    return features


def solve_square(a, b):
    n = len(a)
    aug = [a[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise SystemExit("Least-squares matrix is singular. Try lower --order or more training data.")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        for c in range(col, n + 1):
            aug[col][c] /= scale
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            for c in range(col, n + 1):
                aug[r][c] -= factor * aug[col][c]
    return [row[-1] for row in aug]


def fit_model(rows, order):
    # Fit correction(measured) = reference - measured, in degrees.
    # Runtime use: heading_corrected = heading + correction(heading).
    m = 1 + 2 * order
    normal = [[0.0 for _ in range(m)] for _ in range(m)]
    rhs = [0.0 for _ in range(m)]
    for row in rows:
        x = feature_vector(row["measured"], order)
        y = -circular_error(row["measured"], row["reference"])
        for i in range(m):
            rhs[i] += x[i] * y
            for j in range(m):
                normal[i][j] += x[i] * x[j]
    return solve_square(normal, rhs)


def predict_correction(measured, coeffs, order):
    x = feature_vector(measured, order)
    return sum(c * v for c, v in zip(coeffs, x))


def evaluate(rows, coeffs, order):
    raw_errors = []
    corrected_errors = []
    evaluated = []
    for row in rows:
        raw_error = circular_error(row["measured"], row["reference"])
        correction = predict_correction(row["measured"], coeffs, order)
        corrected = wrap360(row["measured"] + correction)
        corrected_error = circular_error(corrected, row["reference"])
        raw_errors.append(raw_error)
        corrected_errors.append(corrected_error)
        evaluated.append(
            {
                **row,
                "raw_error": raw_error,
                "correction": correction,
                "corrected": corrected,
                "corrected_error": corrected_error,
            }
        )
    return raw_errors, corrected_errors, evaluated


def summarize(errors):
    return {
        "rmse": rmse(errors),
        "abs_mean": abs_mean(errors),
        "abs_max": max((abs(e) for e in errors), default=math.nan),
    }


def discover_inputs(root, measurement, min_directions):
    data_dir = root / "data"
    paths = []
    for path in sorted(data_dir.glob("heading_12dir_*.csv"), key=lambda p: p.stat().st_mtime):
        rows = read_group(path, measurement)
        refs = {round(row["reference"], 6) for row in rows}
        if len(refs) >= min_directions:
            paths.append(path)
    return paths


def format_coeffs(coeffs, order):
    lines = [f"offset={coeffs[0]:.9f}"]
    idx = 1
    for k in range(1, order + 1):
        lines.append(f"sin{k}={coeffs[idx]:.9f}")
        lines.append(f"cos{k}={coeffs[idx + 1]:.9f}")
        idx += 2
    return "\n".join(lines)


def format_c_array(coeffs):
    return (
        "static const float HEADING_RESIDUAL_COEF[] = {\n"
        + "\n".join(f"    {value:.9f}f," for value in coeffs)
        + "\n};"
    )


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    inputs = [Path(p) for p in args.inputs] if args.inputs else discover_inputs(root, args.measurement, args.min_directions)
    if len(inputs) < 2:
        raise SystemExit("Need at least two full heading CSV files. Prefer three: first two train, last validates.")

    groups = [(path, read_group(path, args.measurement)) for path in inputs]
    train_count = args.train_count if args.train_count > 0 else len(groups) - 1
    if train_count < 1 or train_count >= len(groups):
        raise SystemExit("--train-count must leave at least one validation file.")

    train_rows = [row for _, rows in groups[:train_count] for row in rows]
    valid_rows = [row for _, rows in groups[train_count:] for row in rows]
    coeffs = fit_model(train_rows, args.order)
    train_raw, train_corr, train_eval = evaluate(train_rows, coeffs, args.order)
    valid_raw, valid_corr, valid_eval = evaluate(valid_rows, coeffs, args.order)

    data_dir = root / "data"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = data_dir / f"heading_residual_cv_{stamp}.txt"
    csv_path = data_dir / f"heading_residual_cv_{stamp}.csv"

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        fieldnames = ["phase", "source", "reference", "measured", "raw_error", "correction", "corrected", "corrected_error"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for phase, rows in (("train", train_eval), ("valid", valid_eval)):
            for row in rows:
                writer.writerow({key: row.get(key, phase if key == "phase" else "") for key in fieldnames})

    train_raw_s = summarize(train_raw)
    train_corr_s = summarize(train_corr)
    valid_raw_s = summarize(valid_raw)
    valid_corr_s = summarize(valid_corr)
    with report_path.open("w", encoding="utf-8") as f:
        f.write("Heading residual compensation cross validation\n\n")
        f.write(f"measurement: {args.measurement}\n")
        f.write(f"harmonic_order: {args.order}\n")
        f.write("train_files:\n")
        for path, _ in groups[:train_count]:
            f.write(f"- {path}\n")
        f.write("validation_files:\n")
        for path, _ in groups[train_count:]:
            f.write(f"- {path}\n")
        f.write("\ncoefficients_deg:\n")
        f.write(format_coeffs(coeffs, args.order))
        f.write("\n\n")
        f.write(format_c_array(coeffs))
        f.write("\n\nsummary_deg:\n")
        f.write(
            f"train_raw_rmse={train_raw_s['rmse']:.3f}, "
            f"train_raw_abs_mean={train_raw_s['abs_mean']:.3f}, "
            f"train_raw_abs_max={train_raw_s['abs_max']:.3f}\n"
        )
        f.write(
            f"train_corrected_rmse={train_corr_s['rmse']:.3f}, "
            f"train_corrected_abs_mean={train_corr_s['abs_mean']:.3f}, "
            f"train_corrected_abs_max={train_corr_s['abs_max']:.3f}\n"
        )
        f.write(
            f"valid_raw_rmse={valid_raw_s['rmse']:.3f}, "
            f"valid_raw_abs_mean={valid_raw_s['abs_mean']:.3f}, "
            f"valid_raw_abs_max={valid_raw_s['abs_max']:.3f}\n"
        )
        f.write(
            f"valid_corrected_rmse={valid_corr_s['rmse']:.3f}, "
            f"valid_corrected_abs_mean={valid_corr_s['abs_mean']:.3f}, "
            f"valid_corrected_abs_max={valid_corr_s['abs_max']:.3f}\n"
        )

    print("Heading residual compensation cross validation")
    print(f"measurement: {args.measurement}")
    print(f"train files: {train_count}, validation files: {len(groups) - train_count}")
    print("\nvalidation before correction:")
    print(
        f"RMSE={valid_raw_s['rmse']:.3f} deg, "
        f"abs_mean={valid_raw_s['abs_mean']:.3f} deg, "
        f"abs_max={valid_raw_s['abs_max']:.3f} deg"
    )
    print("validation after correction:")
    print(
        f"RMSE={valid_corr_s['rmse']:.3f} deg, "
        f"abs_mean={valid_corr_s['abs_mean']:.3f} deg, "
        f"abs_max={valid_corr_s['abs_max']:.3f} deg"
    )
    print(f"\nSaved CSV:    {csv_path}")
    print(f"Saved report: {report_path}")


if __name__ == "__main__":
    main()
