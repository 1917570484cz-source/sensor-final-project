import argparse
import csv
import math
from datetime import datetime
from pathlib import Path

import numpy as np


IMU_COLS = ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]


def parse_args():
    parser = argparse.ArgumentParser(description="Train a tiny 1D-convolution IMU denoiser on static data.")
    parser.add_argument("csv", help="Static SENSOR CSV file.")
    parser.add_argument("--window", type=int, default=21, help="Odd convolution window length.")
    parser.add_argument("--target-window", type=int, default=101, help="Long smoothing window used as weak clean target.")
    parser.add_argument("--ridge", type=float, default=1e-3, help="Ridge regularization.")
    parser.add_argument("--kalman-q-scale", type=float, default=0.02, help="Kalman process variance as a fraction of measurement variance.")
    parser.add_argument("--train-ratio", type=float, default=0.7)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_csv(path):
    rows = []
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            item = {k: to_float(v) for k, v in row.items()}
            if all(math.isfinite(item.get(c, math.nan)) for c in IMU_COLS):
                rows.append(item)
    return rows


def moving_average(x, window):
    pad = window // 2
    xp = np.pad(x, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(xp, kernel, mode="valid")


def kalman_1d(z, process_var, meas_var):
    out = np.zeros_like(z, dtype=float)
    x = float(z[0])
    p = float(meas_var)
    q = float(process_var)
    r = float(meas_var)
    for i, value in enumerate(z):
        p += q
        k = p / (p + r)
        x += k * (float(value) - x)
        p = (1.0 - k) * p
        out[i] = x
    return out


def build_windows(x, window):
    pad = window // 2
    xp = np.pad(x, (pad, pad), mode="edge")
    return np.vstack([xp[i : i + window] for i in range(len(x))])


def train_conv_denoiser(x_train, y_train, window, ridge):
    xw = build_windows(x_train, window)
    design = np.column_stack([xw, np.ones(len(xw))])
    reg = np.eye(design.shape[1]) * ridge
    reg[-1, -1] = 0.0
    weights = np.linalg.solve(design.T @ design + reg, design.T @ y_train)
    return weights


def apply_conv_denoiser(x, weights, window):
    xw = build_windows(x, window)
    design = np.column_stack([xw, np.ones(len(xw))])
    return design @ weights


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else math.nan


def std(x):
    return float(np.std(x, ddof=1)) if len(x) > 1 else math.nan


def snr_db(signal_level, noise_std):
    if signal_level <= 0.0 or noise_std <= 0.0:
        return math.nan
    return 20.0 * math.log10(signal_level / noise_std)


def fmt(value, digits=6):
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def main():
    args = parse_args()
    if args.window < 3 or args.window % 2 == 0:
        raise SystemExit("--window must be an odd integer >= 3")
    if args.target_window <= args.window or args.target_window % 2 == 0:
        raise SystemExit("--target-window must be an odd integer larger than --window")

    source = Path(args.csv)
    rows = read_csv(source)
    if len(rows) < args.window * 4:
        raise SystemExit("Not enough static rows for denoise experiment")

    data = {col: np.array([row[col] for row in rows], dtype=float) for col in IMU_COLS}
    split = int(len(rows) * args.train_ratio)
    split = max(args.window * 2, min(len(rows) - args.window * 2, split))

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = source.with_name(f"{source.stem}_ai_imu_denoise_{stamp}.csv")
    out_txt = Path(args.output) if args.output else source.with_name(f"{source.stem}_ai_imu_denoise_{stamp}.txt")

    results = []
    denoised = {}
    lowpass = {}
    hybrid = {}
    weights_by_col = {}
    for col in IMU_COLS:
        x = data[col]
        target_all = moving_average(x, args.target_window)
        x_train = x[:split]
        y_train = target_all[:split]
        x_val = x[split:]
        y_val = target_all[split:]

        weights = train_conv_denoiser(x_train, y_train, args.window, args.ridge)
        weights_by_col[col] = weights
        ai = apply_conv_denoiser(x, weights, args.window)
        lp = moving_average(x, args.window)
        train_residual = ai[:split] - target_all[:split]
        meas_var = float(np.var(train_residual)) if len(train_residual) > 1 else 1e-8
        process_var = max(meas_var * args.kalman_q_scale, 1e-10)
        hy = kalman_1d(ai, process_var, max(meas_var, 1e-10))
        denoised[col] = ai
        lowpass[col] = lp
        hybrid[col] = hy

        target = float(np.mean(y_train))
        raw_noise = x_val - y_val
        lp_noise = lp[split:] - y_val
        ai_noise = ai[split:] - y_val
        hybrid_noise = hy[split:] - y_val

        raw_std = std(raw_noise)
        lp_std = std(lp_noise)
        ai_std = std(ai_noise)
        hybrid_std = std(hybrid_noise)
        signal_level = max(abs(target), rms(y_val), 1e-12)
        raw_snr = snr_db(signal_level, raw_std)
        lp_snr = snr_db(signal_level, lp_std)
        ai_snr = snr_db(signal_level, ai_std)
        hybrid_snr = snr_db(signal_level, hybrid_std)

        results.append(
            {
                "axis": col,
                "target_mean": target,
                "raw_std": raw_std,
                "lowpass_std": lp_std,
                "ai_std": ai_std,
                "hybrid_std": hybrid_std,
                "lowpass_noise_reduction": raw_std / lp_std if lp_std > 0 else math.nan,
                "ai_noise_reduction": raw_std / ai_std if ai_std > 0 else math.nan,
                "hybrid_noise_reduction": raw_std / hybrid_std if hybrid_std > 0 else math.nan,
                "raw_snr_db": raw_snr,
                "lowpass_snr_db": lp_snr,
                "ai_snr_db": ai_snr,
                "hybrid_snr_db": hybrid_snr,
                "lowpass_snr_gain_db": lp_snr - raw_snr if math.isfinite(lp_snr) and math.isfinite(raw_snr) else math.nan,
                "ai_snr_gain_db": ai_snr - raw_snr if math.isfinite(ai_snr) and math.isfinite(raw_snr) else math.nan,
                "hybrid_snr_gain_db": hybrid_snr - raw_snr if math.isfinite(hybrid_snr) and math.isfinite(raw_snr) else math.nan,
            }
        )

    with out_csv.open("w", newline="", encoding="utf-8") as f:
        fieldnames = ["index"]
        for col in IMU_COLS:
            fieldnames.extend([col, f"{col}_lowpass", f"{col}_ai", f"{col}_ai_kalman"])
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(len(rows)):
            row = {"index": i}
            for col in IMU_COLS:
                row[col] = data[col][i]
                row[f"{col}_lowpass"] = lowpass[col][i]
                row[f"{col}_ai"] = denoised[col][i]
                row[f"{col}_ai_kalman"] = hybrid[col][i]
            writer.writerow(row)

    lines = [
        "AI-enhanced IMU denoise experiment",
        "",
        f"source: {source}",
        f"rows: {len(rows)}",
        f"train_rows: {split}",
        f"validation_rows: {len(rows) - split}",
        f"model: tiny 1D convolution ridge denoiser, window={args.window}",
        f"hybrid: 1D convolution output followed by first-order Kalman smoothing, q_scale={args.kalman_q_scale}",
        f"weak clean target: moving_average(window={args.target_window})",
        "",
        "Validation summary:",
    ]
    for r in results:
        lines.append(
            f"{r['axis']}: raw_std={fmt(r['raw_std'])}, lowpass_std={fmt(r['lowpass_std'])}, "
            f"ai_std={fmt(r['ai_std'])}, hybrid_std={fmt(r['hybrid_std'])}, "
            f"lowpass_gain={fmt(r['lowpass_snr_gain_db'], 3)} dB, "
            f"ai_gain={fmt(r['ai_snr_gain_db'], 3)} dB, "
            f"hybrid_gain={fmt(r['hybrid_snr_gain_db'], 3)} dB"
        )

    accel = [r for r in results if r["axis"].startswith("a")]
    gyro = [r for r in results if r["axis"].startswith("g")]
    lines.extend(
        [
            "",
            f"accel_mean_lowpass_snr_gain_db: {fmt(float(np.mean([r['lowpass_snr_gain_db'] for r in accel])), 3)}",
            f"accel_mean_ai_snr_gain_db: {fmt(float(np.mean([r['ai_snr_gain_db'] for r in accel])), 3)}",
            f"accel_mean_hybrid_snr_gain_db: {fmt(float(np.mean([r['hybrid_snr_gain_db'] for r in accel])), 3)}",
            f"gyro_mean_lowpass_snr_gain_db: {fmt(float(np.mean([r['lowpass_snr_gain_db'] for r in gyro])), 3)}",
            f"gyro_mean_ai_snr_gain_db: {fmt(float(np.mean([r['ai_snr_gain_db'] for r in gyro])), 3)}",
            f"gyro_mean_hybrid_snr_gain_db: {fmt(float(np.mean([r['hybrid_snr_gain_db'] for r in gyro])), 3)}",
            "",
            "Judgement:",
            "This validates static IMU denoising, 1D convolution denoise, and 1D convolution + first-order Kalman hybrid denoise.",
            "The inertial-position drift target is intentionally not evaluated in this no-GPS version because no trajectory ground truth was collected.",
            "",
            f"Saved denoised CSV: {out_csv}",
        ]
    )

    report = "\n".join(lines) + "\n"
    out_txt.write_text(report, encoding="utf-8")
    print(report)
    print(f"Saved report: {out_txt}")


if __name__ == "__main__":
    main()
