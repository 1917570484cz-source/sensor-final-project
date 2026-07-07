import argparse
import csv
import math
from datetime import datetime
from pathlib import Path


EARTH_R = 6378137.0
G0 = 9.80665


def parse_args():
    parser = argparse.ArgumentParser(description="Offline GNSS/INS loose coupling with a 2D Kalman filter.")
    parser.add_argument("--sensor-csv", required=True, help="SENSOR CSV from log_sensor_csv.py.")
    parser.add_argument("--gps-parsed-csv", required=True, help="Firmware-parsed GPS CSV from log_sensor_csv.py.")
    parser.add_argument("--output", default="")
    parser.add_argument("--report", default="")
    parser.add_argument("--fig", default="")
    parser.add_argument("--accel-gain", type=float, default=0.02, help="Scale factor for IMU horizontal acceleration in prediction.")
    parser.add_argument("--gps-sigma-m", type=float, default=3.0, help="GPS position noise when HDOP is unavailable.")
    parser.add_argument("--process-accel-sigma", type=float, default=1.5, help="Process acceleration noise, m/s^2.")
    return parser.parse_args()


def read_csv(path):
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def to_int(value):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return -1


def finite(values):
    return [v for v in values if math.isfinite(v)]


def mean(values):
    values = finite(values)
    return sum(values) / len(values) if values else math.nan


def stdev(values):
    values = finite(values)
    if len(values) < 2:
        return math.nan
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def rms(values):
    values = finite(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else math.nan


def fmt(value, digits=3):
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def latlon_to_enu(lat, lon, lat0, lon0):
    lat_r = math.radians(lat)
    lat0_r = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east = EARTH_R * math.cos((lat_r + lat0_r) * 0.5) * dlon
    north = EARTH_R * dlat
    return east, north


def load_gps_rows(path):
    rows = []
    for row in read_csv(path):
        lat = to_float(row.get("lat_deg"))
        lon = to_float(row.get("lon_deg"))
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        kind = row.get("kind", "")
        quality = to_int(row.get("status_or_quality"))
        status = row.get("status_or_quality", "")
        checksum_ok = row.get("checksum_ok") == "1"
        if not checksum_ok:
            continue
        if kind == "GGA" and quality <= 0:
            continue
        if kind == "RMC" and status != "A":
            continue
        t_us = to_float(row.get("esp_t_us"))
        if not math.isfinite(t_us):
            continue
        hdop = to_float(row.get("hdop"))
        rows.append(
            {
                "t_us": t_us,
                "kind": kind,
                "lat": lat,
                "lon": lon,
                "hdop": hdop,
            }
        )
    rows.sort(key=lambda r: r["t_us"])
    return rows


def mat4_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat4_transpose(a):
    return [[a[j][i] for j in range(4)] for i in range(4)]


def predict_state(x, p, dt, ae, an, q_accel):
    dt2 = dt * dt
    x = [
        x[0] + x[2] * dt + 0.5 * ae * dt2,
        x[1] + x[3] * dt + 0.5 * an * dt2,
        x[2] + ae * dt,
        x[3] + an * dt,
    ]
    f = [
        [1.0, 0.0, dt, 0.0],
        [0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    q = q_accel * q_accel
    q11 = 0.25 * dt2 * dt2 * q
    q13 = 0.5 * dt2 * dt * q
    q33 = dt2 * q
    q_mat = [
        [q11, 0.0, q13, 0.0],
        [0.0, q11, 0.0, q13],
        [q13, 0.0, q33, 0.0],
        [0.0, q13, 0.0, q33],
    ]
    fp = mat4_mul(f, p)
    p = mat4_mul(fp, mat4_transpose(f))
    for i in range(4):
        for j in range(4):
            p[i][j] += q_mat[i][j]
    return x, p


def update_position(x, p, ze, zn, sigma):
    # H selects position states. The 2x2 innovation covariance can be inverted directly.
    r = sigma * sigma
    y0 = ze - x[0]
    y1 = zn - x[1]
    s00 = p[0][0] + r
    s01 = p[0][1]
    s10 = p[1][0]
    s11 = p[1][1] + r
    det = s00 * s11 - s01 * s10
    if abs(det) < 1e-12:
        return x, p, math.nan
    inv_s = [[s11 / det, -s01 / det], [-s10 / det, s00 / det]]
    k = []
    for i in range(4):
        k.append([
            p[i][0] * inv_s[0][0] + p[i][1] * inv_s[1][0],
            p[i][0] * inv_s[0][1] + p[i][1] * inv_s[1][1],
        ])
    x = [x[i] + k[i][0] * y0 + k[i][1] * y1 for i in range(4)]
    hp = [p[0][:], p[1][:]]
    new_p = [[p[i][j] - k[i][0] * hp[0][j] - k[i][1] * hp[1][j] for j in range(4)] for i in range(4)]
    residual = math.sqrt(y0 * y0 + y1 * y1)
    return x, new_p, residual


def sensor_accel_nav(row, accel_gain):
    ax = to_float(row.get("ax_g"))
    ay = to_float(row.get("ay_g"))
    az = to_float(row.get("az_g"))
    roll = math.radians(to_float(row.get("roll_deg")))
    pitch = math.radians(to_float(row.get("pitch_deg")))
    yaw = math.radians(to_float(row.get("yaw_tilt_deg")))
    if not all(math.isfinite(v) for v in (ax, ay, az, roll, pitch, yaw)):
        return 0.0, 0.0

    # Body-to-navigation rotation, ZYX convention. Then remove gravity from up axis.
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    ae = (r00 * ax + r01 * ay + r02 * az) * G0 * accel_gain
    an = (r10 * ax + r11 * ay + r12 * az) * G0 * accel_gain
    au = (r20 * ax + r21 * ay + r22 * az) * G0 - G0
    if not all(math.isfinite(v) for v in (ae, an, au)):
        return 0.0, 0.0
    # Clamp acceleration to reduce the effect of attitude/magnetic heading errors in static tests.
    limit = 3.0
    ae = max(-limit, min(limit, ae))
    an = max(-limit, min(limit, an))
    return ae, an


def svg_plot(path, gps_points, fused_points):
    path = Path(path)
    width, height = 900, 560
    left, right, top, bottom = 70, 35, 55, 70
    plot_w, plot_h = width - left - right, height - top - bottom
    all_e = [p[0] for p in gps_points + fused_points]
    all_n = [p[1] for p in gps_points + fused_points]
    if not all_e or not all_n:
        return
    emin, emax = min(all_e), max(all_e)
    nmin, nmax = min(all_n), max(all_n)
    span = max(emax - emin, nmax - nmin, 1.0)
    ec = 0.5 * (emin + emax)
    nc = 0.5 * (nmin + nmax)
    emin, emax = ec - span * 0.6, ec + span * 0.6
    nmin, nmax = nc - span * 0.6, nc + span * 0.6

    def sx(e):
        return left + (e - emin) * plot_w / (emax - emin)

    def sy(n):
        return top + plot_h - (n - nmin) * plot_h / (nmax - nmin)

    def poly(points):
        return " ".join(f"{sx(e):.1f},{sy(n):.1f}" for e, n in points)

    gps_circles = "\n".join(
        f'<circle cx="{sx(e):.1f}" cy="{sy(n):.1f}" r="3" fill="#2563eb" opacity="0.75"/>'
        for e, n in gps_points
    )
    fused_line = f'<polyline points="{poly(fused_points)}" fill="none" stroke="#dc2626" stroke-width="2.2"/>'
    body = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>text{{font-family:Arial,"Microsoft YaHei",sans-serif;font-size:13px;fill:#111827}}.title{{font-size:18px;font-weight:700}}.axis{{stroke:#374151;stroke-width:1}}.grid{{stroke:#e5e7eb;stroke-width:1}}</style>
<rect width="{width}" height="{height}" fill="white"/>
<text x="{left}" y="30" class="title">GNSS/INS 松耦合轨迹（局部 EN 坐标）</text>
<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_h}" class="axis"/>
<line x1="{left}" y1="{top+plot_h}" x2="{left+plot_w}" y2="{top+plot_h}" class="axis"/>
<text x="{width/2-40}" y="{height-22}">East / m</text>
<text x="16" y="{top+plot_h/2}" transform="rotate(-90 16 {top+plot_h/2})">North / m</text>
{gps_circles}
{fused_line}
<circle cx="{left+plot_w-185}" cy="31" r="5" fill="#2563eb"/><text x="{left+plot_w-174}" y="36">GPS 观测</text>
<line x1="{left+plot_w-92}" y1="31" x2="{left+plot_w-62}" y2="31" stroke="#dc2626" stroke-width="2.2"/><text x="{left+plot_w-55}" y="36">融合轨迹</text>
</svg>
'''
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body, encoding="utf-8")


def main():
    args = parse_args()
    sensor_rows = read_csv(args.sensor_csv)
    gps_rows = load_gps_rows(args.gps_parsed_csv)
    if len(sensor_rows) < 2:
        raise SystemExit("sensor CSV has too few rows")
    if len(gps_rows) < 2:
        raise SystemExit("GPS parsed CSV has too few valid rows")

    lat0 = mean([r["lat"] for r in gps_rows[: min(8, len(gps_rows))]])
    lon0 = mean([r["lon"] for r in gps_rows[: min(8, len(gps_rows))]])
    for row in gps_rows:
        row["east"], row["north"] = latlon_to_enu(row["lat"], row["lon"], lat0, lon0)

    first_gps = gps_rows[0]
    x = [first_gps["east"], first_gps["north"], 0.0, 0.0]
    p = [
        [25.0, 0.0, 0.0, 0.0],
        [0.0, 25.0, 0.0, 0.0],
        [0.0, 0.0, 4.0, 0.0],
        [0.0, 0.0, 0.0, 4.0],
    ]

    out_rows = []
    gps_idx = 0
    last_t = None
    residuals = []
    gps_points = []
    fused_at_gps = []

    for row in sensor_rows:
        t_s = to_float(row.get("t_s"))
        if not math.isfinite(t_s):
            continue
        t_us = t_s * 1000000.0
        if last_t is None:
            last_t = t_us
        dt = max(0.001, min(0.25, (t_us - last_t) / 1000000.0))
        last_t = t_us
        ae, an = sensor_accel_nav(row, args.accel_gain)
        x, p = predict_state(x, p, dt, ae, an, args.process_accel_sigma)

        update_count = 0
        last_residual = math.nan
        while gps_idx < len(gps_rows) and gps_rows[gps_idx]["t_us"] <= t_us:
            gps = gps_rows[gps_idx]
            hdop = gps["hdop"]
            sigma = args.gps_sigma_m * (hdop if math.isfinite(hdop) and hdop > 0.0 else 1.0)
            x, p, last_residual = update_position(x, p, gps["east"], gps["north"], sigma)
            residuals.append(last_residual)
            gps_points.append((gps["east"], gps["north"]))
            fused_at_gps.append((x[0], x[1]))
            update_count += 1
            gps_idx += 1

        out_rows.append(
            {
                "t_s": f"{t_s:.3f}",
                "east_m": f"{x[0]:.3f}",
                "north_m": f"{x[1]:.3f}",
                "ve_mps": f"{x[2]:.4f}",
                "vn_mps": f"{x[3]:.4f}",
                "ae_mps2": f"{ae:.4f}",
                "an_mps2": f"{an:.4f}",
                "gps_updates": str(update_count),
                "last_gps_residual_m": f"{last_residual:.3f}" if math.isfinite(last_residual) else "",
                "p_pos_e_m2": f"{p[0][0]:.3f}",
                "p_pos_n_m2": f"{p[1][1]:.3f}",
            }
        )

    sensor_path = Path(args.sensor_csv)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.output) if args.output else sensor_path.with_name(f"{sensor_path.stem}_gnss_ins_{stamp}.csv")
    report_path = Path(args.report) if args.report else sensor_path.with_name(f"{sensor_path.stem}_gnss_ins_{stamp}.txt")
    fig_path = Path(args.fig) if args.fig else sensor_path.parents[0].parents[0] / "figures" / f"{sensor_path.stem}_gnss_ins_{stamp}.svg"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        writer.writeheader()
        writer.writerows(out_rows)

    gps_e = [p[0] for p in gps_points]
    gps_n = [p[1] for p in gps_points]
    fused_e = [p[0] for p in fused_at_gps]
    fused_n = [p[1] for p in fused_at_gps]
    gps_spread = [math.hypot(e - mean(gps_e), n - mean(gps_n)) for e, n in gps_points]
    fused_spread = [math.hypot(e - mean(fused_e), n - mean(fused_n)) for e, n in fused_at_gps]

    lines = [
        "GNSS/INS loose coupling summary",
        "",
        f"sensor_csv: {args.sensor_csv}",
        f"gps_parsed_csv: {args.gps_parsed_csv}",
        f"output_csv: {out_path}",
        f"figure_svg: {fig_path}",
        "",
        "Model:",
        "state = [east_m, north_m, ve_mps, vn_mps]",
        f"IMU acceleration is used for prediction with accel_gain={args.accel_gain}; GPS position is used for Kalman measurement update.",
        "",
        f"sensor_rows: {len(sensor_rows)}",
        f"gps_valid_updates_used: {len(gps_points)}",
        f"gps_residual_mean_m: {fmt(mean(residuals))}",
        f"gps_residual_rms_m: {fmt(rms(residuals))}",
        f"gps_residual_max_m: {fmt(max(finite(residuals)) if finite(residuals) else math.nan)}",
        f"gps_position_spread_mean_m: {fmt(mean(gps_spread))}",
        f"fused_position_spread_mean_m: {fmt(mean(fused_spread))}",
        f"final_east_north_m: {fmt(x[0])}, {fmt(x[1])}",
        f"final_velocity_mps: {fmt(x[2], 4)}, {fmt(x[3], 4)}",
        "",
        "Judgement:",
        "The loose-coupled GNSS/INS pipeline is complete and can run on the collected data.",
        "This static dataset verifies algorithm execution and GPS update stability; a walking/outdoor dataset is needed to evaluate trajectory accuracy.",
    ]
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    svg_plot(fig_path, gps_points, [(to_float(r["east_m"]), to_float(r["north_m"])) for r in out_rows])

    print("\n".join(lines))
    print("")
    print(f"Saved CSV: {out_path}")
    print(f"Saved report: {report_path}")
    print(f"Saved figure: {fig_path}")


if __name__ == "__main__":
    main()
