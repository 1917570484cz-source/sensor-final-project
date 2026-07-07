import argparse
import json
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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
]


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>实时姿态显示</title>
<style>
:root { color-scheme: light; --bg:#f5f7fb; --panel:#ffffff; --line:#d7dde8; --text:#162033; --muted:#667085; --blue:#2563eb; --green:#16a34a; --red:#dc2626; --amber:#d97706; }
* { box-sizing: border-box; }
body { margin:0; font-family: "Microsoft YaHei", Arial, sans-serif; background:var(--bg); color:var(--text); }
main { max-width: 1180px; margin: 0 auto; padding: 20px; }
header { display:flex; align-items:flex-end; justify-content:space-between; gap:16px; margin-bottom:16px; }
h1 { font-size:22px; margin:0; }
.status { color:var(--muted); font-size:13px; }
.grid { display:grid; grid-template-columns: 1.15fr .85fr; gap:16px; align-items:stretch; }
.panel { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:16px; box-shadow: 0 1px 2px rgba(16,24,40,.04); }
.panel h2 { font-size:15px; margin:0 0 12px; }
.cards { display:grid; grid-template-columns: repeat(3, 1fr); gap:12px; margin-bottom:16px; }
.card { background:#fbfcff; border:1px solid var(--line); border-radius:8px; padding:12px; min-height:82px; }
.label { color:var(--muted); font-size:12px; }
.value { font-size:30px; font-variant-numeric: tabular-nums; margin-top:7px; }
.unit { font-size:14px; color:var(--muted); margin-left:3px; }
#board3d { width:100%; aspect-ratio: 16 / 9; border:1px solid var(--line); border-radius:8px; background:#f8fafc; display:block; margin-bottom:12px; }
#horizon { width:100%; aspect-ratio: 16 / 9; border:1px solid var(--line); border-radius:8px; background:#eef2f8; display:block; }
.bars { display:grid; gap:14px; }
.bar-row { display:grid; grid-template-columns: 86px 1fr 70px; align-items:center; gap:10px; }
.track { position:relative; height:18px; border-radius:9px; background:#eef1f6; overflow:hidden; border:1px solid var(--line); }
.zero { position:absolute; left:50%; top:0; bottom:0; width:1px; background:#475467; opacity:.45; }
.fill { position:absolute; top:0; bottom:0; left:50%; width:0; background:var(--blue); }
.num { text-align:right; font-variant-numeric: tabular-nums; color:var(--muted); }
.mini { display:grid; grid-template-columns: repeat(2, 1fr); gap:10px; margin-top:12px; }
.kv { border-top:1px solid var(--line); padding-top:10px; }
.kv strong { display:block; font-size:18px; font-variant-numeric: tabular-nums; margin-top:4px; }
.toolbar { display:flex; gap:8px; flex-wrap:wrap; margin-bottom:12px; }
.toolbar button { border:1px solid var(--line); background:#fff; color:var(--text); border-radius:6px; padding:7px 10px; cursor:pointer; font-family:inherit; }
.toolbar button:hover { background:#f1f5f9; }
@media (max-width: 860px) { .grid, .cards, .mini { grid-template-columns: 1fr; } header { align-items:flex-start; flex-direction:column; } }
</style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>实时姿态显示</h1>
      <div class="status">串口数据：<span id="conn">等待数据</span> · 最近样本：<span id="sampleTime">--</span></div>
    </div>
    <div class="status">互补滤波 Roll/Pitch，Mahony Roll/Pitch/Yaw</div>
  </header>

  <section class="cards">
    <div class="card"><div class="label">Roll</div><div class="value"><span id="roll">--</span><span class="unit">deg</span></div></div>
    <div class="card"><div class="label">Pitch</div><div class="value"><span id="pitch">--</span><span class="unit">deg</span></div></div>
    <div class="card"><div class="label">Yaw</div><div class="value"><span id="yaw">--</span><span class="unit">deg</span></div></div>
  </section>

  <section class="grid">
    <div class="panel">
      <h2>虚拟板子模型</h2>
      <div class="toolbar">
        <button id="toggleFront" type="button">FRONT：正向</button>
      </div>
      <canvas id="board3d" width="900" height="506"></canvas>
      <h2>姿态水平仪</h2>
      <canvas id="horizon" width="900" height="506"></canvas>
    </div>
    <div class="panel">
      <h2>角度与传感器状态</h2>
      <div class="bars">
        <div class="bar-row"><div class="label">Roll</div><div class="track"><div class="zero"></div><div class="fill" id="barRoll"></div></div><div class="num" id="numRoll">--</div></div>
        <div class="bar-row"><div class="label">Pitch</div><div class="track"><div class="zero"></div><div class="fill" id="barPitch"></div></div><div class="num" id="numPitch">--</div></div>
        <div class="bar-row"><div class="label">Yaw</div><div class="track"><div class="zero"></div><div class="fill" id="barYaw"></div></div><div class="num" id="numYaw">--</div></div>
      </div>
      <div class="mini">
        <div class="kv"><span class="label">气压高度</span><strong><span id="altitude">--</span> m</strong></div>
        <div class="kv"><span class="label">BMP 温度</span><strong><span id="bmpTemp">--</span> °C</strong></div>
        <div class="kv"><span class="label">磁场模长</span><strong><span id="mag">--</span> uT</strong></div>
        <div class="kv"><span class="label">样本计数</span><strong id="rows">0</strong></div>
      </div>
    </div>
  </section>
</main>
<script>
const el = id => document.getElementById(id);
const canvas = el('horizon');
const ctx = canvas.getContext('2d');
const boardCanvas = el('board3d');
const boardCtx = boardCanvas.getContext('2d');
let rows = 0;
let frontSign = -1;
let lastPose = { roll: 0, pitch: 0, yaw: 0 };

function fmt(v, digits=2) {
  return Number.isFinite(v) ? v.toFixed(digits) : '--';
}

function setBar(id, value, maxAbs) {
  const fill = el(id);
  const v = Number.isFinite(value) ? Math.max(-maxAbs, Math.min(maxAbs, value)) : 0;
  const pct = Math.abs(v) / maxAbs * 50;
  if (v >= 0) {
    fill.style.left = '50%';
    fill.style.width = pct + '%';
  } else {
    fill.style.left = (50 - pct) + '%';
    fill.style.width = pct + '%';
  }
  fill.style.background = Math.abs(v) > maxAbs * 0.75 ? 'var(--amber)' : 'var(--blue)';
}

function drawHorizon(roll, pitch) {
  const displayRoll = -roll;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.save();
  ctx.translate(w/2, h/2);
  ctx.rotate(-displayRoll * Math.PI / 180);
  const offset = Math.max(-h, Math.min(h, pitch * 5));
  ctx.fillStyle = '#5b8def';
  ctx.fillRect(-w*2, -h*2 + offset, w*4, h*2);
  ctx.fillStyle = '#c58b4b';
  ctx.fillRect(-w*2, offset, w*4, h*2);
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(-w*2, offset);
  ctx.lineTo(w*2, offset);
  ctx.stroke();
  ctx.restore();

  ctx.strokeStyle = '#111827';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(w/2 - 80, h/2);
  ctx.lineTo(w/2 - 20, h/2);
  ctx.moveTo(w/2 + 20, h/2);
  ctx.lineTo(w/2 + 80, h/2);
  ctx.moveTo(w/2, h/2 - 12);
  ctx.lineTo(w/2, h/2 + 12);
  ctx.stroke();

  ctx.fillStyle = '#111827';
  ctx.font = '18px Microsoft YaHei, Arial';
  ctx.fillText(`Roll ${fmt(roll,1)}°`, 18, 32);
  ctx.fillText(`Pitch ${fmt(pitch,1)}°`, 18, 58);
}

function rotatePoint(p, rollDeg, pitchDeg, yawDeg) {
  const roll = rollDeg * Math.PI / 180;
  const pitch = pitchDeg * Math.PI / 180;
  // 页面俯视方向与航向角正方向相反，显示时取反，保证实物左转时模型也左转。
  const yaw = -yawDeg * Math.PI / 180;
  let [x, y, z] = p;

  // 本模型把 USB/FRONT 定义在前后方向(Y轴)。
  // 因此 Pitch 应该让前后边抬起，绕左右方向(X轴)转。
  let cy = Math.cos(pitch), sy = Math.sin(pitch);
  let y1 = y * cy - z * sy;
  let z1 = y * sy + z * cy;
  y = y1; z = z1;

  // Roll 应该让左右边抬起，绕前后方向(Y轴)转。
  cy = Math.cos(roll); sy = Math.sin(roll);
  let x2 = x * cy + z * sy;
  let z2 = -x * sy + z * cy;
  x = x2; z = z2;

  // Yaw: rotate around vertical Z axis.
  cy = Math.cos(yaw); sy = Math.sin(yaw);
  let x3 = x * cy - y * sy;
  let y3 = x * sy + y * cy;
  return [x3, y3, z];
}

function project3D(p, w, h) {
  const [x, y, z] = p;
  const distance = 560;
  const scale = distance / (distance - z);
  return [w / 2 + x * scale, h / 2 + y * scale];
}

function drawBoard3D(roll, pitch, yaw) {
  const displayRoll = -roll;
  const displayPitch = -pitch;
  const w = boardCanvas.width, h = boardCanvas.height;
  const b = boardCtx;
  b.clearRect(0, 0, w, h);

  const gradient = b.createLinearGradient(0, 0, 0, h);
  gradient.addColorStop(0, '#f8fafc');
  gradient.addColorStop(1, '#e7edf6');
  b.fillStyle = gradient;
  b.fillRect(0, 0, w, h);

  // 实物板子近似正方形，只用 FRONT 标记区分朝向。
  const halfW = 135;
  const halfH = 135;
  const topZ = 12;
  const bottomZ = -12;
  const top = [
    [-halfW, -halfH, topZ],
    [halfW, -halfH, topZ],
    [halfW, halfH, topZ],
    [-halfW, halfH, topZ],
  ].map(p => rotatePoint(p, displayRoll, displayPitch, yaw));
  const bottom = [
    [-halfW, -halfH, bottomZ],
    [halfW, -halfH, bottomZ],
    [halfW, halfH, bottomZ],
    [-halfW, halfH, bottomZ],
  ].map(p => rotatePoint(p, displayRoll, displayPitch, yaw));
  const top2 = top.map(p => project3D(p, w, h));
  const bottom2 = bottom.map(p => project3D(p, w, h));

  function polygon(points, fill, stroke) {
    b.beginPath();
    b.moveTo(points[0][0], points[0][1]);
    for (let i = 1; i < points.length; i++) b.lineTo(points[i][0], points[i][1]);
    b.closePath();
    b.fillStyle = fill;
    b.fill();
    b.strokeStyle = stroke;
    b.lineWidth = 2;
    b.stroke();
  }

  // Draw side faces first.
  for (let i = 0; i < 4; i++) {
    const j = (i + 1) % 4;
    polygon([bottom2[i], bottom2[j], top2[j], top2[i]], '#94a3b8', '#64748b');
  }
  polygon(top2, '#2563eb', '#1d4ed8');

  // Board details: sensor blocks and orientation arrow on top plane.
  const center = project3D(rotatePoint([0, 0, topZ + 4], displayRoll, displayPitch, yaw), w, h);
  const front = project3D(rotatePoint([0, frontSign * (halfH - 34), topZ + 4], displayRoll, displayPitch, yaw), w, h);
  const right = project3D(rotatePoint([halfW - 26, 0, topZ + 4], displayRoll, displayPitch, yaw), w, h);
  const left = project3D(rotatePoint([-halfW + 30, 44, topZ + 4], displayRoll, displayPitch, yaw), w, h);

  b.strokeStyle = '#ffffff';
  b.lineWidth = 5;
  b.beginPath();
  b.moveTo(center[0], center[1]);
  b.lineTo(front[0], front[1]);
  b.stroke();
  b.fillStyle = '#ffffff';
  b.beginPath();
  b.arc(front[0], front[1], 8, 0, Math.PI * 2);
  b.fill();

  b.fillStyle = '#0f172a';
  b.fillRect(right[0] - 18, right[1] - 12, 36, 24);
  b.fillStyle = '#16a34a';
  b.fillRect(left[0] - 16, left[1] - 10, 32, 20);
  b.fillStyle = '#ffffff';
  b.font = '16px Microsoft YaHei, Arial';
  b.fillText('FRONT', front[0] + 10, front[1] - 8);

  // Axes indicator.
  const origin = project3D(rotatePoint([0, 0, 80], displayRoll, displayPitch, yaw), w, h);
  const axes = [
    { p: [80, 0, 80], c: '#dc2626', label: 'X' },
    { p: [0, 80, 80], c: '#16a34a', label: 'Y' },
    { p: [0, 0, 160], c: '#2563eb', label: 'Z' },
  ];
  for (const axis of axes) {
    const pt = project3D(rotatePoint(axis.p, displayRoll, displayPitch, yaw), w, h);
    b.strokeStyle = axis.c;
    b.lineWidth = 3;
    b.beginPath();
    b.moveTo(origin[0], origin[1]);
    b.lineTo(pt[0], pt[1]);
    b.stroke();
    b.fillStyle = axis.c;
    b.font = '16px Arial';
    b.fillText(axis.label, pt[0] + 5, pt[1] + 5);
  }

  b.fillStyle = '#111827';
  b.font = '18px Microsoft YaHei, Arial';
  b.fillText(`Roll ${fmt(roll,1)}°   Pitch ${fmt(pitch,1)}°   Yaw ${fmt(yaw,1)}°`, 18, 32);
}

function update(d) {
  rows += 1;
  const roll = Number.isFinite(d.mahony_roll_deg) ? d.mahony_roll_deg : d.roll_deg;
  const pitch = Number.isFinite(d.mahony_pitch_deg) ? d.mahony_pitch_deg : d.pitch_deg;
  const yaw = Number.isFinite(d.mahony_yaw_deg) ? d.mahony_yaw_deg : d.yaw_tilt_deg;

  el('conn').textContent = '正常';
  el('sampleTime').textContent = fmt(d.t_s, 3) + ' s';
  el('roll').textContent = fmt(roll, 2);
  el('pitch').textContent = fmt(pitch, 2);
  el('yaw').textContent = fmt(yaw, 2);
  el('numRoll').textContent = fmt(roll, 1) + '°';
  el('numPitch').textContent = fmt(pitch, 1) + '°';
  el('numYaw').textContent = fmt(yaw, 1) + '°';
  el('altitude').textContent = fmt(d.altitude_m, 2);
  el('bmpTemp').textContent = fmt(d.bmp_temp_c, 2);
  el('mag').textContent = fmt(d.mag_cal_uT, 2);
  el('rows').textContent = rows;
  setBar('barRoll', roll, 90);
  setBar('barPitch', pitch, 90);
  setBar('barYaw', ((yaw + 180) % 360) - 180, 180);
  lastPose = { roll: roll || 0, pitch: pitch || 0, yaw: yaw || 0 };
  drawBoard3D(roll || 0, pitch || 0, yaw || 0);
  drawHorizon(roll || 0, pitch || 0);
}

el('toggleFront').onclick = () => {
  frontSign *= -1;
  el('toggleFront').textContent = frontSign < 0 ? 'FRONT：正向' : 'FRONT：反向';
  drawBoard3D(lastPose.roll, lastPose.pitch, lastPose.yaw);
};

drawBoard3D(0, 0, 0);
drawHorizon(0, 0);
const source = new EventSource('/events');
source.onmessage = evt => update(JSON.parse(evt.data));
source.onerror = () => { el('conn').textContent = '中断/等待重连'; };
</script>
</body>
</html>
"""


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.count = 0
        self.error = ""

    def update(self, row):
        with self.lock:
            self.latest = row
            self.count += 1
            self.error = ""

    def set_error(self, error):
        with self.lock:
            self.error = str(error)

    def snapshot(self):
        with self.lock:
            data = dict(self.latest)
            data["_count"] = self.count
            data["_error"] = self.error
            return data


def clean_line(raw: bytes) -> str:
    text = raw.decode("utf-8", errors="ignore").strip()
    return ANSI_RE.sub("", text)


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def parse_sensor_row(header, fields):
    row = {}
    for key, value in zip(header, fields):
        row[key] = to_float(value)
    return row


def serial_worker(state, port, baud):
    header = list(DEFAULT_HEADER)
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                state.set_error("")
                while True:
                    line = clean_line(ser.readline())
                    if not line:
                        continue
                    if line.startswith("SENSOR_HEADER,"):
                        header = line.split(",")[1:]
                    elif line.startswith("SENSOR,"):
                        fields = line.split(",")[1:]
                        if len(fields) >= len(header):
                            state.update(parse_sensor_row(header, fields[: len(header)]))
        except serial.SerialException as exc:
            state.set_error(f"serial error: {exc}")
            time.sleep(1.0)


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                body = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path.startswith("/events"):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                last_count = -1
                while True:
                    snap = state.snapshot()
                    if snap.get("_count") != last_count:
                        last_count = snap.get("_count")
                        payload = json.dumps(snap, allow_nan=False)
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    time.sleep(0.1)

            self.send_response(404)
            self.end_headers()

    return Handler


def parse_args():
    parser = argparse.ArgumentParser(description="Real-time web attitude display for SENSOR serial output.")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    return parser.parse_args()


def main():
    args = parse_args()
    state = SharedState()
    thread = threading.Thread(target=serial_worker, args=(state, args.port, args.baud), daemon=True)
    thread.start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(state))
    print(f"Opening serial {args.port} at {args.baud} baud")
    print(f"Realtime attitude display: http://{args.host}:{args.http_port}")
    print("Stop ESP-IDF Monitor first, otherwise the serial port will be busy.")
    server.serve_forever()


if __name__ == "__main__":
    main()
