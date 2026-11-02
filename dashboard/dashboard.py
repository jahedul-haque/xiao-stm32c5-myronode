#!/usr/bin/env python3
"""
===============================================================================
KineMirror Teleoperation Dashboard & CAN FD Gateway
Seeed Studio XIAO STM32C5: IMU Leader -> CAN FD -> Servo Follower
===============================================================================
Features:
  - Multi-threaded HTTP server (ThreadingHTTPServer) with zero-hang architecture.
  - High-performance real-time Canvas Oscilloscope / Strip-Chart for Node 2 Roll & Pitch.
  - Target vs Actual Servo Angle gauge & range track for Node 1.
  - Interactive Mode Switcher: [IMU MIMIC] [MANUAL SLIDER] [HOLD CENTER].
  - Hardware Diagnostics: CAN FD Ping & Node 1 User LED Toggle Test with live ACK counter.
  - Standalone Python web server (zero external JS/CSS dependencies, just PySerial).
  - Web UI at http://localhost:8090
===============================================================================
"""

import sys
import os
import json
import time
import threading
import http.server
import serial
import serial.tools.list_ports

PORT = 8090
BAUDRATE = 115200

serial_lock = threading.Lock()
ser = None
connected_port = None

system_state = {
    "connected": False,
    "port": "None",
    "mode": 0,           # 0: Mimic, 1: Manual, 2: Center
    "imu": {
        "pitch": 0.0,
        "roll": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 1.0,
        "gz": 0.0
    },
    "servo": {
        "pos": 512,
        "angle": 150.0,
        "target_pos": 512,
        "target_angle": 150.0,
        "status": 0
    },
    "can_acks": 0,
    "rate_hz": 0,
    "last_rx_time": 0
}

rx_packet_count = 0
last_rate_calc_time = time.time()

def list_ports():
    ports = serial.tools.list_ports.comports()
    return [{"device": p.device, "description": p.description or p.device} for p in ports]

def open_serial(port_name):
    global ser, connected_port
    with serial_lock:
        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
        ser = None
        connected_port = None
        try:
            ser = serial.Serial(port_name, BAUDRATE, timeout=0.1)
            connected_port = port_name
            system_state["connected"] = True
            system_state["port"] = port_name
            print(f"[PORT] Connected to {port_name} @ {BAUDRATE}")
            return True
        except Exception as e:
            print(f"[PORT] Error opening {port_name}: {e}")
            return False

def close_serial():
    global ser, connected_port
    with serial_lock:
        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
        ser = None
        connected_port = None
        system_state["connected"] = False
        system_state["port"] = "None"
        print("[PORT] Closed serial connection")

def send_command(cmd_str):
    global ser
    with serial_lock:
        if ser and ser.is_open:
            try:
                msg = (cmd_str.strip() + "\r\n").encode("utf-8")
                ser.write(msg)
                print(f"[TX] >>> Sent to serial: {cmd_str.strip()}", flush=True)
                return True
            except Exception as e:
                print(f"[TX ERROR] Failed to send '{cmd_str}': {e}", flush=True)
        else:
            print(f"[TX WARN] Cannot send '{cmd_str.strip()}': Serial port is NOT connected!", flush=True)
    return False

def serial_reader_thread():
    global ser, rx_packet_count, last_rate_calc_time, connected_port
    buffer = ""
    stale_count = 0
    while True:
        with serial_lock:
            s = ser
            c_port = connected_port

        if s and s.is_open:
            try:
                available = s.in_waiting
                if available > 0:
                    data = s.read(available)
                    if data:
                        stale_count = 0
                        buffer += data.decode("utf-8", errors="ignore")
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            line = line.strip()
                            if line.startswith("{") and line.endswith("}"):
                                try:
                                    parsed = json.loads(line)
                                    if "imu" in parsed and "servo" in parsed:
                                        system_state["imu"] = parsed["imu"]
                                        system_state["servo"] = parsed["servo"]
                                        system_state["mode"] = parsed.get("mode", system_state["mode"])
                                        if "can_acks" in parsed:
                                            system_state["can_acks"] = parsed["can_acks"]
                                        system_state["last_rx_time"] = time.time()
                                        rx_packet_count += 1
                                except Exception:
                                    pass
                else:
                    stale_count += 1
                    # Seamless reconnect if connection stalled for 4s
                    if stale_count > 400 and system_state["last_rx_time"] > 0 and (time.time() - system_state["last_rx_time"]) > 4.0:
                        stale_count = 0
                        if c_port:
                            print(f"[RECONNECT] Re-establishing connection on {c_port}...", flush=True)
                            close_serial()
                            time.sleep(0.5)
                            open_serial(c_port)
            except Exception as e:
                print(f"[READER] Read error: {e}", flush=True)
                close_serial()

        now = time.time()
        if (now - last_rate_calc_time) >= 1.0:
            system_state["rate_hz"] = round(rx_packet_count / (now - last_rate_calc_time), 1)
            rx_packet_count = 0
            last_rate_calc_time = now
            if (now - system_state["last_rx_time"]) > 2.0:
                system_state["rate_hz"] = 0.0

        time.sleep(0.01)

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MyroNode // Teleoperation & Diagnostics</title>
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #070a12;
    --card: #0f1626;
    --card-border: #1a253c;
    --cyan: #06b6d4;
    --emerald: #10b981;
    --amber: #f59e0b;
    --rose: #f43f5e;
    --text-primary: #f8fafc;
    --text-secondary: #94a3b8;
    --text-muted: #475569;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background-color: var(--bg);
    color: var(--text-primary);
    font-family: 'Space Grotesk', sans-serif;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    padding: 24px;
  }
  .container { max-width: 1000px; margin: 0 auto; width: 100%; }

  /* Top Header */
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 1px solid var(--card-border);
    padding-bottom: 16px;
    margin-bottom: 24px;
  }
  .brand { display: flex; align-items: baseline; gap: 12px; }
  .brand h1 { font-size: 22px; font-weight: 700; letter-spacing: -0.5px; }
  .brand span { font-size: 12px; color: var(--cyan); font-family: 'JetBrains Mono', monospace; font-weight: 600; }

  .port-bar { display: flex; gap: 8px; align-items: center; }
  select, button {
    background: var(--card);
    border: 1px solid var(--card-border);
    color: var(--text-primary);
    padding: 7px 12px;
    border-radius: 6px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    outline: none;
    cursor: pointer;
    transition: 0.15s ease-in-out;
  }
  select:focus, button:hover { border-color: var(--cyan); }
  button.primary { background: #0891b2; border-color: #06b6d4; color: #fff; font-weight: 600; }
  button.danger { background: #991b1b; border-color: #ef4444; font-weight: 600; }

  /* Grid Layout */
  .grid {
    display: grid;
    grid-template-columns: 1.15fr 0.85fr;
    gap: 20px;
    margin-bottom: 20px;
  }
  @media (max-width: 840px) { .grid { grid-template-columns: 1fr; } }

  .card {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 12px;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
    box-shadow: 0 4px 20px rgba(0,0,0,0.3);
  }
  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .card-title {
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 1px;
    color: var(--text-secondary);
    font-weight: 600;
  }
  .tag {
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
    padding: 3px 8px;
    border-radius: 4px;
    background: rgba(6, 182, 212, 0.12);
    color: var(--cyan);
    border: 1px solid rgba(6, 182, 212, 0.25);
  }

  /* Live IMU Strip-Chart Canvas */
  .chart-container {
    position: relative;
    width: 100%;
    height: 180px;
    border-radius: 8px;
    background: #080d18;
    overflow: hidden;
    border: 1px solid rgba(255,255,255,0.06);
  }
  canvas#imuChart {
    width: 100%;
    height: 100%;
    display: block;
  }
  .chart-legend {
    position: absolute;
    top: 8px;
    left: 12px;
    display: flex;
    gap: 14px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
    background: rgba(8, 13, 24, 0.75);
    padding: 3px 8px;
    border-radius: 4px;
    border: 1px solid rgba(255,255,255,0.06);
    backdrop-filter: blur(4px);
  }
  .legend-item { display: flex; align-items: center; gap: 5px; }
  .legend-dot { width: 8px; height: 8px; border-radius: 50%; }

  /* Node 1 Gauge & Indicators */
  .servo-display {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 10px 0;
    gap: 6px;
  }
  .big-angle {
    font-size: 46px;
    font-weight: 700;
    font-family: 'JetBrains Mono', monospace;
    color: var(--emerald);
    line-height: 1;
    text-shadow: 0 0 20px rgba(16, 185, 129, 0.25);
  }
  .target-badge {
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    color: var(--text-secondary);
  }
  .target-badge b { color: var(--cyan); }

  /* Track Progress Bar */
  .range-track {
    width: 100%;
    height: 10px;
    background: #080d18;
    border-radius: 5px;
    position: relative;
    overflow: hidden;
    border: 1px solid rgba(255,255,255,0.06);
    margin: 4px 0;
  }
  .range-fill {
    height: 100%;
    background: linear-gradient(90deg, #0891b2, #10b981);
    width: 50%;
    transition: width 0.05s linear;
  }
  .range-labels {
    display: flex;
    justify-content: space-between;
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
    color: var(--text-muted);
  }

  .stats-row {
    display: flex;
    justify-content: space-between;
    border-top: 1px solid var(--card-border);
    padding-top: 12px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 12px;
  }
  .stat-item span:first-child { color: var(--text-secondary); margin-right: 5px; }

  /* Control Strip */
  .controls {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 12px;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
    margin-bottom: 20px;
  }
  .mode-buttons { display: flex; gap: 10px; }
  .mode-btn {
    flex: 1;
    padding: 10px;
    font-weight: 600;
    font-size: 13px;
    border-radius: 8px;
  }
  .mode-btn.active {
    background: var(--cyan);
    color: #070a12;
    border-color: var(--cyan);
    box-shadow: 0 0 12px rgba(6, 182, 212, 0.3);
  }

  .slider-box {
    display: flex;
    align-items: center;
    gap: 16px;
    transition: opacity 0.2s;
  }
  input[type=range] {
    flex: 1;
    accent-color: var(--cyan);
    height: 6px;
    cursor: pointer;
  }

  /* CAN Diagnostics Card */
  .diag-card {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 12px;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 14px;
  }
  .diag-top {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .diag-desc {
    font-size: 13px;
    color: var(--text-secondary);
    line-height: 1.5;
  }
  .diag-actions {
    display: flex;
    gap: 16px;
    align-items: center;
    flex-wrap: wrap;
  }
  .btn-can-test {
    background: linear-gradient(135deg, #0284c7, #0891b2);
    border: 1px solid var(--cyan);
    color: #ffffff;
    font-weight: 600;
    padding: 10px 18px;
    border-radius: 8px;
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 13px;
  }
  .btn-can-test:hover {
    background: linear-gradient(135deg, #0369a1, #0e7490);
    box-shadow: 0 0 14px rgba(6, 182, 212, 0.4);
    transform: translateY(-1px);
  }
  .ack-pill {
    font-family: 'JetBrains Mono', monospace;
    font-size: 12px;
    padding: 7px 14px;
    border-radius: 6px;
    background: rgba(16, 185, 129, 0.12);
    color: var(--emerald);
    border: 1px solid rgba(16, 185, 129, 0.35);
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .ack-pill.waiting {
    background: rgba(245, 158, 11, 0.12);
    color: var(--amber);
    border-color: rgba(245, 158, 11, 0.35);
  }

  .status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 6px;
  }
  .dot-online { background: var(--emerald); box-shadow: 0 0 8px var(--emerald); }
  .dot-offline { background: var(--rose); }
</style>
</head>
<body>
<div class="container">
  <header>
    <div class="brand">
      <h1>MYRONODE</h1>
      <span>[v2.2 REAL-TIME OSCILLOSCOPE // CAN FD]</span>
    </div>
    <div class="port-bar">
      <span class="status-dot dot-offline" id="statusDot"></span>
      <select id="portSelect"><option>Loading ports...</option></select>
      <button onclick="refreshPorts()">↻</button>
      <button class="primary" id="connectBtn" onclick="toggleConnect()">Connect</button>
    </div>
  </header>

  <div class="grid">
    <!-- Node 2: Leader IMU with High-Performance Live Graph -->
    <div class="card">
      <div class="card-header">
        <span class="card-title">Node 2 // IMU Leader Dynamic Telemetry</span>
        <span class="tag">LSM6DS3 @ 104Hz</span>
      </div>

      <!-- Real-time Roll / Pitch Chart -->
      <div class="chart-container">
        <canvas id="imuChart" width="540" height="180"></canvas>
        <div class="chart-legend">
          <div class="legend-item">
            <span class="legend-dot" style="background: var(--cyan);"></span>
            <span>ROLL: <b id="legRoll" style="color: var(--cyan);">0.0°</b></span>
          </div>
          <div class="legend-item">
            <span class="legend-dot" style="background: var(--amber);"></span>
            <span>PITCH: <b id="legPitch" style="color: var(--amber);">0.0°</b></span>
          </div>
        </div>
      </div>

      <div class="stats-row">
        <div class="stat-item"><span>ROLL:</span><b id="valRoll">0.0°</b></div>
        <div class="stat-item"><span>PITCH:</span><b id="valPitch">0.0°</b></div>
        <div class="stat-item"><span>RATE:</span><b id="valRate">0 Hz</b></div>
      </div>
    </div>

    <!-- Node 1: Follower Servo -->
    <div class="card">
      <div class="card-header">
        <span class="card-title">Node 1 // Servo Follower Actuator</span>
        <span class="tag">CAN FD 0x110</span>
      </div>

      <div class="servo-display">
        <div class="big-angle" id="valServoAngle">150.0°</div>
        <div class="target-badge">TARGET: <b id="valServoTarget">150.0°</b> (Δ <span id="valDelta">0.0°</span>)</div>
      </div>

      <!-- Track bar -->
      <div>
        <div class="range-track">
          <div class="range-fill" id="rangeFill"></div>
        </div>
        <div class="range-labels">
          <span>0°</span>
          <span>150°</span>
          <span>300°</span>
        </div>
      </div>

      <div class="stats-row">
        <div class="stat-item"><span>RAW POS:</span><b id="valServoPos">512 / 1023</b></div>
        <div class="stat-item"><span>STATE:</span><b id="valServoStatus" style="color: var(--emerald);">ONLINE</b></div>
      </div>
    </div>
  </div>

  <!-- Interactive Control Strip -->
  <div class="controls">
    <div class="card-header">
      <span class="card-title">Teleoperation Mode</span>
      <span class="tag" id="modeTag">MIMIC MODE</span>
    </div>
    <div class="mode-buttons">
      <button class="mode-btn active" id="btnMimic" onclick="setMode(0)">IMU Mimic (Tilt Control)</button>
      <button class="mode-btn" id="btnManual" onclick="setMode(1)">Manual Slider</button>
      <button class="mode-btn" id="btnCenter" onclick="setMode(2)">Hold Center (150°)</button>
    </div>
    <div class="slider-box" id="sliderGroup" style="opacity: 0.5;">
      <span style="font-family: 'JetBrains Mono', monospace; font-size: 13px;">0°</span>
      <input type="range" id="angleSlider" min="0" max="300" step="1" value="150" disabled oninput="onSliderMove(this.value)">
      <span style="font-family: 'JetBrains Mono', monospace; font-size: 13px;">300°</span>
      <span id="sliderDisplay" style="font-family: 'JetBrains Mono', monospace; font-size: 14px; font-weight: 700; width: 60px; text-align: right; color: var(--cyan);">150°</span>
    </div>
  </div>

  <!-- Hardware CAN FD Test & Node 1 Diagnostics -->
  <div class="diag-card">
    <div class="diag-top">
      <span class="card-title">CAN FD Bus Diagnostics & Node 1 Hardware Verification</span>
      <span class="tag">FDCAN2: PB5/PB13</span>
    </div>
    <p class="diag-desc">
      Broadcasts a real-time CAN test frame (ID <code>0x212</code>) from Node 2 to Node 1 across the CAN FD bus. 
      When received, Node 1 immediately <b>toggles its onboard User LED (PB12)</b> and responds with an ACK frame (ID <code>0x112</code>) back to Node 2.
    </p>
    <div class="diag-actions">
      <button class="btn-can-test" id="btnCanTest" onclick="triggerCanTest()">
        <span>⚡</span> Test CAN: Toggle Node 1 LED
      </button>
      <div class="ack-pill waiting" id="ackPill">
        <span>●</span> <span id="ackText">Awaiting Test (0 ACKs)</span>
      </div>
    </div>
  </div>
</div>

<script>
let currentMode = 0;

// Rolling time-series history for Canvas Graph
const MAX_SAMPLES = 120;
const historyRoll = new Array(MAX_SAMPLES).fill(0);
const historyPitch = new Array(MAX_SAMPLES).fill(0);

const canvas = document.getElementById('imuChart');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  if (canvas.width !== rect.width || canvas.height !== rect.height) {
    canvas.width = rect.width;
    canvas.height = rect.height;
  }
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function drawGraph() {
  resizeCanvas();
  const w = canvas.width;
  const h = canvas.height;

  ctx.clearRect(0, 0, w, h);

  // Background grid lines: +60, +30, 0, -30, -60 degrees
  const degSteps = [60, 30, 0, -30, -60];
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  ctx.fillStyle = 'rgba(148, 163, 184, 0.5)';
  ctx.font = '10px JetBrains Mono, monospace';
  ctx.textAlign = 'right';

  degSteps.forEach(deg => {
    const y = h/2 - (deg / 65) * (h/2 - 14);
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    if (deg === 0) {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
      ctx.setLineDash([4, 4]);
    } else {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.setLineDash([]);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillText((deg > 0 ? '+' : '') + deg + '°', w - 8, y - 3);
  });

  function mapY(deg) {
    const clamped = Math.max(-65, Math.min(65, deg));
    return h/2 - (clamped / 65) * (h/2 - 14);
  }

  // Draw Pitch Waveform (Amber)
  ctx.beginPath();
  for (let i = 0; i < MAX_SAMPLES; i++) {
    const x = (i / (MAX_SAMPLES - 1)) * (w - 45);
    const y = mapY(historyPitch[i]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 2.0;
  ctx.shadowColor = '#f59e0b';
  ctx.shadowBlur = 6;
  ctx.stroke();
  ctx.shadowBlur = 0;

  // Draw Roll Waveform (Cyan)
  ctx.beginPath();
  for (let i = 0; i < MAX_SAMPLES; i++) {
    const x = (i / (MAX_SAMPLES - 1)) * (w - 45);
    const y = mapY(historyRoll[i]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.strokeStyle = '#06b6d4';
  ctx.lineWidth = 2.0;
  ctx.shadowColor = '#06b6d4';
  ctx.shadowBlur = 6;
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function refreshPorts() {
  fetch('/api/ports').then(r => r.json()).then(ports => {
    const sel = document.getElementById('portSelect');
    sel.innerHTML = '';
    if (ports.length === 0) {
      sel.innerHTML = '<option>No ports found</option>';
      return;
    }
    ports.forEach(p => {
      const opt = document.createElement('option');
      opt.value = p.device;
      opt.text = p.device + ' - ' + p.description;
      if (p.device === 'COM21') opt.selected = true;
      sel.appendChild(opt);
    });
  }).catch(() => {});
}

function toggleConnect() {
  const btn = document.getElementById('connectBtn');
  if (btn.innerText === 'Connect') {
    const port = document.getElementById('portSelect').value;
    fetch('/api/connect', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({port})
    }).then(r => r.json()).then(res => {
      if (res.ok) updateUIState(true);
    }).catch(() => {});
  } else {
    fetch('/api/disconnect', {method: 'POST'}).then(() => updateUIState(false)).catch(() => {});
  }
}

function updateUIState(connected) {
  const btn = document.getElementById('connectBtn');
  const dot = document.getElementById('statusDot');
  if (connected) {
    btn.innerText = 'Disconnect';
    btn.className = 'danger';
    dot.className = 'status-dot dot-online';
  } else {
    btn.innerText = 'Connect';
    btn.className = 'primary';
    dot.className = 'status-dot dot-offline';
  }
}

function setMode(mode) {
  currentMode = mode;
  document.getElementById('btnMimic').className = 'mode-btn' + (mode === 0 ? ' active' : '');
  document.getElementById('btnManual').className = 'mode-btn' + (mode === 1 ? ' active' : '');
  document.getElementById('btnCenter').className = 'mode-btn' + (mode === 2 ? ' active' : '');

  const slider = document.getElementById('angleSlider');
  const grp = document.getElementById('sliderGroup');
  const modeTag = document.getElementById('modeTag');

  if (mode === 0) modeTag.innerText = 'MIMIC MODE';
  else if (mode === 1) modeTag.innerText = 'MANUAL SLIDER';
  else modeTag.innerText = 'HOLD CENTER (150°)';

  if (mode === 1) {
    slider.disabled = false;
    grp.style.opacity = '1.0';
  } else {
    slider.disabled = true;
    grp.style.opacity = '0.5';
  }

  fetch('/api/mode', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({mode})
  }).catch(e => console.error("Error setting mode:", e));
}

function onSliderMove(val) {
  document.getElementById('sliderDisplay').innerText = val + '°';
  fetch('/api/target', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({target: parseFloat(val)})
  }).catch(e => console.error("Error setting target:", e));
}

function triggerCanTest() {
  const btn = document.getElementById('btnCanTest');
  btn.disabled = true;
  btn.style.opacity = '0.7';
  btn.innerHTML = '<span>⏳</span> Sending CAN Ping...';

  fetch('/api/can_test', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({})
  }).then(r => r.json()).then(res => {
    setTimeout(() => {
      btn.disabled = false;
      btn.style.opacity = '1.0';
      btn.innerHTML = '<span>⚡</span> Test CAN: Toggle Node 1 LED';
    }, 250);
  }).catch(() => {
    btn.disabled = false;
    btn.style.opacity = '1.0';
    btn.innerHTML = '<span>⚡</span> Test CAN: Toggle Node 1 LED';
  });
}

let isUpdating = false;

async function updateLoop() {
  if (isUpdating) return;
  isUpdating = true;
  try {
    const res = await fetch('/api/state');
    const s = await res.json();
    updateUIState(s.connected);

    // IMU Data update
    const roll = s.imu.roll || 0;
    const pitch = s.imu.pitch || 0;

    historyRoll.shift();
    historyRoll.push(roll);
    historyPitch.shift();
    historyPitch.push(pitch);
    drawGraph();

    document.getElementById('valRoll').innerText = (roll >= 0 ? '+' : '') + roll.toFixed(1) + '°';
    document.getElementById('valPitch').innerText = (pitch >= 0 ? '+' : '') + pitch.toFixed(1) + '°';
    document.getElementById('legRoll').innerText = (roll >= 0 ? '+' : '') + roll.toFixed(1) + '°';
    document.getElementById('legPitch').innerText = (pitch >= 0 ? '+' : '') + pitch.toFixed(1) + '°';
    document.getElementById('valRate').innerText = (s.rate_hz || 0) + ' Hz';

    // Node 1 Servo update
    const curAngle = s.servo.angle != null ? s.servo.angle : 150.0;
    const tgtAngle = s.servo.target_angle != null ? s.servo.target_angle : 150.0;
    document.getElementById('valServoAngle').innerText = curAngle.toFixed(1) + '°';
    document.getElementById('valServoTarget').innerText = tgtAngle.toFixed(1) + '°';
    document.getElementById('valServoPos').innerText = s.servo.pos + ' / 1023';
    document.getElementById('valDelta').innerText = Math.abs(curAngle - tgtAngle).toFixed(1) + '°';

    const pct = Math.max(0, Math.min(100, (curAngle / 300.0) * 100));
    document.getElementById('rangeFill').style.width = pct + '%';

    const statEl = document.getElementById('valServoStatus');
    if (s.servo.status === 0) {
      statEl.innerText = 'ONLINE';
      statEl.style.color = 'var(--emerald)';
    } else {
      statEl.innerText = 'SIMULATED';
      statEl.style.color = 'var(--amber)';
    }

    // Diagnostics ACK counter
    const acks = s.can_acks || 0;
    const pill = document.getElementById('ackPill');
    const text = document.getElementById('ackText');
    if (acks > 0) {
      pill.className = 'ack-pill';
      text.innerText = `Node 1 Verified (${acks} ACKs received)`;
    } else {
      pill.className = 'ack-pill waiting';
      text.innerText = 'Awaiting Test (0 ACKs)';
    }
  } catch (e) {
  } finally {
    isUpdating = false;
  }
}

refreshPorts();
setInterval(updateLoop, 50);
</script>
</body>
</html>
"""

class DashboardHTTPHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/state":
            body = json.dumps(system_state).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/ports":
            body = json.dumps(list_ports()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8") if length > 0 else ""
        try:
            data = json.loads(body) if body else {}
        except Exception:
            data = {}

        if self.path == "/api/connect":
            port = data.get("port")
            ok = open_serial(port) if port else False
            resp = json.dumps({"ok": ok}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(resp)
        elif self.path == "/api/disconnect":
            close_serial()
            resp = json.dumps({"ok": True}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(resp)
        elif self.path == "/api/mode":
            mode = data.get("mode", 0)
            if mode == 0:
                send_command("MIMIC")
            elif mode == 1:
                send_command("MANUAL")
            elif mode == 2:
                send_command("HOLD")
            system_state["mode"] = mode
            resp = json.dumps({"ok": True, "mode": mode}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(resp)
        elif self.path == "/api/target":
            target = data.get("target", 150.0)
            send_command(f"TARGET:{target:.1f}")
            system_state["mode"] = 1
            resp = json.dumps({"ok": True, "target": target}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(resp)
        elif self.path == "/api/can_test":
            ok = send_command("CAN_TEST")
            resp = json.dumps({"ok": ok}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(resp)
        else:
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress HTTP access noise

def run_server():
    reader = threading.Thread(target=serial_reader_thread, daemon=True)
    reader.start()

    server = http.server.ThreadingHTTPServer(("", PORT), DashboardHTTPHandler)
    print("\r\n=======================================================")
    print(" MyroNode Multi-Threaded Dashboard")
    print(f" Web UI running at: http://localhost:{PORT}")
    print("=======================================================\r\n")

    # Auto-connect to COM21 if available
    ports = [p["device"] for p in list_ports()]
    if "COM21" in ports:
        open_serial("COM21")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.server_close()

if __name__ == "__main__":
    run_server()
