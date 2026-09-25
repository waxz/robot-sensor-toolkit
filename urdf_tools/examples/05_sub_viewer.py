"""Example 05 — Subscribe to sensor topics: real-time 2D and 3D LiDAR viewers.

Requires shmbridge installed and 06_irsim_bridge.py (or 04_pub_sensors.py) running.

Usage:
    python 05_sub_viewer.py                     # text print mode
    python 05_sub_viewer.py --live2d            # 2D real-time LiDAR (no world)
    python 05_sub_viewer.py --live3d            # 3D real-time LiDAR (three.js web viewer)

    # With world and robot URDF overlays
    python 05_sub_viewer.py --live2d \\
        --world models/warehouse_world.urdf \\
        --robot models/robot_diff.urdf

    python 05_sub_viewer.py --live3d \\
        --world models/warehouse_world.urdf \\
        --robot models/robot_diff.urdf

    # Fallback matplotlib 3D viewer
    python 05_sub_viewer.py --live3d-mpl \\
        --world models/warehouse_world.urdf \\
        --robot models/robot_diff.urdf

    # Legacy alias
    python 05_sub_viewer.py --live              # same as --live2d
    python 05_sub_viewer.py --count 20          # stop after 20 polls (text mode)
"""

from __future__ import annotations

import json as _json
import math
import queue as _queue
import socket as _socket
import sys
import threading as _threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from urdf_tools.pubsub import SensorSubscriber

SHM_NAME = "/urdf_tools_sensors"

# ── shared style ──────────────────────────────────────────────────────────────

_BG = "#0f172a"  # page / figure background  (slate-900)
_PANE = "#0d1526"  # 3-D pane  (slightly darker)
_GRID = "#1e293b"  # grid lines                 (slate-800)
_TEXT = "#94a3b8"  # axis labels / tick text    (slate-400)
_WORLD_FC = "#1e3a5f"  # world geometry fill
_WORLD_EC = "#2d5fa0"  # world geometry edge
_ROBOT_C = "#3b82f6"  # robot marker / wireframe  (blue-500)
_HDG_C = "#60a5fa"  # heading arrow             (blue-400)
_SCAN_CMAP = "plasma"
_RAY_C = "#ef4444"  # scan ray colour           (red-500)
_SCAN_HIT_EC = "#f97316"  # scan hit edge          (orange-500)

# ── three.js web viewer HTML ──────────────────────────────────────────────────

_HTML_VIEWER = """\
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>3D LiDAR — shmbridge</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html,body{width:100%;height:100%;background:#0f172a;overflow:hidden}
canvas{display:block}
#hud{
  position:fixed;top:14px;left:14px;
  background:rgba(13,21,38,.82);border:1px solid #1e293b;
  border-radius:7px;padding:9px 14px;
  color:#94a3b8;font:11px/1.65 'JetBrains Mono','Fira Code',ui-monospace,monospace;
  pointer-events:none;white-space:pre;backdrop-filter:blur(6px);
}
.hi{color:#60a5fa}.val{color:#e2e8f0}.dim{color:#475569}
#panel{
  position:fixed;top:14px;right:14px;width:210px;
  background:rgba(13,21,38,.82);border:1px solid #1e293b;
  border-radius:7px;padding:10px 12px;
  color:#94a3b8;font:11px/1.55 'JetBrains Mono','Fira Code',ui-monospace,monospace;
  backdrop-filter:blur(6px);
}
#panel .panel-title{color:#e2e8f0;font-weight:bold;margin-bottom:6px;letter-spacing:.03em}
#panel .sensor-row{display:flex;align-items:center;gap:7px;cursor:pointer;user-select:none}
#panel .sensor-row input{accent-color:#3b82f6;cursor:pointer}
#panel .sensor-row.off .sensor-name{color:#475569}
#panel .sensor-info{margin:2px 0 8px 21px;color:#475569;white-space:pre-line}
#panel .sensor-info .v{color:#60a5fa}
</style>
</head>
<body>
<div id="hud"><span class="dim">Connecting…</span></div>
<div id="panel">
  <div class="panel-title">Sensors</div>
  <label class="sensor-row" id="row-scan">
    <input type="checkbox" id="chk-scan" checked>
    <span class="sensor-name">LiDAR 2D</span>
  </label>
  <div class="sensor-info" id="info-scan">type: --<br>-- Hz</div>
  <label class="sensor-row" id="row-cloud">
    <input type="checkbox" id="chk-cloud" checked>
    <span class="sensor-name">LiDAR 3D</span>
  </label>
  <div class="sensor-info" id="info-cloud">type: --<br>-- Hz</div>
  <div class="panel-title" style="margin-top:4px">Telemetry</div>
  <div class="sensor-info" id="info-imu" style="margin-left:0">IMU: --</div>
  <div class="sensor-info" id="info-encoder" style="margin-left:0">Encoder: --</div>
</div>
<script type="importmap">
{
  "imports": {
    "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ── renderer ──────────────────────────────────────────────────────────────────
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
window.addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

// ── scene ─────────────────────────────────────────────────────────────────────
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f172a);
scene.fog = new THREE.FogExp2(0x0f172a, 0.007);

// ── camera (Z-up) ─────────────────────────────────────────────────────────────
const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 600);
camera.up.set(0, 0, 1);
camera.position.set(-8, -22, 32);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(25, 15, 3);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.update();

// R = reset camera
window.addEventListener('keydown', e => {
  if (e.key === 'r' || e.key === 'R') {
    camera.position.set(-8, -22, 32);
    controls.target.set(25, 15, 3);
    controls.update();
  }
});

// ── floor grid ────────────────────────────────────────────────────────────────
const grid = new THREE.GridHelper(300, 300, 0x1e293b, 0x1e293b);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// ── plasma colour LUT ─────────────────────────────────────────────────────────
const PLASMA = [
  [0.050,0.030,0.528],[0.283,0.030,0.632],[0.492,0.012,0.657],
  [0.675,0.079,0.596],[0.829,0.193,0.439],[0.940,0.378,0.220],
  [0.976,0.607,0.100],[0.940,0.975,0.131],
];
function plasma(t) {
  t = Math.max(0, Math.min(1, t));
  const n = PLASMA.length - 1;
  const i = Math.min(Math.floor(t * n), n - 1);
  const f = t * n - i;
  const [r0,g0,b0] = PLASMA[i], [r1,g1,b1] = PLASMA[i+1];
  return [r0+f*(r1-r0), g0+f*(g1-g0), b0+f*(b1-b0)];
}

// ── wireframe builder ─────────────────────────────────────────────────────────
function buildWireframe(data) {
  const group = new THREE.Group();
  for (const { rgba, segments } of data) {
    if (!segments.length) continue;
    const [r, g, b, a = 1] = rgba;
    // blend material colour toward the dark theme (same formula as matplotlib version)
    const ec = new THREE.Color(r*0.55+0.08, g*0.55+0.10, b*0.55+0.15);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.Float32BufferAttribute(segments, 3));
    group.add(new THREE.LineSegments(geom,
      new THREE.LineBasicMaterial({ color: ec, transparent: true,
        opacity: Math.min(a * 0.70, 0.82) })));
  }
  return group;
}

// ── scan cloud (vertex-coloured Points) ───────────────────────────────────────
const MAX_PTS = 1024;
const sPosArr = new Float32Array(MAX_PTS * 3);
const sColArr = new Float32Array(MAX_PTS * 3);
const sGeom   = new THREE.BufferGeometry();
sGeom.setAttribute('position', new THREE.BufferAttribute(sPosArr, 3));
sGeom.setAttribute('color',    new THREE.BufferAttribute(sColArr, 3));
sGeom.setDrawRange(0, 0);
const scanPoints = new THREE.Points(sGeom,
  new THREE.PointsMaterial({ size: 5, vertexColors: true, sizeAttenuation: false }));
scene.add(scanPoints);

// ── scan rays (vertex-coloured LineSegments) ──────────────────────────────────
const MAX_RAYS = 64;
const rPosArr = new Float32Array(MAX_RAYS * 6);
const rColArr = new Float32Array(MAX_RAYS * 6);
const rGeom   = new THREE.BufferGeometry();
rGeom.setAttribute('position', new THREE.BufferAttribute(rPosArr, 3));
rGeom.setAttribute('color',    new THREE.BufferAttribute(rColArr, 3));
rGeom.setDrawRange(0, 0);
const scanRays = new THREE.LineSegments(rGeom,
  new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.30 }));
scene.add(scanRays);

// ── 3D lidar point cloud ──────────────────────────────────────────────────────
const MAX_PTS3D = 2048;
const p3PosArr = new Float32Array(MAX_PTS3D * 3);
const p3ColArr = new Float32Array(MAX_PTS3D * 3);
const p3Geom   = new THREE.BufferGeometry();
p3Geom.setAttribute('position', new THREE.BufferAttribute(p3PosArr, 3));
p3Geom.setAttribute('color',    new THREE.BufferAttribute(p3ColArr, 3));
p3Geom.setDrawRange(0, 0);
const cloud3dPoints = new THREE.Points(p3Geom,
  new THREE.PointsMaterial({ size: 3, vertexColors: true, sizeAttenuation: false }));
scene.add(cloud3dPoints);

// ── robot visual ──────────────────────────────────────────────────────────────
const robotPivot = new THREE.Group();
scene.add(robotPivot);
// heading cone points in +X (local frame)
const coneG = new THREE.ConeGeometry(0.22, 0.60, 10);
coneG.rotateZ(-Math.PI / 2);
robotPivot.add(new THREE.Mesh(coneG,
  new THREE.MeshBasicMaterial({ color: 0x3b82f6 })));
// body disc
robotPivot.add(new THREE.Mesh(new THREE.CircleGeometry(0.22, 24),
  new THREE.MeshBasicMaterial({ color: 0x1e3a5f, side: THREE.DoubleSide })));
// ring outline
robotPivot.add(new THREE.Mesh(new THREE.RingGeometry(0.19, 0.25, 24),
  new THREE.MeshBasicMaterial({ color: 0x93c5fd, side: THREE.DoubleSide })));

let robotWireGroup = null;

// ── per-frame update helpers ──────────────────────────────────────────────────
function updatePose(x, y, th) {
  robotPivot.position.set(x, y, 0.30);
  robotPivot.rotation.z = th;
  if (robotWireGroup) {
    robotWireGroup.position.set(x, y, 0);
    robotWireGroup.rotation.z = th;
  }
}

function updateScan({ ranges, angle_min, angle_increment, range_max }, x0, y0, th, enabled) {
  const LZ = 0.30;
  let nP = 0, nR = 0;
  if (!enabled) {
    sGeom.setDrawRange(0, 0);
    rGeom.setDrawRange(0, 0);
    return 0;
  }
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (r >= range_max * 0.999 || nP >= MAX_PTS) continue;
    const a  = angle_min + i * angle_increment;
    const hx = x0 + r * Math.cos(th + a);
    const hy = y0 + r * Math.sin(th + a);
    const [cr, cg, cb] = plasma(r / range_max);
    sPosArr[nP*3]=hx; sPosArr[nP*3+1]=hy; sPosArr[nP*3+2]=LZ;
    sColArr[nP*3]=cr; sColArr[nP*3+1]=cg; sColArr[nP*3+2]=cb;
    nP++;
    // every 6th hit: draw a ray from robot to hit
    if (nP % 6 === 0 && nR < MAX_RAYS) {
      const k = nR * 6;
      rPosArr[k]=x0;  rPosArr[k+1]=y0;  rPosArr[k+2]=LZ;
      rPosArr[k+3]=hx; rPosArr[k+4]=hy; rPosArr[k+5]=LZ;
      // gradient: dim at robot, full colour at hit
      rColArr[k]=cr*0.2; rColArr[k+1]=cg*0.2; rColArr[k+2]=cb*0.2;
      rColArr[k+3]=cr;   rColArr[k+4]=cg;     rColArr[k+5]=cb;
      nR++;
    }
  }
  sGeom.setDrawRange(0, nP);
  sGeom.attributes.position.needsUpdate = true;
  sGeom.attributes.color.needsUpdate    = true;
  rGeom.setDrawRange(0, nR * 2);
  rGeom.attributes.position.needsUpdate = true;
  rGeom.attributes.color.needsUpdate    = true;
  return nP;
}

function updateCloud3D({ pts, n, z_min, z_max }, enabled) {
  if (!enabled) {
    p3Geom.setDrawRange(0, 0);
    return;
  }
  const zLo = (z_min != null) ? z_min : 0.0;
  const zHi = (z_max != null) ? z_max : 6.0;
  const zRange = Math.max(zHi - zLo, 0.001);
  const count = Math.min(n, MAX_PTS3D);
  for (let i = 0; i < count; i++) {
    p3PosArr[i*3]   = pts[i*3];
    p3PosArr[i*3+1] = pts[i*3+1];
    p3PosArr[i*3+2] = pts[i*3+2];
    const t = (pts[i*3+2] - zLo) / zRange;
    const [cr, cg, cb] = plasma(Math.max(0, Math.min(1, t)));
    p3ColArr[i*3]=cr; p3ColArr[i*3+1]=cg; p3ColArr[i*3+2]=cb;
  }
  p3Geom.setDrawRange(0, count);
  p3Geom.attributes.position.needsUpdate = true;
  p3Geom.attributes.color.needsUpdate    = true;
}

// ── load static geometry ──────────────────────────────────────────────────────
async function initGeometry() {
  const [wData, rData] = await Promise.all([
    fetch('/world.json').then(r => r.json()).catch(() => []),
    fetch('/robot.json').then(r => r.json()).catch(() => []),
  ]);

  if (wData.length) {
    scene.add(buildWireframe(wData));
    // auto-fit camera to world bounding box
    let xlo=Infinity, xhi=-Infinity, ylo=Infinity, yhi=-Infinity, zhi=0;
    for (const { segments: s } of wData)
      for (let i = 0; i < s.length; i += 3) {
        const [x, y, z] = [s[i], s[i+1], s[i+2]];
        xlo=Math.min(xlo,x); xhi=Math.max(xhi,x);
        ylo=Math.min(ylo,y); yhi=Math.max(yhi,y);
        zhi=Math.max(zhi,z);
      }
    const cx=(xlo+xhi)/2, cy=(ylo+yhi)/2;
    const d = Math.max(xhi-xlo, yhi-ylo, 1);
    controls.target.set(cx, cy, zhi*0.4);
    camera.position.set(cx-d*0.35, cy-d*0.55, zhi+d*0.45);
    controls.update();
    grid.position.set(cx, cy, -0.01);
  }

  if (rData.length) {
    robotWireGroup = buildWireframe(rData);
    scene.add(robotWireGroup);
  }
}
initGeometry();

// ── settings panel: per-sensor enable toggle + type/frequency stats ──────────
const chkScan = document.getElementById('chk-scan');
const chkCloud = document.getElementById('chk-cloud');
const rowScan = document.getElementById('row-scan');
const rowCloud = document.getElementById('row-cloud');
const infoScan = document.getElementById('info-scan');
const infoCloud = document.getElementById('info-cloud');
const infoImu = document.getElementById('info-imu');
const infoEncoder = document.getElementById('info-encoder');

let scanEnabled = true, cloudEnabled = true;
chkScan.addEventListener('change', () => {
  scanEnabled = chkScan.checked;
  rowScan.classList.toggle('off', !scanEnabled);
  scanPoints.visible = scanEnabled;
  scanRays.visible = scanEnabled;
});
chkCloud.addEventListener('change', () => {
  cloudEnabled = chkCloud.checked;
  rowCloud.classList.toggle('off', !cloudEnabled);
  cloud3dPoints.visible = cloudEnabled;
});

// per-sensor Hz counters (independent of overall frame rate)
const sensorRate = {
  scan:  { count: 0, last: performance.now(), hz: '--' },
  cloud: { count: 0, last: performance.now(), hz: '--' },
};
function tickRate(key) {
  const s = sensorRate[key];
  s.count++;
  const now = performance.now();
  if (now - s.last >= 1000) {
    s.hz = (s.count * 1000 / (now - s.last)).toFixed(1);
    s.count = 0; s.last = now;
  }
  return s.hz;
}

// ── SSE sensor stream ─────────────────────────────────────────────────────────
const hud = document.getElementById('hud');
let posX=0, posY=0, posTh=0;
let fCount=0, fLast=performance.now(), curFps='--';
let lastHitCount=0;

const es = new EventSource('/stream');
es.onopen = () => {
  hud.innerHTML = '<span class="dim">Connected — waiting for data…</span>';
};
es.onmessage = ({ data }) => {
  const frame = JSON.parse(data);
  if (frame.pose) { [posX, posY, posTh] = frame.pose; updatePose(posX, posY, posTh); }
  if (frame.scan) {
    lastHitCount = updateScan(frame.scan, posX, posY, posTh, scanEnabled);
    const hz = tickRate('scan');
    const { ranges, angle_min, angle_max, range_max } = frame.scan;
    const fovDeg = ((angle_max - angle_min) * 180 / Math.PI).toFixed(0);
    infoScan.innerHTML =
      'type: <span class="v">2D LiDAR</span>, ' + ranges.length + ' beams, ' +
      fovDeg + '° fov, ' + range_max.toFixed(1) + 'm max<br>' +
      '<span class="v">' + hz + '</span> Hz';
  }
  if (frame.cloud) {
    updateCloud3D(frame.cloud, cloudEnabled);
    const hz = tickRate('cloud');
    infoCloud.innerHTML =
      'type: <span class="v">3D LiDAR</span>, ' + frame.cloud.n + ' pts<br>' +
      '<span class="v">' + hz + '</span> Hz';
  }
  if (frame.imu) {
    const [ax_, ay_, az_] = frame.imu.acc;
    const [gx, gy, gz] = frame.imu.gyro;
    infoImu.innerHTML =
      'acc=[' + ax_.toFixed(2) + ',' + ay_.toFixed(2) + ',' + az_.toFixed(2) + ']<br>' +
      'gyro=[' + gx.toFixed(3) + ',' + gy.toFixed(3) + ',' + gz.toFixed(3) + ']';
  }
  if (frame.encoder) {
    const [tl, tr] = frame.encoder.ticks;
    const [sl, sr] = frame.encoder.speed;
    infoEncoder.innerHTML =
      'ticks=[' + tl + ',' + tr + ']<br>' +
      'speed=[' + sl.toFixed(2) + ',' + sr.toFixed(2) + '] rad/s';
  }
  fCount++;
  const now = performance.now();
  if (now - fLast >= 1000) {
    curFps = (fCount * 1000 / (now - fLast)).toFixed(1);
    fCount = 0; fLast = now;
  }
  hud.innerHTML =
    '<span class="hi">3D LiDAR</span> — shmbridge\\n' +
    'x=<span class="val">' + posX.toFixed(3) + '</span>  ' +
    'y=<span class="val">' + posY.toFixed(3) + '</span>  ' +
    'θ=<span class="val">' + posTh.toFixed(3) + '</span>\\n' +
    'hits=<span class="val">' + lastHitCount + '</span>\\n' +
    '<span class="dim">' + curFps + ' fps · drag:orbit  scroll:zoom  R:reset</span>';
};
es.onerror = () => {
  hud.innerHTML = '<span style="color:#ef4444">Stream lost — is 06_irsim_bridge.py running?</span>';
};

// ── render loop ───────────────────────────────────────────────────────────────
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();
</script>
</body>
</html>
"""


# ── URDF → three.js wireframe JSON ───────────────────────────────────────────


def _urdf_to_wireframe_json(robot) -> str:
    """Return a JSON array of {rgba, segments} objects for three.js LineSegments.

    Each element's geometry is pre-transformed into its world (or robot-local)
    frame so the JS side needs no further matrix math per element.
    """
    from urdf_tools.geometry import box_wireframe, cylinder_wireframe, sphere_wireframe
    from urdf_tools.viz import _link_world_blocks

    result = []
    for T, geom, rgba in _link_world_blocks(robot):
        if geom.type == "box":
            corners, edges = box_wireframe(geom.size)
        elif geom.type == "cylinder":
            corners, edges = cylinder_wireframe(geom.radius, geom.length)
        elif geom.type == "sphere":
            corners, edges = sphere_wireframe(geom.radius)
        else:
            continue
        h = np.column_stack([corners, np.ones(len(corners))])
        w = (T @ h.T).T[:, :3]
        # Flat array [x0,y0,z0, x1,y1,z1, ...] — one pair per edge
        segs: list[float] = []
        for i, j in edges:
            p0, p1 = w[i], w[j]
            segs += [
                round(float(p0[0]), 4),
                round(float(p0[1]), 4),
                round(float(p0[2]), 4),
                round(float(p1[0]), 4),
                round(float(p1[1]), 4),
                round(float(p1[2]), 4),
            ]
        result.append(
            {
                "rgba": [round(float(c), 4) for c in rgba],
                "segments": segs,
            }
        )
    return _json.dumps(result)


# ── 3D viewer — three.js web UI ───────────────────────────────────────────────


def _free_port() -> int:
    """Return an available TCP port on localhost."""
    with _socket.socket() as s:
        s.bind(("", 0))
        return s.getsockname()[1]


def live3d(
    sub: SensorSubscriber,
    world_robot=None,
    overlay_robot=None,
    port: int = 0,
) -> None:
    """Serve a three.js 3D viewer over localhost HTTP + SSE and open a browser.

    The browser renders the warehouse wireframe and animates the robot + scan
    cloud using WebGL via three.js. Sensor data is streamed from shmbridge
    via Server-Sent Events (no extra dependencies required beyond stdlib).
    """
    import webbrowser
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    _html = _HTML_VIEWER.encode()
    _world_json = (
        _urdf_to_wireframe_json(world_robot).encode() if world_robot else b"[]"
    )
    _robot_json = (
        _urdf_to_wireframe_json(overlay_robot).encode() if overlay_robot else b"[]"
    )

    data_q: _queue.Queue = _queue.Queue(maxsize=4)

    class _H(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            p = self.path.split("?")[0]
            if p == "/":
                self._reply(200, "text/html; charset=utf-8", _html)
            elif p == "/world.json":
                self._reply(200, "application/json", _world_json)
            elif p == "/robot.json":
                self._reply(200, "application/json", _robot_json)
            elif p == "/stream":
                self._sse()
            else:
                self._reply(404, "text/plain", b"Not found")

        def _reply(self, code: int, ct: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _sse(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            while True:
                try:
                    payload = data_q.get(timeout=0.8)
                    msg = ("data: " + _json.dumps(payload) + "\n\n").encode()
                    self.wfile.write(msg)
                    self.wfile.flush()
                except _queue.Empty:
                    try:  # keepalive comment
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
                    except Exception:
                        break
                except Exception:
                    break

        def log_message(self, *_) -> None:  # suppress HTTP log spam
            pass

    if port == 0:
        port = _free_port()

    srv = ThreadingHTTPServer(("127.0.0.1", port), _H)
    t = _threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    url = f"http://127.0.0.1:{port}/"
    print(f"[web3d] {url}  (Ctrl+C to stop)")
    webbrowser.open(url)

    _has_3d = sub.try_attach_cloud3d(timeout_ms=2000)
    if _has_3d:
        print("[web3d] 3D cloud channel attached")
    else:
        print("[web3d] 3D cloud channel unavailable (no --world or EmbreeLidar3D)")

    pose = [0.0, 0.0, 0.0]
    try:
        while True:
            scan = sub.read_scan()
            odom = sub.read_odom()
            if odom:
                pose[:] = [odom.x, odom.y, odom.theta]

            frame: dict = {
                "pose": [round(pose[0], 4), round(pose[1], 4), round(pose[2], 4)]
            }
            if scan and scan.ranges:
                frame["scan"] = {
                    "ranges": [round(r, 3) for r in scan.ranges],
                    "angle_min": round(scan.angle_min, 6),
                    "angle_max": round(scan.angle_max, 6),
                    "angle_increment": round(scan.angle_increment, 7),
                    "range_max": round(scan.range_max, 3),
                }
            imu = sub.read_imu()
            if imu:
                frame["imu"] = {
                    "acc": [round(v, 4) for v in imu.linear_acceleration],
                    "gyro": [round(v, 5) for v in imu.angular_velocity],
                }
            encoder = sub.read_encoder()
            if encoder:
                frame["encoder"] = {
                    "ticks": list(encoder.ticks[:2]),
                    "speed": [round(s, 3) for s in encoder.speed[:2]],
                }
            if _has_3d:
                pts3d = sub.read_cloud3d()
                if pts3d is not None and len(pts3d):
                    # subsample to MAX_PTS3D for SSE bandwidth
                    step = max(1, len(pts3d) // 2048)
                    pts3d = pts3d[::step][:2048]
                    z_col = pts3d[:, 2]
                    flat = [round(float(v), 3) for v in pts3d[:, :3].ravel()]
                    frame["cloud"] = {
                        "pts": flat,
                        "n": len(pts3d),
                        "z_min": round(float(z_col.min()), 3),
                        "z_max": round(float(z_col.max()), 3),
                    }
            try:
                data_q.put_nowait(frame)
            except _queue.Full:
                pass  # drop frame — client is behind

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[web3d] stopped.")
    finally:
        srv.shutdown()


# ── shared style helpers ───────────────────────────────────────────────────────


def _add_telemetry_text(ax, *, is_3d: bool = False):
    """Add a bottom-left monospace text box for IMU/encoder readouts."""
    kwargs = dict(
        fontsize=8,
        color=_TEXT,
        family="monospace",
        va="bottom",
        ha="left",
        transform=ax.transAxes,
        bbox=dict(fc=_PANE, ec=_GRID, lw=0.8, alpha=0.85, pad=4),
        zorder=20,
    )
    if is_3d:
        return ax.text2D(0.02, 0.02, "", **kwargs)
    return ax.text(0.02, 0.02, "", **kwargs)


def _format_telemetry(imu, encoder) -> str:
    """Format IMU + encoder readings for the telemetry text box."""
    lines = []
    if imu:
        ax_, ay_, az_ = imu.linear_acceleration
        gx, gy, gz = imu.angular_velocity
        lines.append(f"IMU  acc=[{ax_:5.2f},{ay_:5.2f},{az_:5.2f}] m/s2")
        lines.append(f"     gyro=[{gx:6.3f},{gy:6.3f},{gz:6.3f}] rad/s")
    if encoder:
        lines.append(f"Enc  ticks={encoder.ticks[:2]}")
        lines.append(
            f"     speed=[{encoder.speed[0]:5.2f},{encoder.speed[1]:5.2f}] rad/s"
        )
    return "\n".join(lines)


def _style_fig(fig) -> None:
    fig.patch.set_facecolor(_BG)


def _style_ax2d(ax, title: str = "") -> None:
    ax.set_facecolor(_BG)
    ax.set_aspect("equal")
    ax.tick_params(colors=_TEXT, labelsize=8)
    ax.set_xlabel("X (m)", color=_TEXT, fontsize=9)
    ax.set_ylabel("Y (m)", color=_TEXT, fontsize=9)
    for sp in ax.spines.values():
        sp.set_edgecolor(_GRID)
    ax.grid(True, color=_GRID, lw=0.4, alpha=0.7, zorder=0)
    if title:
        ax.set_title(title, color="white", fontweight="bold", fontsize=10)


def _style_ax3d(ax3, title: str = "") -> None:
    ax3.set_facecolor(_PANE)
    ax3.xaxis.pane.fill = False
    ax3.yaxis.pane.fill = False
    ax3.zaxis.pane.fill = False
    for a in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        a.pane.set_edgecolor(_GRID)
        a.line.set_color(_GRID)
    ax3.tick_params(colors=_TEXT, labelsize=7)
    ax3.set_xlabel("X (m)", color=_TEXT, fontsize=9)
    ax3.set_ylabel("Y (m)", color=_TEXT, fontsize=9)
    ax3.set_zlabel("Z (m)", color=_TEXT, fontsize=9)
    if title:
        ax3.set_title(title, color="white", fontweight="bold", fontsize=10)


# ── URDF overlay helpers ───────────────────────────────────────────────────────


def _draw_urdf_2d(ax, robot, *, alpha: float = 0.55, root_T=None) -> None:
    """Draw URDF geometry as 2-D patches onto *ax* (static, called once)."""
    import matplotlib.pyplot as plt

    from urdf_tools.viz import _link_world_blocks, _xy_footprint

    for T, geom, _rgba in _link_world_blocks(robot, root_T=root_T):
        for kind, params, _ in _xy_footprint(T, geom, _rgba):
            if kind == "polygon":
                ax.add_patch(
                    plt.Polygon(
                        params,
                        fc=_WORLD_FC,
                        ec=_WORLD_EC,
                        lw=0.8,
                        alpha=alpha,
                        zorder=2,
                    )
                )
            elif kind == "circle":
                cx, cy, r = params
                ax.add_patch(
                    plt.Circle(
                        (cx, cy),
                        r,
                        fc=_WORLD_FC,
                        ec=_WORLD_EC,
                        lw=0.8,
                        alpha=alpha,
                        zorder=2,
                    )
                )


def _world_bounds_2d(robot) -> tuple[float, float, float, float]:
    """Return (xmin, xmax, ymin, ymax) of the world URDF geometry."""
    from urdf_tools.viz import _link_world_blocks, _xy_footprint

    xs, ys = [], []
    for T, geom, rgba in _link_world_blocks(robot):
        for kind, params, _ in _xy_footprint(T, geom, rgba):
            if kind == "polygon":
                pts = np.asarray(params)
                xs.extend(pts[:, 0])
                ys.extend(pts[:, 1])
            elif kind == "circle":
                cx, cy, r = params
                xs += [cx - r, cx + r]
                ys += [cy - r, cy + r]
    if not xs:
        return -15, 15, -15, 15
    pad = max((max(xs) - min(xs)) * 0.05, 1.0)
    return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


def _draw_urdf_3d_static(ax3, robot) -> list[np.ndarray]:
    """Draw world URDF wireframe onto ax3 once; return world-space corners.

    Edge colour is derived from each geometry's URDF material rgba so that
    different element types (walls, columns, racks, beams) are visually distinct.
    """
    from urdf_tools.geometry import box_wireframe, cylinder_wireframe, sphere_wireframe
    from urdf_tools.viz import _link_world_blocks

    all_pts: list[np.ndarray] = []
    for T, geom, rgba in _link_world_blocks(robot):
        if geom.type == "box":
            corners, edges = box_wireframe(geom.size)
        elif geom.type == "cylinder":
            corners, edges = cylinder_wireframe(geom.radius, geom.length)
        elif geom.type == "sphere":
            corners, edges = sphere_wireframe(geom.radius)
        else:
            continue
        h = np.column_stack([corners, np.ones(len(corners))])
        w = (T @ h.T).T[:, :3]
        all_pts.append(w)
        r, g, b = float(rgba[0]), float(rgba[1]), float(rgba[2])
        mat_a = float(rgba[3]) if len(rgba) > 3 else 1.0
        ec = (r * 0.55 + 0.08, g * 0.55 + 0.10, b * 0.55 + 0.15)
        wire_a = min(mat_a * 0.65, 0.75)
        lw = 0.55 if geom.type == "box" else 0.65
        for i, j in edges:
            p0, p1 = w[i], w[j]
            ax3.plot(
                [p0[0], p1[0]],
                [p0[1], p1[1]],
                [p0[2], p1[2]],
                color=ec,
                lw=lw,
                alpha=wire_a,
            )
    return all_pts


def _robot_wireframe_segments(robot):
    """Pre-compute per-link homogeneous corner matrices + edge indices."""
    from urdf_tools.geometry import box_wireframe, cylinder_wireframe, sphere_wireframe
    from urdf_tools.viz import _link_world_blocks

    segs = []
    for _T, geom, rgba in _link_world_blocks(robot):
        if geom.type == "box":
            corners, edges = box_wireframe(geom.size)
        elif geom.type == "cylinder":
            corners, edges = cylinder_wireframe(geom.radius, geom.length)
        elif geom.type == "sphere":
            corners, edges = sphere_wireframe(geom.radius)
        else:
            continue
        h = np.column_stack([corners, np.ones(len(corners))])
        ec = tuple(max(0.0, c - 0.05) for c in rgba[:3])
        segs.append((h, edges, ec))
    return segs


# ── 2D real-time viewer ────────────────────────────────────────────────────────


def live2d(
    sub: SensorSubscriber,
    world_robot=None,
    overlay_robot=None,
) -> None:
    """Real-time 2D LiDAR viewer: world overlay + robot pose + scan."""
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    RMAX = 12.0
    RAY_STEP = 6  # draw every N-th beam as a ray line

    fig, ax = plt.subplots(figsize=(8, 8))
    _style_fig(fig)
    _style_ax2d(ax, "2D LiDAR — shmbridge")
    fig.tight_layout(pad=1.2)

    # Static world geometry
    if world_robot is not None:
        _draw_urdf_2d(ax, world_robot, alpha=0.55)
        xmin, xmax, ymin, ymax = _world_bounds_2d(world_robot)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        RMAX = min(max(xmax - xmin, ymax - ymin) * 0.4, 30.0)
    else:
        ax.set_xlim(-RMAX, RMAX)
        ax.set_ylim(-RMAX, RMAX)

    # Range rings (muted dashed circles)
    ring_step = max(1, int(RMAX / 4))
    for r in range(ring_step, int(RMAX) + 1, ring_step):
        ax.add_patch(
            plt.Circle((0, 0), r, fc="none", ec=_GRID, lw=0.5, ls="--", alpha=0.5)
        )

    # Animated artists
    scan_scat = ax.scatter(
        [],
        [],
        s=4,
        c=[],
        cmap=_SCAN_CMAP,
        vmin=0,
        vmax=RMAX,
        zorder=5,
        edgecolors="none",
    )
    ray_lines = [
        ax.plot([], [], color=_RAY_C, lw=0.3, alpha=0.18, zorder=3)[0]
        for _ in range(360 // RAY_STEP + 10)
    ]
    robot_circle = plt.Circle((0, 0), 0.2, fc=_ROBOT_C, ec="white", lw=0.8, zorder=7)
    ax.add_patch(robot_circle)
    (hdg_line,) = ax.plot([], [], color=_HDG_C, lw=2.0, zorder=8)
    telemetry_text = _add_telemetry_text(ax)

    state = {"pose": (0.0, 0.0, 0.0), "dynamic_xlim": world_robot is None}

    def update(_frame):
        scan = sub.read_scan()
        odom = sub.read_odom()
        imu = sub.read_imu()
        encoder = sub.read_encoder()
        telemetry_text.set_text(_format_telemetry(imu, encoder))
        if odom:
            state["pose"] = (odom.x, odom.y, odom.theta)
        x0, y0, th = state["pose"]

        # Robot circle + heading
        robot_circle.center = (x0, y0)
        hdg_line.set_data(
            [x0, x0 + 0.6 * math.cos(th)],
            [y0, y0 + 0.6 * math.sin(th)],
        )

        # Dynamic axis follow (when no world urdf)
        if state["dynamic_xlim"]:
            ax.set_xlim(x0 - RMAX, x0 + RMAX)
            ax.set_ylim(y0 - RMAX, y0 + RMAX)

        # Scan
        if scan and scan.ranges:
            rng = np.asarray(scan.ranges, dtype=np.float32)
            a = scan.angle_min + np.arange(len(rng)) * scan.angle_increment
            hit = rng < scan.range_max * 0.999
            hx = x0 + rng[hit] * np.cos(th + a[hit])
            hy = y0 + rng[hit] * np.sin(th + a[hit])
            pts = np.column_stack([hx, hy]) if len(hx) else np.empty((0, 2))
            scan_scat.set_offsets(pts)
            scan_scat.set_array(rng[hit])

            # Ray lines
            ray_idx = np.arange(0, len(rng), RAY_STEP)
            for k, li in enumerate(ray_lines):
                if k < len(ray_idx):
                    i = ray_idx[k]
                    ex = x0 + rng[i] * math.cos(th + float(a[i]))
                    ey = y0 + rng[i] * math.sin(th + float(a[i]))
                    li.set_data([x0, ex], [y0, ey])
                else:
                    li.set_data([], [])
        else:
            scan_scat.set_offsets(np.empty((0, 2)))
            for li in ray_lines:
                li.set_data([], [])

        return (scan_scat, robot_circle, hdg_line, telemetry_text, *ray_lines)

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    _ = ani
    plt.show()


# ── 3D matplotlib viewer (fallback) ───────────────────────────────────────────


def live3d_mpl(
    sub: SensorSubscriber,
    world_robot=None,
    overlay_robot=None,
) -> None:
    """Fallback matplotlib 3D viewer (use --live3d-mpl).  Prefer live3d() for
    the full-featured three.js web viewer (--live3d).
    """
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    from urdf_tools.geometry import pose_to_matrix as _p2m

    fig = plt.figure(figsize=(11, 9))
    _style_fig(fig)
    ax3 = fig.add_subplot(111, projection="3d")
    _style_ax3d(ax3, "3D LiDAR — shmbridge")
    fig.tight_layout(pad=1.0)

    world_pts: list[np.ndarray] = []
    if world_robot is not None:
        world_pts = _draw_urdf_3d_static(ax3, world_robot)

    robot_segs = _robot_wireframe_segments(overlay_robot) if overlay_robot else []
    robot_lines: list[tuple] = []
    for h, edges, ec in robot_segs:
        seg_lines = []
        for i, j in edges:
            (ln,) = ax3.plot([], [], [], color=ec, lw=1.1, alpha=0.95)
            seg_lines.append((ln, h, i, j))
        robot_lines.append(seg_lines)

    scan_holder: list = [None]
    scan_rays: list = []
    robot_dot = ax3.scatter([], [], [], s=90, c=[_ROBOT_C], zorder=6, marker="^")
    telemetry_text = _add_telemetry_text(ax3, is_3d=True)

    if world_pts:
        pts = np.vstack(world_pts)
        xmin, ymin, zmin = pts.min(0)
        xmax, ymax, zmax = pts.max(0)
        xpad = (xmax - xmin) * 0.03 + 1.0
        ypad = (ymax - ymin) * 0.03 + 1.0
        ax3.set_xlim(xmin - xpad, xmax + xpad)
        ax3.set_ylim(ymin - ypad, ymax + ypad)
        ax3.set_zlim(-0.5, zmax + 0.5)
        ax3.view_init(elev=32, azim=-52)
    else:
        ax3.set_xlim(-12, 12)
        ax3.set_ylim(-12, 12)
        ax3.set_zlim(-1, 8)
        ax3.view_init(elev=32, azim=-52)

    scan_rmax = [12.0]
    state = {"pose": (0.0, 0.0, 0.0)}

    def update(_frame):
        scan = sub.read_scan()
        odom = sub.read_odom()
        imu = sub.read_imu()
        encoder = sub.read_encoder()
        telemetry_text.set_text(_format_telemetry(imu, encoder))
        if odom:
            state["pose"] = (odom.x, odom.y, odom.theta)
        x0, y0, th = state["pose"]

        root_T = _p2m([x0, y0, 0.0], [0.0, 0.0, th])
        for seg_lines in robot_lines:
            for ln, h, i, j in seg_lines:
                w = (root_T @ h.T).T[:, :3]
                ln.set_data_3d(
                    [w[i, 0], w[j, 0]], [w[i, 1], w[j, 1]], [w[i, 2], w[j, 2]]
                )
        robot_dot._offsets3d = ([x0], [y0], [0.12])

        if scan_holder[0] is not None:
            try:
                scan_holder[0].remove()
            except Exception:
                pass
            scan_holder[0] = None
        for _ln in scan_rays:
            try:
                _ln.remove()
            except Exception:
                pass
        scan_rays.clear()

        if scan and scan.ranges:
            scan_rmax[0] = scan.range_max
            rng = np.asarray(scan.ranges, dtype=np.float32)
            a = scan.angle_min + np.arange(len(rng)) * scan.angle_increment
            hit = rng < scan.range_max * 0.999
            if hit.any():
                lx = x0 + rng[hit] * np.cos(th + a[hit])
                ly = y0 + rng[hit] * np.sin(th + a[hit])
                _LZ = 0.30
                lz = np.full(hit.sum(), _LZ, dtype=np.float32)
                norm_c = np.clip(rng[hit] / max(scan_rmax[0], 1e-6), 0.0, 1.0)
                colors_rgba = plt.cm.plasma(norm_c)
                scan_holder[0] = ax3.scatter(
                    lx,
                    ly,
                    lz,
                    s=18,
                    c=colors_rgba,
                    alpha=0.95,
                    edgecolors="none",
                    depthshade=False,
                    zorder=5,
                )
                for _k in range(0, hit.sum(), 8):
                    _c = tuple(colors_rgba[_k, :3])
                    (_ray,) = ax3.plot(
                        [x0, float(lx[_k])],
                        [y0, float(ly[_k])],
                        [_LZ, _LZ],
                        color=_c,
                        lw=0.7,
                        alpha=0.35,
                    )
                    scan_rays.append(_ray)

        artists = [robot_dot, telemetry_text] + [
            ln for seg_lines in robot_lines for ln, *_ in seg_lines
        ]
        if scan_holder[0] is not None:
            artists.append(scan_holder[0])
        artists.extend(scan_rays)
        return artists

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    _ = ani
    plt.show()


# ── text mode ─────────────────────────────────────────────────────────────────


def text_mode(sub: SensorSubscriber, count: int) -> None:
    n = 0
    try:
        while True:
            scan = sub.read_scan()
            imu = sub.read_imu()
            odom = sub.read_odom()
            encoder = sub.read_encoder()
            if scan:
                hits = sum(1 for r in scan.ranges if r < scan.range_max)
                print(
                    f"  [scan] t={scan.stamp:.3f}"
                    f"  beams={len(scan.ranges)}  hits={hits}"
                )
            if imu:
                print(
                    f"  [imu]  t={imu.stamp:.3f}"
                    f"  acc={[f'{v:.2f}' for v in imu.linear_acceleration]}"
                    f"  gyro={[f'{v:.3f}' for v in imu.angular_velocity]}"
                )
            if odom:
                print(
                    f"  [odom] t={odom.stamp:.3f}"
                    f"  x={odom.x:.2f}  y={odom.y:.2f}  θ={odom.theta:.2f}"
                )
            if encoder:
                print(
                    f"  [enc]  t={encoder.stamp:.3f}"
                    f"  ticks={encoder.ticks[:2]}"
                    f"  speed(rad/s)={[f'{s:.2f}' for s in encoder.speed[:2]]}"
                )
            n += 1
            if count and n >= count:
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped.")


# ── entry point ───────────────────────────────────────────────────────────────


def main() -> None:
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    import argparse

    ap = argparse.ArgumentParser(
        description="Subscribe to sensor topics and visualise in 2D or 3D"
    )
    ap.add_argument("--shm", default=SHM_NAME, help="Shared memory segment name")
    ap.add_argument(
        "--count", type=int, default=0, help="Stop after N polls (text mode only)"
    )
    ap.add_argument(
        "--live2d",
        "--live",
        dest="live2d",
        action="store_true",
        help="Open real-time 2D LiDAR viewer (matplotlib)",
    )
    ap.add_argument(
        "--live3d",
        action="store_true",
        help="Open real-time 3D LiDAR viewer (three.js web UI, opens browser)",
    )
    ap.add_argument(
        "--live3d-mpl",
        dest="live3d_mpl",
        action="store_true",
        help="Open real-time 3D LiDAR viewer (matplotlib fallback)",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=0,
        help="Port for the three.js web server (0 = auto-select)",
    )
    ap.add_argument(
        "--world",
        default=None,
        metavar="URDF",
        help="World URDF drawn as static background overlay",
    )
    ap.add_argument(
        "--robot",
        default=None,
        metavar="URDF",
        help="Robot URDF model to animate at odometry pose",
    )
    ap.add_argument(
        "--timeout", type=float, default=10000.0, help="Attach timeout in ms"
    )
    args = ap.parse_args()

    world_robot = None
    overlay_robot = None
    if args.world or args.robot:
        from urdf_tools.parser import parse_urdf

        if args.world:
            world_robot = parse_urdf(args.world)
            print(f"World : {world_robot.name!r}  links={len(world_robot.links)}")
        if args.robot:
            overlay_robot = parse_urdf(args.robot)
            print(f"Robot : {overlay_robot.name!r}  links={len(overlay_robot.links)}")

    print(f"Attaching to shm={args.shm!r}  (timeout={args.timeout:.0f} ms) …")
    sub = SensorSubscriber(args.shm, timeout_ms=args.timeout)
    try:
        sub.attach()
    except TimeoutError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print("Attached.  Ctrl+C to stop.\n")

    try:
        if args.live3d:
            live3d(
                sub,
                world_robot=world_robot,
                overlay_robot=overlay_robot,
                port=args.port,
            )
        elif args.live3d_mpl:
            live3d_mpl(sub, world_robot=world_robot, overlay_robot=overlay_robot)
        elif args.live2d:
            live2d(sub, world_robot=world_robot, overlay_robot=overlay_robot)
        else:
            text_mode(sub, args.count)
    finally:
        sub.detach()


if __name__ == "__main__":
    main()
