#!/usr/bin/env python3
"""
bench_notify.py  —  Notification-mechanism latency benchmark

Compares four notification channels that can signal a subscriber after
a new shmbridge state has been written.  Data always flows through shm
(ShmPublisher / ShmSubscriber) with write_ns embedded inside the seqlock
so end-to-end usable_latency = now_ns() - state.write_ns is accurate.

Mechanisms
----------
  uds_dgram   : 1-byte UDP-like datagram over UNIX-domain socket  (baseline)
  posix_sem   : multiprocessing.Semaphore (POSIX futex-backed)
  pipe        : multiprocessing.Pipe (os.pipe2 + buffered read)
  eventfd     : os.eventfd() file-descriptor passed via SCM_RIGHTS

Rates tested  : 100 Hz, 1 000 Hz

Metrics
-------
  wakeup_us   : time from notify send to subscriber wake  (notify_ts inside pub)
  usable_lat_us : now_ns() - state.write_ns  (true data age when used)
  sub_cpu_pct : subscriber process CPU%
  p50 / p99 / p999
"""

import contextlib
import multiprocessing as mp
import os
import resource
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass, field

# ── build path ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# shmbridge _core must be built first
try:
    from shmbridge import _core
except ImportError:
    import subprocess
    subprocess.check_call(
        [sys.executable, "-m", "pip", "install", "-e", ".", "-q"],
        cwd=os.path.join(os.path.dirname(__file__), ".."),
    )
    from shmbridge import _core

# ── helpers ───────────────────────────────────────────────────────────────────

def _now_ns() -> int:
    return time.monotonic_ns()


def _pct(lst: list[float], p: float) -> float:
    if not lst:
        return float("nan")
    s = sorted(lst)
    idx = max(0, int(len(s) * p / 100) - 1)
    return s[idx]


def _cpu_us() -> int:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return int((r.ru_utime + r.ru_stime) * 1e6)


_SHM_NAME  = "/bench_notify_shm"
_UDS_DATA  = "/tmp/bench_notify_data.sock"
_UDS_SETUP = "/tmp/bench_notify_setup.sock"

_WARMUP   = 50
_N_RUNS   = 500
_TIMEOUT  = 2.0   # seconds before declaring a lost notification


@dataclass
class RunResult:
    wakeup_us:     list[float] = field(default_factory=list)
    usable_lat_us: list[float] = field(default_factory=list)
    sub_cpu_pct:   float = 0.0
    pub_cpu_pct:   float = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# Mechanism helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _uds_server(path: str) -> socket.socket:
    with contextlib.suppress(FileNotFoundError):
        os.unlink(path)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(path)
    return s


def _uds_client(path: str) -> socket.socket:
    return socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)


# ═══════════════════════════════════════════════════════════════════════════════
# Publisher worker (common skeleton, mechanism-specific notify)
# ═══════════════════════════════════════════════════════════════════════════════

def _pub_uds(sensor_hz: float, shm_ready: mp.Event, sub_ready: mp.Event,
             done: mp.Event, q: mp.Queue):
    pub = _core.ShmPublisher(_SHM_NAME, 1, 1, 1)
    pub.open()
    shm_ready.set()          # shm exists now — subscriber can attach
    sub_ready.wait(10)       # wait for subscriber UDS socket to be bound

    state = _core.RobotState()
    sock = _uds_client(_UDS_DATA)

    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    period = 1.0 / sensor_hz

    for step in range(_WARMUP + _N_RUNS):
        deadline = time.perf_counter() + period
        state.step = step
        pub.write_state(0, state)
        notify_ts = _now_ns()
        sock.sendto(struct.pack("!q", notify_ts), _UDS_DATA)

        rem = deadline - time.perf_counter()
        if rem > 0:
            time.sleep(rem)

    done.set()
    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({"pub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4)})
    pub.close()
    sock.close()


def _pub_sem(sensor_hz: float, sem: mp.Semaphore, ready: mp.Event,
             done: mp.Event, q: mp.Queue):
    pub = _core.ShmPublisher(_SHM_NAME, 1, 1, 1)
    pub.open()
    state = _core.RobotState()

    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    period = 1.0 / sensor_hz
    ready.set()

    for step in range(_WARMUP + _N_RUNS):
        deadline = time.perf_counter() + period
        state.step = step
        pub.write_state(0, state)
        sem.release()

        rem = deadline - time.perf_counter()
        if rem > 0:
            time.sleep(rem)

    done.set()
    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({"pub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4)})
    pub.close()


def _pub_pipe(sensor_hz: float, send_conn, ready: mp.Event,
              done: mp.Event, q: mp.Queue):
    pub = _core.ShmPublisher(_SHM_NAME, 1, 1, 1)
    pub.open()
    state = _core.RobotState()

    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    period = 1.0 / sensor_hz
    ready.set()

    for step in range(_WARMUP + _N_RUNS):
        deadline = time.perf_counter() + period
        state.step = step
        pub.write_state(0, state)
        send_conn.send_bytes(b"\x01")

        rem = deadline - time.perf_counter()
        if rem > 0:
            time.sleep(rem)

    done.set()
    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({"pub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4)})
    pub.close()
    send_conn.close()


def _pub_eventfd(sensor_hz: float, setup_path: str, ready: mp.Event,
                 done: mp.Event, q: mp.Queue):
    """Publisher creates eventfd, hands it to subscriber via SCM_RIGHTS."""
    efd = os.eventfd(0, os.EFD_SEMAPHORE)

    # Send fd to subscriber via a setup UDS
    setup_srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    with contextlib.suppress(FileNotFoundError):
        os.unlink(setup_path)
    setup_srv.bind(setup_path)
    setup_srv.listen(1)

    pub = _core.ShmPublisher(_SHM_NAME, 1, 1, 1)
    pub.open()
    state = _core.RobotState()

    ready.set()  # subscriber can now connect

    conn, _ = setup_srv.accept()
    socket.send_fds(conn, [b"efd"], [efd])
    conn.close()
    setup_srv.close()

    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    period = 1.0 / sensor_hz

    for step in range(_WARMUP + _N_RUNS):
        deadline = time.perf_counter() + period
        state.step = step
        pub.write_state(0, state)
        os.eventfd_write(efd, 1)

        rem = deadline - time.perf_counter()
        if rem > 0:
            time.sleep(rem)

    done.set()
    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({"pub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4)})
    pub.close()
    os.close(efd)
    with contextlib.suppress(FileNotFoundError):
        os.unlink(setup_path)


# ═══════════════════════════════════════════════════════════════════════════════
# Subscriber workers
# ═══════════════════════════════════════════════════════════════════════════════

def _sub_uds(shm_ready: mp.Event, sub_ready: mp.Event, done: mp.Event, q: mp.Queue):
    # Bind UDS socket first (so publisher knows we're ready to receive)
    srv = _uds_server(_UDS_DATA)
    srv.settimeout(_TIMEOUT)

    shm_ready.wait(10)      # shm exists now
    sub = _core.ShmSubscriber(_SHM_NAME, 1)
    sub.attach(5_000)
    sub_ready.set()          # publisher can now start writing

    wakeup_us: list[float] = []
    usable_lat: list[float] = []
    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    step = 0

    while not done.is_set() or step < _WARMUP + _N_RUNS:
        try:
            data, _ = srv.recvfrom(8)
        except TimeoutError:
            break
        notify_ts = struct.unpack("!q", data)[0]
        wake_ts   = _now_ns()
        state = sub.read_state_spin(0, 64)
        use_ts = _now_ns()
        if state is None:
            continue
        step += 1
        if step <= _WARMUP:
            continue
        wk = (wake_ts - notify_ts) / 1e3
        ul = (use_ts - state.write_ns) / 1e3
        if 0 < wk < 50_000:
            wakeup_us.append(wk)
        if 0 < ul < 50_000:
            usable_lat.append(ul)

    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({
        "wakeup_us": wakeup_us,
        "usable_lat_us": usable_lat,
        "sub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4),
    })
    sub.detach()
    srv.close()
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_DATA)


def _sub_sem(sem: mp.Semaphore, done: mp.Event, q: mp.Queue):
    sub = _core.ShmSubscriber(_SHM_NAME, 1)
    sub.attach(30_000)

    wakeup_us: list[float] = []
    usable_lat: list[float] = []
    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    step = 0

    while not done.is_set() or step < _WARMUP + _N_RUNS:
        if not sem.acquire(timeout=_TIMEOUT):
            break
        wake_ts = _now_ns()
        state = sub.read_state_spin(0, 64)
        use_ts = _now_ns()
        if state is None:
            continue
        step += 1
        if step <= _WARMUP:
            continue
        # write_ns captured inside seqlock right before sem.release() → true wakeup ref
        wk = (wake_ts - state.write_ns) / 1e3
        ul = (use_ts - state.write_ns) / 1e3
        if 0 < wk < 50_000:
            wakeup_us.append(wk)
        if 0 < ul < 50_000:
            usable_lat.append(ul)

    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({
        "wakeup_us": wakeup_us,
        "usable_lat_us": usable_lat,
        "sub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4),
    })
    sub.detach()


def _sub_pipe(recv_conn, done: mp.Event, q: mp.Queue):
    sub = _core.ShmSubscriber(_SHM_NAME, 1)
    sub.attach(30_000)

    wakeup_us: list[float] = []
    usable_lat: list[float] = []
    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    step = 0

    while not done.is_set() or step < _WARMUP + _N_RUNS:
        if not recv_conn.poll(_TIMEOUT):
            break
        recv_conn.recv_bytes()
        wake_ts = _now_ns()
        state = sub.read_state_spin(0, 64)
        use_ts = _now_ns()
        if state is None:
            continue
        step += 1
        if step <= _WARMUP:
            continue
        # write_ns captured inside seqlock right before pipe write → true wakeup ref
        wk = (wake_ts - state.write_ns) / 1e3
        ul = (use_ts - state.write_ns) / 1e3
        if 0 < wk < 50_000:
            wakeup_us.append(wk)
        if 0 < ul < 50_000:
            usable_lat.append(ul)

    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({
        "wakeup_us": wakeup_us,
        "usable_lat_us": usable_lat,
        "sub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4),
    })
    sub.detach()
    recv_conn.close()


def _sub_eventfd(setup_path: str, done: mp.Event, q: mp.Queue):
    """Subscriber receives eventfd via SCM_RIGHTS, then blocks with select."""
    # Connect to publisher's setup socket and receive the fd
    setup_cli = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    for _ in range(50):
        try:
            setup_cli.connect(setup_path)
            break
        except (FileNotFoundError, ConnectionRefusedError):
            time.sleep(0.05)
    else:
        q.put({"error": "setup socket not found"})
        return

    _, fds, _, _ = socket.recv_fds(setup_cli, 4, 1)
    setup_cli.close()
    if not fds:
        q.put({"error": "no fd received"})
        return
    efd = fds[0]

    sub = _core.ShmSubscriber(_SHM_NAME, 1)
    sub.attach(30_000)

    wakeup_us: list[float] = []
    usable_lat: list[float] = []
    cpu0 = _cpu_us()
    t0   = time.perf_counter()
    step = 0

    while not done.is_set() or step < _WARMUP + _N_RUNS:
        rlist, _, _ = select.select([efd], [], [], _TIMEOUT)
        if not rlist:
            break
        os.eventfd_read(efd)
        wake_ts = _now_ns()
        state = sub.read_state_spin(0, 64)
        use_ts = _now_ns()
        if state is None:
            continue
        step += 1
        if step <= _WARMUP:
            continue
        # write_ns captured inside seqlock right before eventfd_write → true wakeup ref
        wk = (wake_ts - state.write_ns) / 1e3
        ul = (use_ts - state.write_ns) / 1e3
        if 0 < wk < 50_000:
            wakeup_us.append(wk)
        if 0 < ul < 50_000:
            usable_lat.append(ul)

    elapsed = time.perf_counter() - t0
    cpu1 = _cpu_us()
    q.put({
        "wakeup_us": wakeup_us,
        "usable_lat_us": usable_lat,
        "sub_cpu_pct": (cpu1 - cpu0) / (elapsed * 1e4),
    })
    sub.detach()
    os.close(efd)


# ═══════════════════════════════════════════════════════════════════════════════
# Run helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _collect(pub_p: mp.Process, sub_p: mp.Process, q: mp.Queue) -> RunResult:
    pub_p.join(timeout=(_WARMUP + _N_RUNS) / 10 + 5)
    sub_p.join(timeout=3)
    result = RunResult()
    items = {}
    while not q.empty():
        items.update(q.get_nowait())
    result.wakeup_us     = items.get("wakeup_us", [])
    result.usable_lat_us = items.get("usable_lat_us", [])
    result.sub_cpu_pct   = items.get("sub_cpu_pct", float("nan"))
    result.pub_cpu_pct   = items.get("pub_cpu_pct", float("nan"))
    return result


def run_uds(sensor_hz: float) -> RunResult:
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_DATA)
    q         = mp.Queue()
    shm_ready = mp.Event()
    sub_ready = mp.Event()
    done      = mp.Event()
    pub_p = mp.Process(
        target=_pub_uds, args=(sensor_hz, shm_ready, sub_ready, done, q), daemon=True
    )
    sub_p = mp.Process(
        target=_sub_uds, args=(shm_ready, sub_ready, done, q), daemon=True
    )
    pub_p.start()
    sub_p.start()
    sub_ready.wait(10)
    return _collect(pub_p, sub_p, q)


def run_sem(sensor_hz: float) -> RunResult:
    sem   = mp.Semaphore(0)
    q     = mp.Queue()
    ready = mp.Event()
    done  = mp.Event()
    pub_p = mp.Process(
        target=_pub_sem, args=(sensor_hz, sem, ready, done, q), daemon=True
    )
    sub_p = mp.Process(
        target=_sub_sem, args=(sem, done, q), daemon=True
    )
    pub_p.start()
    sub_p.start()
    ready.wait(5)
    return _collect(pub_p, sub_p, q)


def run_pipe(sensor_hz: float) -> RunResult:
    recv_conn, send_conn = mp.Pipe(duplex=False)
    q     = mp.Queue()
    ready = mp.Event()
    done  = mp.Event()
    pub_p = mp.Process(
        target=_pub_pipe, args=(sensor_hz, send_conn, ready, done, q), daemon=True
    )
    sub_p = mp.Process(
        target=_sub_pipe, args=(recv_conn, done, q), daemon=True
    )
    pub_p.start()
    sub_p.start()
    ready.wait(5)
    return _collect(pub_p, sub_p, q)


def run_eventfd(sensor_hz: float) -> RunResult:
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_SETUP)
    q     = mp.Queue()
    ready = mp.Event()
    done  = mp.Event()
    sub_p = mp.Process(
        target=_sub_eventfd, args=(_UDS_SETUP, done, q), daemon=True
    )
    pub_p = mp.Process(
        target=_pub_eventfd, args=(sensor_hz, _UDS_SETUP, ready, done, q), daemon=True
    )
    # publisher must bind setup socket before subscriber connects
    pub_p.start()
    ready.wait(5)
    sub_p.start()
    return _collect(pub_p, sub_p, q)


# ═══════════════════════════════════════════════════════════════════════════════
# Report
# ═══════════════════════════════════════════════════════════════════════════════

MECHANISMS = [
    ("uds_dgram",  "UDS DGRAM",    run_uds),
    ("posix_sem",  "POSIX sem",    run_sem),
    ("pipe",       "mp.Pipe",      run_pipe),
    ("eventfd",    "eventfd SCM",  run_eventfd),
]

RATES = [100, 1_000]


def _fmt(v: float) -> str:
    if v != v:
        return "   N/A"
    return f"{v:6.1f}"


def main() -> None:
    mp.set_start_method("fork", force=True)  # fork keeps Semaphore/Pipe handles

    print(f"\n{'Notification mechanism benchmark':^72}")
    print(f"{'n_runs=' + str(_N_RUNS) + '  warmup=' + str(_WARMUP):^72}\n")

    rows: list[dict] = []

    for hz in RATES:
        print(f"── {hz} Hz ─────────────────────────────────────────────────────────")
        hdr = f"  {'Mechanism':<14}  {'wakeup p50':>10}  {'p99':>7}  "
        hdr += f"{'usable p50':>10}  {'p99':>7}  {'p999':>8}  {'sub_cpu%':>8}"
        print(hdr)
        print("  " + "-" * 68)

        for key, label, runner in MECHANISMS:
            try:
                r = runner(float(hz))
            except Exception as e:
                print(f"  {label:<14}  ERROR: {e}")
                continue

            wp50  = _pct(r.wakeup_us, 50)
            wp99  = _pct(r.wakeup_us, 99)
            up50  = _pct(r.usable_lat_us, 50)
            up99  = _pct(r.usable_lat_us, 99)
            up999 = _pct(r.usable_lat_us, 99.9)

            row = {
                "hz": hz, "key": key, "label": label,
                "wp50": wp50, "wp99": wp99,
                "up50": up50, "up99": up99, "up999": up999,
                "sub_cpu": r.sub_cpu_pct,
            }
            rows.append(row)

            line = (
                f"  {label:<14}"
                f"  {_fmt(wp50)} µs"
                f"  {_fmt(wp99)} µs"
                f"  {_fmt(up50)} µs"
                f"  {_fmt(up99)} µs"
                f"  {_fmt(up999)} µs"
                f"  {r.sub_cpu_pct:7.1f}%"
            )
            print(line)
        print()

    _build_report(rows)


# ═══════════════════════════════════════════════════════════════════════════════
# HTML report
# ═══════════════════════════════════════════════════════════════════════════════

def _build_report(rows: list[dict]) -> None:
    import json

    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, "notify_report.html")

    # ── color map ──────────────────────────────────────────────────────────
    mech_colors = {
        "uds_dgram": "#6b93c4",
        "posix_sem": "#5bb874",
        "pipe":      "#e09c44",
        "eventfd":   "#c4627a",
    }
    mech_labels = {
        "uds_dgram": "UDS DGRAM",
        "posix_sem": "POSIX sem",
        "pipe":      "mp.Pipe",
        "eventfd":   "eventfd SCM",
    }

    # ── build JSON datasets for Chart.js ──────────────────────────────────
    keys = [m[0] for m in MECHANISMS]

    def chart_data(hz_val: int, metric: str) -> str:
        """Return Chart.js datasets JSON for a given hz and metric."""
        hz_rows = [r for r in rows if r["hz"] == hz_val]
        datasets = []
        labels_x = ["p50", "p99", "p999"] if "999" in metric or metric == "up" else ["p50", "p99"]
        for k in keys:
            row = next((r for r in hz_rows if r["key"] == k), None)
            if row is None:
                continue
            if metric == "wakeup":
                data = [row["wp50"], row["wp99"]]
            else:  # usable
                data = [row["up50"], row["up99"], row["up999"]]
            datasets.append({
                "label": mech_labels[k],
                "data": [round(v, 2) if v == v else None for v in data],
                "backgroundColor": mech_colors[k] + "cc",
                "borderColor": mech_colors[k],
                "borderWidth": 2,
                "borderRadius": 4,
            })
        return json.dumps({"labels": labels_x if metric == "usable" else ["p50", "p99"],
                           "datasets": datasets})

    def cpu_data(hz_val: int) -> str:
        hz_rows = [r for r in rows if r["hz"] == hz_val]
        labels_x = [mech_labels[k] for k in keys if any(r["key"] == k for r in hz_rows)]
        data_cpu = []
        colors = []
        for k in keys:
            row = next((r for r in hz_rows if r["key"] == k), None)
            if row is None:
                continue
            data_cpu.append(round(row["sub_cpu"], 2))
            colors.append(mech_colors[k] + "cc")
        return json.dumps({
            "labels": labels_x,
            "datasets": [{"label": "sub CPU%", "data": data_cpu,
                           "backgroundColor": colors, "borderWidth": 1, "borderRadius": 4}]
        })

    cw100 = chart_data(100, "wakeup")
    cu100 = chart_data(100, "usable")
    cc100 = cpu_data(100)
    cw1k  = chart_data(1000, "wakeup")
    cu1k  = chart_data(1000, "usable")
    cc1k  = cpu_data(1000)

    # ── build summary table rows ───────────────────────────────────────────
    def tr(row: dict) -> str:
        def td(v: float, unit: str = "µs") -> str:
            if v != v:
                return "<td>—</td>"
            return f"<td>{v:.1f} {unit}</td>"

        color = mech_colors.get(row["key"], "#888")
        badge = (
            f'<span style="display:inline-block;width:10px;height:10px;'
            f'border-radius:50%;background:{color};margin-right:6px;"></span>'
        )
        return (
            f"<tr>"
            f"<td>{row['hz']} Hz</td>"
            f"<td>{badge}{mech_labels[row['key']]}</td>"
            f"{td(row['wp50'])}{td(row['wp99'])}"
            f"{td(row['up50'])}{td(row['up99'])}{td(row['up999'])}"
            f"<td>{row['sub_cpu']:.1f}%</td>"
            f"</tr>"
        )

    table_rows = "\n".join(tr(r) for r in rows)

    html = f"""<title>Notify Mechanism Benchmark</title>
<style>
:root {{
  --bg: #f8f8f6;
  --surface: #ffffff;
  --border: #e0ddd8;
  --text: #1a1a1a;
  --muted: #6b6b6b;
  --accent: #2d6a9f;
  --font-body: 'Inter', system-ui, sans-serif;
  --font-mono: 'JetBrains Mono', 'Fira Code', monospace;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    --bg: #141416;
    --surface: #1e2024;
    --border: #2e3238;
    --text: #e8e6e2;
    --muted: #909090;
    --accent: #6fa8d8;
  }}
}}
:root[data-theme="dark"] {{
  --bg: #141416;
  --surface: #1e2024;
  --border: #2e3238;
  --text: #e8e6e2;
  --muted: #909090;
  --accent: #6fa8d8;
}}
* {{ box-sizing: border-box; margin: 0; padding: 0; }}
body {{
  font-family: var(--font-body);
  background: var(--bg);
  color: var(--text);
  font-size: 14px;
  line-height: 1.6;
  padding: 2rem 1.5rem;
}}
h1 {{ font-size: 1.5rem; font-weight: 700; margin-bottom: 0.25rem; }}
.subtitle {{ color: var(--muted); font-size: 0.85rem; margin-bottom: 2rem; }}
h2 {{ font-size: 1rem; font-weight: 600; margin: 2rem 0 1rem; }}
.grid {{ display: grid; grid-template-columns: repeat(3, 1fr); gap: 1.25rem; margin-bottom: 2rem; }}
.card {{
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 1rem;
}}
.card h3 {{ font-size: 0.78rem; font-weight: 600; color: var(--muted);
             text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 0.75rem; }}
canvas {{ width: 100% !important; }}
table {{
  width: 100%;
  border-collapse: collapse;
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 8px;
  overflow: hidden;
  font-variant-numeric: tabular-nums;
  font-size: 0.82rem;
}}
th {{
  background: var(--bg);
  border-bottom: 1px solid var(--border);
  padding: 0.6rem 0.8rem;
  text-align: left;
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--muted);
  text-transform: uppercase;
  letter-spacing: 0.04em;
}}
td {{ padding: 0.55rem 0.8rem; border-bottom: 1px solid var(--border); }}
tr:last-child td {{ border-bottom: none; }}
tr:hover td {{ background: color-mix(in srgb, var(--accent) 6%, var(--surface)); }}
.insight {{
  background: var(--surface);
  border-left: 3px solid var(--accent);
  border-radius: 4px;
  padding: 0.9rem 1rem;
  margin: 1.25rem 0;
  font-size: 0.85rem;
  line-height: 1.7;
}}
.insight b {{ color: var(--accent); }}
</style>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&display=swap">
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>

<h1>Notification Mechanism Benchmark</h1>
<p class="subtitle">shmbridge data plane &nbsp;·&nbsp; {_N_RUNS} samples per cell &nbsp;·&nbsp; {_WARMUP} warmup discarded &nbsp;·&nbsp; Linux eventfd / POSIX sem / mp.Pipe / UDS DGRAM</p>

<h2>100 Hz sensor rate</h2>
<div class="grid">
  <div class="card">
    <h3>Wakeup latency (ns sent → wake)</h3>
    <canvas id="cw100"></canvas>
  </div>
  <div class="card">
    <h3>Usable latency (now - write_ns)</h3>
    <canvas id="cu100"></canvas>
  </div>
  <div class="card">
    <h3>Subscriber CPU%</h3>
    <canvas id="cc100"></canvas>
  </div>
</div>

<h2>1 000 Hz sensor rate</h2>
<div class="grid">
  <div class="card">
    <h3>Wakeup latency (ns sent → wake)</h3>
    <canvas id="cw1k"></canvas>
  </div>
  <div class="card">
    <h3>Usable latency (now - write_ns)</h3>
    <canvas id="cu1k"></canvas>
  </div>
  <div class="card">
    <h3>Subscriber CPU%</h3>
    <canvas id="cc1k"></canvas>
  </div>
</div>

<h2>Full results</h2>
<table>
  <thead>
    <tr>
      <th>Rate</th><th>Mechanism</th>
      <th>Wakeup p50</th><th>Wakeup p99</th>
      <th>Usable p50</th><th>Usable p99</th><th>Usable p999</th>
      <th>Sub CPU%</th>
    </tr>
  </thead>
  <tbody>
    {table_rows}
  </tbody>
</table>

<div class="insight">
  <b>Key insight:</b>
  The <em>usable latency</em> (now - write_ns) is the metric that matters for control quality — it
  is the actual age of the sensor data when the algorithm uses it.  Wakeup latency measures the
  notification cost alone.  All notification mechanisms share the same shm data plane, so usable
  latency differences reveal scheduling/buffering overhead, not transport delay.
  <br><br>
  <b>eventfd SCM_RIGHTS</b> is expected to show the lowest wakeup latency on Linux because the
  kernel write to the eventfd is a direct futex-wake on the reading process without a UDS
  queue copy.  <b>POSIX sem</b> (futex) is comparable.  <b>mp.Pipe</b> is slightly slower due
  to its read buffer.  <b>UDS DGRAM</b> carries the highest per-call syscall overhead.
</div>

<script>
const OPTS = (title) => ({{
  responsive: true,
  plugins: {{
    legend: {{ position: 'bottom', labels: {{ boxWidth: 10, font: {{ size: 11 }} }} }},
    title: {{ display: false }},
    tooltip: {{ callbacks: {{ label: ctx => ctx.dataset.label + ': ' + ctx.parsed.y + ' µs' }} }}
  }},
  scales: {{
    x: {{ grid: {{ display: false }}, ticks: {{ font: {{ size: 11 }} }} }},
    y: {{ beginAtZero: true, ticks: {{ font: {{ size: 11 }} }},
          title: {{ display: true, text: 'µs', font: {{ size: 10 }} }} }}
  }}
}});
const CPU_OPTS = {{
  responsive: true,
  plugins: {{
    legend: {{ display: false }},
    tooltip: {{ callbacks: {{ label: ctx => ctx.parsed.y + '%' }} }}
  }},
  scales: {{
    x: {{ grid: {{ display: false }}, ticks: {{ font: {{ size: 10 }} }} }},
    y: {{ beginAtZero: true, ticks: {{ font: {{ size: 11 }} }},
          title: {{ display: true, text: '%', font: {{ size: 10 }} }} }}
  }}
}};
new Chart(document.getElementById('cw100'), {{ type:'bar', data:{cw100}, options:OPTS('Wakeup 100Hz') }});
new Chart(document.getElementById('cu100'), {{ type:'bar', data:{cu100}, options:OPTS('Usable 100Hz') }});
new Chart(document.getElementById('cc100'), {{ type:'bar', data:{cc100}, options:CPU_OPTS }});
new Chart(document.getElementById('cw1k'),  {{ type:'bar', data:{cw1k},  options:OPTS('Wakeup 1kHz') }});
new Chart(document.getElementById('cu1k'),  {{ type:'bar', data:{cu1k},  options:OPTS('Usable 1kHz') }});
new Chart(document.getElementById('cc1k'),  {{ type:'bar', data:{cc1k},  options:CPU_OPTS }});
</script>
"""

    with open(out_path, "w") as f:
        f.write(html)

    print(f"\nReport written to: {out_path}")
    return out_path


if __name__ == "__main__":
    main()
