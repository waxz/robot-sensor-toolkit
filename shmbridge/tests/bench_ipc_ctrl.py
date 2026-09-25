#!/usr/bin/env python3
"""
bench_ipc_ctrl.py  —  Controller-cycle IPC latency benchmark

Separates the IPC pipeline into three independently measurable stages:

  write_us          : publisher's write-call duration (shm seqlock or socket send)
  usable_latency_us : age of the data when the consumer reads it
                      = now_ns() - state.write_ns   (system-monotonic clock)
  read_us           : consumer's read-call duration (shm poll or socket recv)

Three communication modes:
  shm_poll          : controller polls shm at ctrl_hz; no notification path
  shm_uds_notify    : publisher writes shm then sends 1-byte UDS signal;
                      subscriber blocks on UDS, then reads shm
  uds_only          : publisher sends write_ns + payload over UDS; subscriber
                      blocks on recv (reference for OS-event-driven wakeup cost)

Three rate scenarios:
  1:1   sensor_hz = ctrl_hz = 100 Hz   (perfectly coupled)
  10:1  sensor_hz = 1000 Hz, ctrl_hz = 100 Hz  (controller reads latest; discards 9/10)
  1:3   sensor_hz = 30 Hz,  ctrl_hz = 100 Hz   (controller reads stale data 2x/3 cycles)

Metrics reported per scenario:
  write_us p50/p99        publisher write overhead
  read_us  p50/p99        consumer read overhead (how much cycle budget is consumed)
  usable_latency_us p50/p99/p999   true end-to-end data age
  sub_cpu_pct             subscriber CPU% (overhead to controller core)
  stale_rate              % reads where usable_latency > 2 x sensor period
"""

import contextlib
import multiprocessing as mp
import os
import resource
import socket
import struct
import time
from dataclasses import dataclass, field

# ── helpers ──────────────────────────────────────────────────────────────────

def _now_ns() -> int:
    return time.monotonic_ns()


def _pct(lst: list[float], p: float) -> float:
    if not lst:
        return float("nan")
    lst_sorted = sorted(lst)
    idx = max(0, int(len(lst_sorted) * p / 100) - 1)
    return lst_sorted[idx]


def _cpu_us() -> int:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return int((r.ru_utime + r.ru_stime) * 1e6)


_SHM_CTRL = "/bench_ctrl_shm"
_UDS_NOTIFY = "/tmp/bench_ctrl_notify.sock"
_UDS_ONLY   = "/tmp/bench_ctrl_uds.sock"
_WARMUP = 30
_MAX_VALID_US = 50_000  # 50 ms — anything larger is a measurement artifact


@dataclass
class RunResult:
    write_us:          list[float] = field(default_factory=list)
    read_us:           list[float] = field(default_factory=list)
    usable_lat_us:     list[float] = field(default_factory=list)
    sub_cpu_pct:       float = 0.0
    pub_cpu_pct:       float = 0.0
    n_stale:           int = 0
    n_total:           int = 0


# ── Mode B: shm_poll ─────────────────────────────────────────────────────────
# Publisher writes shm at sensor_hz; controller reads at ctrl_hz (no notify).

def _shm_poll_pub(sensor_hz, n_pub, pub_ready, done_ev, q):
    from shmbridge import _core
    pub = _core.ShmPublisher(_SHM_CTRL, 1, 1, 1)  # heartbeat_every=1 → write_ns always set
    pub.open()
    pub_ready.set()

    loop = _core.LoopSleeper(sensor_hz)
    state = _core.RobotState()
    write_times = []
    cpu0 = _cpu_us()
    t0 = _now_ns()

    for i in range(n_pub + _WARMUP):
        loop.start()
        state.step = i
        t_w0 = _now_ns()
        pub.write_state(0, state)
        t_w1 = _now_ns()
        if i >= _WARMUP:
            wt = (t_w1 - t_w0) / 1e3
            if 0 < wt < _MAX_VALID_US:
                write_times.append(wt)
        loop.sleep()

    done_ev.set()
    cpu1 = _cpu_us()
    wall = (_now_ns() - t0) / 1e3
    pub.close()
    q.put(("write_us", write_times, (cpu1 - cpu0) / wall * 100))


def _shm_poll_sub(ctrl_hz, n_ctrl, sensor_period_us, pub_ready, done_ev, q):
    from shmbridge import _core
    pub_ready.wait(15)
    sub = _core.ShmSubscriber(_SHM_CTRL, 1)
    sub.attach(10_000)

    loop = _core.LoopSleeper(ctrl_hz)
    read_times = []
    usable_lats = []
    n_stale = 0
    cpu0 = _cpu_us()
    t0 = _now_ns()
    last_step = -1

    for i in range(n_ctrl + _WARMUP):
        loop.start()
        t_r0 = _now_ns()
        state = sub.read_state_spin(0, 64)
        t_r1 = _now_ns()
        t_use = _now_ns()

        if state is not None and i >= _WARMUP:
            rt = (t_r1 - t_r0) / 1e3
            if 0 < rt < _MAX_VALID_US:
                read_times.append(rt)

            if state.write_ns > 0:
                lat = (t_use - state.write_ns) / 1e3
                if 0 < lat < _MAX_VALID_US:
                    usable_lats.append(lat)
                    if lat > 2 * sensor_period_us:
                        n_stale += 1
            last_step = state.step if state else last_step
        loop.sleep()

    cpu1 = _cpu_us()
    wall = (_now_ns() - t0) / 1e3
    sub_cpu = (cpu1 - cpu0) / wall * 100

    res = RunResult(read_us=read_times, usable_lat_us=usable_lats,
                    sub_cpu_pct=sub_cpu, n_stale=n_stale, n_total=len(usable_lats))
    q.put(("sub", res))


def run_shm_poll(sensor_hz: float, ctrl_hz: float, n: int) -> RunResult:
    n_pub = int(n * sensor_hz / ctrl_hz)
    sensor_period_us = 1e6 / sensor_hz
    pub_ready = mp.Event()
    done_ev   = mp.Event()
    q = mp.Queue()

    pp = mp.Process(target=_shm_poll_pub, args=(sensor_hz, n_pub, pub_ready, done_ev, q))
    sp = mp.Process(target=_shm_poll_sub, args=(ctrl_hz, n, sensor_period_us, pub_ready, done_ev, q))
    pp.start()
    sp.start()
    pp.join()
    sp.join()

    result = RunResult()
    while not q.empty():
        tag, *data = q.get()
        if tag == "write_us":
            result.write_us, result.pub_cpu_pct = data[0], data[1]
        else:
            sub_res: RunResult = data[0]
            result.read_us       = sub_res.read_us
            result.usable_lat_us = sub_res.usable_lat_us
            result.sub_cpu_pct   = sub_res.sub_cpu_pct
            result.n_stale       = sub_res.n_stale
            result.n_total       = sub_res.n_total
    return result


# ── Mode A: shm_uds_notify ───────────────────────────────────────────────────
# Publisher writes shm then sends 1-byte UDS datagram; subscriber blocks on recv.

def _notify_pub(sensor_hz, n_pub, pub_ready, sub_ready, done_ev, q):
    from shmbridge import _core
    pub = _core.ShmPublisher(_SHM_CTRL, 1, 1, 1)
    pub.open()
    pub_ready.set()
    sub_ready.wait(15)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sub_ready.wait(15)  # sub already set, second wait is instant

    loop = _core.LoopSleeper(sensor_hz)
    state = _core.RobotState()
    write_times = []
    cpu0 = _cpu_us()
    t0 = _now_ns()

    for i in range(n_pub + _WARMUP):
        loop.start()
        state.step = i
        t_w0 = _now_ns()
        pub.write_state(0, state)
        t_w1 = _now_ns()
        # Notification: 1-byte signal, not data
        with contextlib.suppress(OSError):
            sock.sendto(b'\x01', _UDS_NOTIFY)
        if i >= _WARMUP:
            wt = (t_w1 - t_w0) / 1e3
            if 0 < wt < _MAX_VALID_US:
                write_times.append(wt)
        loop.sleep()

    done_ev.set()
    cpu1 = _cpu_us()
    wall = (_now_ns() - t0) / 1e3
    sock.close()
    pub.close()
    q.put(("write_us", write_times, (cpu1 - cpu0) / wall * 100))


def _notify_sub(n, sensor_period_us, pub_ready, sub_ready, done_ev, q):
    from shmbridge import _core
    # Bind notify socket before signalling ready
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_NOTIFY)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(_UDS_NOTIFY)
    sock.settimeout(2.0)

    pub_ready.wait(15)
    sub = _core.ShmSubscriber(_SHM_CTRL, 1)
    sub.attach(10_000)
    sub_ready.set()

    read_times = []
    usable_lats = []
    n_stale = 0
    cpu0 = _cpu_us()
    t0 = _now_ns()
    received = 0

    while received < n + _WARMUP and not done_ev.is_set():
        try:
            sock.recv(1)
        except TimeoutError:
            continue
        t_r0 = _now_ns()
        state = sub.read_state_spin(0, 64)
        t_r1 = _now_ns()

        if state is None:
            continue
        received += 1
        if received <= _WARMUP:
            continue

        rt = (t_r1 - t_r0) / 1e3
        if 0 < rt < _MAX_VALID_US:
            read_times.append(rt)

        if state.write_ns > 0:
            lat = (t_r1 - state.write_ns) / 1e3
            if 0 < lat < _MAX_VALID_US:
                usable_lats.append(lat)
                if lat > 2 * sensor_period_us:
                    n_stale += 1

    cpu1 = _cpu_us()
    wall = (_now_ns() - t0) / 1e3
    sock.close()
    res = RunResult(read_us=read_times, usable_lat_us=usable_lats,
                    sub_cpu_pct=(cpu1 - cpu0) / wall * 100,
                    n_stale=n_stale, n_total=len(usable_lats))
    q.put(("sub", res))


def run_shm_notify(sensor_hz: float, ctrl_hz: float, n: int) -> RunResult:
    n_pub = int(n * sensor_hz / ctrl_hz) + n
    sensor_period_us = 1e6 / sensor_hz
    pub_ready = mp.Event()
    sub_ready = mp.Event()
    done_ev   = mp.Event()
    q = mp.Queue()

    pp = mp.Process(target=_notify_pub, args=(sensor_hz, n_pub, pub_ready, sub_ready, done_ev, q))
    sp = mp.Process(target=_notify_sub, args=(n, sensor_period_us, pub_ready, sub_ready, done_ev, q))
    pp.start()
    sp.start()
    pp.join()
    sp.join()

    result = RunResult()
    while not q.empty():
        tag, *data = q.get()
        if tag == "write_us":
            result.write_us, result.pub_cpu_pct = data[0], data[1]
        else:
            sub_res: RunResult = data[0]
            result.read_us       = sub_res.read_us
            result.usable_lat_us = sub_res.usable_lat_us
            result.sub_cpu_pct   = sub_res.sub_cpu_pct
            result.n_stale       = sub_res.n_stale
            result.n_total       = sub_res.n_total
    return result


# ── Reference: uds_only ──────────────────────────────────────────────────────
# Publisher sends write_ns (8 bytes) over UDS DGRAM; subscriber blocks on recv.

_UDS_PAYLOAD_FMT = "Q"   # 8-byte uint64 write_ns
_UDS_PAYLOAD_SIZE = struct.calcsize(_UDS_PAYLOAD_FMT)


def _uds_only_pub(sensor_hz, n_pub, sub_ready, done_ev, q):
    from shmbridge import _core
    sub_ready.wait(15)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    loop = _core.LoopSleeper(sensor_hz)
    write_times = []
    cpu0 = _cpu_us()
    t0 = _now_ns()

    for i in range(n_pub + _WARMUP):
        loop.start()
        t_w0 = _now_ns()
        payload = struct.pack(_UDS_PAYLOAD_FMT, _now_ns())
        with contextlib.suppress(OSError):
            sock.sendto(payload, _UDS_ONLY)
        t_w1 = _now_ns()
        if i >= _WARMUP:
            wt = (t_w1 - t_w0) / 1e3
            if 0 < wt < _MAX_VALID_US:
                write_times.append(wt)
        loop.sleep()

    done_ev.set()
    cpu1 = _cpu_us()
    wall = (_now_ns() - t0) / 1e3
    sock.close()
    q.put(("write_us", write_times, (cpu1 - cpu0) / wall * 100))


def _uds_only_sub(n, sensor_period_us, sub_ready, done_ev, q):
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_ONLY)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(_UDS_ONLY)
    sock.settimeout(2.0)
    sub_ready.set()

    read_times = []
    usable_lats = []
    n_stale = 0
    cpu0 = _cpu_us()
    t0 = _now_ns()
    received = 0

    while received < n + _WARMUP and not done_ev.is_set():
        try:
            data = sock.recv(_UDS_PAYLOAD_SIZE)
        except TimeoutError:
            continue
        t_r1 = _now_ns()
        if len(data) < _UDS_PAYLOAD_SIZE:
            continue

        received += 1
        if received <= _WARMUP:
            continue

        write_ns = struct.unpack(_UDS_PAYLOAD_FMT, data)[0]
        lat = (t_r1 - write_ns) / 1e3
        # read_us for UDS: the recv() call itself (estimated as ~transport cost)
        if 0 < lat < _MAX_VALID_US:
            usable_lats.append(lat)
            read_times.append(lat)   # for UDS, read and transport are inseparable
            if lat > 2 * sensor_period_us:
                n_stale += 1

    cpu1 = _cpu_us()
    wall = (_now_ns() - t0) / 1e3
    sock.close()
    res = RunResult(read_us=read_times, usable_lat_us=usable_lats,
                    sub_cpu_pct=(cpu1 - cpu0) / wall * 100,
                    n_stale=n_stale, n_total=len(usable_lats))
    q.put(("sub", res))


def run_uds_only(sensor_hz: float, ctrl_hz: float, n: int) -> RunResult:
    n_pub = int(n * sensor_hz / ctrl_hz) + n
    sensor_period_us = 1e6 / sensor_hz
    sub_ready = mp.Event()
    done_ev   = mp.Event()
    q = mp.Queue()

    pp = mp.Process(target=_uds_only_pub, args=(sensor_hz, n_pub, sub_ready, done_ev, q))
    sp = mp.Process(target=_uds_only_sub, args=(n, sensor_period_us, sub_ready, done_ev, q))
    pp.start()
    sp.start()
    pp.join()
    sp.join()

    result = RunResult()
    while not q.empty():
        tag, *data = q.get()
        if tag == "write_us":
            result.write_us, result.pub_cpu_pct = data[0], data[1]
        else:
            sub_res: RunResult = data[0]
            result.read_us       = sub_res.read_us
            result.usable_lat_us = sub_res.usable_lat_us
            result.sub_cpu_pct   = sub_res.sub_cpu_pct
            result.n_stale       = sub_res.n_stale
            result.n_total       = sub_res.n_total
    return result


# ── Scenarios ────────────────────────────────────────────────────────────────

SCENARIOS = [
    # label,                sensor_hz, ctrl_hz, description
    ("1:1  100/100 Hz",     100.0,     100.0,   "Sensor and controller tightly coupled at 100 Hz"),
    ("10:1 1000/100 Hz",   1000.0,     100.0,   "Sensor 10x faster; controller reads latest, discards 9/10"),
    ("1:3  30/100 Hz",       30.0,     100.0,   "Sensor slower; controller reads stale data ~2 of 3 cycles"),
]

MODES = [
    ("shm_poll",       run_shm_poll,    "shm read-latest (inline poll at ctrl rate)"),
    ("shm+notify",     run_shm_notify,  "shm data + 1-byte UDS notification"),
    ("uds_only",       run_uds_only,    "pure UDS DGRAM  (reference)"),
]

N = 300  # samples per run (after warmup)


def main():
    results: dict[tuple[str, str], RunResult] = {}

    for sc_label, sensor_hz, ctrl_hz, _desc in SCENARIOS:
        for mode_label, run_fn, _mdesc in MODES:
            key = (sc_label, mode_label)
            print(f"  {mode_label:18s}  {sc_label} … ", end="", flush=True)
            try:
                res = run_fn(sensor_hz, ctrl_hz, N)
                results[key] = res
                print(f"lat p50={_pct(res.usable_lat_us,50):.1f}µs "
                      f"p99={_pct(res.usable_lat_us,99):.1f}µs "
                      f"cpu={res.sub_cpu_pct:.1f}%")
            except Exception as exc:
                print(f"FAILED: {exc}")
                results[key] = RunResult()

    return results


# ── HTML report ──────────────────────────────────────────────────────────────

def _bar(val, max_val, color, width=160):
    w = max(2, int(val / max_val * width)) if max_val > 0 else 0
    return f'<div style="width:{w}px;height:10px;background:{color};border-radius:2px;display:inline-block"></div>'


def build_report(results: dict[tuple[str, str], RunResult]) -> str:
    [s[0] for s in SCENARIOS]
    mode_labels = [m[0] for m in MODES]

    # Colour scheme
    COLORS = {
        "shm_poll":   "#4CAF82",
        "shm+notify": "#5B9CF6",
        "uds_only":   "#F5A623",
    }

    def row_cells(sc, mode):
        res = results.get((sc, mode), RunResult())
        lp50  = _pct(res.usable_lat_us, 50)
        lp99  = _pct(res.usable_lat_us, 99)
        lp999 = _pct(res.usable_lat_us, 99.9)
        wp50  = _pct(res.write_us, 50)
        wp99  = _pct(res.write_us, 99)
        rp50  = _pct(res.read_us, 50)
        rp99  = _pct(res.read_us, 99)
        cpu   = res.sub_cpu_pct
        stale = (res.n_stale / res.n_total * 100) if res.n_total else 0
        return lp50, lp99, lp999, wp50, wp99, rp50, rp99, cpu, stale

    # Build table HTML for each scenario
    tables = ""
    for sc_label, sensor_hz, ctrl_hz, sc_desc in SCENARIOS:
        sensor_period_us = 1e6 / sensor_hz
        ctrl_period_us   = 1e6 / ctrl_hz

        max_lat = max(
            (_pct(results.get((sc_label, m), RunResult()).usable_lat_us, 99) or 0)
            for m in mode_labels
        )

        rows = ""
        for mode_label, _, _mode_desc in MODES:
            lp50, lp99, lp999, wp50, wp99, rp50, rp99, cpu, stale = row_cells(sc_label, mode_label)
            color = COLORS.get(mode_label, "#888")
            bar_html = _bar(lp99, max(max_lat, 1), color)
            stale_cls = ' class="stale"' if stale > 5 else ""
            rows += f"""
            <tr>
              <td><span class="badge" style="background:{color}22;color:{color}">{mode_label}</span></td>
              <td class="num">{wp50:.1f}</td>
              <td class="num">{wp99:.1f}</td>
              <td class="num">{rp50:.1f}</td>
              <td class="num">{rp99:.1f}</td>
              <td class="num lat-cell">{lp50:.1f}</td>
              <td class="num lat-cell">{lp99:.1f}</td>
              <td class="num lat-cell">{lp999:.1f}</td>
              <td class="num">{cpu:.1f}%</td>
              <td{stale_cls} class="num">{stale:.1f}%</td>
              <td>{bar_html}</td>
            </tr>"""

        tables += f"""
        <section class="scenario">
          <div class="scenario-header">
            <h2>{sc_label}</h2>
            <span class="scenario-desc">{sc_desc}</span>
            <div class="periods">
              <span>sensor period = <strong>{sensor_period_us:.0f} µs</strong></span>
              <span>ctrl period = <strong>{ctrl_period_us:.0f} µs</strong></span>
            </div>
          </div>
          <div class="table-wrap">
          <table>
            <thead>
              <tr>
                <th>Mode</th>
                <th colspan="2">write_us (µs)</th>
                <th colspan="2">read_us (µs)</th>
                <th colspan="3">usable_latency_us (µs)</th>
                <th>sub CPU%</th>
                <th>stale rate</th>
                <th>latency p99</th>
              </tr>
              <tr class="subhead">
                <th></th>
                <th>p50</th><th>p99</th>
                <th>p50</th><th>p99</th>
                <th>p50</th><th>p99</th><th>p999</th>
                <th></th><th></th><th></th>
              </tr>
            </thead>
            <tbody>{rows}</tbody>
          </table>
          </div>
        </section>"""

    # Build insight cards
    # Find best mode per scenario by usable_latency p50
    insights = []
    for sc_label, _sensor_hz, _ctrl_hz, _sc_desc in SCENARIOS:
        best_mode = min(
            mode_labels,
            key=lambda m: _pct(results.get((sc_label, m), RunResult()).usable_lat_us, 50) or 1e9
        )
        best_res = results.get((sc_label, best_mode), RunResult())
        worst_mode = max(
            mode_labels,
            key=lambda m: _pct(results.get((sc_label, m), RunResult()).usable_lat_us, 50) or 0
        )
        worst_res = results.get((sc_label, worst_mode), RunResult())
        insights.append((sc_label, best_mode, best_res, worst_mode, worst_res))

    insight_html = ""
    for sc_label, best_m, best_r, worst_m, worst_r in insights:
        bl = _pct(best_r.usable_lat_us, 50)
        wl = _pct(worst_r.usable_lat_us, 50)
        bc = best_r.sub_cpu_pct
        insight_html += f"""
        <div class="insight">
          <div class="insight-label">{sc_label}</div>
          <div class="insight-body">
            Best usable latency: <strong>{best_m}</strong> at
            <strong>{bl:.0f} µs p50</strong>, {bc:.0f}% CPU.
            Worst: {worst_m} at {wl:.0f} µs ({wl/bl:.1f}x).
          </div>
        </div>"""

    # Summary stat tiles
    # Overall best across all scenarios for shm_poll
    all_lats = []
    for sc_label, *_ in SCENARIOS:
        res = results.get((sc_label, "shm_poll"), RunResult())
        if res.usable_lat_us:
            all_lats.extend(res.usable_lat_us)
    overall_p50 = _pct(all_lats, 50) if all_lats else 0

    notify_lats = []
    for sc_label, *_ in SCENARIOS:
        res = results.get((sc_label, "shm+notify"), RunResult())
        if res.usable_lat_us:
            notify_lats.extend(res.usable_lat_us)
    notify_p50 = _pct(notify_lats, 50) if notify_lats else 0

    uds_lats = []
    for sc_label, *_ in SCENARIOS:
        res = results.get((sc_label, "uds_only"), RunResult())
        if res.usable_lat_us:
            uds_lats.extend(res.usable_lat_us)
    uds_p50 = _pct(uds_lats, 50) if uds_lats else 0

    poll_cpu = max(results.get((s[0], "shm_poll"), RunResult()).sub_cpu_pct for s in SCENARIOS)
    notify_cpu = max(results.get((s[0], "shm+notify"), RunResult()).sub_cpu_pct for s in SCENARIOS)
    uds_cpu = max(results.get((s[0], "uds_only"), RunResult()).sub_cpu_pct for s in SCENARIOS)

    tiles = f"""
    <div class="tiles">
      <div class="tile">
        <div class="tile-label">shm_poll p50</div>
        <div class="tile-val">{overall_p50:.0f} <span class="unit">µs</span></div>
        <div class="tile-sub">usable latency</div>
      </div>
      <div class="tile">
        <div class="tile-label">shm+notify p50</div>
        <div class="tile-val">{notify_p50:.0f} <span class="unit">µs</span></div>
        <div class="tile-sub">usable latency</div>
      </div>
      <div class="tile">
        <div class="tile-label">uds_only p50</div>
        <div class="tile-val">{uds_p50:.0f} <span class="unit">µs</span></div>
        <div class="tile-sub">usable latency</div>
      </div>
      <div class="tile">
        <div class="tile-label">shm_poll CPU</div>
        <div class="tile-val">{poll_cpu:.0f} <span class="unit">%</span></div>
        <div class="tile-sub">peak sub process</div>
      </div>
      <div class="tile">
        <div class="tile-label">shm+notify CPU</div>
        <div class="tile-val">{notify_cpu:.0f} <span class="unit">%</span></div>
        <div class="tile-sub">peak sub process</div>
      </div>
      <div class="tile">
        <div class="tile-label">uds_only CPU</div>
        <div class="tile-val">{uds_cpu:.0f} <span class="unit">%</span></div>
        <div class="tile-sub">peak sub process</div>
      </div>
    </div>"""

    return f"""<title>IPC Controller Benchmark</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
:root {{
  --bg:      #0b0f14;
  --bg2:     #131920;
  --bg3:     #1a2230;
  --border:  #253040;
  --text:    #d4dce8;
  --muted:   #6b7d90;
  --accent1: #4CAF82;
  --accent2: #5B9CF6;
  --accent3: #F5A623;
  --stale:   #EF5350;
  --head:    #e8f0fe;
}}
@media (prefers-color-scheme: light) {{
  :root:not([data-theme="dark"]) {{
    --bg:     #f4f6f9;
    --bg2:    #ffffff;
    --bg3:    #edf0f5;
    --border: #d0d8e4;
    --text:   #1a2233;
    --muted:  #5a6a80;
    --head:   #0d1a2e;
  }}
}}
:root[data-theme="light"] {{
  --bg:     #f4f6f9;
  --bg2:    #ffffff;
  --bg3:    #edf0f5;
  --border: #d0d8e4;
  --text:   #1a2233;
  --muted:  #5a6a80;
  --head:   #0d1a2e;
}}
*, *::before, *::after {{ box-sizing: border-box; }}
body {{
  background: var(--bg);
  color: var(--text);
  font-family: 'IBM Plex Sans', system-ui, sans-serif;
  font-size: 14px;
  line-height: 1.6;
  margin: 0;
  padding: 0;
}}
header {{
  background: var(--bg2);
  border-bottom: 1px solid var(--border);
  padding: 28px 32px 20px;
}}
header h1 {{
  font-family: 'IBM Plex Mono', monospace;
  font-size: 20px;
  font-weight: 600;
  color: var(--head);
  margin: 0 0 4px;
  letter-spacing: -0.3px;
}}
header p {{
  color: var(--muted);
  margin: 0;
  font-size: 13px;
}}
.pipeline {{
  background: var(--bg3);
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 14px 20px;
  margin: 20px 32px;
  font-family: 'IBM Plex Mono', monospace;
  font-size: 12px;
  color: var(--muted);
  display: flex;
  align-items: center;
  gap: 8px;
  flex-wrap: wrap;
}}
.pipeline .stage {{ color: var(--text); font-weight: 600; }}
.pipeline .arrow {{ color: var(--border); }}
.tiles {{
  display: grid;
  grid-template-columns: repeat(6, 1fr);
  gap: 12px;
  padding: 0 32px 24px;
}}
.tile {{
  background: var(--bg2);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 16px;
}}
.tile-label {{ font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: .05em; margin-bottom: 4px; }}
.tile-val {{ font-family: 'IBM Plex Mono', monospace; font-size: 28px; font-weight: 600; color: var(--head); line-height: 1.1; }}
.tile-val .unit {{ font-size: 14px; color: var(--muted); font-weight: 400; }}
.tile-sub {{ font-size: 11px; color: var(--muted); margin-top: 2px; }}
main {{ padding: 0 32px 32px; }}
.scenario {{
  background: var(--bg2);
  border: 1px solid var(--border);
  border-radius: 8px;
  margin-bottom: 20px;
  overflow: hidden;
}}
.scenario-header {{
  padding: 14px 20px;
  background: var(--bg3);
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: baseline;
  gap: 16px;
  flex-wrap: wrap;
}}
.scenario-header h2 {{
  font-family: 'IBM Plex Mono', monospace;
  font-size: 14px;
  font-weight: 600;
  color: var(--head);
  margin: 0;
}}
.scenario-desc {{ color: var(--muted); font-size: 12px; flex: 1; }}
.periods {{ display: flex; gap: 16px; font-size: 12px; color: var(--muted); }}
.periods strong {{ color: var(--text); }}
.table-wrap {{ overflow-x: auto; }}
table {{
  width: 100%;
  border-collapse: collapse;
  font-size: 13px;
}}
th, td {{ padding: 8px 12px; text-align: left; border-bottom: 1px solid var(--border); }}
th {{ color: var(--muted); font-weight: 500; font-size: 11px; text-transform: uppercase; letter-spacing: .04em; background: var(--bg3); }}
th.subhead {{ background: var(--bg2); padding-top: 4px; padding-bottom: 4px; }}
td.num {{ font-family: 'IBM Plex Mono', monospace; font-variant-numeric: tabular-nums; text-align: right; }}
td.lat-cell {{ color: var(--accent2); }}
td.stale {{ color: var(--stale); }}
tbody tr:hover {{ background: var(--bg3); }}
.badge {{
  display: inline-block;
  padding: 2px 8px;
  border-radius: 4px;
  font-family: 'IBM Plex Mono', monospace;
  font-size: 11px;
  font-weight: 600;
}}
.insights {{
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 12px;
  margin-bottom: 24px;
}}
.insight {{
  background: var(--bg2);
  border: 1px solid var(--border);
  border-left: 3px solid var(--accent2);
  border-radius: 6px;
  padding: 14px 16px;
}}
.insight-label {{ font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: .04em; margin-bottom: 6px; }}
.insight-body {{ font-size: 13px; line-height: 1.5; }}
.insight-body strong {{ color: var(--accent2); }}
.explainer {{
  background: var(--bg2);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 20px 24px;
  margin-bottom: 20px;
}}
.explainer h3 {{ margin: 0 0 12px; font-size: 14px; color: var(--head); }}
.def-grid {{ display: grid; grid-template-columns: auto 1fr; gap: 6px 20px; }}
.def-term {{ font-family: 'IBM Plex Mono', monospace; font-size: 12px; color: var(--accent1); white-space: nowrap; }}
.def-body {{ font-size: 13px; color: var(--muted); }}
</style>

<header>
  <h1>IPC Controller-Cycle Benchmark</h1>
  <p>write_us · usable_latency_us · read_us decomposed per mode and rate scenario  ·  n={N} samples</p>
</header>

<div class="pipeline">
  <span class="stage">publish</span>
  <span class="arrow">→</span>
  <span>write_us <em>(seqlock or send())</em></span>
  <span class="arrow">→</span>
  <span class="stage">shm / socket</span>
  <span class="arrow">→</span>
  <span>wire time</span>
  <span class="arrow">→</span>
  <span class="stage">consumer wakes</span>
  <span class="arrow">→</span>
  <span>read_us <em>(read_state_spin or recv())</em></span>
  <span class="arrow">→</span>
  <span class="stage">usable_latency = now - write_ns</span>
</div>

{tiles}

<main>
  <div class="explainer">
    <h3>Metric definitions</h3>
    <div class="def-grid">
      <span class="def-term">write_us</span>
      <span class="def-body">Duration of the publisher's write call (seqlock write or socket send). Directly consumed from the publisher's cycle budget.</span>
      <span class="def-term">read_us</span>
      <span class="def-body">Duration of the consumer's read call after being woken (shm read_state_spin or implicit in socket recv). Consumed from the controller's cycle budget.</span>
      <span class="def-term">usable_latency_us</span>
      <span class="def-body">Age of the data at the moment the consumer reads it: <code>now_ns() - state.write_ns</code>. The write_ns timestamp is captured inside the seqlock window. This is the end-to-end figure that determines control quality.</span>
      <span class="def-term">stale_rate</span>
      <span class="def-body">Fraction of reads where usable_latency &gt; 2x sensor_period. Indicates the consumer fell more than one publish interval behind.</span>
    </div>
  </div>

  <div class="insights">{insight_html}</div>

  {tables}
</main>"""


if __name__ == "__main__":
    mp.set_start_method("forkserver", force=True)
    print("IPC Controller-Cycle Benchmark")
    print(f"  {N} samples per run (after {_WARMUP} warmup iterations)")
    print()
    res = main()
    html = build_report(res)
    out = "/tmp/ipc_ctrl_report.html"
    with open(out, "w") as f:
        f.write(html)
    print(f"\nReport written to {out}")
