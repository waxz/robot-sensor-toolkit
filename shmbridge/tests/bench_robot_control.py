#!/usr/bin/env python3
"""
bench_robot_control.py — Real-time robot control-loop benchmark for shmbridge.

Simulates the actual usage pattern of a robot control stack built on
``shmbridge.bridge_ext.ExtShmBridge``:

  * a "sim" process writes RobotState at a fixed control-loop rate
  * a "controller" process reads state, runs a trivial control law, and
    writes back a RobotCmd
  * the sim process closes the loop by reading that RobotCmd

This is the same publisher/subscriber pattern used by
``urdf_tools.pubsub.SensorPublisher/SensorSubscriber`` and by
``irsim_devices`` sensor/actuator bridges, run here as two real OS
processes (via ``multiprocessing``) so IPC and scheduling costs are
representative, not just an in-process microbenchmark.

Metrics collected per configuration (target rate x backend):

  * one-way latency (sim write -> controller read), ns
  * round-trip latency (sim write -> controller read+write -> sim read), ns
  * control-loop period jitter (achieved vs. target loop period)
  * CPU usage of both processes (via psutil, wall-clock-normalized)
  * achieved write rate vs. target rate (missed-deadline count)

Exercises ``shmbridge.bridge_ext.ExtShmBridge``, a thin wrapper over the
compiled ``_core`` C++ extension -- the only implementation shmbridge
ships; a ``backend`` label is still threaded through results/CLI for
historical report compatibility (older runs of this script also measured
a pure-Python ctypes ``_PyExtShmBridge``, since removed in favor of making
the C++ extension mandatory).

Usage::

    python bench_robot_control.py --rates 50,100,250,500,1000 --duration 3
    python bench_robot_control.py --backend cpp --rates 500 --json out.json
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import statistics
import sys
import time
from dataclasses import asdict, dataclass

try:
    import psutil

    _HAS_PSUTIL = True
except ImportError:
    _HAS_PSUTIL = False

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

SHM_NAME = "/shmbridge_bench_ctrl_v1"


def _make_bridge(backend: str, shm_name: str):
    """Return an ExtShmBridge instance for the requested backend."""
    if backend != "cpp":
        raise ValueError(backend)
    from shmbridge.bridge_ext import ExtShmBridge as _Bridge

    return _Bridge(shm_name)


# ── worker processes ─────────────────────────────────────────────────────────


def _sim_process(
    backend: str,
    shm_name: str,
    rate_hz: float,
    duration_s: float,
    warmup_s: float,
    ready_evt,
    stop_evt,
    result_q: mp.Queue,
) -> None:
    bridge = _make_bridge(backend, shm_name)
    bridge.open()
    ready_evt.set()

    period_ns = int(1e9 / rate_hz)
    proc = psutil.Process(os.getpid()) if _HAS_PSUTIL else None
    if proc is not None:
        proc.cpu_percent(None)  # prime the internal counter (first call is 0)

    round_trip_ns: list[int] = []
    period_actual_ns: list[int] = []
    missed_deadlines = 0
    n_written = 0
    n_cmd_hits = 0

    # perf_counter_ns() reads the same OS-wide monotonic clock in every
    # process, so timestamps taken in the sim process and compared against
    # readings taken in the controller process are directly comparable --
    # no clock sync or offset exchange needed, just use the raw value.
    start = time.perf_counter_ns()
    next_tick = start
    end_at = start + int((warmup_s + duration_s) * 1e9)
    warmup_until = start + int(warmup_s * 1e9)

    last_tick_time = start

    while True:
        now = time.perf_counter_ns()
        if now >= end_at:
            break
        if now < next_tick:
            remaining = (next_tick - now) / 1e9
            if remaining > 0.0005:
                time.sleep(remaining - 0.0003)
            while time.perf_counter_ns() < next_tick:
                pass
            now = time.perf_counter_ns()

        t0 = now  # raw absolute monotonic ns, exact (uint64 wire field)
        if now >= warmup_until:
            period_actual_ns.append(now - last_tick_time)
            if now - next_tick > period_ns // 2:
                missed_deadlines += 1
        last_tick_time = now

        bridge.write_state(
            x=float(n_written),
            y=0.0,
            heading=0.0,
            vx=0.0,
            vy=0.0,
            omega=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_dist=0.0,
            step=t0,
            sim_time=t0 / 1e9,
        )
        n_written += 1

        # Poll for the controller's echo of this exact tick. `cmd.angular` is
        # a 32-bit float (16-byte wire struct), which cannot hold a raw ns
        # offset exactly once it grows past ~2^24 -- so the echo is
        # quantized to microseconds (exact in float32 up to ~16.7s, well
        # past this benchmark's per-rate duration) purely as a matching key.
        # The actual round-trip latency below is computed from `t0`, the
        # unquantized value this process already holds. The key wraps every
        # 8s (still exact in float32) -- safe as long as duration+warmup
        # stays under that per rate, true for this benchmark's defaults.
        t0_us_key = float((t0 // 1000) % 8_000_000)
        deadline = now + period_ns * 4
        while True:
            cmd = bridge.read_cmd()
            if cmd is not None and abs(cmd.angular - t0_us_key) < 0.5:
                if now >= warmup_until - period_ns:
                    round_trip_ns.append(time.perf_counter_ns() - t0)
                    n_cmd_hits += 1
                break
            if time.perf_counter_ns() > deadline:
                break

        next_tick += period_ns

    if proc is not None:
        cpu_pct = proc.cpu_percent(None)
    else:
        cpu_pct = None

    bridge.close()
    result_q.put(
        {
            "round_trip_ns": round_trip_ns,
            "period_actual_ns": period_actual_ns,
            "missed_deadlines": missed_deadlines,
            "n_written": n_written,
            "n_cmd_hits": n_cmd_hits,
            "cpu_pct_sim": cpu_pct,
        }
    )


def _controller_process(
    backend: str,
    shm_name: str,
    duration_s: float,
    warmup_s: float,
    poll_sleep_us: float,
    ready_evt,
    stop_evt,
    result_q: mp.Queue,
) -> None:
    ready_evt.wait(timeout=10.0)
    bridge = _make_bridge(backend, shm_name)
    bridge.attach(timeout_ms=5000.0)

    proc = psutil.Process(os.getpid()) if _HAS_PSUTIL else None
    if proc is not None:
        proc.cpu_percent(None)

    one_way_ns: list[int] = []
    start = time.perf_counter_ns()
    warmup_until = start + int(warmup_s * 1e9)
    end_at = start + int((warmup_s + duration_s + 0.5) * 1e9)  # small drain tail

    last_step_seen = -1
    n_read = 0
    poll_sleep_s = poll_sleep_us / 1e6

    while time.perf_counter_ns() < end_at and not stop_evt.is_set():
        state = bridge.read_state()
        if state is None or state.step == last_step_seen:
            if poll_sleep_s > 0:
                time.sleep(poll_sleep_s)
            continue
        last_step_seen = state.step
        now = time.perf_counter_ns()
        t0 = state.step  # raw perf_counter_ns() value from the sim process

        # Trivial control law: bang-bang toward goal_dist (unused here, just
        # representative per-tick work), then echo t0 in `angular` so the
        # sim can compute round-trip latency.
        linear = 0.5 if state.goal_dist > 0.1 else 0.0
        bridge.write_cmd(linear=linear, angular=float((t0 // 1000) % 8_000_000))
        n_read += 1

        if now >= warmup_until:
            one_way_ns.append(now - t0)

    if proc is not None:
        cpu_pct = proc.cpu_percent(None)
    else:
        cpu_pct = None

    bridge.detach() if hasattr(bridge, "detach") else bridge.close()
    result_q.put({"one_way_ns": one_way_ns, "n_read": n_read, "cpu_pct_ctrl": cpu_pct})


# ── stats ────────────────────────────────────────────────────────────────────


@dataclass
class RunResult:
    backend: str
    rate_hz: float
    poll_sleep_us: float
    n_written: int
    n_read: int
    n_cmd_hits: int
    missed_deadlines: int
    one_way_ns_mean: float
    one_way_ns_p50: float
    one_way_ns_p99: float
    one_way_ns_max: float
    round_trip_ns_mean: float
    round_trip_ns_p50: float
    round_trip_ns_p99: float
    round_trip_ns_max: float
    period_jitter_ns_stdev: float
    period_jitter_ns_max: float
    cpu_pct_sim: float | None
    cpu_pct_ctrl: float | None


def _pctl(data: list[float], q: float) -> float:
    if not data:
        return float("nan")
    s = sorted(data)
    idx = min(len(s) - 1, int(round(q * (len(s) - 1))))
    return s[idx]


def run_one(
    backend: str,
    rate_hz: float,
    duration_s: float,
    warmup_s: float,
    poll_sleep_us: float = 0.0,
) -> RunResult:
    ctx = mp.get_context("spawn")
    ready_evt = ctx.Event()
    stop_evt = ctx.Event()
    result_q: mp.Queue = ctx.Queue()

    shm_name = f"{SHM_NAME}_{int(rate_hz)}_{backend}"

    p_sim = ctx.Process(
        target=_sim_process,
        args=(backend, shm_name, rate_hz, duration_s, warmup_s, ready_evt, stop_evt, result_q),
    )
    p_ctrl = ctx.Process(
        target=_controller_process,
        args=(backend, shm_name, duration_s, warmup_s, poll_sleep_us, ready_evt, stop_evt, result_q),
    )
    p_ctrl.start()
    p_sim.start()

    p_sim.join(timeout=duration_s + warmup_s + 15.0)
    stop_evt.set()
    p_ctrl.join(timeout=5.0)

    results: dict = {}
    while not result_q.empty():
        results.update(result_q.get())

    for p in (p_sim, p_ctrl):
        if p.is_alive():
            p.terminate()
        p.join(timeout=2.0)

    one_way = results.get("one_way_ns", [])
    round_trip = results.get("round_trip_ns", [])
    period = results.get("period_actual_ns", [])
    jitter = [abs(x - statistics.median(period)) for x in period] if period else [0.0]

    return RunResult(
        backend=backend,
        rate_hz=rate_hz,
        poll_sleep_us=poll_sleep_us,
        n_written=results.get("n_written", 0),
        n_read=results.get("n_read", 0),
        n_cmd_hits=results.get("n_cmd_hits", 0),
        missed_deadlines=results.get("missed_deadlines", 0),
        one_way_ns_mean=statistics.fmean(one_way) if one_way else float("nan"),
        one_way_ns_p50=_pctl(one_way, 0.50),
        one_way_ns_p99=_pctl(one_way, 0.99),
        one_way_ns_max=max(one_way) if one_way else float("nan"),
        round_trip_ns_mean=statistics.fmean(round_trip) if round_trip else float("nan"),
        round_trip_ns_p50=_pctl(round_trip, 0.50),
        round_trip_ns_p99=_pctl(round_trip, 0.99),
        round_trip_ns_max=max(round_trip) if round_trip else float("nan"),
        period_jitter_ns_stdev=statistics.pstdev(jitter) if len(jitter) > 1 else 0.0,
        period_jitter_ns_max=max(jitter) if jitter else 0.0,
        cpu_pct_sim=results.get("cpu_pct_sim"),
        cpu_pct_ctrl=results.get("cpu_pct_ctrl"),
    )


def _fmt_us(ns: float) -> str:
    return f"{ns / 1e3:8.2f}" if ns == ns else "     n/a"


def print_table(results: list[RunResult]) -> None:
    hdr = (
        f"{'backend':7} {'rate':>6} {'poll':>6} {'1-way p50':>10} {'1-way p99':>10} "
        f"{'RT p50':>9} {'RT p99':>9} {'jitter sd':>10} {'missed':>7} "
        f"{'cpu sim':>8} {'cpu ctrl':>9}"
    )
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        print(
            f"{r.backend:7} {r.rate_hz:6.0f} {r.poll_sleep_us:6.0f} "
            f"{_fmt_us(r.one_way_ns_p50):>10} {_fmt_us(r.one_way_ns_p99):>10} "
            f"{_fmt_us(r.round_trip_ns_p50):>9} {_fmt_us(r.round_trip_ns_p99):>9} "
            f"{_fmt_us(r.period_jitter_ns_stdev):>10} {r.missed_deadlines:7d} "
            f"{r.cpu_pct_sim or 0:8.1f} {r.cpu_pct_ctrl or 0:9.1f}"
        )
    print("(all times in microseconds except poll (us) and cpu (% of one core))")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rates", default="50,100,250,500,1000", help="comma-separated target Hz")
    ap.add_argument("--duration", type=float, default=3.0, help="measured seconds per run")
    ap.add_argument("--warmup", type=float, default=0.5, help="warmup seconds per run (discarded)")
    ap.add_argument(
        "--backend",
        choices=["cpp"],
        default="cpp",
        help="which ExtShmBridge implementation to benchmark",
    )
    ap.add_argument(
        "--poll-sleep-us",
        default="0",
        help="comma-separated controller poll sleep(s) in microseconds "
        "(0 = busy-spin, lowest latency/highest CPU; >0 trades latency for CPU)",
    )
    ap.add_argument("--json", default=None, help="write raw results to this JSON file")
    args = ap.parse_args()

    if not _HAS_PSUTIL:
        print("warning: psutil not installed, CPU columns will be empty (pip install psutil)")

    rates = [float(r) for r in args.rates.split(",")]
    poll_sleeps = [float(p) for p in args.poll_sleep_us.split(",")]
    backends = [args.backend]

    all_results: list[RunResult] = []
    for backend in backends:
        for rate in rates:
            for poll_us in poll_sleeps:
                print(
                    f"running backend={backend} rate={rate:.0f}Hz "
                    f"poll_sleep={poll_us:.0f}us duration={args.duration}s ...",
                    file=sys.stderr,
                )
                try:
                    r = run_one(backend, rate, args.duration, args.warmup, poll_us)
                except RuntimeError as exc:
                    print(f"  skipped: {exc}", file=sys.stderr)
                    continue
                all_results.append(r)

    print()
    print_table(all_results)

    if args.json:
        with open(args.json, "w") as f:
            json.dump([asdict(r) for r in all_results], f, indent=2)
        print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
