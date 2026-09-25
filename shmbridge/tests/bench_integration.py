#!/usr/bin/env python3
"""
bench_integration.py — Evaluate TaskScheduler / spin_sleep impact on shmbridge.

Measures three specific integration points identified in bridge.py / python_writer.py:

  A. Publisher timing jitter:     time.sleep(0.01) vs LoopSleeper(100)
  B. read_cmd_blocking CPU burn:  tight busy-poll vs spin_sleep hybrid
  C. Cross-proc round-trip:       spin sub vs TaskScheduler @various rates

Outputs JSON via --json.  Human-readable table always printed.
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import resource
import statistics
import sys
import time
from typing import Any

# Bootstrap
try:
    from shmbridge._core import (
        LoopSleeper,
        ShmPublisher,
        ShmSubscriber,
        TaskScheduler,
        now_ns_mono,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../src"))
    from shmbridge._core import (  # type: ignore[no-redef]
        LoopSleeper,
        ShmPublisher,
        ShmSubscriber,
        TaskScheduler,
        now_ns_mono,
    )


# ── helpers ──────────────────────────────────────────────────────────────────

def _pin(cpu: int) -> None:
    try:
        if cpu in os.sched_getaffinity(0):
            os.sched_setaffinity(0, {cpu})
    except (AttributeError, OSError):
        pass


def _pct(data, p):
    s = sorted(data)
    return s[max(0, min(len(s) - 1, int(len(s) * p / 100)))]


def _stats(data):
    return {
        "min":  min(data),
        "p50":  _pct(data, 50),
        "p95":  _pct(data, 95),
        "p99":  _pct(data, 99),
        "max":  max(data),
        "mean": statistics.mean(data),
        "n":    len(data),
    }


def _fmt(v, d=1): return f"{v:.{d}f}" if v is not None else "—"


_SHM_A = "/bench_integ_a"
_SHM_B = "/bench_integ_b"


# ══════════════════════════════════════════════════════════════════════════════
# A. PUBLISHER TIMING JITTER
#    Compare time.sleep(dt) vs LoopSleeper for the 100 Hz sim loop pattern.
# ══════════════════════════════════════════════════════════════════════════════

def bench_publisher_jitter(hz=100, n=400, warmup=20):
    """Measure inter-write interval jitter for each timing strategy."""
    target_us = 1e6 / hz
    results = {}

    # — A1: time.sleep --------------------------------------------------------
    pub = ShmPublisher(_SHM_A, 1, 1, 0)
    pub.open()
    try:
        from shmbridge._core import RobotState
        s = RobotState()
        # warmup
        for i in range(warmup):
            s.step = i
            pub.write_state(0, s)
            time.sleep(1.0 / hz)

        intervals = []
        t_last = now_ns_mono()
        for i in range(n):
            s.step = i
            pub.write_state(0, s)
            t = now_ns_mono()
            intervals.append((t - t_last) / 1e3)
            t_last = t
            time.sleep(1.0 / hz)

        errs = [iv - target_us for iv in intervals[1:]]
        results["time_sleep"] = _stats(errs)
        results["time_sleep"]["target_us"] = target_us
    finally:
        pub.close()

    # — A2: LoopSleeper -------------------------------------------------------
    pub = ShmPublisher(_SHM_A, 1, 1, 0)
    pub.open()
    try:
        sleeper = LoopSleeper(float(hz))
        # warmup
        for i in range(warmup):
            sleeper.start()
            s.step = i
            pub.write_state(0, s)
            sleeper.sleep()

        intervals = []
        t_last = now_ns_mono()
        for i in range(n):
            sleeper.start()
            s.step = i
            pub.write_state(0, s)
            t = now_ns_mono()
            intervals.append((t - t_last) / 1e3)
            t_last = t
            sleeper.sleep()

        errs = [iv - target_us for iv in intervals[1:]]
        results["loop_sleeper"] = _stats(errs)
        results["loop_sleeper"]["target_us"] = target_us
    finally:
        pub.close()

    return results


# ══════════════════════════════════════════════════════════════════════════════
# B. read_cmd_blocking CPU EFFICIENCY
#
# Four variants compared over a 10 ms wall-clock budget (no cmd ever arrives):
#
#   Py-tight    Python tight while loop  — baseline (100% CPU)
#   Py-sleep    Python + time.sleep(500µs) — OS yield via Python
#   C++-busy    C++ read_cmd_blocking(poll_sleep_ns=0) — pure-spin in C++
#   C++-nanosleep C++ read_cmd_blocking(poll_sleep_ns=500_000) — nanosleep in C++
#
# The key insight: spin_sleep_us < 1 ms stays in pure-spin (Welford estimate =
# 1 ms → no nanosleep issued), so it never reduces CPU.  Real CPU reduction
# requires an OS-level nanosleep — either via Python time.sleep or directly
# from C++ with nanosleep(). The C++ path has lower per-poll overhead and
# releases the GIL for the whole call duration.
# ══════════════════════════════════════════════════════════════════════════════

def _cpu_us() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return (r.ru_utime + r.ru_stime) * 1e6


def bench_blocking_read_cpu(wall_us=10_000, n=200):
    """
    Create a publisher with NO cmd posted.  Measure CPU consumed while waiting
    for a cmd that never arrives, across four polling strategies.
    """
    pub = ShmPublisher(_SHM_A, 1, 1, 0)
    pub.open()

    results = {}

    # — B1: Python tight poll (baseline) ---------------------------------------
    cpu_costs = []
    for _ in range(n):
        deadline = time.monotonic() + wall_us * 1e-6
        c0 = _cpu_us()
        while time.monotonic() < deadline:
            pub.read_cmd(0, 0)
        c1 = _cpu_us()
        cpu_costs.append(c1 - c0)
    results["py_tight_poll"] = _stats(cpu_costs)
    results["py_tight_poll"]["wall_us"] = wall_us

    # — B2: Python + time.sleep(500 µs) ----------------------------------------
    cpu_costs = []
    for _ in range(n):
        deadline = time.monotonic() + wall_us * 1e-6
        c0 = _cpu_us()
        while time.monotonic() < deadline:
            pub.read_cmd(0, 0)
            time.sleep(0.0005)
        c1 = _cpu_us()
        cpu_costs.append(c1 - c0)
    results["py_sleep_500us"] = _stats(cpu_costs)
    results["py_sleep_500us"]["wall_us"] = wall_us

    # — B3: C++ busy-poll (poll_sleep_ns=0, GIL released for full call) --------
    cpu_costs = []
    for _ in range(n):
        c0 = _cpu_us()
        pub.read_cmd_blocking(wall_us / 1000.0, 0, 0, 0)   # timeout=wall, no sleep
        c1 = _cpu_us()
        cpu_costs.append(c1 - c0)
    results["cpp_busy_poll"] = _stats(cpu_costs)
    results["cpp_busy_poll"]["wall_us"] = wall_us

    # — B4: C++ nanosleep 500 µs between polls (GIL released for full call) ----
    cpu_costs = []
    for _ in range(n):
        c0 = _cpu_us()
        pub.read_cmd_blocking(wall_us / 1000.0, 500_000, 0, 0)
        c1 = _cpu_us()
        cpu_costs.append(c1 - c0)
    results["cpp_nanosleep_500us"] = _stats(cpu_costs)
    results["cpp_nanosleep_500us"]["wall_us"] = wall_us

    pub.close()
    return results


# ══════════════════════════════════════════════════════════════════════════════
# C. CROSS-PROC ROUND-TRIP LATENCY
#    Publisher: ShmPublisher on CPU 0, writes at pub_hz.
#    Subscriber: ShmSubscriber on CPU 1, uses different strategies.
#    Latency = T(cmd_available) - T(state_written).
#    Protocol: publisher embeds write timestamp in state.sim_time (as float64 µs).
#              subscriber reads sim_time, echoes via cmd.seq (mod 2^32).
#              publisher measures cmd arrival time vs state.sim_time.
# ══════════════════════════════════════════════════════════════════════════════

def _sub_process(shm_name: str, strategy: str, sub_hz: float,
                 n_rounds: int, ready_ev: mp.Event, done_ev: mp.Event,
                 latencies_q: mp.Queue) -> None:
    """Subscriber subprocess.  strategy: 'spin' | 'sched'."""
    _pin(1)

    sub = ShmSubscriber(shm_name, 1)
    sub.attach(10_000)

    ready_ev.set()   # signal publisher that we're attached

    latencies: list[float] = []
    rounds = 0

    # Token encoding: angular = step + 1  (always >= 1, never matches default 0)
    def _token(step: int) -> float:
        return float((step % (2**15)) + 1)

    if strategy == "spin":
        last_step = -1
        while rounds < n_rounds and not done_ev.is_set():
            state = sub.read_state_spin(0, 128)
            if state is None or state.step == last_step:
                continue
            last_step = state.step
            write_us = state.sim_time           # publisher stored write timestamp here
            detect_us = now_ns_mono() / 1e3     # when we first saw this step
            latencies.append(detect_us - write_us)
            sub.write_cmd(0, 0, 0.0, _token(state.step))
            rounds += 1

    elif strategy == "sched":
        sched = TaskScheduler(sub_hz, 10)
        last_step = -1

        def poll():
            nonlocal last_step, rounds
            if rounds >= n_rounds:
                return True
            state = sub.read_state_spin(0, 64)
            if state is None or state.step == last_step:
                return True
            last_step = state.step
            write_us = state.sim_time
            detect_us = now_ns_mono() / 1e3
            latencies.append(detect_us - write_us)
            sub.write_cmd(0, 0, 0.0, _token(state.step))
            rounds += 1
            return True

        sched.add_task("poll", poll, 1000.0 / sub_hz)
        while rounds < n_rounds and not done_ev.is_set():
            sched.run()

    sub.detach()
    latencies_q.put(latencies)


def bench_cross_proc(pub_hz=100, n_rounds=200, warmup=30, strategies=None):
    if strategies is None:
        strategies = [
            ("spin",        0),
            ("sched",  2000.0),
            ("sched",  1000.0),
            ("sched",   500.0),
        ]

    results = {}
    _pin(0)

    for strategy, sub_hz in strategies:
        key = "spin" if strategy == "spin" else f"sched_{int(sub_hz)}hz"

        pub = ShmPublisher(_SHM_B, 1, 1, 0)
        pub.open()

        ready_ev = mp.Event()
        done_ev  = mp.Event()
        q        = mp.Queue()

        proc = mp.Process(
            target=_sub_process,
            args=(_SHM_B, strategy, sub_hz, n_rounds + warmup, ready_ev, done_ev, q),
            daemon=True,
        )
        proc.start()
        ready_ev.wait(timeout=15)

        from shmbridge._core import RobotState
        st = RobotState()
        sleeper = LoopSleeper(float(pub_hz))
        cmd_latencies: list[float] = []

        # Token encoding must match subscriber: (step % 2**15) + 1
        def _pub_token(step: int) -> int:
            return (step % (2**15)) + 1

        # burn through warmup rounds without recording
        for i in range(warmup + n_rounds):
            sleeper.start()
            t_write = now_ns_mono() / 1e3       # µs
            st.step = i
            st.sim_time = t_write               # encode write timestamp as µs
            pub.write_state(0, st)

            # wait for cmd echo; yield to OS so subscriber subprocess gets CPU
            expected_token = _pub_token(i)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.5:  # 500 ms budget per round
                cmd = pub.read_cmd(0, 0)
                if cmd is not None and int(cmd.angular) == expected_token:
                    if i >= warmup:
                        t_cmd = now_ns_mono() / 1e3
                        cmd_latencies.append(t_cmd - t_write)
                    break
                time.sleep(0)  # voluntary yield — let subscriber process run

            sleeper.sleep()

        done_ev.set()
        proc.join(timeout=5)

        q.get(timeout=5) if not q.empty() else []
        pub.close()

        results[key] = {
            "pub_hz":    pub_hz,
            "sub_hz":    sub_hz if strategy == "sched" else "spin",
            "strategy":  strategy,
            "round_trip": _stats(cmd_latencies) if cmd_latencies else {},
        }

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", metavar="PATH", default=None)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    n_pub   = 150 if args.quick else 400
    n_block = 50  if args.quick else 200
    n_cross = 60  if args.quick else 200

    out: dict[str, Any] = {}

    # A. Publisher jitter
    print("=== A. Publisher timing jitter (100 Hz, error vs target) ===")
    jitter = bench_publisher_jitter(hz=100, n=n_pub)
    out["publisher_jitter"] = jitter
    for name, s in jitter.items():
        print(f"  {name:20}  p50={_fmt(s['p50'])}µs  p99={_fmt(s['p99'])}µs  max={_fmt(s['max'])}µs")

    # B. read_cmd_blocking CPU
    print(f"\n=== B. Blocking-read CPU burn (10 ms wall, N={n_block}) ===")
    cpu = bench_blocking_read_cpu(wall_us=10_000, n=n_block)
    out["blocking_read_cpu"] = cpu
    labels = {
        "py_tight_poll":      "Py tight poll",
        "py_sleep_500us":     "Py + time.sleep(500µs)",
        "cpp_busy_poll":      "C++ busy-poll",
        "cpp_nanosleep_500us":"C++ nanosleep(500µs)",
    }
    for name, s in cpu.items():
        util = min(100.0, s["p50"] / s["wall_us"] * 100)
        label = labels.get(name, name)
        print(f"  {label:26}  CPU p50={_fmt(s['p50'])}µs  ({util:.0f}% of {s['wall_us']}µs wall)")

    # C. Cross-proc round-trip
    print(f"\n=== C. Cross-proc round-trip (pub=100Hz, N={n_cross}) ===")
    cross = bench_cross_proc(
        pub_hz=100, n_rounds=n_cross, warmup=20,
        strategies=[
            ("spin",        0),
            ("sched",  2000.0),
            ("sched",  1000.0),
            ("sched",   500.0),
        ],
    )
    out["cross_proc"] = cross
    print(f"  {'strategy':20}  {'sub_hz':8}  p50 ms   p99 ms   max ms")
    for key, r in cross.items():
        rt = r["round_trip"]
        if not rt:
            print(f"  {key:20}  (no data)")
            continue
        sh = r["sub_hz"] if isinstance(r["sub_hz"], str) else f"{int(r['sub_hz'])} Hz"
        print(f"  {key:20}  {sh:8}  "
              f"{_fmt(rt['p50']/1000, 3)} ms  "
              f"{_fmt(rt['p99']/1000, 3)} ms  "
              f"{_fmt(rt['max']/1000, 3)} ms")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nSaved {args.json}")


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    main()
