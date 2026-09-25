#!/usr/bin/env python3
"""
bench_scheduler.py — p99 latency and TaskScheduler accuracy benchmark.

Measures:
  1. spin_sleep_us accuracy across targets (50 µs – 10 ms), N=500 samples each.
  2. LoopSleeper inter-iteration jitter at 100 / 500 / 1000 / 2000 Hz.
  3. TaskScheduler per-task period accuracy: actual vs target call period.
  4. TaskScheduler overhead per tick (pure scheduling cost, no task body).

Outputs JSON and a human-readable table.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
from typing import Any

# ------------------------------------------------------------------
# Bootstrap: prefer installed shmbridge, fall back to source tree.
# ------------------------------------------------------------------
try:
    from shmbridge._core import (
        LoopSleeper,
        TaskScheduler,
        now_ns_mono,
        spin_sleep_us,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../src"))
    from shmbridge._core import (  # type: ignore[no-redef]
        LoopSleeper,
        TaskScheduler,
        now_ns_mono,
        spin_sleep_us,
    )


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _pin_cpu(cpu: int) -> bool:
    try:
        avail = os.sched_getaffinity(0)
        if cpu in avail:
            os.sched_setaffinity(0, {cpu})
            return True
    except (AttributeError, PermissionError, OSError):
        pass
    return False


def _percentiles(data: list[float], ps=(50, 95, 99)):
    s = sorted(data)
    n = len(s)
    result = {}
    for p in ps:
        idx = max(0, min(n - 1, int(n * p / 100)))
        result[f"p{p}"] = s[idx]
    result["min"] = s[0]
    result["max"] = s[-1]
    result["mean"] = statistics.mean(data)
    result["n"] = n
    return result


def _fmt(v, digits=2):
    return f"{v:.{digits}f}" if v is not None else "—"


# ------------------------------------------------------------------
# 1. spin_sleep accuracy
# ------------------------------------------------------------------

def bench_spin_sleep(targets_us=(50, 100, 200, 500, 1000, 2000, 5000, 10000),
                     n=500, warmup=20):
    """For each target, measure (actual - target) error distribution."""
    results = {}
    for target in targets_us:
        # warmup — let Welford estimator converge
        for _ in range(warmup):
            spin_sleep_us(target)

        errors = []
        for _ in range(n):
            t0 = now_ns_mono()
            spin_sleep_us(target)
            actual_us = (now_ns_mono() - t0) / 1e3
            errors.append(actual_us - target)

        stats = _percentiles(errors)
        stats["target_us"] = target
        results[f"sleep_{target}us"] = stats

    return results


# ------------------------------------------------------------------
# 2. LoopSleeper inter-iteration jitter
# ------------------------------------------------------------------

def bench_loop_sleeper(hz_list=(100, 500, 1000, 2000), n=300, warmup=20):
    results = {}
    for hz in hz_list:
        target_us = 1e6 / hz
        sleeper = LoopSleeper(float(hz))

        # warmup
        for _ in range(warmup):
            sleeper.start()
            sleeper.sleep()

        periods = []
        t_last = now_ns_mono()
        sleeper.start()
        sleeper.sleep()
        t_last = now_ns_mono()

        for _ in range(n):
            sleeper.start()
            t_now = now_ns_mono()
            periods.append((t_now - t_last) / 1e3)   # µs
            t_last = t_now
            sleeper.sleep()

        errors = [p - target_us for p in periods]
        stats = _percentiles(errors)
        stats["target_us"] = target_us
        stats["hz"] = hz
        results[f"loop_{hz}hz"] = stats

    return results


# ------------------------------------------------------------------
# 3. TaskScheduler per-task period accuracy
# ------------------------------------------------------------------

def bench_scheduler_tasks(base_hz=1000.0, n_ticks=1000, warmup_ticks=50):
    """
    Build a scheduler with tasks at several periods and record when each
    actually fires.  Compute error distribution vs the target period.
    """
    sched = TaskScheduler(base_hz, 10)
    base_ms = 1000.0 / base_hz

    task_configs = [
        ("1ms",   1.0),
        ("2ms",   2.0),
        ("5ms",   5.0),
        ("10ms", 10.0),
        ("20ms", 20.0),
    ]

    call_times: dict[str, list[int]] = {name: [] for name, _ in task_configs}

    def make_cb(name):
        def cb():
            call_times[name].append(now_ns_mono())
            return True
        return cb

    for name, period_ms in task_configs:
        sched.add_task(name, make_cb(name), period_ms)

    # warmup
    for _ in range(warmup_ticks):
        sched.run()

    for name in call_times:
        call_times[name].clear()

    # measurement
    for _ in range(n_ticks):
        sched.run()

    results = {}
    for name, period_ms in task_configs:
        ts = call_times[name]
        if len(ts) < 2:
            continue
        intervals_us = [(ts[i+1] - ts[i]) / 1e3 for i in range(len(ts) - 1)]
        target_us = period_ms * 1000.0
        errors = [iv - target_us for iv in intervals_us]
        stats = _percentiles(errors)
        stats["target_us"] = target_us
        stats["n_calls"] = len(ts)
        results[f"task_{name}"] = stats

    return results, sched.report()


# ------------------------------------------------------------------
# 4. Scheduler overhead (empty tick cost)
# ------------------------------------------------------------------

def bench_scheduler_overhead(base_hz=2000.0, n=500):
    """Measure raw per-tick overhead with zero tasks."""
    sched = TaskScheduler(base_hz, 10)
    overhead_ns = []
    for _ in range(n):
        t0 = now_ns_mono()
        # single tick without sleeping (we time just the dispatch loop)
        # Patch: use internal run() but override loop to 0 cost
        # Instead just time the overhead of our own bookkeeping
        sched.run()
        overhead_ns.append(now_ns_mono() - t0)

    # The measured values include the sleep; subtract target period.
    target_ns = int(1e9 / base_hz)
    pure_overhead = [max(0, v - target_ns) for v in overhead_ns]
    return {
        "target_period_us": target_ns / 1e3,
        **_percentiles([v / 1e3 for v in pure_overhead]),
    }


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="shmbridge scheduler benchmark")
    parser.add_argument("--json", metavar="PATH", default=None)
    parser.add_argument("--quick", action="store_true",
                        help="Fewer samples for faster CI run")
    args = parser.parse_args()

    n_sleep  = 100 if args.quick else 500
    n_loop   = 100 if args.quick else 300
    n_ticks  = 300 if args.quick else 1000

    _pin_cpu(0)

    print("=== spin_sleep_us accuracy ===")
    sleep_results = bench_spin_sleep(n=n_sleep)
    for key, s in sleep_results.items():
        target = s["target_us"]
        print(f"  {target:6}µs  "
              f"p50={_fmt(s['p50'])}µs  "
              f"p95={_fmt(s['p95'])}µs  "
              f"p99={_fmt(s['p99'])}µs  "
              f"max={_fmt(s['max'])}µs")

    print("\n=== LoopSleeper jitter ===")
    loop_results = bench_loop_sleeper(n=n_loop)
    for key, s in loop_results.items():
        hz = s["hz"]
        print(f"  {hz:5}Hz  target={s['target_us']:.0f}µs  "
              f"p50={_fmt(s['p50'])}µs  "
              f"p95={_fmt(s['p95'])}µs  "
              f"p99={_fmt(s['p99'])}µs  "
              f"max={_fmt(s['max'])}µs")

    print("\n=== TaskScheduler period accuracy (base=1kHz) ===")
    sched_results, sched_report = bench_scheduler_tasks(n_ticks=n_ticks)
    for key, s in sched_results.items():
        target_ms = s["target_us"] / 1000
        print(f"  {target_ms:4}ms task  "
              f"p50={_fmt(s['p50'])}µs  "
              f"p95={_fmt(s['p95'])}µs  "
              f"p99={_fmt(s['p99'])}µs  "
              f"calls={s['n_calls']}")
    print(sched_report)

    print("=== Scheduler overhead (2kHz, empty ticks) ===")
    overhead = bench_scheduler_overhead()
    print(f"  p50={_fmt(overhead['p50'])}µs  "
          f"p99={_fmt(overhead['p99'])}µs  "
          f"max={_fmt(overhead['max'])}µs  "
          f"(target period={overhead['target_period_us']:.0f}µs)")

    if args.json:
        out: dict[str, Any] = {
            "spin_sleep": sleep_results,
            "loop_sleeper": loop_results,
            "scheduler_tasks": sched_results,
            "scheduler_overhead": overhead,
        }
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nSaved {args.json}")


if __name__ == "__main__":
    main()
