#!/usr/bin/env python3
"""
bench_pubsub_matrix.py — size x rate x backend pub/sub benchmark for
shmbridge's generic per-topic transport (``shmbridge.topics``).

Complements ``bench_robot_control.py`` (which fixes message shape/size and
sweeps rate + poll strategy for the ``ExtShmBridge`` combined-segment API)
by instead fixing the transport pattern to a single publisher -> single
subscriber topic and sweeping **payload size** and **publish rate**, to
characterize how shmbridge behaves for realtime robot topics of varying
shape: small fixed-rate control/IMU-like messages up to large bursty
sensor payloads (LiDAR/point-cloud/image-like blobs).

Two transports are exercised, both "own named shm segment, keep only the
latest sample" (seqlock) semantics:

  * **raw** — ``TopicPublisher``/``TopicSubscriber`` (up to 88 bytes/msg,
    the generic fixed-size channel used for small structured messages like
    Pose2d/Imu/BatteryState).
  * **pointcloud** — ``PointCloudPublisher``/``PointCloudSubscriber``
    (bulk payload, up to 65536 points x 16 bytes = 1 MiB/msg), representing
    large sensor blobs that don't fit the 88-byte generic slot.

Exercises the compiled C++ extension (``shmbridge._core``'s
``RawChannelPublisher``/``PointCloudPublisher`` etc, via
``shmbridge.topics``) -- the only implementation shmbridge ships; a
``backend`` label is still threaded through results/CLI for historical
report compatibility (older runs of this script also measured a
pure-Python ctypes mirror, since removed in favor of making the C++
extension mandatory).

Metrics per (mode, size, rate, backend) configuration, both processes
started as separate OS processes (``multiprocessing``, spawn) so IPC and
scheduling cost is representative:

  * one-way latency (publish -> first observed by subscriber), ns
  * delivery ratio (distinct messages observed / messages published) —
    this transport keeps only the latest sample, so a subscriber slower
    than the publish rate is *expected* to coalesce/miss samples; this is
    not a dropped-packet bug, it is the keep-latest design working as
    intended, and the ratio quantifies how much coalescing occurs at each
    rate/size.
  * achieved throughput (distinct messages/sec, MB/sec)
  * CPU usage of both processes (via psutil), to characterize a fully
    busy-spun (lowest latency, highest CPU) run in both directions.

Usage::

    python bench_pubsub_matrix.py --duration 2 --json out.json
    python bench_pubsub_matrix.py --mode raw --raw-sizes 8,88 --raw-rates 1000
    python bench_pubsub_matrix.py --mode pointcloud --pc-sizes 1048576 --pc-rates 50
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import statistics
import struct
import sys
import time
from dataclasses import asdict, dataclass

try:
    import psutil

    _HAS_PSUTIL = True
except ImportError:
    _HAS_PSUTIL = False

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

RAW_MAX_BYTES = 88  # shmbridge.topics.RAW_PAYLOAD_BYTES
PC_POINT_BYTES = 16
PC_MAX_POINTS = 65536
PC_MAX_BYTES = PC_MAX_POINTS * PC_POINT_BYTES

_HDR_FMT = "<qq"  # seq (int64), t0_ns (int64, perf_counter_ns absolute)
_HDR_BYTES = struct.calcsize(_HDR_FMT)


def _make_raw(backend: str):
    if backend == "cpp":
        from shmbridge._core import RawChannelPublisher as Pub
        from shmbridge.topics import TopicSubscriber as Sub

        return Pub, Sub
    raise ValueError(backend)


def _make_pc(backend: str):
    if backend == "cpp":
        from shmbridge._core import PointCloudPublisher as Pub
        from shmbridge.topics import PointCloudSubscriber as Sub

        return Pub, Sub
    raise ValueError(backend)


# ── worker processes: raw (<=88 byte) transport ─────────────────────────────


def _raw_pub_process(backend, name, size_bytes, rate_hz, duration_s, warmup_s, ready_evt, epoch_ns, result_q):
    Pub, _ = _make_raw(backend)
    pub = Pub()
    pub.open(name)
    start = time.perf_counter_ns()
    epoch_ns.value = start
    ready_evt.set()

    proc = psutil.Process(os.getpid()) if _HAS_PSUTIL else None
    if proc is not None:
        proc.cpu_percent(None)

    pad = b"\xab" * max(0, size_bytes - _HDR_BYTES)
    period_ns = int(1e9 / rate_hz)
    next_tick = start
    warmup_until = start + int(warmup_s * 1e9)
    end_at = start + int((warmup_s + duration_s) * 1e9)

    seq = 0
    n_published = 0
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

        seq += 1
        t0 = now
        body = struct.pack(_HDR_FMT, seq, t0) + pad
        pub.write(body[:size_bytes])
        if now >= warmup_until:
            n_published += 1
        next_tick += period_ns

    cpu_pct = proc.cpu_percent(None) if proc is not None else None
    pub.close()
    result_q.put({"n_published": n_published, "cpu_pct_pub": cpu_pct})


def _raw_sub_process(backend, name, duration_s, warmup_s, ready_evt, epoch_ns, stop_evt, result_q):
    ready_evt.wait(timeout=10.0)
    _, Sub = _make_raw(backend)
    sub = Sub()
    sub.attach(name, timeout_ms=5000.0)

    proc = psutil.Process(os.getpid()) if _HAS_PSUTIL else None
    if proc is not None:
        proc.cpu_percent(None)

    one_way_ns: list[int] = []
    last_seq = -1
    n_distinct = 0
    epoch = epoch_ns.value
    warmup_until = epoch + int(warmup_s * 1e9)
    end_at = epoch + int((warmup_s + duration_s + 0.5) * 1e9)

    while time.perf_counter_ns() < end_at and not stop_evt.is_set():
        data = sub.read()
        if data is None:
            continue
        seq, t0 = struct.unpack_from(_HDR_FMT, bytes(data), 0)
        if seq == last_seq:
            continue
        last_seq = seq
        now = time.perf_counter_ns()
        if now >= warmup_until:
            n_distinct += 1
            one_way_ns.append(now - t0)

    cpu_pct = proc.cpu_percent(None) if proc is not None else None
    sub.detach()
    result_q.put({"one_way_ns": one_way_ns, "n_distinct": n_distinct, "cpu_pct_sub": cpu_pct})


# ── worker processes: pointcloud (bulk) transport ───────────────────────────


def _pc_pub_process(backend, name, size_bytes, rate_hz, duration_s, warmup_s, ready_evt, epoch_ns, result_q):
    import numpy as np

    Pub, _ = _make_pc(backend)
    pub = Pub()
    pub.open(name)
    start = time.perf_counter_ns()
    epoch_ns.value = start
    ready_evt.set()

    proc = psutil.Process(os.getpid()) if _HAS_PSUTIL else None
    if proc is not None:
        proc.cpu_percent(None)

    n_points = min(PC_MAX_POINTS, max(1, size_bytes // PC_POINT_BYTES))
    points = np.ones((n_points, 4), dtype=np.float32)

    period_ns = int(1e9 / rate_hz)
    next_tick = start
    warmup_until = start + int(warmup_s * 1e9)
    end_at = start + int((warmup_s + duration_s) * 1e9)

    n_published = 0
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

        t0 = now
        pub.write(points, ts=float(t0))
        if now >= warmup_until:
            n_published += 1
        next_tick += period_ns

    cpu_pct = proc.cpu_percent(None) if proc is not None else None
    pub.close()
    result_q.put({"n_published": n_published, "cpu_pct_pub": cpu_pct, "n_points": n_points})


def _pc_sub_process(backend, name, duration_s, warmup_s, ready_evt, epoch_ns, stop_evt, result_q):
    ready_evt.wait(timeout=10.0)
    _, Sub = _make_pc(backend)
    sub = Sub()
    sub.attach(name, timeout_ms=10000.0)

    proc = psutil.Process(os.getpid()) if _HAS_PSUTIL else None
    if proc is not None:
        proc.cpu_percent(None)

    one_way_ns: list[int] = []
    last_ts = -1.0
    n_distinct = 0
    n_bytes_received = 0
    epoch = epoch_ns.value
    warmup_until = epoch + int(warmup_s * 1e9)
    end_at = epoch + int((warmup_s + duration_s + 0.5) * 1e9)

    while time.perf_counter_ns() < end_at and not stop_evt.is_set():
        hdr = sub.read_header()
        if hdr is None or hdr.ts == last_ts:
            continue
        last_ts = hdr.ts
        payload = sub.read_bytes()
        now = time.perf_counter_ns()
        if now >= warmup_until:
            n_distinct += 1
            one_way_ns.append(now - int(hdr.ts))
            if payload is not None:
                n_bytes_received += len(payload)

    cpu_pct = proc.cpu_percent(None) if proc is not None else None
    sub.detach()
    result_q.put(
        {
            "one_way_ns": one_way_ns,
            "n_distinct": n_distinct,
            "cpu_pct_sub": cpu_pct,
            "n_bytes_received": n_bytes_received,
        }
    )


# ── stats / orchestration ───────────────────────────────────────────────────


@dataclass
class RunResult:
    mode: str
    backend: str
    size_bytes: int
    rate_hz: float
    n_published: int
    n_distinct: int
    delivery_ratio: float
    achieved_rate_hz: float
    throughput_MBps: float
    one_way_ns_mean: float
    one_way_ns_p50: float
    one_way_ns_p99: float
    one_way_ns_max: float
    cpu_pct_pub: float | None
    cpu_pct_sub: float | None


def _pctl(data: list[float], q: float) -> float:
    if not data:
        return float("nan")
    s = sorted(data)
    idx = min(len(s) - 1, int(round(q * (len(s) - 1))))
    return s[idx]


def run_one(mode: str, backend: str, size_bytes: int, rate_hz: float, duration_s: float, warmup_s: float) -> RunResult:
    ctx = mp.get_context("spawn")
    ready_evt = ctx.Event()
    stop_evt = ctx.Event()
    result_q: mp.Queue = ctx.Queue()
    epoch_ns = ctx.Value("q", 0)

    name = f"bench_matrix_{mode}_{size_bytes}_{int(rate_hz)}_{backend}"

    if mode == "raw" and size_bytes < _HDR_BYTES:
        raise ValueError(
            f"raw mode requires size_bytes >= {_HDR_BYTES} (seq+timestamp header); got {size_bytes}"
        )

    if mode == "raw":
        pub_fn, sub_fn = _raw_pub_process, _raw_sub_process
        pub_args = (backend, name, size_bytes, rate_hz, duration_s, warmup_s, ready_evt, epoch_ns, result_q)
        sub_args = (backend, name, duration_s, warmup_s, ready_evt, epoch_ns, stop_evt, result_q)
    elif mode == "pointcloud":
        pub_fn, sub_fn = _pc_pub_process, _pc_sub_process
        pub_args = (backend, name, size_bytes, rate_hz, duration_s, warmup_s, ready_evt, epoch_ns, result_q)
        sub_args = (backend, name, duration_s, warmup_s, ready_evt, epoch_ns, stop_evt, result_q)
    else:
        raise ValueError(mode)

    p_pub = ctx.Process(target=pub_fn, args=pub_args)
    p_sub = ctx.Process(target=sub_fn, args=sub_args)
    p_sub.start()
    p_pub.start()

    p_pub.join(timeout=duration_s + warmup_s + 15.0)
    stop_evt.set()
    p_sub.join(timeout=5.0)

    results: dict = {}
    while not result_q.empty():
        results.update(result_q.get())

    for p in (p_pub, p_sub):
        if p.is_alive():
            p.terminate()
        p.join(timeout=2.0)

    one_way = results.get("one_way_ns", [])
    n_published = results.get("n_published", 0)
    n_distinct = results.get("n_distinct", 0)
    delivery_ratio = (n_distinct / n_published) if n_published else float("nan")
    achieved_rate = n_distinct / duration_s if duration_s else float("nan")
    throughput_mbps = (n_distinct * size_bytes / duration_s) / 1e6 if duration_s else float("nan")

    return RunResult(
        mode=mode,
        backend=backend,
        size_bytes=size_bytes,
        rate_hz=rate_hz,
        n_published=n_published,
        n_distinct=n_distinct,
        delivery_ratio=delivery_ratio,
        achieved_rate_hz=achieved_rate,
        throughput_MBps=throughput_mbps,
        one_way_ns_mean=statistics.fmean(one_way) if one_way else float("nan"),
        one_way_ns_p50=_pctl(one_way, 0.50),
        one_way_ns_p99=_pctl(one_way, 0.99),
        one_way_ns_max=max(one_way) if one_way else float("nan"),
        cpu_pct_pub=results.get("cpu_pct_pub"),
        cpu_pct_sub=results.get("cpu_pct_sub"),
    )


def _fmt_us(ns: float) -> str:
    return f"{ns / 1e3:9.2f}" if ns == ns else "      n/a"


def _fmt_size(n: int) -> str:
    if n >= 1024 * 1024:
        return f"{n / (1024 * 1024):.1f}MiB"
    if n >= 1024:
        return f"{n / 1024:.1f}KiB"
    return f"{n}B"


def print_table(results: list[RunResult]) -> None:
    hdr = (
        f"{'mode':10} {'backend':7} {'size':>8} {'rate':>6} {'1-way p50':>10} {'1-way p99':>10} "
        f"{'delivery':>9} {'achieved':>9} {'MB/s':>8} {'cpu pub':>8} {'cpu sub':>8}"
    )
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        print(
            f"{r.mode:10} {r.backend:7} {_fmt_size(r.size_bytes):>8} {r.rate_hz:6.0f} "
            f"{_fmt_us(r.one_way_ns_p50):>10} {_fmt_us(r.one_way_ns_p99):>10} "
            f"{r.delivery_ratio * 100:8.1f}% {r.achieved_rate_hz:9.1f} {r.throughput_MBps:8.2f} "
            f"{r.cpu_pct_pub or 0:8.1f} {r.cpu_pct_sub or 0:8.1f}"
        )
    print("(latency in microseconds; delivery = distinct msgs seen / msgs published; cpu = % of one core)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["raw", "pointcloud", "both"], default="both")
    ap.add_argument(
        "--raw-sizes",
        default="16,32,88",
        help="comma-separated payload sizes in bytes (16<=size<=88; "
        "16 bytes are reserved for the seq+timestamp header)",
    )
    ap.add_argument("--raw-rates", default="10,100,500,1000", help="comma-separated target Hz for raw mode")
    ap.add_argument(
        "--pc-sizes",
        default="4096,65536,262144,1048576",
        help="comma-separated payload sizes in bytes (<=1048576) for pointcloud mode",
    )
    ap.add_argument("--pc-rates", default="10,50,100", help="comma-separated target Hz for pointcloud mode")
    ap.add_argument("--backend", choices=["cpp"], default="cpp")
    ap.add_argument("--duration", type=float, default=2.0, help="measured seconds per run")
    ap.add_argument("--warmup", type=float, default=0.3, help="warmup seconds per run (discarded)")
    ap.add_argument("--json", default=None, help="write raw results to this JSON file")
    args = ap.parse_args()

    if not _HAS_PSUTIL:
        print("warning: psutil not installed, CPU columns will be empty (pip install psutil)", file=sys.stderr)

    modes = ["raw", "pointcloud"] if args.mode == "both" else [args.mode]
    backends = [args.backend]
    raw_sizes = [int(s) for s in args.raw_sizes.split(",")]
    raw_rates = [float(r) for r in args.raw_rates.split(",")]
    pc_sizes = [int(s) for s in args.pc_sizes.split(",")]
    pc_rates = [float(r) for r in args.pc_rates.split(",")]

    all_results: list[RunResult] = []
    for mode in modes:
        sizes = raw_sizes if mode == "raw" else pc_sizes
        rates = raw_rates if mode == "raw" else pc_rates
        for backend in backends:
            for size_bytes in sizes:
                for rate in rates:
                    print(
                        f"running mode={mode} backend={backend} size={size_bytes}B "
                        f"rate={rate:.0f}Hz duration={args.duration}s ...",
                        file=sys.stderr,
                    )
                    try:
                        r = run_one(mode, backend, size_bytes, rate, args.duration, args.warmup)
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
