"""
bench_ipc.py - Multi-core IPC benchmark for shmbridge.

Benchmarks
----------
A. In-process C++ write latency
B. Cross-process seqlock latency (CPU-pinned, no sleep between writes)
C. Torn-read (retry) statistics
D. ZMQ PUSH/PULL TCP cross-process latency (comparison)
E. Single atomic uint64 cache-propagation micro-benchmark

Usage
-----
    cd shmbridge
    python tests/bench_ipc.py [--quick] [--n-iter N] [--json PATH]
"""

from __future__ import annotations

import argparse
import json
import mmap
import multiprocessing as mp
import os
import sys
import time

import numpy as np

# ── shmbridge import ──────────────────────────────────────────────────────────
try:
    import shmbridge
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(_here, "..", "src"))
    import shmbridge

from shmbridge import RobotState, ShmPublisher, ShmSubscriber

# Sentinel step value: large uint64 that signals "done"
_DONE_STEP: int = 2**32 - 1  # 0xFFFFFFFF — fits in uint32 and uint64

# ── defaults ──────────────────────────────────────────────────────────────────
N_WARMUP = 200
N_ITER = 5000
DURATION = 3.0  # seconds for cross-process benchmarks


# ── CPU-affinity helper ───────────────────────────────────────────────────────

def _pin_cpu(cpu: int) -> bool:
    """Pin current process to *cpu*. Returns True if successful."""
    try:
        avail = os.sched_getaffinity(0)
        if len(avail) >= 2 and cpu in avail:
            os.sched_setaffinity(0, {cpu})
            return True
    except (AttributeError, PermissionError, OSError):
        pass
    return False


def _can_pin() -> bool:
    """Return True if at least 2 CPUs are available for affinity pinning."""
    try:
        return len(os.sched_getaffinity(0)) >= 2
    except (AttributeError, OSError):
        return False


# ── stats helper ─────────────────────────────────────────────────────────────

def _stats(samples: list[float], cpu_pinned: bool = False) -> dict:
    a = np.array(samples, dtype=np.float64)
    return {
        "min": float(np.min(a)),
        "p25": float(np.percentile(a, 25)),
        "p50": float(np.percentile(a, 50)),
        "p75": float(np.percentile(a, 75)),
        "p95": float(np.percentile(a, 95)),
        "p99": float(np.percentile(a, 99)),
        "max": float(np.max(a)),
        "mean": float(np.mean(a)),
        "n": len(a),
        "cpu_pinned": cpu_pinned,
    }


# ── A. In-process C++ write latency ──────────────────────────────────────────

def bench_inproc_write(n_warmup: int = N_WARMUP, n_iter: int = N_ITER) -> dict:
    """Measure raw C++ seqlock write latency in-process."""
    shm_name = f"/bench_inproc_{os.getpid()}"
    pub = ShmPublisher(shm_name)
    pub.open()
    state = RobotState()

    # Warmup
    for _ in range(n_warmup):
        pub.write_state(0, state)

    # Measure
    samples: list[float] = []
    for i in range(n_iter):
        state.step = i
        t0 = time.monotonic_ns()
        pub.write_state(0, state)
        t1 = time.monotonic_ns()
        samples.append((t1 - t0) / 1_000.0)  # → µs

    pub.close()
    return _stats(samples, cpu_pinned=False)


# ── B. Cross-process seqlock latency ─────────────────────────────────────────

def _xproc_publisher(shm_name: str, ready_ev: mp.Event, done_ev: mp.Event,
                     duration: float, cpu: int) -> None:
    """Publisher subprocess: writes continuously, embeds monotonic_ns as sim_time."""
    _pin_cpu(cpu)
    pub = ShmPublisher(shm_name)
    pub.open()
    state = RobotState()
    ready_ev.set()

    t_end = time.monotonic() + duration
    step = 0
    while time.monotonic() < t_end:
        state.step = step
        state.sim_time = float(time.monotonic_ns())
        pub.write_state(0, state)
        step += 1

    # Signal done: write sentinel step so subscriber can exit
    state.step = _DONE_STEP
    state.sim_time = float(time.monotonic_ns())
    pub.write_state(0, state)
    done_ev.set()
    # Keep segment alive briefly so subscriber finishes
    time.sleep(0.5)
    pub.close()


def _xproc_subscriber(shm_name: str, ready_ev: mp.Event, done_ev: mp.Event,
                      result_q: mp.Queue, cpu: int) -> None:
    """Subscriber subprocess: spins, detects step changes, records latency."""
    _pin_cpu(cpu)
    ready_ev.wait(timeout=5.0)
    sub = ShmSubscriber(shm_name)
    sub.attach(5000)  # int timeout in ms

    latencies: list[float] = []
    last_step = 0

    while True:
        state = sub.read_state_spin(0, 256)
        if state is None:
            if done_ev.is_set():
                break
            continue
        s = int(state.step)
        if s == _DONE_STEP:
            break
        if s > last_step:
            now_ns = time.monotonic_ns()
            pub_ts = int(state.sim_time)
            lat_us = (now_ns - pub_ts) / 1_000.0
            if 0.0 < lat_us < 10_000.0:  # sanity filter (< 10ms)
                latencies.append(lat_us)
            last_step = s

    sub.detach()
    result_q.put(latencies)


def bench_xproc(duration: float = DURATION) -> dict:
    """Cross-process seqlock latency with CPU pinning."""
    pinned = _can_pin()
    pub_cpu, sub_cpu = (0, 1) if pinned else (0, 0)

    ctx = mp.get_context("fork")
    shm_name = f"/bench_xproc_{os.getpid()}"
    ready_ev = ctx.Event()
    done_ev = ctx.Event()
    result_q: mp.Queue = ctx.Queue()

    pub_proc = ctx.Process(
        target=_xproc_publisher,
        args=(shm_name, ready_ev, done_ev, duration, pub_cpu),
        daemon=True,
    )
    sub_proc = ctx.Process(
        target=_xproc_subscriber,
        args=(shm_name, ready_ev, done_ev, result_q, sub_cpu),
        daemon=True,
    )

    pub_proc.start()
    sub_proc.start()
    pub_proc.join(timeout=duration + 5)
    sub_proc.join(timeout=duration + 5)

    latencies: list[float] = []
    if not result_q.empty():
        latencies = result_q.get_nowait()

    if not latencies:
        latencies = [0.0]

    return _stats(latencies, cpu_pinned=pinned)


# ── C. Torn-read retry stats ──────────────────────────────────────────────────

def _retry_publisher(shm_name: str, ready_ev: mp.Event, done_ev: mp.Event,
                     duration: float) -> None:
    pub = ShmPublisher(shm_name)
    pub.open()
    state = RobotState()
    ready_ev.set()

    t_end = time.monotonic() + duration
    step = 0
    while time.monotonic() < t_end:
        state.step = step
        state.sim_time = float(time.monotonic_ns())
        pub.write_state(0, state)
        step += 1

    state.step = _DONE_STEP
    pub.write_state(0, state)
    done_ev.set()
    time.sleep(0.3)
    pub.close()


def _retry_subscriber(shm_name: str, ready_ev: mp.Event, done_ev: mp.Event,
                      result_q: mp.Queue) -> None:
    """Count retries per successful read using read_state (non-spinning manually)."""
    ready_ev.wait(timeout=5.0)
    sub = ShmSubscriber(shm_name)
    sub.attach(5000)  # int ms

    retry_counts: list[int] = []
    last_step = 0

    while True:
        retries = 0
        state = None
        # Manually spin with retry counting
        for attempt in range(256):
            state = sub.read_state(0)
            if state is not None:
                retries = attempt
                break
        if state is None:
            if done_ev.is_set():
                break
            continue
        s = int(state.step)
        if s == _DONE_STEP:
            break
        if s > last_step:
            retry_counts.append(retries)
            last_step = s

    sub.detach()
    result_q.put(retry_counts)


def bench_retry_stats(duration: float = 1.0) -> dict:
    """Measure torn-read retry distribution."""
    ctx = mp.get_context("fork")
    shm_name = f"/bench_retry_{os.getpid()}"
    ready_ev = ctx.Event()
    done_ev = ctx.Event()
    result_q: mp.Queue = ctx.Queue()

    pub_proc = ctx.Process(
        target=_retry_publisher,
        args=(shm_name, ready_ev, done_ev, duration),
        daemon=True,
    )
    sub_proc = ctx.Process(
        target=_retry_subscriber,
        args=(shm_name, ready_ev, done_ev, result_q),
        daemon=True,
    )

    pub_proc.start()
    sub_proc.start()
    pub_proc.join(timeout=duration + 5)
    sub_proc.join(timeout=duration + 5)

    counts: list[int] = []
    if not result_q.empty():
        counts = result_q.get_nowait()

    if not counts:
        counts = [0]

    arr = np.array(counts, dtype=np.int64)
    total = len(arr)
    gt1 = int(np.sum(arr > 0))
    return {
        "n_reads": total,
        "mean_retries": float(np.mean(arr)),
        "max_retries": int(np.max(arr)),
        "pct_with_retry": 100.0 * gt1 / total if total > 0 else 0.0,
    }


# ── D. ZMQ PUSH/PULL TCP cross-process latency ───────────────────────────────

def _zmq_publisher(ready_ev: mp.Event, done_ev: mp.Event,
                   result_q: mp.Queue, duration: float, port: int) -> None:
    try:
        import zmq
    except ImportError:
        result_q.put(None)
        return

    ctx_zmq = zmq.Context()
    sock = ctx_zmq.socket(zmq.PUSH)
    sock.set_hwm(0)  # unlimited HWM to avoid drops
    sock.bind(f"tcp://127.0.0.1:{port}")
    ready_ev.set()
    # Give subscriber time to connect
    time.sleep(0.1)

    t_end = time.monotonic() + duration
    step = 0
    while time.monotonic() < t_end:
        ts_ns = time.monotonic_ns()
        # Pack: 8-byte LE timestamp + 8-byte step
        msg = ts_ns.to_bytes(8, "little") + step.to_bytes(8, "little")
        try:
            sock.send(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass
        step += 1

    # Send sentinel
    try:
        sock.send(b"\xff" * 16, flags=0)
    except Exception:
        pass
    done_ev.set()
    time.sleep(0.3)
    sock.close()
    ctx_zmq.term()


def _zmq_subscriber(ready_ev: mp.Event, done_ev: mp.Event,
                    result_q: mp.Queue, port: int) -> None:
    try:
        import zmq
    except ImportError:
        result_q.put([])
        return

    ctx_zmq = zmq.Context()
    sock = ctx_zmq.socket(zmq.PULL)
    sock.connect(f"tcp://127.0.0.1:{port}")
    sock.setsockopt(zmq.RCVTIMEO, 5000)
    ready_ev.wait(timeout=5.0)

    latencies: list[float] = []
    while True:
        try:
            msg = sock.recv()
        except Exception:
            break
        now_ns = time.monotonic_ns()
        if len(msg) != 16:
            continue
        if msg == b"\xff" * 16:
            break
        pub_ts = int.from_bytes(msg[:8], "little")
        lat_us = (now_ns - pub_ts) / 1_000.0
        if 0.0 < lat_us < 100_000.0:
            latencies.append(lat_us)

    sock.close()
    ctx_zmq.term()
    result_q.put(latencies)


def bench_zmq(duration: float = DURATION) -> dict | None:
    """ZMQ PUSH/PULL TCP cross-process latency."""
    try:
        import zmq  # noqa: F401
    except ImportError:
        return None

    port = 15777
    ctx = mp.get_context("fork")
    ready_ev = ctx.Event()
    done_ev = ctx.Event()
    result_q: mp.Queue = ctx.Queue()

    pub_proc = ctx.Process(
        target=_zmq_publisher,
        args=(ready_ev, done_ev, result_q, duration, port),
        daemon=True,
    )
    sub_proc = ctx.Process(
        target=_zmq_subscriber,
        args=(ready_ev, done_ev, result_q, port),
        daemon=True,
    )

    sub_proc.start()
    pub_proc.start()
    pub_proc.join(timeout=duration + 5)
    sub_proc.join(timeout=duration + 5)

    latencies: list[float] = []
    while not result_q.empty():
        val = result_q.get_nowait()
        if isinstance(val, list):
            latencies.extend(val)

    if not latencies:
        return None

    return _stats(latencies, cpu_pinned=False)


# ── E. Single atomic uint64 cache-propagation micro-benchmark ─────────────────
#
# Layout in the mmap page (offsets in bytes):
#   0..7   : uint64 counter (written by publisher)
#   8..15  : uint64 sentinel flag (written by publisher to signal done)
# Publisher writes counter = 1..N, then sentinel[8..15] = 0xDEAD
# Subscriber spins on counter at offset 0.

_ATOMIC_DONE_FLAG: int = 0xDEADBEEF
_MMAP_SIZE = 4096


def _atomic_publisher(shm_name: str, ready_ev: mp.Event,
                      result_q: mp.Queue, n_iter: int, cpu: int) -> None:
    """Write a sequence of uint64 values into the mmap."""
    _pin_cpu(cpu)
    from shmbridge._libc import _libc
    from shmbridge._platform import O_CREAT, O_EXCL, O_RDWR

    name_b = shm_name.encode()
    _libc.shm_unlink(name_b)
    fd = _libc.shm_open(name_b, O_CREAT | O_RDWR | O_EXCL, 0o666)
    _libc.ftruncate(fd, _MMAP_SIZE)
    mm = mmap.mmap(fd, _MMAP_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(fd)

    # Zero the region
    mm[0:16] = b"\x00" * 16

    ready_ev.set()
    time.sleep(0.05)  # let subscriber attach

    pub_timestamps: list[int] = []
    for i in range(1, n_iter + 1):
        ts = time.monotonic_ns()
        pub_timestamps.append(ts)
        # Write counter value at offset 0 as little-endian uint64
        mm[0:8] = i.to_bytes(8, "little")
        # Yield so subscriber gets a chance to detect (minimal overhead on Linux)
        time.sleep(0)

    # Write sentinel at offset 8
    mm[8:16] = _ATOMIC_DONE_FLAG.to_bytes(8, "little")

    time.sleep(0.5)
    mm.close()
    _libc.shm_unlink(name_b)
    result_q.put(pub_timestamps)


def _atomic_subscriber(shm_name: str, ready_ev: mp.Event,
                       result_q: mp.Queue, n_iter: int, cpu: int) -> None:
    """Spin-detect uint64 changes and record detection timestamps."""
    _pin_cpu(cpu)
    from shmbridge._libc import _libc
    from shmbridge._platform import O_RDWR

    ready_ev.wait(timeout=5.0)
    time.sleep(0.02)  # wait for publisher to open

    name_b = shm_name.encode()
    fd = _libc.shm_open(name_b, O_RDWR, 0o666)
    mm = mmap.mmap(fd, _MMAP_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(fd)

    sub_timestamps: list[int] = []
    sub_values: list[int] = []
    last_val = 0

    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        v = int.from_bytes(mm[0:8], "little")
        done_flag = int.from_bytes(mm[8:16], "little")
        if done_flag == _ATOMIC_DONE_FLAG:
            break
        if v > last_val:
            ts = time.monotonic_ns()
            sub_timestamps.append(ts)
            sub_values.append(v)
            last_val = v
        if last_val >= n_iter:
            break

    mm.close()
    result_q.put((sub_timestamps, sub_values))


def bench_atomic_cache(n_iter: int = 500) -> dict | None:
    """Single atomic uint64 cache-propagation micro-benchmark."""
    pinned = _can_pin()
    pub_cpu, sub_cpu = (0, 1) if pinned else (0, 0)

    ctx = mp.get_context("fork")
    shm_name = f"/bench_atomic_{os.getpid()}"
    ready_ev = ctx.Event()
    result_q: mp.Queue = ctx.Queue()

    pub_proc = ctx.Process(
        target=_atomic_publisher,
        args=(shm_name, ready_ev, result_q, n_iter, pub_cpu),
        daemon=True,
    )
    sub_proc = ctx.Process(
        target=_atomic_subscriber,
        args=(shm_name, ready_ev, result_q, n_iter, sub_cpu),
        daemon=True,
    )

    pub_proc.start()
    sub_proc.start()
    pub_proc.join(timeout=90)
    sub_proc.join(timeout=90)

    pub_timestamps = None
    sub_result = None
    items: list = []
    while not result_q.empty():
        items.append(result_q.get_nowait())

    for item in items:
        if isinstance(item, list):
            pub_timestamps = item
        elif isinstance(item, tuple):
            sub_result = item

    if pub_timestamps is None or sub_result is None:
        return None

    sub_timestamps, sub_values = sub_result
    if not sub_timestamps:
        return None

    # Match by sequence number
    latencies: list[float] = []
    pub_map = {i + 1: pub_timestamps[i] for i in range(len(pub_timestamps))}
    for sub_ts, v in zip(sub_timestamps, sub_values):
        pub_ts = pub_map.get(v)
        if pub_ts is not None:
            lat_us = (sub_ts - pub_ts) / 1_000.0
            if 0.0 < lat_us < 10_000.0:
                latencies.append(lat_us)

    if not latencies:
        return None

    return _stats(latencies, cpu_pinned=pinned)


# ── formatting ────────────────────────────────────────────────────────────────

def _bar(val: float, scale: float, width: int = 40) -> str:
    if scale <= 0:
        return ""
    filled = max(1, min(width, int(val / scale * width)))
    return "█" * filled


def _fmt_row(name: str, s: dict) -> str:
    pin = "✓" if s.get("cpu_pinned") else " "
    return (
        f"  {pin} {name:<36s}  "
        f"{s['min']:8.2f}  {s['p50']:8.2f}  {s['p95']:8.2f}  "
        f"{s['p99']:8.2f}  {s['max']:8.2f}  {s['n']:>8d}"
    )


def print_table(results: dict) -> None:
    hdr = (
        f"\n  {'CPU':1s} {'Benchmark':<36s}  "
        f"{'min µs':>8s}  {'p50 µs':>8s}  {'p95 µs':>8s}  "
        f"{'p99 µs':>8s}  {'max µs':>8s}  {'n':>8s}"
    )
    sep = "  " + "-" * (len(hdr) - 4)
    print(hdr)
    print(sep)
    order = [
        ("cpp_inproc_write", "C++ in-proc write"),
        ("cpp_xproc", "C++ cross-proc (seqlock)"),
        ("atomic_cache", "Atomic uint64 cache-prop"),
        ("zmq_tcp", "ZMQ PUSH/PULL TCP"),
    ]
    for key, label in order:
        if key in results and results[key] is not None:
            print(_fmt_row(label, results[key]))
    print()


def print_decomposition(results: dict) -> None:
    inproc = results.get("cpp_inproc_write")
    xproc = results.get("cpp_xproc")
    atomic = results.get("atomic_cache")

    if inproc is None or xproc is None:
        return

    t_write = inproc["p50"]
    t_prop = atomic["p50"] if atomic else 0.08  # fallback estimate
    t_total = xproc["p50"]
    t_detect = max(0.0, t_total - t_write - t_prop)

    scale = t_total if t_total > 0 else 1.0

    print("Latency source breakdown (cross-process, µs):")
    print(
        f"  T_write    (in-proc p50):      {t_write:6.2f} µs  "
        f"{_bar(t_write, scale, 36)}"
    )
    prop_src = "measured" if atomic else "estimate"
    print(
        f"  T_propagate (cache, {prop_src}):  {t_prop:6.2f} µs  "
        f"{_bar(t_prop, scale, 36)}"
    )
    print(
        f"  T_detect   (spin lag, p50):    {t_detect:6.2f} µs  "
        f"{_bar(t_detect, scale, 36)}"
    )
    print("  " + "─" * 60)
    print(f"  Total measured p50:            {t_total:6.2f} µs")
    print()


def print_retry_stats(stats: dict) -> None:
    print("Torn-read retry statistics:")
    print(f"  Total reads:      {stats['n_reads']:,}")
    print(f"  Mean retries:     {stats['mean_retries']:.4f}")
    print(f"  Max retries:      {stats['max_retries']}")
    print(f"  % with retry:     {stats['pct_with_retry']:.2f}%")
    print()


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    mp.set_start_method("fork", force=True)

    parser = argparse.ArgumentParser(description="shmbridge IPC benchmark")
    parser.add_argument("--json", default="bench_ipc.json", metavar="PATH",
                        help="Output JSON path (default: bench_ipc.json)")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: N_ITER=1000, DURATION=1.0s")
    parser.add_argument("--n-iter", type=int, default=None, metavar="N",
                        help="Override N_ITER for in-proc benchmark")
    args = parser.parse_args()

    n_iter = args.n_iter or (1000 if args.quick else N_ITER)
    duration = 1.0 if args.quick else DURATION
    atomic_n = 200 if args.quick else 500

    print(f"shmbridge IPC Benchmark  [backend={shmbridge._BACKEND}]")
    print(f"  n_iter={n_iter}, duration={duration:.1f}s, quick={args.quick}")
    print()

    results: dict = {}
    retry_stats: dict = {}

    # A. In-process write latency
    print("A. In-process C++ write latency ... ", end="", flush=True)
    results["cpp_inproc_write"] = bench_inproc_write(
        n_warmup=N_WARMUP, n_iter=n_iter
    )
    s = results["cpp_inproc_write"]
    print(f"p50={s['p50']:.2f} µs  p99={s['p99']:.2f} µs")

    # B. Cross-process seqlock latency
    print("B. Cross-process seqlock latency ... ", end="", flush=True)
    results["cpp_xproc"] = bench_xproc(duration=duration)
    s = results["cpp_xproc"]
    pin_str = "CPU-pinned" if s["cpu_pinned"] else "no-pin"
    print(f"p50={s['p50']:.2f} µs  p99={s['p99']:.2f} µs  n={s['n']:,}  [{pin_str}]")

    # C. Torn-read retry stats
    print("C. Torn-read retry stats ... ", end="", flush=True)
    retry_stats = bench_retry_stats(duration=min(1.0, duration))
    print(
        f"mean_retries={retry_stats['mean_retries']:.4f}  "
        f"pct_with_retry={retry_stats['pct_with_retry']:.2f}%"
    )

    # D. ZMQ PUSH/PULL TCP
    print("D. ZMQ PUSH/PULL TCP latency ... ", end="", flush=True)
    zmq_res = bench_zmq(duration=duration)
    results["zmq_tcp"] = zmq_res
    if zmq_res is not None:
        print(
            f"p50={zmq_res['p50']:.2f} µs  "
            f"p99={zmq_res['p99']:.2f} µs  n={zmq_res['n']:,}"
        )
    else:
        print("(zmq not available)")

    # E. Atomic uint64 cache-propagation
    print("E. Atomic uint64 cache-propagation ... ", end="", flush=True)
    atomic_res = bench_atomic_cache(n_iter=atomic_n)
    results["atomic_cache"] = atomic_res
    if atomic_res is not None:
        print(
            f"p50={atomic_res['p50']:.2f} µs  "
            f"p99={atomic_res['p99']:.2f} µs  n={atomic_res['n']:,}"
        )
    else:
        print("(inconclusive)")

    # Print table
    print_table(results)

    # Print retry stats
    print_retry_stats(retry_stats)

    # Print latency decomposition
    print_decomposition(results)

    # Write JSON
    output = {
        "benchmarks": results,
        "retry_stats": retry_stats,
        "backend": shmbridge._BACKEND,
        "n_iter": n_iter,
        "duration": duration,
    }
    out_path = args.json
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"Results written to {out_path}")


if __name__ == "__main__":
    main()
