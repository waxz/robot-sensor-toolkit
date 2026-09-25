#!/usr/bin/env python3
"""
bench_ipc_cpu.py — IPC latency vs CPU consumption, normal vs intensive load.

Each mechanism is evaluated under two conditions:
  IDLE:    no background load
  LOADED:  (nCPU - 1) CPU-spinning processes saturating all spare cores

Per mechanism the benchmark reports:
  - One-way detection latency  (p50 / p95 / p99 / max)
  - Subscriber CPU%  (fraction of wall time consumed by the subscriber process)
  - Publisher  CPU%  (fraction of wall time consumed by the publisher side)
  - Latency ratio    (loaded_p50 / idle_p50) — sensitivity to CPU contention

The subscriber CPU% is the most revealing number:
  - shm spinners (~100%) compete with the load — latency spikes under contention
  - blocking sockets ( ~0%) yield the CPU to the OS — stable under load
"""
from __future__ import annotations

import argparse
import contextlib
import ctypes
import json
import mmap
import multiprocessing as mp
import os
import resource
import socket
import statistics
import struct
import sys
import threading
import time
from typing import Any

# ── bootstrap shmbridge ──────────────────────────────────────────────────────
try:
    from shmbridge._core import RobotState as _RS
    from shmbridge._core import (
        ShmPublisher,
        ShmSubscriber,
        now_ns_mono,
    )

    _HAS_SHMBRIDGE = True
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../src"))
    try:
        from shmbridge._core import RobotState as _RS
        from shmbridge._core import (  # type: ignore[no-redef]
            ShmPublisher,
            ShmSubscriber,
            now_ns_mono,
        )

        _HAS_SHMBRIDGE = True
    except ImportError:
        _HAS_SHMBRIDGE = False

        def now_ns_mono() -> int:  # type: ignore[misc]
            return time.monotonic_ns()


try:
    import zmq

    _HAS_ZMQ = True
except ImportError:
    _HAS_ZMQ = False

_NCPU = os.cpu_count() or 2

# ── libc for raw shm ─────────────────────────────────────────────────────────
_libc = ctypes.CDLL("libc.so.6", use_errno=True)
_libc.shm_open.restype = ctypes.c_int
_libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
_libc.shm_unlink.restype = ctypes.c_int
_libc.ftruncate.restype = ctypes.c_int
_libc.close.restype = ctypes.c_int

_RAW_STRUCT = struct.Struct("=QqQ")
_RAW_SHM_SIZE = 64
_MAX_VALID_US = 100_000.0


# ══════════════════════════════════════════════════════════════════════════════
# CPU spinner (runs as a daemon process to create load)
# ══════════════════════════════════════════════════════════════════════════════


def _cpu_spin_worker(stop_ev: mp.Event) -> None:
    x = 1.0
    while not stop_ev.is_set():
        x = x * 1.000_001 + 0.000_001
    _ = x


# ══════════════════════════════════════════════════════════════════════════════
# Stats helpers
# ══════════════════════════════════════════════════════════════════════════════


def _pct(data: list[float], p: float) -> float:
    s = sorted(data)
    return s[max(0, min(len(s) - 1, int(len(s) * p / 100)))]


def _stats(data: list[float]) -> dict:
    return {
        "p50": _pct(data, 50),
        "p95": _pct(data, 95),
        "p99": _pct(data, 99),
        "max": max(data),
        "mean": statistics.mean(data),
        "stdev": statistics.stdev(data) if len(data) > 1 else 0.0,
        "n": len(data),
    }


def _cpu_us() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return (r.ru_utime + r.ru_stime) * 1e6


def _fmt(v: float, d: int = 1) -> str:
    return f"{v:.{d}f}" if v is not None else "—"


# ══════════════════════════════════════════════════════════════════════════════
# Generic result type returned by every sub-process
# ══════════════════════════════════════════════════════════════════════════════
# tuple: (latencies_us, sub_cpu_us, sub_wall_us)


# ══════════════════════════════════════════════════════════════════════════════
# 1. shmbridge
# ══════════════════════════════════════════════════════════════════════════════

_SHM_BRIDGE = "/bench_cpu_bridge"


def _shmbridge_sub(ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    sub = ShmSubscriber(_SHM_BRIDGE, 1)
    sub.attach(10_000)
    ready.set()

    cpu0 = _cpu_us()
    t0 = time.monotonic()
    latencies: list[float] = []
    last_step = -1
    while len(latencies) < n and not done.is_set():
        st = sub.read_state_spin(0, 256)
        if st is None or st.step == last_step:
            continue
        recv_ns = now_ns_mono()
        send_ns = int(st.sim_time * 1e3)
        latencies.append((recv_ns - send_ns) / 1e3)
        last_step = st.step

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    sub.detach()
    q.put((latencies, cpu_us, wall_us))


def bench_shmbridge(n: int, warmup: int) -> dict:
    if not _HAS_SHMBRIDGE:
        return {"error": "shmbridge unavailable"}
    pub = ShmPublisher(_SHM_BRIDGE, 1, 1, 0)
    pub.open()
    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_shmbridge_sub, args=(ready, done, q, n + warmup), daemon=True)
    proc.start()
    ready.wait(10)

    st = _RS()
    cpu0, t0 = _cpu_us(), time.monotonic()
    for i in range(warmup + n):
        send_ns = now_ns_mono()
        st.step = i
        st.sim_time = send_ns / 1e3
        pub.write_state(0, st)
        time.sleep(0)
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)
    lats, sub_cpu_us, sub_wall_us = q.get(timeout=5)
    pub.close()

    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 2. Raw POSIX shm
# ══════════════════════════════════════════════════════════════════════════════

_RAW_SHM_NAME = b"/bench_cpu_rawshm"


def _raw_shm_sub(ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    O_RDONLY = 0
    fd = _libc.shm_open(_RAW_SHM_NAME, O_RDONLY, 0)
    buf = mmap.mmap(fd, _RAW_SHM_SIZE, access=mmap.ACCESS_READ)
    _libc.close(fd)
    ready.set()

    cpu0 = _cpu_us()
    t0 = time.monotonic()
    latencies: list[float] = []
    last_ver = 0
    while len(latencies) < n and not done.is_set():
        ver_lo, send_ns, ver_hi = _RAW_STRUCT.unpack_from(buf, 0)
        if ver_lo != ver_hi or ver_lo % 2 != 0 or ver_lo <= last_ver:
            continue
        recv_ns = now_ns_mono()
        lat = (recv_ns - send_ns) / 1e3
        last_ver = ver_lo
        if 0 < lat < _MAX_VALID_US:
            latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    buf.close()
    q.put((latencies, cpu_us, wall_us))


def bench_raw_posix_shm(n: int, warmup: int) -> dict:
    O_CREAT, O_RDWR = 0o100, 0o2
    fd = _libc.shm_open(_RAW_SHM_NAME, O_CREAT | O_RDWR, 0o600)
    if fd < 0:
        return {"error": "shm_open failed"}
    _libc.ftruncate(fd, _RAW_SHM_SIZE)
    buf = mmap.mmap(fd, _RAW_SHM_SIZE)
    _libc.close(fd)
    _RAW_STRUCT.pack_into(buf, 0, 0, 0, 0)

    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_raw_shm_sub, args=(ready, done, q, n + warmup), daemon=True)
    proc.start()
    ready.wait(10)

    cpu0, t0 = _cpu_us(), time.monotonic()
    ver = 0
    for _ in range(warmup + n):
        ver += 2
        _RAW_STRUCT.pack_into(buf, 0, ver - 1, 0, 0)
        send_ns = now_ns_mono()
        _RAW_STRUCT.pack_into(buf, 0, ver, send_ns, ver)
        time.sleep(0)
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)
    buf.close()
    _libc.shm_unlink(_RAW_SHM_NAME)

    lats, sub_cpu_us, sub_wall_us = q.get(timeout=5)
    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 3. ZMQ PUSH/PULL inproc (threads — best ZMQ inproc variant)
# ══════════════════════════════════════════════════════════════════════════════


def bench_zmq_inproc(n: int, warmup: int) -> dict:
    if not _HAS_ZMQ:
        return {"error": "pyzmq not installed"}
    ctx = zmq.Context.instance()
    push = ctx.socket(zmq.PUSH)
    pull = ctx.socket(zmq.PULL)
    push.bind("inproc://bench_cpu_push")
    pull.connect("inproc://bench_cpu_push")
    pull.setsockopt(zmq.RCVTIMEO, 500)

    latencies: list[float] = []
    sub_stats: list[tuple[float, float]] = []
    barrier = threading.Barrier(2)

    def _pull():
        cpu0 = _cpu_us()
        t0 = time.monotonic()
        barrier.wait()
        received = 0
        while received < warmup + n:
            try:
                data = pull.recv()
            except zmq.Again:
                continue
            recv_ns = now_ns_mono()
            send_ns = struct.unpack("q", data)[0]
            received += 1
            if received > warmup:
                latencies.append((recv_ns - send_ns) / 1e3)
        sub_stats.append((_cpu_us() - cpu0, (time.monotonic() - t0) * 1e6))

    t = threading.Thread(target=_pull, daemon=True)
    t.start()
    barrier.wait()
    time.sleep(0.01)

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(warmup + n):
        send_ns = now_ns_mono()
        push.send(struct.pack("q", send_ns))
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    t.join(5)
    push.close()
    pull.close()

    sub_cpu_us, sub_wall_us = sub_stats[0] if sub_stats else (0, 1)
    return {
        **(_stats(latencies) if latencies else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 4. ZMQ PUSH/PULL ipc:// (cross-process)
# ══════════════════════════════════════════════════════════════════════════════

_ZMQ_IPC = "/tmp/bench_cpu_zmq.ipc"


def _zmq_ipc_sub(ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.PULL)
    sub.connect(f"ipc://{_ZMQ_IPC}")
    sub.setsockopt(zmq.RCVTIMEO, 500)
    ready.set()

    cpu0 = _cpu_us()
    t0 = time.monotonic()
    latencies: list[float] = []
    while len(latencies) < n and not done.is_set():
        try:
            data = sub.recv()
        except zmq.Again:
            continue
        recv_ns = now_ns_mono()
        latencies.append((now_ns_mono() - struct.unpack("q", data)[0]) / 1e3)
        # overwrite with accurate timing
        latencies[-1] = (recv_ns - struct.unpack("q", data)[0]) / 1e3

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    sub.close()
    ctx.term()
    q.put((latencies, cpu_us, wall_us))


def bench_zmq_ipc(n: int, warmup: int) -> dict:
    if not _HAS_ZMQ:
        return {"error": "pyzmq not installed"}
    ctx = zmq.Context()
    push = ctx.socket(zmq.PUSH)
    push.bind(f"ipc://{_ZMQ_IPC}")

    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_zmq_ipc_sub, args=(ready, done, q, n + warmup), daemon=True)
    proc.start()
    ready.wait(10)
    time.sleep(0.05)

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(warmup + n):
        send_ns = now_ns_mono()
        push.send(struct.pack("q", send_ns))
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)
    push.close()
    ctx.term()
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_ZMQ_IPC)

    lats, sub_cpu_us, sub_wall_us = q.get(timeout=5)
    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 5. Unix domain socket DGRAM (cross-process)
# ══════════════════════════════════════════════════════════════════════════════

_UDS_PATH = "/tmp/bench_cpu_uds.sock"


def _uds_sub(ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(_UDS_PATH)
    s.settimeout(0.5)
    ready.set()

    cpu0 = _cpu_us()
    t0 = time.monotonic()
    latencies: list[float] = []
    while len(latencies) < n and not done.is_set():
        try:
            data = s.recv(8)
        except TimeoutError:
            continue
        recv_ns = now_ns_mono()
        latencies.append((recv_ns - struct.unpack("q", data)[0]) / 1e3)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    s.close()
    q.put((latencies, cpu_us, wall_us))


def bench_uds(n: int, warmup: int) -> dict:
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_PATH)

    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_uds_sub, args=(ready, done, q, n + warmup), daemon=True)
    proc.start()
    ready.wait(10)

    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(warmup + n):
        send_ns = now_ns_mono()
        s.sendto(struct.pack("q", send_ns), _UDS_PATH)
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)
    s.close()
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_PATH)

    lats, sub_cpu_us, sub_wall_us = q.get(timeout=5)
    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 6. Pipe  (mp.Pipe, cross-process)
# ══════════════════════════════════════════════════════════════════════════════


def _pipe_sub(
    conn_r: mp.connection.Connection,
    ready: mp.Event,
    done: mp.Event,
    q: mp.Queue,
    n: int,
) -> None:
    cpu0 = _cpu_us()
    t0 = time.monotonic()
    ready.set()
    latencies: list[float] = []
    while len(latencies) < n and not done.is_set():
        if not conn_r.poll(0.5):
            continue
        recv_ns = now_ns_mono()
        send_ns = conn_r.recv()
        latencies.append((recv_ns - send_ns) / 1e3)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    conn_r.close()
    q.put((latencies, cpu_us, wall_us))


def bench_pipe(n: int, warmup: int) -> dict:
    conn_r, conn_w = mp.Pipe(duplex=False)
    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_pipe_sub, args=(conn_r, ready, done, q, n + warmup), daemon=True)
    proc.start()
    conn_r.close()
    ready.wait(10)

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(warmup + n):
        conn_w.send(now_ns_mono())
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)
    conn_w.close()

    lats, sub_cpu_us, sub_wall_us = q.get(timeout=5)
    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 7. TCP loopback (cross-process)
# ══════════════════════════════════════════════════════════════════════════════

_TCP_PORT = 59_322


def _tcp_sub(ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", _TCP_PORT))
    srv.listen(1)
    ready.set()
    conn, _ = srv.accept()
    conn.settimeout(0.5)

    cpu0 = _cpu_us()
    t0 = time.monotonic()
    latencies: list[float] = []
    buf = b""
    while len(latencies) < n and not done.is_set():
        try:
            chunk = conn.recv(64)
        except TimeoutError:
            continue
        if not chunk:
            break
        buf += chunk
        while len(buf) >= 8:
            recv_ns = now_ns_mono()
            latencies.append((recv_ns - struct.unpack("q", buf[:8])[0]) / 1e3)
            buf = buf[8:]

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    conn.close()
    srv.close()
    q.put((latencies, cpu_us, wall_us))


def bench_tcp(n: int, warmup: int) -> dict:
    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_tcp_sub, args=(ready, done, q, n + warmup), daemon=True)
    proc.start()
    ready.wait(10)
    time.sleep(0.05)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect(("127.0.0.1", _TCP_PORT))

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(warmup + n):
        s.sendall(struct.pack("q", now_ns_mono()))
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)
    s.close()

    lats, sub_cpu_us, sub_wall_us = q.get(timeout=5)
    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 8. mp.Queue (cross-process)
# ══════════════════════════════════════════════════════════════════════════════


def _mpq_sub(mq: mp.Queue, ready: mp.Event, done: mp.Event, out: mp.Queue, n: int) -> None:
    cpu0 = _cpu_us()
    t0 = time.monotonic()
    ready.set()
    latencies: list[float] = []
    while len(latencies) < n and not done.is_set():
        try:
            send_ns = mq.get(timeout=0.5)
        except Exception:
            continue
        latencies.append((now_ns_mono() - send_ns) / 1e3)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    out.put((latencies, cpu_us, wall_us))


def bench_mpqueue(n: int, warmup: int) -> dict:
    mq: mp.Queue = mp.Queue()
    ready, done, out = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(target=_mpq_sub, args=(mq, ready, done, out, n + warmup), daemon=True)
    proc.start()
    ready.wait(10)

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(warmup + n):
        mq.put(now_ns_mono())
        time.sleep(1e-4)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done.set()
    proc.join(5)

    lats, sub_cpu_us, sub_wall_us = out.get(timeout=5)
    usable = lats[warmup:] if len(lats) > warmup else lats
    return {
        **(_stats(usable) if usable else {"error": "no samples"}),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Runner: execute a benchmark under optional CPU load
# ══════════════════════════════════════════════════════════════════════════════

BENCHMARKS = [
    ("shmbridge", "shmbridge (C++ seqlock)", bench_shmbridge),
    ("raw_posix_shm", "Raw POSIX shm (Python)", bench_raw_posix_shm),
    ("zmq_inproc", "ZMQ PUSH/PULL inproc", bench_zmq_inproc),
    ("zmq_ipc", "ZMQ PUSH/PULL ipc://", bench_zmq_ipc),
    ("uds", "Unix domain socket DGRAM", bench_uds),
    ("pipe", "mp.Pipe (anonymous)", bench_pipe),
    ("tcp", "TCP loopback", bench_tcp),
    ("mpqueue", "mp.Queue", bench_mpqueue),
]


def run_all(n: int, warmup: int, n_spinners: int = 0) -> dict[str, Any]:
    spinners: list[mp.Process] = []
    stop_ev = mp.Event()
    if n_spinners > 0:
        for _ in range(n_spinners):
            p = mp.Process(target=_cpu_spin_worker, args=(stop_ev,), daemon=True)
            p.start()
            spinners.append(p)
        time.sleep(0.2)  # let spinners saturate

    results: dict[str, Any] = {}
    for key, label, fn in BENCHMARKS:
        print(f"    {label} …", end="", flush=True)
        try:
            r = fn(n=n, warmup=warmup)
        except Exception as exc:
            r = {"error": str(exc)}
        results[key] = {"label": label, **r}
        if "error" in r:
            print(f"\r    {label:42}  ERROR: {r['error']}")
        else:
            print(
                f"\r    {label:42}  "
                f"p50={_fmt(r['p50'])}µs  "
                f"p99={_fmt(r['p99'])}µs  "
                f"sub={_fmt(r.get('sub_cpu_pct', 0), 0)}%  "
                f"pub={_fmt(r.get('pub_cpu_pct', 0), 0)}%"
            )

    if spinners:
        stop_ev.set()
        for p in spinners:
            p.join(2)

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", metavar="PATH", default=None)
    parser.add_argument("--quick", action="store_true", help="n=150, warmup=20")
    args = parser.parse_args()

    n = 150 if args.quick else 400
    warmup = 20 if args.quick else 50
    # Use nCPU-1 spinners; at least 1 for the loaded case
    n_spinners = max(1, _NCPU - 1)

    print(f"\nIPC CPU+Latency benchmark  (n={n}, warmup={warmup}, spinners={n_spinners})\n")

    print("[ IDLE — no background load ]")
    idle = run_all(n, warmup, n_spinners=0)

    print(f"\n[ LOADED — {n_spinners} CPU spinners on spare cores ]")
    loaded = run_all(n, warmup, n_spinners=n_spinners)

    out: dict[str, Any] = {
        "n": n,
        "warmup": warmup,
        "n_spinners": n_spinners,
        "ncpu": _NCPU,
        "idle": idle,
        "loaded": loaded,
    }

    print(f"\n{'Mechanism':42}  {'idle p50':>8}  {'load p50':>8}  {'ratio':>6}  {'sub CPU%':>8}")
    print("  " + "─" * 78)
    for key, label, _ in BENCHMARKS:
        ir = idle.get(key, {})
        lr = loaded.get(key, {})
        if "p50" not in ir:
            continue
        ratio = lr["p50"] / ir["p50"] if lr.get("p50") else 0
        print(
            f"  {label:40}  "
            f"{_fmt(ir['p50']):>8}  "
            f"{_fmt(lr.get('p50', 0)):>8}  "
            f"{ratio:>5.2f}x  "
            f"{_fmt(ir.get('sub_cpu_pct', 0), 0):>7}%"
        )

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nSaved {args.json}")


if __name__ == "__main__":
    mp.set_start_method("forkserver", force=True)
    main()
