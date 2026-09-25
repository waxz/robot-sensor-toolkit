#!/usr/bin/env python3
"""
bench_ipc_rates.py — IPC latency and CPU at robotics-typical publish rates.

Publish rates tested:
    30 Hz  — LiDAR / pointcloud
   100 Hz  — wheel encoder / low-rate IMU
  1000 Hz  — high-rate IMU / control loop

For shmbridge, the subscriber-side polling strategy matters enormously:
  spin      — busy-poll forever; lowest latency, ~100% CPU
  p100us    — spin, then sleep 100 µs if no new message; trade ~100µs latency for CPU
  p500us    — spin + 500 µs sleep
  p1ms      — spin + 1 ms sleep
  p5ms      — spin + 5 ms sleep  (only sensible at ≤100 Hz)

Blocking OS-scheduled mechanisms (natural sleep, wake on data):
  UDS DGRAM, TCP loopback, mp.Pipe, ZMQ ipc://

Metrics per (mechanism, strategy, hz):
  detection latency:  p50 / p95 / p99 / max
  subscriber CPU%  :  fraction of wall time the subscriber process consumed CPU
  pub CPU%         :  same for publisher
"""
from __future__ import annotations

import argparse
import contextlib
import ctypes
import json
import mmap
import multiprocessing as mp
import os
import socket
import statistics
import struct
import sys
import time
from typing import Any

# ── bootstrap shmbridge ──────────────────────────────────────────────────────
try:
    from shmbridge._core import LoopSleeper, ShmPublisher, ShmSubscriber, now_ns_mono
    from shmbridge._core import RobotState as _RS

    _HAS_SHMBRIDGE = True
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../src"))
    try:
        from shmbridge._core import (  # type: ignore[no-redef]
            LoopSleeper,
            ShmPublisher,
            ShmSubscriber,
            now_ns_mono,
        )
        from shmbridge._core import RobotState as _RS  # type: ignore[no-redef]

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

import resource

# ── libc for raw shm ─────────────────────────────────────────────────────────
_libc = ctypes.CDLL("libc.so.6", use_errno=True)
_libc.shm_open.restype = ctypes.c_int
_libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
_libc.shm_unlink.restype = ctypes.c_int
_libc.ftruncate.restype = ctypes.c_int
_libc.close.restype = ctypes.c_int

_RAW_STRUCT = struct.Struct("=QqQ")  # ver_lo, send_ns, ver_hi
_RAW_SHM_SIZE = 64
_MAX_VALID_US = 500_000.0  # discard > 500 ms (startup artefacts)

RATES_HZ = [30, 100, 1000]


# ── helpers ──────────────────────────────────────────────────────────────────

def _cpu_us() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return (r.ru_utime + r.ru_stime) * 1e6


def _pct(data: list[float], p: float) -> float:
    s = sorted(data)
    return s[max(0, min(len(s) - 1, int(len(s) * p / 100)))]


def _stats(data: list[float]) -> dict:
    if not data:
        return {"error": "no samples"}
    return {
        "p50":   _pct(data, 50),
        "p95":   _pct(data, 95),
        "p99":   _pct(data, 99),
        "max":   max(data),
        "mean":  statistics.mean(data),
        "stdev": statistics.stdev(data) if len(data) > 1 else 0.0,
        "n":     len(data),
    }


def _fmt(v: float, d: int = 1) -> str:
    return f"{v:.{d}f}" if v is not None else "—"


# ══════════════════════════════════════════════════════════════════════════════
# shmbridge: publisher side (common across all subscriber strategies)
# ══════════════════════════════════════════════════════════════════════════════

_SHM_RATE = "/bench_rate_shm"


def _shm_publish(hz: float, n: int, pub_ready: mp.Event, sub_ready: mp.Event,
                 done_ev: mp.Event, q: mp.Queue) -> None:
    pub = ShmPublisher(_SHM_RATE, 1, 1, 0)
    pub.open()
    pub_ready.set()      # SHM now exists; subscriber may attach
    sub_ready.wait(15)   # wait until subscriber has attached
    st = _RS()

    cpu0, t0 = _cpu_us(), time.monotonic()
    sleeper = LoopSleeper(float(hz))
    for i in range(n):
        sleeper.start()
        st.step = i
        st.sim_time = now_ns_mono() / 1e3  # µs timestamp
        pub.write_state(0, st)
        sleeper.sleep()
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done_ev.set()
    pub.close()
    q.put(("pub", pub_cpu_us, pub_wall_us))


def _shm_subscribe(sleep_s: float, n: int, warmup: int,
                   pub_ready: mp.Event, sub_ready: mp.Event,
                   done_ev: mp.Event, q: mp.Queue) -> None:
    """Generic shmbridge subscriber: spin briefly, sleep sleep_s if nothing new."""
    pub_ready.wait(15)   # wait until SHM is created
    sub = ShmSubscriber(_SHM_RATE, 1)
    sub.attach(10_000)
    sub_ready.set()      # signal publisher that we're attached

    cpu0, t0 = _cpu_us(), time.monotonic()
    latencies: list[float] = []
    last_step = -1
    received = 0

    while received < warmup + n and not done_ev.is_set():
        st = sub.read_state_spin(0, 32)
        if st is None or st.step == last_step:
            if sleep_s > 0:
                time.sleep(sleep_s)
            continue
        recv_ns = now_ns_mono()
        send_ns = int(st.sim_time * 1e3)
        lat = (recv_ns - send_ns) / 1e3
        last_step = st.step
        received += 1
        if received > warmup and 0 < lat < _MAX_VALID_US:
            latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    sub.detach()
    q.put(("sub", latencies, cpu_us, wall_us))


def bench_shmbridge_strategy(hz: float, sleep_s: float,
                              n: int = 200, warmup: int = 30) -> dict:
    if not _HAS_SHMBRIDGE:
        return {"error": "shmbridge unavailable"}
    total = warmup + n
    pub_ready = mp.Event()
    sub_ready = mp.Event()
    done_ev = mp.Event()
    q: mp.Queue = mp.Queue()

    pub_proc = mp.Process(target=_shm_publish,
                          args=(hz, total, pub_ready, sub_ready, done_ev, q), daemon=True)
    sub_proc = mp.Process(target=_shm_subscribe,
                          args=(sleep_s, n, warmup, pub_ready, sub_ready, done_ev, q), daemon=True)
    pub_proc.start()
    sub_proc.start()
    pub_proc.join(total / hz + 15)
    sub_proc.join(10)

    pub_cpu_us = pub_wall_us = 0.0
    latencies: list[float] = []
    sub_cpu_us = sub_wall_us = 0.0
    for _ in range(2):
        item = q.get(timeout=10)
        if item[0] == "pub":
            _, pub_cpu_us, pub_wall_us = item
        else:
            _, latencies, sub_cpu_us, sub_wall_us = item

    return {
        **_stats(latencies),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Raw POSIX shm (Python spin)
# ══════════════════════════════════════════════════════════════════════════════

_RAW_SHM_NAME = b"/bench_rate_rawshm"


def _raw_shm_sub_rate(sleep_s: float, n: int, warmup: int,
                      pub_ready: mp.Event, sub_ready: mp.Event,
                      done_ev: mp.Event, q: mp.Queue) -> None:
    pub_ready.wait(15)
    O_RDONLY = 0
    fd = _libc.shm_open(_RAW_SHM_NAME, O_RDONLY, 0)
    buf = mmap.mmap(fd, _RAW_SHM_SIZE, access=mmap.ACCESS_READ)
    _libc.close(fd)
    sub_ready.set()

    cpu0, t0 = _cpu_us(), time.monotonic()
    latencies: list[float] = []
    last_ver = 0
    received = 0

    while received < warmup + n and not done_ev.is_set():
        ver_lo, send_ns, ver_hi = _RAW_STRUCT.unpack_from(buf, 0)
        if ver_lo != ver_hi or ver_lo % 2 != 0 or ver_lo <= last_ver:
            if sleep_s > 0:
                time.sleep(sleep_s)
            continue
        recv_ns = now_ns_mono()
        lat = (recv_ns - send_ns) / 1e3
        last_ver = ver_lo
        received += 1
        if received > warmup and 0 < lat < _MAX_VALID_US:
            latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    buf.close()
    q.put(("sub", latencies, cpu_us, wall_us))


def _raw_shm_pub_rate(hz: float, n: int, pub_ready: mp.Event, sub_ready: mp.Event,
                      done_ev: mp.Event, q: mp.Queue) -> None:
    O_CREAT, O_RDWR = 0o100, 0o2
    fd = _libc.shm_open(_RAW_SHM_NAME, O_CREAT | O_RDWR, 0o600)
    _libc.ftruncate(fd, _RAW_SHM_SIZE)
    buf = mmap.mmap(fd, _RAW_SHM_SIZE)
    _libc.close(fd)
    _RAW_STRUCT.pack_into(buf, 0, 0, 0, 0)
    pub_ready.set()
    sub_ready.wait(15)

    cpu0, t0 = _cpu_us(), time.monotonic()
    sleeper = LoopSleeper(float(hz)) if _HAS_SHMBRIDGE else None
    ver = 0
    for _ in range(n):
        if sleeper:
            sleeper.start()
        ver += 2
        _RAW_STRUCT.pack_into(buf, 0, ver - 1, 0, 0)
        send_ns = now_ns_mono()
        _RAW_STRUCT.pack_into(buf, 0, ver, send_ns, ver)
        if sleeper:
            sleeper.sleep()
        else:
            time.sleep(1.0 / hz)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done_ev.set()
    buf.close()
    _libc.shm_unlink(_RAW_SHM_NAME)
    q.put(("pub", pub_cpu_us, pub_wall_us))


def bench_raw_shm_strategy(hz: float, sleep_s: float,
                            n: int = 200, warmup: int = 30) -> dict:
    total = warmup + n
    pub_ready = mp.Event()
    sub_ready = mp.Event()
    done_ev = mp.Event()
    q: mp.Queue = mp.Queue()

    pub_proc = mp.Process(target=_raw_shm_pub_rate,
                          args=(hz, total, pub_ready, sub_ready, done_ev, q), daemon=True)
    sub_proc = mp.Process(target=_raw_shm_sub_rate,
                          args=(sleep_s, n, warmup, pub_ready, sub_ready, done_ev, q), daemon=True)
    pub_proc.start()
    sub_proc.start()
    pub_proc.join(total / hz + 15)
    sub_proc.join(10)

    pub_cpu_us = pub_wall_us = 0.0
    latencies: list[float] = []
    sub_cpu_us = sub_wall_us = 0.0
    for _ in range(2):
        item = q.get(timeout=10)
        if item[0] == "pub":
            _, pub_cpu_us, pub_wall_us = item
        else:
            _, latencies, sub_cpu_us, sub_wall_us = item

    return {
        **_stats(latencies),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Unix domain socket DGRAM (blocking)
# ══════════════════════════════════════════════════════════════════════════════

_UDS_PATH = "/tmp/bench_rate_uds.sock"


def _uds_sub_rate(n: int, warmup: int, sub_ready: mp.Event,
                  done_ev: mp.Event, q: mp.Queue) -> None:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(_UDS_PATH)
    s.settimeout(1.0)
    sub_ready.set()

    cpu0, t0 = _cpu_us(), time.monotonic()
    latencies: list[float] = []
    received = 0
    while received < warmup + n and not done_ev.is_set():
        try:
            data = s.recv(8)
        except TimeoutError:
            continue
        recv_ns = now_ns_mono()
        lat = (recv_ns - struct.unpack("q", data)[0]) / 1e3
        received += 1
        if received > warmup and 0 < lat < _MAX_VALID_US:
            latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    s.close()
    q.put(("sub", latencies, cpu_us, wall_us))


def _uds_pub_rate(hz: float, n: int, sub_ready: mp.Event,
                  done_ev: mp.Event, q: mp.Queue) -> None:
    sub_ready.wait(15)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sleeper = LoopSleeper(float(hz)) if _HAS_SHMBRIDGE else None

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(n):
        if sleeper:
            sleeper.start()
        s.sendto(struct.pack("q", now_ns_mono()), _UDS_PATH)
        if sleeper:
            sleeper.sleep()
        else:
            time.sleep(1.0 / hz)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done_ev.set()
    s.close()
    q.put(("pub", pub_cpu_us, pub_wall_us))


def bench_uds_at_rate(hz: float, n: int = 200, warmup: int = 30) -> dict:
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_PATH)
    total = warmup + n
    sub_ready = mp.Event()
    done_ev = mp.Event()
    q: mp.Queue = mp.Queue()

    sub_proc = mp.Process(target=_uds_sub_rate,
                          args=(n, warmup, sub_ready, done_ev, q), daemon=True)
    pub_proc = mp.Process(target=_uds_pub_rate,
                          args=(hz, total, sub_ready, done_ev, q), daemon=True)
    sub_proc.start()
    pub_proc.start()
    pub_proc.join(total / hz + 15)
    sub_proc.join(10)

    pub_cpu_us = pub_wall_us = 0.0
    latencies: list[float] = []
    sub_cpu_us = sub_wall_us = 0.0
    for _ in range(2):
        item = q.get(timeout=10)
        if item[0] == "pub":
            _, pub_cpu_us, pub_wall_us = item
        else:
            _, latencies, sub_cpu_us, sub_wall_us = item

    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_PATH)
    return {
        **_stats(latencies),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# TCP loopback (blocking, TCP_NODELAY)
# ══════════════════════════════════════════════════════════════════════════════

_TCP_PORT = 59_323


def _tcp_sub_rate(n: int, warmup: int, sub_ready: mp.Event,
                  done_ev: mp.Event, q: mp.Queue) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", _TCP_PORT))
    srv.listen(1)
    sub_ready.set()
    conn, _ = srv.accept()
    conn.settimeout(1.0)

    cpu0, t0 = _cpu_us(), time.monotonic()
    latencies: list[float] = []
    buf = b""
    received = 0
    while received < warmup + n and not done_ev.is_set():
        try:
            chunk = conn.recv(128)
        except TimeoutError:
            continue
        if not chunk:
            break
        buf += chunk
        while len(buf) >= 8:
            recv_ns = now_ns_mono()
            lat = (recv_ns - struct.unpack("q", buf[:8])[0]) / 1e3
            buf = buf[8:]
            received += 1
            if received > warmup and 0 < lat < _MAX_VALID_US:
                latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    conn.close()
    srv.close()
    q.put(("sub", latencies, cpu_us, wall_us))


def _tcp_pub_rate(hz: float, n: int, sub_ready: mp.Event,
                  done_ev: mp.Event, q: mp.Queue) -> None:
    sub_ready.wait(15)
    time.sleep(0.02)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect(("127.0.0.1", _TCP_PORT))
    sleeper = LoopSleeper(float(hz)) if _HAS_SHMBRIDGE else None

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(n):
        if sleeper:
            sleeper.start()
        s.sendall(struct.pack("q", now_ns_mono()))
        if sleeper:
            sleeper.sleep()
        else:
            time.sleep(1.0 / hz)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done_ev.set()
    s.close()
    q.put(("pub", pub_cpu_us, pub_wall_us))


def bench_tcp_at_rate(hz: float, n: int = 200, warmup: int = 30) -> dict:
    total = warmup + n
    sub_ready = mp.Event()
    done_ev = mp.Event()
    q: mp.Queue = mp.Queue()

    sub_proc = mp.Process(target=_tcp_sub_rate,
                          args=(n, warmup, sub_ready, done_ev, q), daemon=True)
    pub_proc = mp.Process(target=_tcp_pub_rate,
                          args=(hz, total, sub_ready, done_ev, q), daemon=True)
    sub_proc.start()
    pub_proc.start()
    pub_proc.join(total / hz + 15)
    sub_proc.join(10)

    pub_cpu_us = pub_wall_us = 0.0
    latencies: list[float] = []
    sub_cpu_us = sub_wall_us = 0.0
    for _ in range(2):
        item = q.get(timeout=10)
        if item[0] == "pub":
            _, pub_cpu_us, pub_wall_us = item
        else:
            _, latencies, sub_cpu_us, sub_wall_us = item

    return {
        **_stats(latencies),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# mp.Pipe (blocking poll + recv)
# ══════════════════════════════════════════════════════════════════════════════


def _pipe_sub_rate(conn_r: mp.connection.Connection, n: int, warmup: int,
                   sub_ready: mp.Event, done_ev: mp.Event,
                   q: mp.Queue) -> None:
    sub_ready.set()
    cpu0, t0 = _cpu_us(), time.monotonic()
    latencies: list[float] = []
    received = 0
    while received < warmup + n and not done_ev.is_set():
        if not conn_r.poll(1.0):
            continue
        recv_ns = now_ns_mono()
        send_ns = conn_r.recv()
        received += 1
        lat = (recv_ns - send_ns) / 1e3
        if received > warmup and 0 < lat < _MAX_VALID_US:
            latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    conn_r.close()
    q.put(("sub", latencies, cpu_us, wall_us))


def _pipe_pub_rate(conn_w: mp.connection.Connection, hz: float, n: int,
                   sub_ready: mp.Event, done_ev: mp.Event,
                   q: mp.Queue) -> None:
    sub_ready.wait(15)
    sleeper = LoopSleeper(float(hz)) if _HAS_SHMBRIDGE else None
    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(n):
        if sleeper:
            sleeper.start()
        conn_w.send(now_ns_mono())
        if sleeper:
            sleeper.sleep()
        else:
            time.sleep(1.0 / hz)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done_ev.set()
    conn_w.close()
    q.put(("pub", pub_cpu_us, pub_wall_us))


def bench_pipe_at_rate(hz: float, n: int = 200, warmup: int = 30) -> dict:
    total = warmup + n
    conn_r, conn_w = mp.Pipe(duplex=False)
    sub_ready = mp.Event()
    done_ev = mp.Event()
    q: mp.Queue = mp.Queue()

    sub_proc = mp.Process(target=_pipe_sub_rate,
                          args=(conn_r, n, warmup, sub_ready, done_ev, q), daemon=True)
    pub_proc = mp.Process(target=_pipe_pub_rate,
                          args=(conn_w, hz, total, sub_ready, done_ev, q), daemon=True)
    sub_proc.start()
    pub_proc.start()
    conn_r.close()
    conn_w.close()
    pub_proc.join(total / hz + 15)
    sub_proc.join(10)

    pub_cpu_us = pub_wall_us = 0.0
    latencies: list[float] = []
    sub_cpu_us = sub_wall_us = 0.0
    for _ in range(2):
        item = q.get(timeout=10)
        if item[0] == "pub":
            _, pub_cpu_us, pub_wall_us = item
        else:
            _, latencies, sub_cpu_us, sub_wall_us = item

    return {
        **_stats(latencies),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# ZMQ ipc:// (blocking, cross-process)
# ══════════════════════════════════════════════════════════════════════════════

_ZMQ_IPC = "/tmp/bench_rate_zmq.ipc"


def _zmq_sub_rate(n: int, warmup: int, pub_ready: mp.Event,
                  done_ev: mp.Event, q: mp.Queue) -> None:
    pub_ready.wait(15)
    ctx = zmq.Context()
    sub = ctx.socket(zmq.PULL)
    sub.connect(f"ipc://{_ZMQ_IPC}")
    sub.setsockopt(zmq.RCVTIMEO, 1000)

    cpu0, t0 = _cpu_us(), time.monotonic()
    latencies: list[float] = []
    received = 0
    while received < warmup + n and not done_ev.is_set():
        try:
            data = sub.recv()
        except zmq.Again:
            continue
        recv_ns = now_ns_mono()
        lat = (recv_ns - struct.unpack("q", data)[0]) / 1e3
        received += 1
        if received > warmup and 0 < lat < _MAX_VALID_US:
            latencies.append(lat)

    cpu_us = _cpu_us() - cpu0
    wall_us = (time.monotonic() - t0) * 1e6
    sub.close()
    ctx.term()
    q.put(("sub", latencies, cpu_us, wall_us))


def _zmq_pub_rate(hz: float, n: int, pub_ready: mp.Event,
                  done_ev: mp.Event, q: mp.Queue) -> None:
    ctx = zmq.Context()
    push = ctx.socket(zmq.PUSH)
    push.bind(f"ipc://{_ZMQ_IPC}")
    pub_ready.set()
    time.sleep(0.08)  # allow subscriber to connect
    sleeper = LoopSleeper(float(hz)) if _HAS_SHMBRIDGE else None

    cpu0, t0 = _cpu_us(), time.monotonic()
    for _ in range(n):
        if sleeper:
            sleeper.start()
        push.send(struct.pack("q", now_ns_mono()))
        if sleeper:
            sleeper.sleep()
        else:
            time.sleep(1.0 / hz)
    pub_cpu_us = _cpu_us() - cpu0
    pub_wall_us = (time.monotonic() - t0) * 1e6

    done_ev.set()
    push.close()
    ctx.term()
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_ZMQ_IPC)
    q.put(("pub", pub_cpu_us, pub_wall_us))


def bench_zmq_at_rate(hz: float, n: int = 200, warmup: int = 30) -> dict:
    if not _HAS_ZMQ:
        return {"error": "pyzmq not installed"}
    total = warmup + n
    pub_ready = mp.Event()
    done_ev = mp.Event()
    q: mp.Queue = mp.Queue()

    sub_proc = mp.Process(target=_zmq_sub_rate,
                          args=(n, warmup, pub_ready, done_ev, q), daemon=True)
    pub_proc = mp.Process(target=_zmq_pub_rate,
                          args=(hz, total, pub_ready, done_ev, q), daemon=True)
    sub_proc.start()
    pub_proc.start()
    pub_proc.join(total / hz + 15)
    sub_proc.join(10)

    pub_cpu_us = pub_wall_us = 0.0
    latencies: list[float] = []
    sub_cpu_us = sub_wall_us = 0.0
    for _ in range(2):
        item = q.get(timeout=10)
        if item[0] == "pub":
            _, pub_cpu_us, pub_wall_us = item
        else:
            _, latencies, sub_cpu_us, sub_wall_us = item

    return {
        **_stats(latencies),
        "sub_cpu_pct": sub_cpu_us / sub_wall_us * 100 if sub_wall_us else 0,
        "pub_cpu_pct": pub_cpu_us / pub_wall_us * 100 if pub_wall_us else 0,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Main runner
# ══════════════════════════════════════════════════════════════════════════════

# (sleep_s, label, relevant_rates_hz)
SHM_STRATEGIES = [
    (0.0,    "spin",   {30, 100, 1000}),
    (100e-6, "p100us", {30, 100, 1000}),
    (500e-6, "p500us", {30, 100, 1000}),
    (1e-3,   "p1ms",   {30, 100, 1000}),
    (5e-3,   "p5ms",   {30, 100}),       # at 1000 Hz this misses messages
]


def run_hz(hz: float, n: int, warmup: int) -> dict[str, Any]:
    period_ms = 1000 / hz
    results: dict[str, Any] = {}

    # shmbridge strategies
    for sleep_s, label, rates in SHM_STRATEGIES:
        if hz not in rates:
            continue
        key = f"shm_{label}"
        print(f"    shm/{label:8} @ {hz:4.0f} Hz ...", end="", flush=True)
        r = bench_shmbridge_strategy(hz, sleep_s, n=n, warmup=warmup)
        results[key] = {"label": f"shmbridge/{label}", "sleep_ms": sleep_s * 1e3,
                        "period_ms": period_ms, **r}
        tag = " ⚠ sleep > period" if sleep_s * 1e3 > period_ms * 0.8 else ""
        if "error" in r:
            print(f"\r    shm/{label:8} @ {hz:4.0f} Hz  ERROR: {r['error']}")
        else:
            print(
                f"\r    shm/{label:8} @ {hz:4.0f} Hz  "
                f"p50={_fmt(r['p50'])}µs  p99={_fmt(r['p99'])}µs  "
                f"sub={_fmt(r.get('sub_cpu_pct', 0), 0)}%{tag}"
            )

    # raw POSIX shm (spin only — Python, no LoopSleeper in subprocess)
    print(f"    rawshm/spin @ {hz:4.0f} Hz ...", end="", flush=True)
    r = bench_raw_shm_strategy(hz, 0.0, n=n, warmup=warmup)
    results["rawshm_spin"] = {"label": "raw POSIX shm/spin", "sleep_ms": 0,
                               "period_ms": period_ms, **r}
    if "error" not in r:
        print(
            f"\r    rawshm/spin @ {hz:4.0f} Hz  "
            f"p50={_fmt(r['p50'])}µs  p99={_fmt(r['p99'])}µs  "
            f"sub={_fmt(r.get('sub_cpu_pct', 0), 0)}%"
        )

    # blocking mechanisms
    for name, fn in [("uds", bench_uds_at_rate),
                     ("tcp", bench_tcp_at_rate),
                     ("pipe", bench_pipe_at_rate),
                     ("zmq_ipc", bench_zmq_at_rate)]:
        print(f"    {name:12} @ {hz:4.0f} Hz ...", end="", flush=True)
        r = fn(hz=hz, n=n, warmup=warmup)
        labels = {
            "uds":     "UDS DGRAM (blocking)",
            "tcp":     "TCP loopback (blocking)",
            "pipe":    "mp.Pipe (blocking)",
            "zmq_ipc": "ZMQ ipc:// (blocking)",
        }
        results[name] = {"label": labels[name], "sleep_ms": None,
                         "period_ms": period_ms, **r}
        if "error" in r:
            print(f"\r    {name:12} @ {hz:4.0f} Hz  ERROR: {r['error']}")
        else:
            print(
                f"\r    {name:12} @ {hz:4.0f} Hz  "
                f"p50={_fmt(r['p50'])}µs  p99={_fmt(r['p99'])}µs  "
                f"sub={_fmt(r.get('sub_cpu_pct', 0), 0)}%"
            )

    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", metavar="PATH", default=None)
    parser.add_argument("--quick", action="store_true",
                        help="n=100, warmup=15; faster but noisier")
    args = parser.parse_args()

    n = 100 if args.quick else 250
    warmup = 15 if args.quick else 40

    print(f"\nIPC Rates benchmark  (n={n}/rate, warmup={warmup})\n")
    print("Legend: p50/p99 = one-way detection latency, sub% = subscriber CPU of wall time\n")

    out: dict[str, Any] = {"n": n, "warmup": warmup, "rates": {}}

    for hz in RATES_HZ:
        print(f"\n── {hz} Hz  (period={1000/hz:.1f} ms) ──────────────────────────────────")
        out["rates"][str(hz)] = run_hz(hz, n=n, warmup=warmup)

    # Summary table
    print(f"\n{'':30}  {'':6}  {'30 Hz':>16}  {'100 Hz':>16}  {'1000 Hz':>16}")
    print(f"{'Mechanism / strategy':30}  {'sub%':6}  {'p50µs   p99µs':>16}  {'p50µs   p99µs':>16}  {'p50µs   p99µs':>16}")
    print("  " + "─" * 100)

    all_keys = []
    for k in out["rates"].get("30", {}):
        if k not in all_keys:
            all_keys.append(k)
    for k in out["rates"].get("1000", {}):
        if k not in all_keys:
            all_keys.append(k)

    for key in all_keys:
        cols = []
        sub_pct = "—"
        label = key
        for hz in RATES_HZ:
            r = out["rates"].get(str(hz), {}).get(key, {})
            label = r.get("label", key)
            if "p50" not in r:
                cols.append("  —         —   ")
            else:
                cols.append(f"  {_fmt(r['p50']):>6}  {_fmt(r['p99']):>7}")
                sub_pct = f"{_fmt(r.get('sub_cpu_pct', 0), 0):>4}%"
        print(f"  {label:30}  {sub_pct:6}  {''.join(cols)}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nSaved {args.json}")


if __name__ == "__main__":
    mp.set_start_method("forkserver", force=True)
    main()
