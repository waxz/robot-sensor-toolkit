#!/usr/bin/env python3
"""
bench_ipc_latency.py — Publish-subscribe round-trip latency comparison.

Mechanisms benchmarked:
  1. shmbridge (seqlock + POSIX shm, C++ extension)
  2. Raw POSIX shm  (mmap + ctypes spinlock via atomic-like CAS)
  3. ZMQ PUB/SUB   (inproc transport, single-process)
  4. ZMQ PUSH/PULL (inproc, lower-overhead than pub/sub for 1:1)
  5. Unix domain socket (SOCK_DGRAM, sendto/recvfrom)
  6. Anonymous pipe  (os.pipe, write/read)
  7. multiprocessing.Queue  (mp.Queue via Pipe)

Measurement: publisher stamps now_ns_mono() into the message; subscriber
reads it back and computes one-way latency.  For mechanisms without
shared memory (pipes, sockets, ZMQ), the publisher sends the timestamp
inside the payload and the subscriber measures receive_time - send_time.
For shm-based mechanisms, the timestamp is written into the shm slot and
the subscriber polls for a new sequence number.

All tests are single-publisher / single-subscriber, same host.
Cross-process tests use multiprocessing.Process; inproc tests use threads.

Outputs a JSON results file and a human-readable table.
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


# ── stats helpers ─────────────────────────────────────────────────────────────


def _pct(data: list[float], p: float) -> float:
    s = sorted(data)
    return s[max(0, min(len(s) - 1, int(len(s) * p / 100)))]


def _stats(data: list[float]) -> dict:
    return {
        "min": min(data),
        "p50": _pct(data, 50),
        "p95": _pct(data, 95),
        "p99": _pct(data, 99),
        "max": max(data),
        "mean": statistics.mean(data),
        "stdev": statistics.stdev(data) if len(data) > 1 else 0.0,
        "n": len(data),
    }


def _fmt(v: float, d: int = 1) -> str:
    return f"{v:.{d}f}" if v is not None else "—"


# ══════════════════════════════════════════════════════════════════════════════
# 1. shmbridge (seqlock POSIX shm, C++ extension)
# ══════════════════════════════════════════════════════════════════════════════

_SHM_BRIDGE = "/bench_ipc_bridge"


def _shmbridge_sub(ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    sub = ShmSubscriber(_SHM_BRIDGE, 1)
    sub.attach(10_000)
    ready.set()
    latencies: list[float] = []
    last_step = -1
    while len(latencies) < n and not done.is_set():
        st = sub.read_state_spin(0, 256)
        if st is None or st.step == last_step:
            continue
        recv_ns = now_ns_mono()
        send_ns = int(st.sim_time * 1e3)  # stored as µs → convert to ns
        latencies.append((recv_ns - send_ns) / 1e3)  # µs
        last_step = st.step
    sub.detach()
    q.put(latencies)


def bench_shmbridge(n: int = 500, warmup: int = 50) -> dict:
    if not _HAS_SHMBRIDGE:
        return {"error": "shmbridge C++ extension not available"}
    pub = ShmPublisher(_SHM_BRIDGE, 1, 1, 0)
    pub.open()
    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_shmbridge_sub, args=(ready, done, q, n + warmup), daemon=True
    )
    proc.start()
    ready.wait(10)

    st = _RS()
    for i in range(warmup + n):
        send_ns = now_ns_mono()
        st.step = i
        st.sim_time = send_ns / 1e3  # store as µs
        pub.write_state(0, st)
        time.sleep(0)  # voluntary yield
        time.sleep(1e-4)  # ~100 µs pacing to let sub catch up

    done.set()
    proc.join(5)
    sub_lats: list[float] = q.get(timeout=5) if not q.empty() else []
    pub.close()
    # use subscriber-measured latencies (warmup already trimmed by n limit)
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 2. Raw POSIX shm (mmap + struct, Python spinlock)
#    Layout: [seq_lo: uint64][send_ns: int64][seq_hi: uint64]
#    Writer increments seq_lo (odd while writing), writes send_ns, increments
#    seq_hi (matches seq_lo → read complete).  Reader spins on seq.
# ══════════════════════════════════════════════════════════════════════════════

# Raw shm layout (64 bytes, one cache line):
#   offset  0: uint64 version counter (odd = writer active, even = stable)
#   offset  8: int64  send_ns (timestamp written BEFORE version committed)
#   offset 16: uint64 version echo (reader checks version_lo == version_hi)
# Protocol: writer increments version_lo (→ odd), writes send_ns,
#            increments version_lo again (→ even, matches version_hi).
# Reader spins until version_lo == version_hi and both are even and > last.
# A sanity check on latency discards readings > 100 ms (partial-write artifact).

_RAW_SHM_SIZE = 64
_RAW_STRUCT = struct.Struct("=QqQ")  # ver_lo, send_ns, ver_hi  (24 bytes)

_libc = ctypes.CDLL("libc.so.6", use_errno=True)
_libc.shm_open.restype = ctypes.c_int
_libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
_libc.shm_unlink.restype = ctypes.c_int
_libc.ftruncate.restype = ctypes.c_int
_libc.close.restype = ctypes.c_int

_MAX_VALID_US = 100_000.0  # discard readings > 100 ms (clock artefact)


def _raw_shm_sub(
    shm_name: bytes, ready: mp.Event, done: mp.Event, q: mp.Queue, n: int
) -> None:
    O_RDONLY = 0
    fd = _libc.shm_open(shm_name, O_RDONLY, 0)
    buf = mmap.mmap(fd, _RAW_SHM_SIZE, access=mmap.ACCESS_READ)
    _libc.close(fd)
    ready.set()
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
    buf.close()
    q.put(latencies)


def bench_raw_posix_shm(n: int = 500, warmup: int = 50) -> dict:
    shm_name = b"/bench_ipc_rawshm"
    O_CREAT, O_RDWR = 0o100, 0o2
    fd = _libc.shm_open(shm_name, O_CREAT | O_RDWR, 0o600)
    if fd < 0:
        return {"error": f"shm_open failed: {ctypes.get_errno()}"}
    _libc.ftruncate(fd, _RAW_SHM_SIZE)
    buf = mmap.mmap(fd, _RAW_SHM_SIZE)
    _libc.close(fd)
    _RAW_STRUCT.pack_into(buf, 0, 0, 0, 0)

    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_raw_shm_sub, args=(shm_name, ready, done, q, n + warmup), daemon=True
    )
    proc.start()
    ready.wait(10)

    ver = 0
    for _i in range(warmup + n):
        ver += 2  # next even version
        _RAW_STRUCT.pack_into(buf, 0, ver - 1, 0, 0)  # mark in-progress (odd)
        send_ns = now_ns_mono()
        _RAW_STRUCT.pack_into(buf, 0, ver, send_ns, ver)  # commit (even)
        time.sleep(0)
        time.sleep(1e-4)

    done.set()
    proc.join(5)
    buf.close()
    _libc.shm_unlink(shm_name)
    sub_lats: list[float] = q.get(timeout=5) if not q.empty() else []
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 3. ZMQ PUB/SUB  (inproc — both sides in same process, threads)
# ══════════════════════════════════════════════════════════════════════════════


def bench_zmq_pubsub(n: int = 500, warmup: int = 50) -> dict:
    if not _HAS_ZMQ:
        return {"error": "pyzmq not installed"}
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    sub = ctx.socket(zmq.SUB)
    pub.bind("inproc://bench_pubsub")
    sub.connect("inproc://bench_pubsub")
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 1000)

    latencies: list[float] = []
    barrier = threading.Barrier(2)

    def _sub():
        barrier.wait()
        received = 0
        while received < warmup + n:
            try:
                data = sub.recv()
            except zmq.Again:
                continue
            recv_ns = now_ns_mono()
            send_ns = struct.unpack("q", data)[0]
            received += 1
            if received > warmup:
                latencies.append((recv_ns - send_ns) / 1e3)

    t = threading.Thread(target=_sub, daemon=True)
    t.start()
    barrier.wait()
    time.sleep(0.01)  # allow sub to connect

    for _ in range(warmup + n):
        send_ns = now_ns_mono()
        pub.send(struct.pack("q", send_ns))
        time.sleep(1e-4)

    t.join(5)
    pub.close()
    sub.close()
    return _stats(latencies) if latencies else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 4. ZMQ PUSH/PULL  (inproc — lower PUB/SUB overhead for 1:1)
# ══════════════════════════════════════════════════════════════════════════════


def bench_zmq_pushpull(n: int = 500, warmup: int = 50) -> dict:
    if not _HAS_ZMQ:
        return {"error": "pyzmq not installed"}
    ctx = zmq.Context.instance()
    push = ctx.socket(zmq.PUSH)
    pull = ctx.socket(zmq.PULL)
    push.bind("inproc://bench_pushpull")
    pull.connect("inproc://bench_pushpull")
    pull.setsockopt(zmq.RCVTIMEO, 1000)

    latencies: list[float] = []
    barrier = threading.Barrier(2)

    def _pull():
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

    t = threading.Thread(target=_pull, daemon=True)
    t.start()
    barrier.wait()
    time.sleep(0.01)

    for _ in range(warmup + n):
        send_ns = now_ns_mono()
        push.send(struct.pack("q", send_ns))
        time.sleep(1e-4)

    t.join(5)
    push.close()
    pull.close()
    return _stats(latencies) if latencies else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 5. ZMQ PAIR over ipc:// (cross-process, Unix domain socket under the hood)
# ══════════════════════════════════════════════════════════════════════════════

_ZMQ_IPC_PATH = "/tmp/bench_ipc_zmq.ipc"


def _zmq_ipc_sub(
    path: str, ready: mp.Event, done: mp.Event, q: mp.Queue, n: int
) -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.PULL)
    sub.connect(f"ipc://{path}")
    sub.setsockopt(zmq.RCVTIMEO, 500)
    ready.set()
    latencies: list[float] = []
    while len(latencies) < n and not done.is_set():
        try:
            data = sub.recv()
        except zmq.Again:
            continue
        recv_ns = now_ns_mono()
        send_ns = struct.unpack("q", data)[0]
        latencies.append((recv_ns - send_ns) / 1e3)
    sub.close()
    ctx.term()
    q.put(latencies)


def bench_zmq_ipc(n: int = 500, warmup: int = 50) -> dict:
    if not _HAS_ZMQ:
        return {"error": "pyzmq not installed"}
    ctx = zmq.Context()
    push = ctx.socket(zmq.PUSH)
    push.bind(f"ipc://{_ZMQ_IPC_PATH}")

    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_zmq_ipc_sub,
        args=(_ZMQ_IPC_PATH, ready, done, q, n + warmup),
        daemon=True,
    )
    proc.start()
    ready.wait(10)
    time.sleep(0.05)  # let PULL connect

    for _i in range(warmup + n):
        send_ns = now_ns_mono()
        push.send(struct.pack("q", send_ns))
        time.sleep(1e-4)

    done.set()
    proc.join(5)
    push.close()
    ctx.term()
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_ZMQ_IPC_PATH)

    sub_lats: list[float] = q.get(timeout=5) if not q.empty() else []
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 6. Unix domain socket  SOCK_DGRAM  (cross-process)
# ══════════════════════════════════════════════════════════════════════════════

_UDS_PATH = "/tmp/bench_ipc_uds.sock"


def _uds_sub(path: str, ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(path)
    s.settimeout(0.5)
    ready.set()
    latencies: list[float] = []
    while len(latencies) < n and not done.is_set():
        try:
            data = s.recv(8)
        except TimeoutError:
            continue
        recv_ns = now_ns_mono()
        send_ns = struct.unpack("q", data)[0]
        latencies.append((recv_ns - send_ns) / 1e3)
    s.close()
    q.put(latencies)


def bench_uds(n: int = 500, warmup: int = 50) -> dict:
    for p in (_UDS_PATH,):
        with contextlib.suppress(FileNotFoundError):
            os.unlink(p)

    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_uds_sub, args=(_UDS_PATH, ready, done, q, n + warmup), daemon=True
    )
    proc.start()
    ready.wait(10)

    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    for _i in range(warmup + n):
        send_ns = now_ns_mono()
        s.sendto(struct.pack("q", send_ns), _UDS_PATH)
        time.sleep(1e-4)

    done.set()
    proc.join(5)
    s.close()
    with contextlib.suppress(FileNotFoundError):
        os.unlink(_UDS_PATH)

    sub_lats: list[float] = q.get(timeout=5) if not q.empty() else []
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 7. Pipe  (multiprocessing.Pipe — duplex=False, cross-process)
# ══════════════════════════════════════════════════════════════════════════════


def _pipe_sub(
    conn_r: mp.connection.Connection,
    ready: mp.Event,
    done: mp.Event,
    q: mp.Queue,
    n: int,
) -> None:
    latencies: list[float] = []
    ready.set()
    while len(latencies) < n and not done.is_set():
        if not conn_r.poll(0.5):
            continue
        recv_ns = now_ns_mono()
        send_ns = conn_r.recv()
        latencies.append((recv_ns - send_ns) / 1e3)
    conn_r.close()
    q.put(latencies)


def bench_pipe(n: int = 500, warmup: int = 50) -> dict:
    conn_r, conn_w = mp.Pipe(duplex=False)
    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_pipe_sub, args=(conn_r, ready, done, q, n + warmup), daemon=True
    )
    proc.start()
    conn_r.close()  # close read end in parent
    ready.wait(10)

    for _i in range(warmup + n):
        send_ns = now_ns_mono()
        conn_w.send(send_ns)
        time.sleep(1e-4)

    done.set()
    proc.join(5)
    conn_w.close()

    sub_lats: list[float] = q.get(timeout=5) if not q.empty() else []
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 8. multiprocessing.Queue  (backed by Pipe)
# ══════════════════════════════════════════════════════════════════════════════


def _mpq_sub(
    mq: mp.Queue, ready: mp.Event, done: mp.Event, out_q: mp.Queue, n: int
) -> None:
    latencies: list[float] = []
    ready.set()
    while len(latencies) < n and not done.is_set():
        try:
            send_ns = mq.get(timeout=0.5)
        except Exception:
            continue
        recv_ns = now_ns_mono()
        latencies.append((recv_ns - send_ns) / 1e3)
    out_q.put(latencies)


def bench_mpqueue(n: int = 500, warmup: int = 50) -> dict:
    mq: mp.Queue = mp.Queue()
    ready, done, out_q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_mpq_sub, args=(mq, ready, done, out_q, n + warmup), daemon=True
    )
    proc.start()
    ready.wait(10)

    for _i in range(warmup + n):
        send_ns = now_ns_mono()
        mq.put(send_ns)
        time.sleep(1e-4)

    done.set()
    proc.join(5)

    sub_lats: list[float] = out_q.get(timeout=5) if not out_q.empty() else []
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# 9. TCP loopback  (127.0.0.1, for comparison)
# ══════════════════════════════════════════════════════════════════════════════


def _tcp_sub(port: int, ready: mp.Event, done: mp.Event, q: mp.Queue, n: int) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(1)
    ready.set()
    conn, _ = s.accept()
    conn.settimeout(0.5)
    latencies: list[float] = []
    buf = b""
    while len(latencies) < n and not done.is_set():
        try:
            chunk = conn.recv(8)
        except TimeoutError:
            continue
        if not chunk:
            break
        buf += chunk
        while len(buf) >= 8:
            recv_ns = now_ns_mono()
            send_ns = struct.unpack("q", buf[:8])[0]
            latencies.append((recv_ns - send_ns) / 1e3)
            buf = buf[8:]
    conn.close()
    s.close()
    q.put(latencies)


def bench_tcp(n: int = 500, warmup: int = 50) -> dict:
    port = 59_321
    ready, done, q = mp.Event(), mp.Event(), mp.Queue()
    proc = mp.Process(
        target=_tcp_sub, args=(port, ready, done, q, n + warmup), daemon=True
    )
    proc.start()
    ready.wait(10)
    time.sleep(0.05)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect(("127.0.0.1", port))

    for _i in range(warmup + n):
        send_ns = now_ns_mono()
        s.sendall(struct.pack("q", send_ns))
        time.sleep(1e-4)

    done.set()
    proc.join(5)
    s.close()

    sub_lats: list[float] = q.get(timeout=5) if not q.empty() else []
    usable = sub_lats[warmup:] if len(sub_lats) > warmup else sub_lats
    return _stats(usable) if usable else {"error": "no samples"}


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

BENCHMARKS = [
    ("shmbridge", "shmbridge (seqlock POSIX shm, C++)", bench_shmbridge),
    ("raw_posix_shm", "Raw POSIX shm (mmap + seqlock, Python)", bench_raw_posix_shm),
    ("zmq_inproc_pub", "ZMQ PUB/SUB inproc (threads)", bench_zmq_pubsub),
    ("zmq_inproc_push", "ZMQ PUSH/PULL inproc (threads)", bench_zmq_pushpull),
    ("zmq_ipc", "ZMQ PUSH/PULL ipc:// (cross-process)", bench_zmq_ipc),
    ("uds", "Unix domain socket DGRAM (cross-proc)", bench_uds),
    ("pipe", "Anonymous pipe (cross-process)", bench_pipe),
    ("mpqueue", "multiprocessing.Queue (cross-proc)", bench_mpqueue),
    ("tcp_loopback", "TCP loopback 127.0.0.1 (cross-proc)", bench_tcp),
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", metavar="PATH", default=None)
    parser.add_argument(
        "--quick", action="store_true", help="Fewer samples (n=200, warmup=30)"
    )
    parser.add_argument(
        "--only", metavar="KEY", nargs="*", help="Run only these benchmark keys"
    )
    args = parser.parse_args()

    n = 200 if args.quick else 500
    warmup = 30 if args.quick else 50

    out: dict[str, Any] = {"n": n, "warmup": warmup, "results": {}}

    print(f"IPC latency benchmark  (n={n}, warmup={warmup})\n")
    print(
        f"  {'Mechanism':45}  {'p50 µs':>8}  {'p95 µs':>8}  {'p99 µs':>8}  {'max µs':>8}"
    )
    print("  " + "─" * 90)

    for key, label, fn in BENCHMARKS:
        if args.only and key not in args.only:
            continue
        print(f"  running {label} …", end="", flush=True)
        try:
            r = fn(n=n, warmup=warmup)
        except Exception as exc:
            r = {"error": str(exc)}
        out["results"][key] = {"label": label, **r}
        if "error" in r:
            print(f"\r  {label:45}  ERROR: {r['error']}")
        else:
            print(
                f"\r  {label:45}  "
                f"{_fmt(r['p50'], 1):>8}  "
                f"{_fmt(r['p95'], 1):>8}  "
                f"{_fmt(r['p99'], 1):>8}  "
                f"{_fmt(r['max'], 1):>8}"
            )

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nSaved {args.json}")


if __name__ == "__main__":
    mp.set_start_method("forkserver", force=True)
    main()
