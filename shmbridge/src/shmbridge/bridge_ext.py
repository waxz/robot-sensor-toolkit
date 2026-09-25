"""
bridge_ext.py - Extended ShmBridge with IMU, encoder, and point-cloud channels.

All extended channels use the same seqlock + writer_ts_ns heartbeat protocol
as the base bridge, so liveness checks work uniformly across all data streams.
"""

from __future__ import annotations

import ctypes
import errno
import mmap
import os
import sys
import time

try:
    import numpy as np

    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False

from ._libc import _libc, _libshm, _monotonic_ns
from ._platform import O_CREAT, O_EXCL, O_RDWR
from ._types import (
    EXT_SHM_NAME_DEFAULT,
    EXT_SHM_SIZE,
    MAGIC,
    PC_MAX_POINTS,
    PC_POINT_BYTES,
    SCHEMA_VERSION,
    _ShmEncoder,
    _ShmExtBlock,
    _ShmImu,
    _ShmPcHdr,
)

# Expose RobotState/RobotCmd from the base bridge for convenience
from .bridge import RobotCmd, RobotState, ShmBridge  # noqa: F401


class ExtShmBridge:
    """
    Extended shared-memory bridge with IMU, encoder, and point-cloud support.

    Contains a full base bridge (state + cmd) plus three additional channels:

    - **IMU** (Python→C++): 9-axis IMU (accel + gyro + mag) + timestamp
    - **Encoder** (Python→C++): 4-wheel tick counts + speeds
    - **Point cloud** (Python→C++): up to 65 536 points (x, y, z, intensity)

    Parameters
    ----------
    shm_name : str
        POSIX shm name for the extended segment (default ``/shmbridge_ext_v2``).
    """

    def __init__(self, shm_name: str = EXT_SHM_NAME_DEFAULT) -> None:
        self._name = shm_name.encode()
        self._size = EXT_SHM_SIZE
        self._mm: mmap.mmap | None = None
        self._blk: _ShmExtBlock | None = None
        self._pc_data_mv: memoryview | None = None  # raw point-cloud bytes
        # seqlock counters
        self._state_seq: int = 0
        self._imu_seq: int = 0
        self._encoder_seq: int = 0
        self._pc_seq: int = 0
        # cached slot refs (set in open/attach)
        self._state_slot = None
        self._cmd_slot = None
        self._imu_slot = None
        self._enc_slot = None
        self._pc_slot_ref = None

    # ── lifecycle ─────────────────────────────────────────────────────────

    def open(self) -> None:
        """Create and initialise the extended shm segment."""
        if sys.platform == "win32":
            tag = self._name.decode().lstrip("/")
            self._mm = mmap.mmap(-1, self._size, tagname=tag, access=mmap.ACCESS_WRITE)
            self._mm.seek(0)
            self._mm.write(b"\x00" * self._size)
            self._mm.seek(0)
        else:
            _libshm.shm_unlink(self._name)
            fd = _libshm.shm_open(self._name, O_CREAT | O_RDWR | O_EXCL, 0o666)
            if fd < 0:
                err = ctypes.get_errno()
                raise OSError(err, os.strerror(err), self._name.decode())
            if _libc.ftruncate(fd, self._size) != 0:
                err = ctypes.get_errno()
                os.close(fd)
                raise OSError(err, os.strerror(err))
            self._mm = mmap.mmap(
                fd, self._size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE
            )
            os.close(fd)
            self._mm.write(b"\x00" * self._size)
            self._mm.seek(0)
        self._blk = _ShmExtBlock.from_buffer(self._mm)
        hdr = self._blk.header
        hdr.magic = MAGIC
        hdr.schema_version = SCHEMA_VERSION
        hdr.n_robots = 1
        hdr.ready = 1
        self._cache_slots()

    def attach(self, timeout_ms: float = 30_000.0) -> None:
        """Attach to an existing extended segment; blocks until ready or timeout.

        Safe to call before the publisher has started: retries until the segment
        appears (up to *timeout_ms* ms), then waits for the ready flag.
        Pass timeout_ms=0 for a single non-blocking attempt.
        """
        self.detach()  # release old mapping before (re-)attaching

        if sys.platform == "win32":
            tag = self._name.decode().lstrip("/")
            deadline = time.monotonic() + timeout_ms * 1e-3
            # ACCESS_READ uses OpenFileMappingA which fails cleanly when the
            # mapping doesn't exist yet — use it to poll until publisher starts.
            while True:
                try:
                    test = mmap.mmap(-1, 128, tagname=tag, access=mmap.ACCESS_READ)
                    test.close()
                    break
                except OSError as exc:
                    if time.monotonic() > deadline:
                        raise TimeoutError(
                            f"attach timeout: segment '{tag}' not found "
                            "— is the publisher running?"
                        ) from exc
                    time.sleep(0.05)
            # ACCESS_WRITE uses CreateFileMappingA which opens an existing mapping
            self._mm = mmap.mmap(-1, self._size, tagname=tag, access=mmap.ACCESS_WRITE)
        else:
            deadline = time.monotonic() + timeout_ms * 1e-3
            while True:
                fd = _libshm.shm_open(self._name, O_RDWR, 0o666)
                if fd >= 0:
                    break
                err = ctypes.get_errno()
                if err not in (errno.ENOENT, errno.EACCES):
                    raise OSError(err, os.strerror(err), self._name.decode())
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"attach timeout: segment '{self._name.decode()}' not found "
                        "— is the publisher running?"
                    )
                time.sleep(0.05)
            self._mm = mmap.mmap(
                fd, self._size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE
            )
            os.close(fd)

        self._blk = _ShmExtBlock.from_buffer(self._mm)
        hdr = self._blk.header

        # Wait for ready flag
        deadline = time.monotonic() + timeout_ms * 1e-3
        while not hdr.ready:
            if time.monotonic() > deadline:
                self.detach()
                raise TimeoutError("attach timeout: publisher not ready")
            time.sleep(0.0005)

        if hdr.magic and hdr.magic != MAGIC:
            self.detach()
            raise ValueError(
                f"Schema mismatch: magic=0x{hdr.magic:08X} (expected 0x{MAGIC:08X})"
            )
        self._cache_slots()

    def _cache_slots(self) -> None:
        blk = self._blk
        self._state_slot = blk.state
        self._cmd_slot = blk.cmd
        self._imu_slot = blk.imu
        self._enc_slot = blk.encoder
        self._pc_slot_ref = blk.pc_slot
        # memoryview over the raw point-cloud data region (after the 768 B header)
        self._pc_data_mv = memoryview(self._mm)[
            768 : 768 + PC_MAX_POINTS * PC_POINT_BYTES
        ]

    def close(self) -> None:
        """Unmap and delete the segment."""
        self._state_slot = self._cmd_slot = self._imu_slot = None
        self._enc_slot = self._pc_slot_ref = None
        if self._pc_data_mv is not None:
            self._pc_data_mv.release()
            self._pc_data_mv = None
        self._blk = None
        import gc

        gc.collect()
        if self._mm is not None:
            try:
                self._mm.close()
            except BufferError:
                pass
            self._mm = None
        if sys.platform != "win32":
            _libshm.shm_unlink(self._name)

    def detach(self) -> None:
        """Unmap without unlinking — safe for secondary clients (subscribers)."""
        self._state_slot = self._cmd_slot = self._imu_slot = None
        self._enc_slot = self._pc_slot_ref = None
        if self._pc_data_mv is not None:
            self._pc_data_mv.release()
            self._pc_data_mv = None
        self._blk = None
        import gc

        gc.collect()
        if self._mm is not None:
            try:
                self._mm.close()
            except BufferError:
                pass
            self._mm = None

    def __enter__(self) -> ExtShmBridge:
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ── base state / cmd (delegate slot logic) ────────────────────────────

    def write_state(
        self,
        x,
        y,
        heading,
        vx,
        vy,
        omega,
        goal_x,
        goal_y,
        goal_dist,
        step,
        sim_time,
        reached=False,
        collision=False,
    ) -> None:
        slot = self._state_slot
        s = slot.state
        self._state_seq += 1
        slot.seq = self._state_seq
        s.x = x
        s.y = y
        s.heading = heading
        s.vx = vx
        s.vy = vy
        s.omega = omega
        s.goal_x = goal_x
        s.goal_y = goal_y
        s.goal_dist = goal_dist
        s.step = step
        s.sim_time = sim_time
        s.reached = int(reached)
        s.collision = int(collision)
        self._state_seq += 1
        slot.seq = self._state_seq
        slot.seq2 = self._state_seq
        slot.writer_ts_ns = _monotonic_ns()

    def read_cmd(self):
        slot = self._cmd_slot
        s1 = slot.seq
        linear = slot.cmd.linear
        angular = slot.cmd.angular
        valid = slot.cmd.valid
        s2 = slot.seq2
        if s1 != s2 or (s1 & 1) or not valid:
            return None
        return RobotCmd(linear=float(linear), angular=float(angular), seq=slot.cmd.seq)

    # ── IMU ───────────────────────────────────────────────────────────────

    def write_imu(
        self,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
        mx: float = 0.0,
        my: float = 0.0,
        mz: float = 0.0,
        ts: float = 0.0,
    ) -> None:
        slot = self._imu_slot
        self._imu_seq += 1
        slot.seq = self._imu_seq
        im = slot.imu
        im.ax = ax
        im.ay = ay
        im.az = az
        im.gx = gx
        im.gy = gy
        im.gz = gz
        im.mx = mx
        im.my = my
        im.mz = mz
        im.ts = ts
        self._imu_seq += 1
        slot.seq = self._imu_seq
        slot.seq2 = self._imu_seq
        slot.writer_ts_ns = _monotonic_ns()

    def read_imu(self) -> _ShmImu | None:
        slot = self._imu_slot
        s1 = slot.seq
        im = slot.imu
        s2 = slot.seq2
        if s1 != s2 or (s1 & 1) or s1 == 0:
            return None
        return im

    # ── Encoder ───────────────────────────────────────────────────────────

    def write_encoder(
        self,
        ticks: tuple[int, int, int, int],
        speeds: tuple[float, float, float, float],
        ts: float = 0.0,
    ) -> None:
        slot = self._enc_slot
        self._encoder_seq += 1
        slot.seq = self._encoder_seq
        enc = slot.encoder
        for i in range(4):
            enc.ticks[i] = ticks[i]
            enc.speed[i] = speeds[i]
        enc.ts = ts
        self._encoder_seq += 1
        slot.seq = self._encoder_seq
        slot.seq2 = self._encoder_seq
        slot.writer_ts_ns = _monotonic_ns()

    def read_encoder(self) -> _ShmEncoder | None:
        slot = self._enc_slot
        s1 = slot.seq
        enc = slot.encoder
        s2 = slot.seq2
        if s1 != s2 or (s1 & 1) or s1 == 0:
            return None
        return enc

    # ── Point cloud ───────────────────────────────────────────────────────

    def write_pointcloud(self, points, ts: float = 0.0) -> None:
        """
        Write a point cloud from *points*.

        Parameters
        ----------
        points : array-like, shape (N, 4)
            Each row: [x, y, z, intensity] as float32.
            N must be <= PC_MAX_POINTS (65 536).
        ts : float
            Timestamp (e.g. sim_time) attached to the cloud.
        """
        if not _HAS_NUMPY:
            raise RuntimeError("numpy required for write_pointcloud()")
        arr = np.asarray(points, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(-1, 4)
        n = len(arr)
        if n > PC_MAX_POINTS:
            raise ValueError(f"Too many points: {n} > {PC_MAX_POINTS}")

        ps = self._pc_slot_ref
        self._pc_seq += 1
        ps.seq = self._pc_seq

        # write header
        hdr = ps.hdr
        hdr.seq = self._pc_seq
        hdr.n_points = n
        hdr.max_pts = PC_MAX_POINTS
        hdr.ts = ts

        # write raw point data directly into the mapped region
        byte_len = n * PC_POINT_BYTES
        self._pc_data_mv[:byte_len] = arr.tobytes()

        self._pc_seq += 1
        ps.seq = self._pc_seq
        ps.seq2 = self._pc_seq
        ps.writer_ts_ns = _monotonic_ns()

    def read_pointcloud_header(self) -> _ShmPcHdr | None:
        ps = self._pc_slot_ref
        s1 = ps.seq
        hdr = ps.hdr
        s2 = ps.seq2
        if s1 != s2 or (s1 & 1) or s1 == 0:
            return None
        return hdr

    def read_pointcloud(self):
        """
        Return a copy of the latest point cloud as a numpy array (N, 4),
        or None if no valid cloud is available.
        """
        if not _HAS_NUMPY:
            raise RuntimeError("numpy required for read_pointcloud()")
        ps = self._pc_slot_ref
        s1 = ps.seq
        hdr = ps.hdr
        n = hdr.n_points
        raw = bytes(self._pc_data_mv[: n * PC_POINT_BYTES])
        s2 = ps.seq2
        if s1 != s2 or (s1 & 1) or s1 == 0:
            return None
        return np.frombuffer(raw, dtype=np.float32).reshape(n, 4)

    # ── liveness ──────────────────────────────────────────────────────────

    def is_imu_alive(self, max_age_ms: float = 100.0) -> bool:
        ts = self._imu_slot.writer_ts_ns
        if ts == 0:
            return True
        return (_monotonic_ns() - ts) / 1e6 < max_age_ms

    def is_pointcloud_alive(self, max_age_ms: float = 100.0) -> bool:
        ts = self._pc_slot_ref.writer_ts_ns
        if ts == 0:
            return True
        return (_monotonic_ns() - ts) / 1e6 < max_age_ms
