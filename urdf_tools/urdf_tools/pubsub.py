"""Sensor pub/sub backed by shmbridge shared memory.

Channel mapping (all in one ExtShmBridge segment):
    scan    →  pointcloud channel  (N, 4) float32: [x, y, range, angle]
    imu     →  imu channel
    odom    →  state channel: x, y, heading, vx, vy, omega
    encoder →  encoder channel: 4 wheel slots [ticks, speed (rad/s)] —
               diff-drive robots use slots 0=left, 1=right; 2, 3 are unused.
    cloud3d → secondary ExtShmBridge segment (shm_name + "_3d"):
               pointcloud channel (N, 4) float32: [x, y, z, dist]

Usage
-----
Publisher (sensor simulator):
    with SensorPublisher("/urdf_sensors") as pub:
        pub.publish_scan(LaserScan(...))
        pub.publish_imu(Imu(...))
        pub.publish_odom(Odometry(...))
        pub.publish_encoder(EncoderState(...))
        pub.publish_cloud3d(pts_nx4)   # float32 (N,4): x,y,z,dist

Subscriber (visualizer / controller):
    with SensorSubscriber("/urdf_sensors") as sub:
        sub.try_attach_cloud3d()       # optional; enables read_cloud3d()
        scan    = sub.read_scan()      # LaserScan or None
        imu     = sub.read_imu()       # Imu or None
        odom    = sub.read_odom()      # Odometry or None
        encoder = sub.read_encoder()   # EncoderState or None
        cloud   = sub.read_cloud3d()   # (N,4) float32 or None
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

import numpy as np

try:
    from shmbridge.bridge_ext import ExtShmBridge

    _HAVE_SHM = True
except ImportError:
    _HAVE_SHM = False

SHM_NAME_DEFAULT = "/urdf_tools_sensors"

# ── Message types ─────────────────────────────────────────────────────────────


@dataclass
class LaserScan:
    frame_id: str = "laser"
    stamp: float = 0.0
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = float(2 * math.pi / 1080)
    range_min: float = 0.05
    range_max: float = 30.0
    ranges: list[float] = field(default_factory=list)


@dataclass
class Imu:
    frame_id: str = "imu_link"
    stamp: float = 0.0
    linear_acceleration: list[float] = field(default_factory=lambda: [0.0, 0.0, 9.81])
    angular_velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class Odometry:
    frame_id: str = "odom"
    stamp: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class EncoderState:
    frame_id: str = "encoder"
    stamp: float = 0.0
    # Up to 4 wheel slots. Diff-drive: [left, right, 0, 0].
    ticks: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    speed: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


@dataclass
class PointCloud3D:
    frame_id: str = "lidar3d"
    stamp: float = 0.0
    # (N, 4) float32 array: x, y, z, dist  (world frame)
    points: np.ndarray = field(
        default_factory=lambda: np.empty((0, 4), dtype=np.float32)
    )


# ── Publisher ─────────────────────────────────────────────────────────────────


class SensorPublisher:
    """Publish sensor data over shmbridge shared memory.

    Creates the shm segment on :meth:`open` (or ``__enter__``).
    Only one publisher per shm_name should exist at a time.
    """

    def __init__(self, shm_name: str = SHM_NAME_DEFAULT) -> None:
        if not _HAVE_SHM:
            raise RuntimeError(
                "shmbridge package not found — install it from ../shmbridge/"
            )
        self._shm_name = shm_name
        self._ext = ExtShmBridge(shm_name)
        self._ext3d = ExtShmBridge(shm_name + "_3d")
        self._step = 0

    def open(self) -> None:
        self._ext.open()
        self._ext3d.open()

    def close(self) -> None:
        self._ext.close()
        try:
            self._ext3d.close()
        except Exception:
            pass

    def __enter__(self) -> SensorPublisher:
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def publish_scan(self, scan: LaserScan) -> None:
        """Pack scan ranges into XY points and write to pointcloud channel."""
        n = len(scan.ranges)
        if n == 0:
            return
        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
        rng = np.asarray(scan.ranges, dtype=np.float32)
        pts = np.empty((n, 4), dtype=np.float32)
        pts[:, 0] = rng * np.cos(angles)  # x
        pts[:, 1] = rng * np.sin(angles)  # y
        pts[:, 2] = rng  # range (for reconstruction)
        pts[:, 3] = angles  # angle (for reconstruction)
        self._ext.write_pointcloud(pts, ts=scan.stamp or time.time())

    def publish_imu(self, imu: Imu) -> None:
        acc = imu.linear_acceleration
        gyro = imu.angular_velocity
        self._ext.write_imu(
            acc[0],
            acc[1],
            acc[2],
            gyro[0],
            gyro[1],
            gyro[2],
            ts=imu.stamp or time.time(),
        )

    def publish_encoder(self, enc: EncoderState) -> None:
        """Write per-wheel tick counts and speeds to the encoder channel."""
        ticks = (list(enc.ticks) + [0, 0, 0, 0])[:4]
        speed = (list(enc.speed) + [0.0, 0.0, 0.0, 0.0])[:4]
        self._ext.write_encoder(
            tuple(int(t) for t in ticks),
            tuple(float(s) for s in speed),
            ts=enc.stamp or time.time(),
        )

    def publish_cloud3d(self, pts: np.ndarray, stamp: float = 0.0) -> None:
        """Write 3D point cloud to secondary shm segment.

        Args:
            pts:   float32 ``(N, 4)`` array — columns x, y, z, dist (world frame).
                   Also accepts ``(N, 3)``; a zero dist column is appended.
            stamp: Sensor timestamp in seconds.
        """
        if pts is None or len(pts) == 0:
            return
        pts4 = np.asarray(pts, dtype=np.float32)
        if pts4.ndim == 2 and pts4.shape[1] == 3:
            pts4 = np.column_stack([pts4, np.zeros(len(pts4), dtype=np.float32)])
        if pts4.ndim != 2 or pts4.shape[1] != 4:
            return
        self._ext3d.write_pointcloud(pts4, ts=stamp or time.time())

    def publish_odom(self, odom: Odometry) -> None:
        self._step += 1
        self._ext.write_state(
            odom.x,
            odom.y,
            odom.theta,
            odom.vx,
            odom.vy,
            odom.omega,
            0.0,
            0.0,
            0.0,  # goal fields unused
            self._step,
            odom.stamp or time.time(),
        )


# ── Subscriber ────────────────────────────────────────────────────────────────


class SensorSubscriber:
    """Subscribe to sensor data published over shmbridge shared memory.

    Attaches to an existing segment.  If the publisher hasn't started yet,
    :meth:`attach` polls up to *timeout_ms* milliseconds before raising.
    """

    def __init__(
        self,
        shm_name: str = SHM_NAME_DEFAULT,
        timeout_ms: float = 10_000.0,
    ) -> None:
        if not _HAVE_SHM:
            raise RuntimeError(
                "shmbridge package not found — install it from ../shmbridge/"
            )
        self._shm_name = shm_name
        self._ext = ExtShmBridge(shm_name)
        self._ext3d: ExtShmBridge | None = None
        self._timeout_ms = timeout_ms

    def attach(self) -> None:
        """Attach to the publisher's shm segment, blocking until ready."""
        self._ext.attach(timeout_ms=self._timeout_ms)

    def try_attach_cloud3d(self, timeout_ms: float = 2000.0) -> bool:
        """Attempt to attach to the 3D cloud shm segment.

        Returns True on success, False if the segment is not available
        (e.g. publisher not started or no 3D lidar configured).
        """
        try:
            ext = ExtShmBridge(self._shm_name + "_3d")
            ext.attach(timeout_ms=timeout_ms)
            self._ext3d = ext
            return True
        except Exception:
            self._ext3d = None
            return False

    def detach(self) -> None:
        try:
            self._ext.detach()
        except Exception:
            pass
        if self._ext3d is not None:
            try:
                self._ext3d.detach()
            except Exception:
                pass

    def __enter__(self) -> SensorSubscriber:
        self.attach()
        return self

    def __exit__(self, *_: object) -> None:
        self.detach()

    def read_scan(self) -> LaserScan | None:
        """Return the latest scan, or None if nothing new."""
        pts = self._ext.read_pointcloud()
        if pts is None or len(pts) == 0:
            return None
        hdr = self._ext.read_pointcloud_header()
        stamp = float(hdr.ts) if hdr is not None else 0.0
        rng = pts[:, 2]
        angles = pts[:, 3]
        if len(angles) == 0:
            return None
        inc = float(angles[1] - angles[0]) if len(angles) > 1 else 0.0
        return LaserScan(
            stamp=stamp,
            angle_min=float(angles[0]),
            angle_max=float(angles[-1]),
            angle_increment=inc,
            range_max=float(np.max(rng)) if len(rng) else 30.0,
            ranges=rng.tolist(),
        )

    def read_imu(self) -> Imu | None:
        raw = self._ext.read_imu()
        if raw is None:
            return None
        return Imu(
            stamp=float(raw.ts),
            linear_acceleration=[float(raw.ax), float(raw.ay), float(raw.az)],
            angular_velocity=[float(raw.gx), float(raw.gy), float(raw.gz)],
        )

    def read_odom(self) -> Odometry | None:
        """Read odometry from the state channel."""
        slot = self._ext._state_slot
        if slot is None:
            return None
        s1 = slot.seq
        s = slot.state
        s2 = slot.seq2
        if s1 != s2 or (s1 & 1) or s1 == 0:
            return None
        return Odometry(
            stamp=float(s.sim_time),
            x=float(s.x),
            y=float(s.y),
            theta=float(s.heading),
            vx=float(s.vx),
            vy=float(s.vy),
            omega=float(s.omega),
        )

    def read_encoder(self) -> EncoderState | None:
        """Read per-wheel tick counts and speeds from the encoder channel."""
        raw = self._ext.read_encoder()
        if raw is None:
            return None
        return EncoderState(
            stamp=float(raw.ts),
            ticks=[int(t) for t in raw.ticks],
            speed=[float(s) for s in raw.speed],
        )

    def read_cloud3d(self) -> np.ndarray | None:
        """Return latest 3D cloud as (N, 4) float32 [x, y, z, dist], or None.

        Returns None when :meth:`try_attach_cloud3d` has not been called or
        when no new data is available.
        """
        if self._ext3d is None:
            return None
        pts = self._ext3d.read_pointcloud()
        if pts is None or len(pts) == 0:
            return None
        return np.asarray(pts, dtype=np.float32)

    def read_scan_points(self) -> np.ndarray | None:
        """Return (N, 4) float32 [x, y, range, angle], or None."""
        return self._ext.read_pointcloud()

    def receive_iter(self) -> Iterator[dict[str, object]]:
        """Yield dicts with keys 'scan', 'imu', 'odom', 'encoder' as data arrives."""
        while True:
            scan = self.read_scan()
            imu = self.read_imu()
            odom = self.read_odom()
            encoder = self.read_encoder()
            if scan or imu or odom or encoder:
                yield {"scan": scan, "imu": imu, "odom": odom, "encoder": encoder}
            else:
                time.sleep(0.005)


# ── spin helper ───────────────────────────────────────────────────────────────


def spin(
    sub: SensorSubscriber,
    callbacks: dict[str, Callable],
    *,
    rate_hz: float = 50.0,
    max_iters: int = 0,
) -> None:
    """Poll subscriber at *rate_hz* and dispatch to per-key callbacks.

    Callback keys: ``"scan"``, ``"imu"``, ``"odom"``, ``"encoder"``.
    Set *max_iters* > 0 to stop after that many poll cycles.
    """
    dt = 1.0 / rate_hz
    count = 0
    while True:
        t0 = time.monotonic()
        scan = sub.read_scan() if "scan" in callbacks else None
        imu = sub.read_imu() if "imu" in callbacks else None
        odom = sub.read_odom() if "odom" in callbacks else None
        encoder = sub.read_encoder() if "encoder" in callbacks else None
        if scan and "scan" in callbacks:
            callbacks["scan"](scan)
        if imu and "imu" in callbacks:
            callbacks["imu"](imu)
        if odom and "odom" in callbacks:
            callbacks["odom"](odom)
        if encoder and "encoder" in callbacks:
            callbacks["encoder"](encoder)
        count += 1
        if max_iters and count >= max_iters:
            break
        elapsed = time.monotonic() - t0
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)
