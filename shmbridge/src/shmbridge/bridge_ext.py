"""
bridge_ext.py - ExtShmBridge: state/cmd plus IMU, encoder, and point-cloud channels.

Thin Python-facing adapter over ``shmbridge._core.ExtShmBridge`` (C++,
ext_core.hpp) -- see :class:`ExtShmBridge` below. All extended channels use
the same seqlock + writer_ts_ns heartbeat protocol as the base bridge, so
liveness checks work uniformly across all data streams.
"""

from __future__ import annotations

from dataclasses import dataclass

try:
    import numpy as np

    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False

from ._core import ExtShmBridge as _CppExtShmBridgeCore  # type: ignore[import]
from ._types import EXT_SHM_NAME_DEFAULT

# Expose RobotState/RobotCmd for convenience (shmbridge._core's pybind11 classes).
from ._core import RobotCmd, RobotState  # noqa: F401  # type: ignore[import]


@dataclass
class ShmTopic:
    """One channel of an :class:`ExtShmBridge` segment, for topic discovery.

    ``alive`` is False until the channel has been written at least once,
    then reflects whether its ``writer_ts_ns`` heartbeat is fresher than the
    ``max_age_ms`` passed to :meth:`ExtShmBridge.list_topics`.
    """

    name: str
    kind: str
    alive: bool
    age_ms: float | None


class ExtShmBridge:
    """Adapts ``_core.ExtShmBridge`` (ext_core.hpp) to a Python-friendly API.

    Every hot-path method (called every sensor tick) gets an explicit
    passthrough here rather than relying on ``__getattr__`` — measured at
    ~90 ns/call of avoidable indirection each, small individually but this
    class exists specifically to shave overhead off a per-tick loop.
    ``__getattr__`` still covers the rarely-called remainder (open/close/
    is_open/is_attached/is_imu_alive/is_pointcloud_alive/list_topics).

    Two methods need real translation, not just a passthrough:

    * ``write_state`` takes flat args here vs. a ``RobotState`` object in
      C++. The obvious adapter — build a ``RobotState()``, set 11
      attributes, then call ``write_state(state)`` — measured *slower* than
      building the object on the C++ side of the boundary from one call
      carrying all 11 values (~2.3us for 11 pybind11 attribute setters vs.
      ~0.2us for the actual seqlock write): ``write_state_fields`` (see
      ext_core.hpp's pybind11 binding) does the latter; that's the version
      used below.
    * ``read_pointcloud`` returns raw bytes in C++ (the extension has no
      numpy build dependency) vs. an (N, 4) ndarray here; wrapped with
      ``np.frombuffer`` to match.

    ``attach``'s timeout also needs translating: the C++ side can't
    distinguish "not found yet" from any other runtime error at the
    language level, so it raises plain RuntimeError; this wraps that as
    TimeoutError to match every other attach()/open() pair in this package.
    A schema/magic mismatch raises ValueError on both sides already
    (pybind11 maps std::invalid_argument to ValueError automatically).
    """

    def __init__(self, shm_name: str = EXT_SHM_NAME_DEFAULT) -> None:
        self._bridge = _CppExtShmBridgeCore(shm_name)

    def __getattr__(self, name: str):
        return getattr(self._bridge, name)

    def __enter__(self) -> ExtShmBridge:
        self._bridge.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self._bridge.close()

    def attach(self, timeout_ms: float = 30_000.0) -> None:
        try:
            self._bridge.attach(timeout_ms)
        except RuntimeError as e:
            raise TimeoutError(str(e)) from e

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
        self._bridge.write_state_fields(
            x, y, heading, vx, vy, omega,
            goal_x, goal_y, goal_dist, step, sim_time,
            reached, collision,
        )

    def read_state(self):
        return self._bridge.read_state()

    def write_cmd(self, linear: float, angular: float) -> None:
        self._bridge.write_cmd(linear, angular)

    def read_cmd(self):
        return self._bridge.read_cmd()

    def write_imu(self, ax, ay, az, gx, gy, gz, mx=0.0, my=0.0, mz=0.0, ts=0.0) -> None:
        self._bridge.write_imu(ax, ay, az, gx, gy, gz, mx, my, mz, ts)

    def read_imu(self):
        return self._bridge.read_imu()

    def write_encoder(self, ticks, speeds, ts: float = 0.0) -> None:
        self._bridge.write_encoder(ticks, speeds, ts)

    def read_encoder(self):
        return self._bridge.read_encoder()

    def write_pointcloud(self, points, ts: float = 0.0) -> None:
        self._bridge.write_pointcloud(points, ts)

    def read_pointcloud(self):
        """Return the latest cloud as an (N, 4) float32 ndarray, or None."""
        if not _HAS_NUMPY:
            raise RuntimeError("numpy required for read_pointcloud()")
        raw = self._bridge.read_pointcloud_bytes()
        if raw is None:
            return None
        return np.frombuffer(raw, dtype=np.float32).reshape(-1, 4)

    def read_pointcloud_header(self):
        return self._bridge.read_pointcloud_header()
