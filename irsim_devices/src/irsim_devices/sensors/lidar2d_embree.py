"""Embree4-backed 2D LiDAR sensor.

Drop-in replacement for :class:`~irsim_devices.sensors.lidar2d.Lidar2D` that
substitutes the AVX2 scalar kernel with Intel Embree4's BVH ray traversal.

Requires the ``lidar_embree`` native extension (``lidar_embree.*.so`` / ``.pyd``
built from ``irsim_devices/cpp/lidar_embree.cpp``).

The interface is identical to :class:`Lidar2D` — the only difference visible
to the caller is which backend computes the range values.  The sensor can be
used in two modes:

* **scene mode** (recommended): call :meth:`build_embree_scene` once, then
  ``step()`` uses the Embree BVH for the fixed static scene plus a fallback to
  the standard kernel for any segments that arrive dynamically.

* **passthrough mode** (default, drop-in): if the Embree scene has not been
  built, every ``step()`` is forwarded to the parent ``Lidar2D`` path unchanged.
"""

from __future__ import annotations

import importlib
import os
import sys
from math import cos, sin
from typing import Any

import numpy as np

from irsim_devices.sensors.lidar2d import Lidar2D


def _load_lidar_embree():
    """Import lidar_embree from the package root, cpp/ build dir, or sys.path."""
    # 1. irsim_devices package root (where the installed .so lives)
    pkg_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
    # 2. cpp/ build directory (development builds)
    cpp_dir = os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "cpp")
    )
    for d in (pkg_root, cpp_dir):
        if os.path.isdir(d) and d not in sys.path:
            sys.path.insert(0, d)
    return importlib.import_module("lidar_embree")


class EmbreeLidar2D(Lidar2D):
    """2D LiDAR backed by Intel Embree4 BVH ray traversal.

    Inherits all configuration, profiling, noise, and visualisation from
    :class:`~irsim_devices.sensors.lidar2d.Lidar2D`.  The only difference is
    the ray-casting kernel: once :meth:`build_embree_scene` is called with
    the static obstacle segments, ``_step_fast`` routes rays through Embree
    instead of the AVX2 scalar kernel.

    Args:
        use_packet8: If ``True`` (default when Embree supports it) use the
            8-ray SIMD packet path; otherwise scalar.
        *args, **kwargs: Forwarded to :class:`Lidar2D`.

    Example::

        lidar = EmbreeLidar2D(state, obj_id, range_max=30, number=1500)
        lidar.build_embree_scene(static_segs_f32)   # float32 [N,4]
        lidar.step(state)
    """

    def __init__(
        self,
        *args: Any,
        use_packet8: bool = True,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self._embree_scene = None
        self._use_packet8 = use_packet8
        try:
            self._lidar_embree = _load_lidar_embree()
        except ImportError:
            self._lidar_embree = None  # fall back to parent kernel silently

    # ── Public API ─────────────────────────────────────────────────────────────

    def build_embree_scene(
        self, segs: np.ndarray, use_packet8: bool | None = None
    ) -> None:
        """Build the Embree BVH from a static 2D segment array.

        Args:
            segs: float32 ndarray of shape ``[N, 4]`` — columns ``ax ay bx by``.
            use_packet8: Override the instance-level ``use_packet8`` setting.
        """
        if self._lidar_embree is None:
            raise RuntimeError(
                "lidar_embree native module not found. "
                "Build it from irsim_devices/cpp/lidar_embree.cpp."
            )
        segs_f32 = np.ascontiguousarray(segs, dtype=np.float32)
        if segs_f32.ndim != 2 or segs_f32.shape[1] != 4:
            raise ValueError("segs must be float32 [N,4]: ax ay bx by")
        scene = self._lidar_embree.EmbreeScene2D()
        scene.build(segs_f32)
        self._embree_scene = scene
        if use_packet8 is not None:
            self._use_packet8 = use_packet8

    @property
    def embree_ready(self) -> bool:
        """``True`` when the Embree BVH has been built and is ready."""
        return self._embree_scene is not None

    # ── Override _step_fast ──────────────────────────────────────────────────

    def _step_fast(self, ox: float, oy: float, world_theta: float) -> None:
        """Override: use Embree BVH when the scene is built.

        Falls back to the parent AVX2 kernel when the scene is absent.
        """
        if self._embree_scene is None:
            super()._step_fast(ox, oy, world_theta)
            return

        rmax_f = np.float32(self.range_max)
        ox_f = np.float32(ox)
        oy_f = np.float32(oy)
        cw_f = np.float32(cos(world_theta))
        sw_f = np.float32(sin(world_theta))

        # Rotate directions in-place (identical to parent)
        np.multiply(self._local_dir_cos_f32, cw_f, out=self._dir_dx_f32)
        np.multiply(self._local_dir_sin_f32, sw_f, out=self._tmp_f32)
        np.subtract(self._dir_dx_f32, self._tmp_f32, out=self._dir_dx_f32)
        np.multiply(self._local_dir_cos_f32, sw_f, out=self._dir_dy_f32)
        np.multiply(self._local_dir_sin_f32, cw_f, out=self._tmp_f32)
        np.add(self._dir_dy_f32, self._tmp_f32, out=self._dir_dy_f32)

        self._origin_f32[0] = ox_f
        self._origin_f32[1] = oy_f

        cast_fn = (
            self._embree_scene.cast8_inplace
            if self._use_packet8
            else self._embree_scene.cast_inplace
        )
        cast_fn(
            self._origin_f32,
            self._dir_dx_f32,
            self._dir_dy_f32,
            float(rmax_f),
            self._out_ranges_f32,
            self._out_hit_i32,
        )

        if self.noise:
            from irsim_devices.core.random_utils import rng

            self.range_data[:] = self._out_ranges_f32 + rng.normal(
                0, self.std, self.number
            )
        else:
            self.range_data[:] = self._out_ranges_f32
