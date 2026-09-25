"""Embree4-backed 3D LiDAR sensor.

Drop-in replacement for :class:`~irsim_devices.sensors.lidar3d.Lidar3D` that
directly calls Intel Embree4 without going through Open3D, removing the
O3D Python overhead (~100-150 µs per step on a 1M-triangle scene).

Requires the ``lidar_embree`` native extension (``lidar_embree.*.so`` / ``.pyd``
built from ``irsim_devices/cpp/lidar_embree.cpp``).
"""

from __future__ import annotations

import importlib
import os
import sys
from typing import Any

import numpy as np

from irsim_devices.sensors.lidar3d import Lidar3D


def _load_lidar_embree():
    pkg_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
    cpp_dir = os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "cpp")
    )
    for d in (pkg_root, cpp_dir):
        if os.path.isdir(d) and d not in sys.path:
            sys.path.insert(0, d)
    return importlib.import_module("lidar_embree")


class EmbreeLidar3D(Lidar3D):
    """3D LiDAR backed by Intel Embree4 BVH ray traversal.

    Eliminates the Open3D Python overhead (~100-150 µs/step) by calling
    Embree4 directly from C++ via pybind11.  All sensor profiles, geometry,
    and the ``scan`` output format are identical to
    :class:`~irsim_devices.sensors.lidar3d.Lidar3D`.

    The Embree scene is built from a triangle mesh provided via
    :meth:`build_embree_scene`.  The ``scene`` attribute from the parent class
    is *not* used; set ``scene = None`` (default) to avoid Open3D imports.

    Args:
        *args, **kwargs: Forwarded to :class:`Lidar3D`.

    Example::

        lidar = EmbreeLidar3D(state, obj_id, profile="vlp16", range_max=50)
        lidar.build_embree_scene(vertices_f32, triangles_i32)
        lidar.step(state)
        pts = lidar.get_scan()   # float32 [N_hits, 4]
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._embree_scene3d = None
        try:
            self._lidar_embree = _load_lidar_embree()
        except ImportError:
            self._lidar_embree = None

    # ── Public API ─────────────────────────────────────────────────────────────

    def build_embree_scene(
        self,
        vertices: np.ndarray | None = None,
        triangles: np.ndarray | None = None,
        soup: np.ndarray | None = None,
    ) -> None:
        """Build the Embree BVH from a 3D triangle mesh.

        Call with either ``(vertices, triangles)`` or ``soup``.

        Args:
            vertices:  float32 ``[V, 3]`` vertex positions.
            triangles: int32   ``[T, 3]`` vertex-index triangles.
            soup:      float32 ``[T, 3, 3]`` triangle soup (alternative).
        """
        if self._lidar_embree is None:
            raise RuntimeError(
                "lidar_embree native module not found. "
                "Build it from irsim_devices/cpp/lidar_embree.cpp."
            )
        scene = self._lidar_embree.EmbreeScene3D()
        if soup is not None:
            scene.build_soup(np.ascontiguousarray(soup, dtype=np.float32))
        elif vertices is not None and triangles is not None:
            scene.build(
                np.ascontiguousarray(vertices, dtype=np.float32),
                np.ascontiguousarray(triangles, dtype=np.int32),
            )
        else:
            raise ValueError("Provide either (vertices, triangles) or soup.")
        self._embree_scene3d = scene

    @property
    def embree_ready(self) -> bool:
        """``True`` when the Embree BVH has been built."""
        return self._embree_scene3d is not None

    # ── Override step ─────────────────────────────────────────────────────────

    def step(self, state: np.ndarray) -> None:
        """Cast a full 3D scan using the Embree BVH.

        Falls back to the parent Open3D path when the Embree scene is absent.
        """
        if self._embree_scene3d is None:
            super().step(state)
            return

        x = float(state[0])
        y = float(state[1])
        z = self.sensor_height + self.offset[2]
        origin = np.array([x + self.offset[0], y + self.offset[1], z], dtype=np.float32)

        profile = self.PROFILES[self.profile]
        n_vertical, n_horizontal, elev_min, elev_max = profile

        self.scan = self._embree_scene3d.cast_3d_lidar(
            origin,
            n_vertical,
            n_horizontal,
            float(elev_min),
            float(elev_max),
            float(self.range_max),
        )
