"""3D LiDAR sensor backed by an open3d Embree raycasting scene.

Requires ``pip install irsim-devices[lidar3d]`` (open3d with Embree support).

The sensor delegates every scan to ``scene.cast_3d_lidar(...)`` and requires
that a :class:`~irsim_devices.core.world_model.Scene3DProtocol`-conforming
instance be assigned to the ``scene`` attribute after the sensor is
created::

    lidar = Lidar3D(state, obj_id, profile="vlp16", range_max=30.0)
    lidar.scene = my_scene3d_instance   # set before stepping
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, ClassVar

import numpy as np

if TYPE_CHECKING:
    from irsim_devices.core.world_model import GeometryObject2D as ObjectBase
    from irsim_devices.core.world_model import Scene3DProtocol as Scene3D


class Lidar3D:
    """Simulated 3D LiDAR sensor using open3d Embree BVH ray-casting.

    Wraps a :class:`~irsim_devices.core.world_model.Scene3DProtocol`-conforming
    scene's ``cast_3d_lidar`` method.  Sixteen spinning-LiDAR profiles are
    available via :attr:`PROFILES`:

    ============ ====== ======= ===================== =============================
    Profile      ch     Azimuth Elevation             Sensor
    ============ ====== ======= ===================== =============================
    ``vlp16``    16     1 800   -15 to +15 deg        Velodyne VLP-16 / Puck
    ``vlp32c``   32     1 800   -25 to +15 deg        Velodyne VLP-32C
    ``hdl64e``   64     1 800   -24.9 to +2 deg       Velodyne HDL-64E
    ``vls128``   128    1 800   -25 to +15 deg        Velodyne VLS-128
    ``os0_32``   32     1 024   -45 to +45 deg        Ouster OS0-32
    ``os0_64``   64     1 024   -45 to +45 deg        Ouster OS0-64
    ``os0_128``  128    2 048   -45 to +45 deg        Ouster OS0-128
    ``os1_32``   32     1 024   -22.5 to +22.5 deg    Ouster OS1-32
    ``os1_64``   64     1 024   -22.5 to +22.5 deg    Ouster OS1-64
    ``os1_128``  128    2 048   -22.5 to +22.5 deg    Ouster OS1-128
    ``xt32``     32     1 800   -15 to +15 deg        Hesai Pandar XT32
    ``qt128``    128    3 600   -52.1 to +52.1 deg    Hesai QT128C2X
    ``rs16``     16     1 800   -15 to +15 deg        Robosense RS-LiDAR-16
    ``rs32``     32     1 800   -15 to +15 deg        Robosense RS-LiDAR-32
    ``os64``     64     1 024   -45 to +45 deg        legacy alias → os0_64
    ``os128``    128    2 048   -45 to +45 deg        legacy alias → os0_128
    ============ ====== ======= ===================== =============================

    Args:
        state: Initial [x, y, theta] state of the parent object (unused).
        obj_id: ID of the associated object.
        profile: Sensor beam pattern; see table above (default ``"vlp16"``).
        range_max: Maximum detection range in metres.
        sensor_height: Height of the sensor above the floor plane (m).
        offset: Sensor offset [x, y, z] from the object's XY position (m).
        **kwargs: Ignored extra keyword arguments passed by SensorFactory.

    Attr:
        sensor_type (str): ``"lidar3d"``.
        scene (Scene3D | None): Reference to a
            :class:`~irsim_devices.core.world_model.Scene3DProtocol`-conforming
            scene.  Assign this before stepping; returns empty scan when
            ``None``.
        scan (np.ndarray): Latest scan, shape ``(N, 4)`` — columns
            ``(x, y, z, distance)`` in the world frame.
        parent (ObjectBase | None): Set by the owning object after construction.
    """

    sensor_type: str = "lidar3d"

    # (n_vertical, n_horizontal, elev_min_deg, elev_max_deg)
    PROFILES: ClassVar[dict[str, tuple[int, int, float, float]]] = {
        # ── Velodyne ──────────────────────────────────────────────────────────
        "vlp16": (16, 1800, -15.0, 15.0),  # VLP-16 / Puck
        "vlp32c": (32, 1800, -25.0, 15.0),  # VLP-32C
        "hdl64e": (64, 1800, -24.9, 2.0),  # HDL-64E
        "vls128": (128, 1800, -25.0, 15.0),  # VLS-128 / Alpha Prime
        # ── Ouster OS0  (90° vertical FoV) ────────────────────────────────────
        "os0_32": (32, 1024, -45.0, 45.0),
        "os0_64": (64, 1024, -45.0, 45.0),
        "os0_128": (128, 2048, -45.0, 45.0),
        # ── Ouster OS1  (45° vertical FoV) ────────────────────────────────────
        "os1_32": (32, 1024, -22.5, 22.5),
        "os1_64": (64, 1024, -22.5, 22.5),
        "os1_128": (128, 2048, -22.5, 22.5),
        # ── Hesai ─────────────────────────────────────────────────────────────
        "xt32": (32, 1800, -15.0, 15.0),  # Hesai Pandar XT32
        "qt128": (128, 3600, -52.1, 52.1),  # Hesai QT128C2X
        # ── Robosense ─────────────────────────────────────────────────────────
        "rs16": (16, 1800, -15.0, 15.0),  # RS-LiDAR-16
        "rs32": (32, 1800, -15.0, 15.0),  # RS-LiDAR-32
        # ── Backward-compatibility aliases ────────────────────────────────────
        "os64": (64, 1024, -45.0, 45.0),  # legacy → os0_64
        "os128": (128, 2048, -45.0, 45.0),  # legacy → os0_128
    }

    def __init__(
        self,
        state=None,
        obj_id: int = 0,
        profile: str = "vlp16",
        range_max: float = 50.0,
        sensor_height: float = 0.3,
        offset: list | np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        if profile not in self.PROFILES:
            raise ValueError(
                f"Unknown lidar3d profile {profile!r}. Available: {list(self.PROFILES)}"
            )
        self.obj_id = obj_id
        self.profile = profile
        self.range_max = float(range_max)
        self.sensor_height = float(sensor_height)
        self.offset = np.asarray(
            offset if offset is not None else [0.0, 0.0, 0.0], dtype=float
        )
        self.scene: Scene3D | None = None
        self.scan: np.ndarray = np.empty((0, 4), dtype=np.float32)
        self.parent: ObjectBase | None = None

    def step(self, state: np.ndarray) -> None:
        """Cast a full 3D scan from the current sensor position.

        Does nothing if :attr:`scene` has not been assigned.

        Args:
            state: Current [x, y, theta] state of the parent object (world frame).
        """
        if self.scene is None:
            return

        x = float(state[0])
        y = float(state[1])
        z = self.sensor_height + self.offset[2]
        origin = [x + self.offset[0], y + self.offset[1], z]

        self.scan = self.scene.cast_3d_lidar(origin, self.profile, self.range_max)

    def get_scan(self) -> np.ndarray:
        """Return the latest scan array.

        Returns:
            Array of shape ``(N, 4)`` — columns ``(x, y, z, distance)``.
            Empty ``(0, 4)`` array when no scene is attached or no hits.
        """
        return self.scan
