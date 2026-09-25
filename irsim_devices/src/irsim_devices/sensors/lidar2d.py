from math import cos, pi, sin
from typing import TYPE_CHECKING, Any, ClassVar

import matplotlib.transforms as mtransforms
import numpy as np
import shapely
import shapely as _shapely
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from shapely import MultiLineString

from irsim_devices.core.geo_utils import (
    ClipTo2Pi,
    geometry_transform,
    transform_point_with_state,
)
from irsim_devices.core.random_utils import rng
from irsim_devices.core.ray_casting_2d import (
    _empty_segments,
    _ray_parameters,
    boundary_segments,
    cast_ray_segments,
    cast_rays,
)

if TYPE_CHECKING:
    from irsim_devices.core.world_model import GeometryObject2D as ObjectBase


class Lidar2D:
    """
    Simulates a 2D Lidar sensor for detecting obstacles in the environment.

    A set of named product profiles matching widely available 2D LiDAR hardware
    is available via :attr:`PROFILES` and :meth:`from_profile`::

        sensor = Lidar2D.from_profile("rplidar_a1m8", state=robot.state, obj_id=robot.id)

    Args:
        state (np.ndarray): Initial state of the sensor.
        obj_id (int): ID of the associated object.
        range_min (float): Minimum detection range.
        range_max (float): Maximum detection range.
        angle_range (float): Total angle range of the sensor.
        number (int): Number of laser beams.
        scan_time (float): Time taken for one complete scan.
        noise (bool): Whether noise is added to measurements.
        std (float): Standard deviation for range noise.
        angle_std (float): Standard deviation for angle noise.
        offset (list): Offset of the sensor from the object's position.
        alpha (float): Transparency for plotting.
        has_velocity (bool): Whether the sensor measures velocity.
        **kwargs: Additional arguments.
            color (str): Color of the sensor.

    Attr:
        - sensor_type (str): Type of sensor ("lidar2d"). Default is "lidar2d".
        - range_min (float): Minimum detection range in meters. Default is 0.
        - range_max (float): Maximum detection range in meters. Default is 10.
        - angle_range (float): Total angle range of the sensor in radians. Default is pi. Clipped to [0, 2*pi].
        - angle_min (float): Starting angle of the sensor's scan relative to the forward direction in radians. Calculated as -angle_range / 2.
        - angle_max (float): Ending angle of the sensor's scan relative to the forward direction in radians. Calculated as angle_range / 2.
        - angle_inc (float): Angular increment between each laser beam in radians. Calculated as angle_range / (number - 1) when multiple beams are used.
          A single-beam sensor has no increment and points straight ahead.
        - number (int): Number of laser beams. Default is 100.
        - scan_time (float): Time taken to complete one full scan in seconds. Default is 0.1.
        - noise (bool): Whether to add noise to the measurements. Default is False.
        - std (float): Standard deviation for range noise in meters. Effective only if `noise` is True. Default is 0.2.
        - angle_std (float): Standard deviation for angle noise in radians. Effective only if `noise` is True. Default is 0.02.
        - offset (np.ndarray): Offset of the sensor relative to the object's position, formatted as [x, y, theta]. Default is [0, 0, 0].
        - lidar_origin (np.ndarray): Origin position of the Lidar sensor, considering offset and the object's state.
        - alpha (float): Transparency level for plotting the laser beams. Default is 0.3.
        - has_velocity (bool): Whether the sensor measures the velocity of detected points. Default is False.
        - velocity (np.ndarray): Velocity data for each laser beam, formatted as (2, number) array. Effective only if `has_velocity` is True. Initialized to zeros.
        - time_inc (float): Time increment for each scan, simulating the sensor's time resolution. Default is 5e-4.
        - range_data (np.ndarray): Array storing range data for each laser beam. Initialized to `range_max` for all beams.
        - angle_list (np.ndarray): Array of angles corresponding to each laser beam, distributed linearly from `angle_min` to `angle_max`.
        - color (str): Color of the sensor's representation in visualizations. Default is "r" (red).
        - obj_id (int): ID of the associated object, used to differentiate between multiple sensors or objects in the environment. Default is 0.
        - plot_patch_list (list): List storing plot patches (e.g., line collections) for visualization purposes.
        - plot_line_list (list): List storing plot lines for visualization purposes.
        - plot_text_list (list): List storing plot text elements for visualization purposes.
    """

    # Named product profiles: {name: {range_min, range_max, angle_range, number, scan_time, std, description}}
    PROFILES: ClassVar[dict[str, dict[str, Any]]] = {
        # Slamtec RPLiDAR
        "rplidar_a1m8": {
            "range_min": 0.15,
            "range_max": 12.0,
            "angle_range": 2 * pi,
            "number": 360,
            "scan_time": 0.182,
            "std": 0.03,
            "description": "Slamtec RPLiDAR A1M8 — 360° / 12 m / 5.5 Hz",
        },
        "rplidar_a3": {
            "range_min": 0.1,
            "range_max": 25.0,
            "angle_range": 2 * pi,
            "number": 720,
            "scan_time": 0.1,
            "std": 0.02,
            "description": "Slamtec RPLiDAR A3 — 360° / 25 m / 10 Hz",
        },
        "rplidar_s2": {
            "range_min": 0.05,
            "range_max": 30.0,
            "angle_range": 2 * pi,
            "number": 720,
            "scan_time": 0.067,
            "std": 0.015,
            "description": "Slamtec RPLiDAR S2 — 360° / 30 m / 15 Hz",
        },
        # Hokuyo
        "hokuyo_urg04lx": {
            "range_min": 0.06,
            "range_max": 4.095,
            "angle_range": 4.189,  # 240 deg
            "number": 682,
            "scan_time": 0.1,
            "std": 0.03,
            "description": "Hokuyo URG-04LX — 240° / 4 m / 10 Hz",
        },
        "hokuyo_utm30lx": {
            "range_min": 0.1,
            "range_max": 30.0,
            "angle_range": 4.712,  # 270 deg
            "number": 1081,
            "scan_time": 0.025,
            "std": 0.03,
            "description": "Hokuyo UTM-30LX — 270° / 30 m / 40 Hz",
        },
        # SICK
        "sick_lms111": {
            "range_min": 0.5,
            "range_max": 20.0,
            "angle_range": 4.712,  # 270 deg
            "number": 541,
            "scan_time": 0.04,
            "std": 0.015,
            "description": "SICK LMS111 — 270° / 20 m / 25 Hz",
        },
        "sick_lms511": {
            "range_min": 0.1,
            "range_max": 80.0,
            "angle_range": 3.316,  # 190 deg
            "number": 761,
            "scan_time": 0.04,
            "std": 0.025,
            "description": "SICK LMS511 — 190° / 80 m / 25 Hz",
        },
        "sick_tim571": {
            "range_min": 0.05,
            "range_max": 25.0,
            "angle_range": 4.712,  # 270 deg
            "number": 811,
            "scan_time": 0.067,
            "std": 0.02,
            "description": "SICK TIM571 — 270° / 25 m / 15 Hz",
        },
        "sick_nav310": {
            "range_min": 0.5,
            "range_max": 250.0,
            "angle_range": 2 * pi,  # 360 deg
            "number": 720,
            "scan_time": 0.125,
            "std": 0.025,
            "description": "SICK NAV310 — 360° / 250 m / 8 Hz",
        },
        # YDLiDAR
        "ydlidar_x4": {
            "range_min": 0.12,
            "range_max": 10.0,
            "angle_range": 2 * pi,
            "number": 720,
            "scan_time": 0.1,
            "std": 0.02,
            "description": "YDLiDAR X4 — 360° / 10 m / 6-12 Hz",
        },
        "ydlidar_tg15": {
            "range_min": 0.02,
            "range_max": 15.0,
            "angle_range": 2 * pi,
            "number": 720,
            "scan_time": 0.067,
            "std": 0.02,
            "description": "YDLiDAR TG15 — 360° / 15 m / 10-20 Hz",
        },
        "ydlidar_g4": {
            "range_min": 0.28,
            "range_max": 16.0,
            "angle_range": 2 * pi,
            "number": 720,
            "scan_time": 0.111,
            "std": 0.02,
            "description": "YDLiDAR G4 — 360° / 16 m / 9 Hz",
        },
    }

    @classmethod
    def from_profile(
        cls,
        name: str,
        state: "np.ndarray | None" = None,
        obj_id: int = 0,
        **overrides: Any,
    ) -> "Lidar2D":
        """Construct a Lidar2D from a named product profile.

        Args:
            name: Profile key from :attr:`PROFILES` (e.g. ``"rplidar_a1m8"``).
            state: Initial state of the parent object.
            obj_id: ID of the associated object.
            **overrides: Override any profile parameter (e.g. ``range_max=8.0``).

        Returns:
            A new :class:`Lidar2D` initialised with the profile's parameters.
        """
        if name not in cls.PROFILES:
            raise ValueError(
                f"Unknown lidar2d profile {name!r}. Available: {list(cls.PROFILES)}"
            )
        params = {k: v for k, v in cls.PROFILES[name].items() if k != "description"}
        params.update(overrides)
        return cls(state=state, obj_id=obj_id, **params)

    def __init__(
        self,
        state: np.ndarray | None = None,
        obj_id: int = 0,
        range_min: float = 0,
        range_max: float = 10,
        angle_range: float = pi,
        number: int = 100,
        scan_time: float = 0.1,
        noise: bool = False,
        std: float = 0.2,
        angle_std: float = 0.02,
        offset: list[float] | None = None,
        alpha: float = 0.3,
        has_velocity: bool = False,
        **kwargs,
    ) -> None:
        """
        Initialize the Lidar2D sensor.


        """
        if offset is None:
            offset = [0, 0, 0]
        self.sensor_type = "lidar2d"

        self.range_min = range_min
        self.range_max = range_max

        self.angle_range = ClipTo2Pi(angle_range)
        self.angle_min = -self.angle_range / 2 if number > 1 else 0.0
        self.angle_max = self.angle_range / 2 if number > 1 else 0.0
        self.angle_inc = self.angle_range / (number - 1) if number > 1 else 0.0

        self.number = number
        self.scan_time = scan_time
        self.noise = noise
        self.std = std
        self.angle_std = angle_std
        self.offset = np.c_[offset]

        # Visualization params may be given under a `plot:` sub-dict
        # (preferred) or as flat top-level keys (backward compatible).
        _plot = kwargs.get("plot") or {}
        self._plot_cfg = _plot
        self.alpha = _plot.get("alpha", alpha)
        self.has_velocity = has_velocity
        self.velocity = np.zeros((2, number))

        # All beams use one instantaneous geometry snapshot.
        self.time_inc = 0.0
        self.range_data = range_max * np.ones(number)

        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=number)

        self._state = state
        self.init_geometry(self._state)

        self.color = _plot.get("color", kwargs.get("color", "r"))

        self.obj_id = obj_id

        # Parent object reference (set by ObjectBase or SensorFactory)
        self.parent: ObjectBase | None = None

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        # Map segment cache: static map geometry is re-queried only when the
        # sensor moves more than ``_map_cache_thresh`` metres from the position
        # at which it was last computed.  Dynamic obstacles are always re-queried
        # every step.  Set thresh to 0 to disable caching entirely.
        self._map_seg_cache: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._map_cache_origin: np.ndarray = np.full(2, np.inf)
        self._map_cache_thresh: float = range_max * 0.05

        # Optional standalone scene (Open3DScene2D or any object with
        # .objects and .GeometryTree).  Returned by _env_param.
        self._scene: object | None = None

        # Precomputed local-frame trig for fast direction computation (f64 + f32)
        self._local_dir_cos = np.cos(self.angle_list)
        self._local_dir_sin = np.sin(self.angle_list)
        self._local_dir_cos_f32 = self._local_dir_cos.astype(np.float32)
        self._local_dir_sin_f32 = self._local_dir_sin.astype(np.float32)

        # Pre-allocated f32 SoA direction/output buffers (zero-alloc fast path)
        self._dir_dx_f32 = np.empty(number, dtype=np.float32)
        self._dir_dy_f32 = np.empty(number, dtype=np.float32)
        self._tmp_f32 = np.empty(number, dtype=np.float32)
        self._out_ranges_f32 = np.empty(number, dtype=np.float32)
        self._out_hit_i32 = np.empty(number, dtype=np.int32)
        self._origin_f32 = np.zeros(2, dtype=np.float32)

        # Precomputed static scene segments (set by _attach_scene / set_scene)
        self._all_seg_sx: np.ndarray | None = None
        self._all_seg_sy: np.ndarray | None = None
        self._all_seg_ex: np.ndarray | None = None
        self._all_seg_ey: np.ndarray | None = None
        self._seg_dvx: np.ndarray | None = None  # ex - sx for each segment
        self._seg_dvy: np.ndarray | None = None  # ey - sy for each segment
        self._seg_len2_safe: np.ndarray | None = None  # |dv|^2, 0→1 for safety
        # Pre-allocated working buffers for compressed-segment output
        self._work_seg_sx: np.ndarray | None = None
        self._work_seg_sy: np.ndarray | None = None
        self._work_seg_ex: np.ndarray | None = None
        self._work_seg_ey: np.ndarray | None = None
        # Pre-allocated prefilter temporaries — these 6 arrays would otherwise
        # be heap-allocated on every step (one per intermediate expression in
        # the closest-point-on-segment test).  Sized to total segment count at
        # _attach_scene time and reused every step.
        self._pf_ax: np.ndarray | None = None
        self._pf_ay: np.ndarray | None = None
        self._pf_t: np.ndarray | None = None
        self._pf_px: np.ndarray | None = None
        self._pf_py: np.ndarray | None = None
        self._pf_mask: np.ndarray | None = None
        # Pose-change cache: skip the prefilter entirely when the robot has not
        # moved more than filter_margin metres from the last filter position.
        # The filter runs at range_max + margin so the cached segment set is
        # always a superset of the true in-range set (triangle inequality).
        self._filter_cache_x: float = np.inf
        self._filter_cache_y: float = np.inf
        self._filter_cache_M: int = 0
        self._filter_margin: float = 1.0  # metres; configurable via set_scene()

        try:
            from irsim_devices.core.ray_casting_2d_omp import (
                cast_ray_segments_avx2,
                cast_ray_segments_avx2_f32,
                cast_ray_segments_avx2_f32_inplace,
                cast_ray_segments_omp,
                is_avx2_available,
                is_avx2_f32_available,
                is_omp_available,
            )

            self._cast_inplace = (
                cast_ray_segments_avx2_f32_inplace if is_avx2_f32_available() else None
            )
            if is_avx2_f32_available():
                self._omp_cast = cast_ray_segments_avx2_f32
            elif is_avx2_available():
                self._omp_cast = cast_ray_segments_avx2
            elif is_omp_available():
                self._omp_cast = cast_ray_segments_omp
            else:
                self._omp_cast = None
        except ImportError:
            self._cast_inplace = None
            self._omp_cast = None

    @property
    def scene(self) -> object | None:
        """Standalone scene providing ``.objects`` and ``.GeometryTree``.

        Assign an :class:`~irsim_devices.core.open3d_scene_2d.Open3DScene2D`
        (or any compatible object) before calling :meth:`step` to run the
        sensor without the ir-sim environment.

        Example::

            from irsim_devices.core.open3d_scene_2d import Open3DScene2D
            lidar = Lidar2D(state=[0, 0, 0], obj_id=0)
            lidar.scene = Open3DScene2D.from_files(["map.obj"], slice_z=0.5)
            lidar.step([0, 0, 0])
        """
        return self._scene

    @scene.setter
    def scene(self, value: object | None) -> None:
        self._attach_scene(value)

    def set_scene(
        self,
        scene: object | None,
        n_omp_threads: int = 2,
        filter_margin: float = 1.0,
    ) -> None:
        """Attach a standalone scene and configure the sensor for low-CPU operation.

        Precomputes all static segment geometry from *scene* at call time so that
        each :meth:`step` call requires no Shapely operations.  Also installs a
        pose-change cache so the per-step range prefilter is skipped whenever the
        sensor has not moved more than *filter_margin* metres from its position at
        the last filter run — which is the common case for robots navigating at
        typical indoor speeds (0.5–1.5 m/s at 30 Hz).

        Also sets the number of OpenMP threads used by the raycasting kernel so
        the sensor does not starve a robot stack running in the same process.

        Args:
            scene: An :class:`~irsim_devices.core.open3d_scene_2d.Open3DScene2D`
                (or any object with ``.objects`` iterable, each with
                ``.linestrings``).  Pass ``None`` to detach.
            n_omp_threads: OMP thread count for the raycasting kernel.  Default
                ``2`` — enough to achieve >200 Hz at 1500 beams / 30 m while
                leaving the remaining cores free for the robot stack.
            filter_margin: Safety margin in metres added to *range_max* when
                computing the cached segment set.  The cache is valid as long as
                the robot moves less than this distance between refilter calls,
                guaranteeing the cached set is always a superset of the true
                in-range segments.  Default ``1.0`` m; increase for faster
                robots or decrease to reduce the extra kernel work on cache hits.
        """
        self._filter_margin = float(filter_margin)
        try:
            from irsim_devices.core.ray_casting_2d_omp import set_omp_threads

            set_omp_threads(n_omp_threads)
        except ImportError:
            pass
        self._attach_scene(scene)

    def _attach_scene(self, scene: object | None) -> None:
        """Precompute static segment geometry from *scene*.

        Extracts every linestring from every object in *scene* and stores them
        as contiguous float32 SoA arrays.  Also pre-allocates per-step working
        buffers sized to the total segment count.  All of this runs **once** at
        scene-attachment time; :meth:`_step_standalone` then runs allocation-free.
        """
        self._scene = scene
        if scene is None:
            self._all_seg_sx = None
            self._all_seg_sy = None
            self._all_seg_ex = None
            self._all_seg_ey = None
            self._seg_dvx = None
            self._seg_dvy = None
            self._seg_len2_safe = None
            self._work_seg_sx = None
            self._work_seg_sy = None
            self._work_seg_ex = None
            self._work_seg_ey = None
            self._pf_ax = None
            self._pf_ay = None
            self._pf_t = None
            self._pf_px = None
            self._pf_py = None
            self._pf_mask = None
            self._filter_cache_x = np.inf
            self._filter_cache_y = np.inf
            self._filter_cache_M = 0
            self._map_seg_cache = None
            self._map_cache_origin = np.full(2, np.inf)
            return

        sx_list: list[np.ndarray] = []
        sy_list: list[np.ndarray] = []
        ex_list: list[np.ndarray] = []
        ey_list: list[np.ndarray] = []
        for obj in scene.objects:
            if not obj.linestrings:
                continue
            s, e = boundary_segments(obj.linestrings)
            if len(s) == 0:
                continue
            sx_list.append(s[:, 0])
            sy_list.append(s[:, 1])
            ex_list.append(e[:, 0])
            ey_list.append(e[:, 1])

        if sx_list:
            sx = np.concatenate(sx_list).astype(np.float32)
            sy = np.concatenate(sy_list).astype(np.float32)
            ex = np.concatenate(ex_list).astype(np.float32)
            ey = np.concatenate(ey_list).astype(np.float32)
        else:
            sx = sy = ex = ey = np.zeros(0, dtype=np.float32)

        self._all_seg_sx = sx
        self._all_seg_sy = sy
        self._all_seg_ex = ex
        self._all_seg_ey = ey

        # Precompute segment delta vectors for the per-step range prefilter
        dvx = (ex - sx).astype(np.float32)
        dvy = (ey - sy).astype(np.float32)
        len2 = dvx * dvx + dvy * dvy
        self._seg_dvx = dvx
        self._seg_dvy = dvy
        # Replace zero-length entries with 1.0 to avoid division by zero
        self._seg_len2_safe = np.where(len2 > 0, len2, np.float32(1.0))

        # Pre-allocate working buffers for filtered segments
        M = len(sx)
        self._work_seg_sx = np.empty(M, dtype=np.float32)
        self._work_seg_sy = np.empty(M, dtype=np.float32)
        self._work_seg_ex = np.empty(M, dtype=np.float32)
        self._work_seg_ey = np.empty(M, dtype=np.float32)

        # Pre-allocate prefilter temporaries (avoids 6 heap allocs per step)
        self._pf_ax = np.empty(M, dtype=np.float32)
        self._pf_ay = np.empty(M, dtype=np.float32)
        self._pf_t = np.empty(M, dtype=np.float32)
        self._pf_px = np.empty(M, dtype=np.float32)
        self._pf_py = np.empty(M, dtype=np.float32)
        self._pf_mask = np.empty(M, dtype=bool)

        # Reset pose-change cache (force refilter on first step)
        self._filter_cache_x = np.inf
        self._filter_cache_y = np.inf
        self._filter_cache_M = 0

        self._map_seg_cache = None
        self._map_cache_origin = np.full(2, np.inf)

    @property
    def _env_param(self):
        """Return the attached standalone scene, or None if none is set."""
        return self._scene

    def init_geometry(self, state):
        """
        Initialize the Lidar's scanning geometry.

        Args:
            state (np.ndarray): Current state of the sensor.
        """
        segment_point_list = []

        for i in range(self.number):
            x = self.range_data[i] * cos(self.angle_list[i])
            y = self.range_data[i] * sin(self.angle_list[i])

            point0 = np.zeros((1, 2))
            point = np.array([[x], [y]]).T

            segment = np.concatenate((point0, point), axis=0)

            segment_point_list.append(segment)

        self.origin_state = self.offset
        geometry = MultiLineString(segment_point_list)
        self._original_geometry = geometry_transform(geometry, self.origin_state)
        self.lidar_origin = transform_point_with_state(self.offset, state)

        self._geometry = geometry_transform(self._original_geometry, state)
        self._init_geometry = self._geometry

    def step(self, state: np.ndarray) -> None:
        """
        Update the Lidar's state and compute per-beam ranges via ray casting.

        Each beam is intersected analytically against the boundary segments of
        nearby obstacles (polygons, linestrings, and map segments); the nearest
        hit along the beam is its range. This reproduces the previous geometry
        ``difference`` result to floating-point precision when the sensor origin
        is in free space, while avoiding the expensive GEOS overlay.

        Static map geometry is gathered with a fast disk query and cached by
        sensor position; dynamic obstacles are re-queried every step.

        When a standalone scene is attached (via :attr:`scene`), this method
        delegates to :meth:`_step_standalone`, which bypasses all Shapely beam
        geometry and processes directions through pure NumPy + the C kernel.

        Args:
            state (np.ndarray): New state of the sensor.
        """
        self._state = state

        if self._scene is not None:
            self._step_standalone(state)
            return

        lidar_geometry = self._world_geometry(state)
        detected_objects = self._get_detected_objects(lidar_geometry)

        ranges, hit_object_indices, origin, directions = self._cast_rays_cached(
            lidar_geometry,
            detected_objects,
        )

        if self.noise:
            self.range_data[:] = ranges + rng.normal(0, self.std, self.number)
        else:
            self.range_data[:] = ranges

        self._rebuild_scan_geometry(origin, directions)

        if self.has_velocity:
            self._assign_velocities(hit_object_indices, detected_objects)

    def _step_standalone(self, state: np.ndarray) -> None:
        """Fast ray-casting path for standalone scene.

        When static segment geometry has been precomputed via :meth:`set_scene`
        or :meth:`_attach_scene`, this path is fully allocation-free per step:

        * Sensor origin and heading are computed with scalar math.
        * World-frame beam directions are written directly into pre-allocated
          float32 SoA buffers via NumPy in-place operations.
        * A vectorised NumPy range prefilter eliminates segments outside
          ``range_max`` (no Shapely disk query).
        * The AVX2 float32 kernel writes hits into pre-allocated output buffers.
        * ``range_data`` is updated with a single NumPy assignment.

        Falls back to the Shapely-based positional-cache path when precomputed
        geometry is unavailable (e.g. scene attached via the ``scene`` property
        before the first call to :meth:`set_scene`).
        """
        # ── World-frame origin ───────────────────────────────────────────────
        s = np.asarray(state).ravel()
        sx, sy = float(s[0]), float(s[1])
        stheta = float(s[2]) if len(s) > 2 else 0.0
        off = self.offset.ravel()
        off_x = float(off[0])
        off_y = float(off[1])
        off_th = float(off[2] if len(off) > 2 else 0.0)
        ct, st = cos(stheta), sin(stheta)
        ox = sx + off_x * ct - off_y * st
        oy = sy + off_x * st + off_y * ct
        world_theta = stheta + off_th
        self.lidar_origin = np.array([[ox], [oy], [world_theta]])

        # ── Fast path: precomputed f32 SoA segments + in-place AVX2 kernel ──
        if self._all_seg_sx is not None and self._cast_inplace is not None:
            self._step_fast(ox, oy, world_theta)
            return

        # ── Fallback: Shapely disk query + positional cache ──────────────────
        import shapely as _sl

        cw, sw = cos(world_theta), sin(world_theta)
        dir_cos = self._local_dir_cos * cw - self._local_dir_sin * sw
        dir_sin = self._local_dir_cos * sw + self._local_dir_sin * cw
        directions = np.stack([dir_cos, dir_sin], axis=1)  # (N, 2) float64
        origin = np.array([ox, oy, world_theta])
        origin_2d = origin[:2]

        cache_hit = (
            self._map_seg_cache is not None
            and np.linalg.norm(origin_2d - self._map_cache_origin)
            <= self._map_cache_thresh
        )
        if not cache_hit:
            disk = _sl.buffer(_sl.points([ox, oy]), self.range_max)
            scene = self._scene
            obj_hits = (
                scene.GeometryTree.query(disk, predicate="intersects")
                if scene.GeometryTree is not None
                else []
            )
            ss_list, se_list = [], []
            for obj_idx in obj_hits:
                obj = scene.objects[obj_idx]
                ls_hits = obj.geometry_tree.query(disk, predicate="intersects")
                if len(ls_hits):
                    geoms = [obj.linestrings[h] for h in ls_hits]
                    s_segs, e_segs = boundary_segments(geoms)
                    if len(s_segs):
                        ss_list.append(s_segs)
                        se_list.append(e_segs)
            if ss_list:
                seg_start = np.concatenate(ss_list)
                seg_end = np.concatenate(se_list)
            else:
                seg_start, seg_end = _empty_segments()[:2]
            dummy_owners = np.arange(len(seg_start), dtype=int)
            self._map_seg_cache = (seg_start, seg_end, dummy_owners)
            self._map_cache_origin = origin_2d.copy()
        else:
            seg_start, seg_end, _ = self._map_seg_cache  # type: ignore[misc]

        if len(seg_start) == 0:
            ranges = np.full(self.number, self.range_max, dtype=np.float64)
        elif self._omp_cast is not None:
            ranges, _ = self._omp_cast(
                origin_2d, directions, seg_start, seg_end, self.range_max
            )
        else:
            ranges, _ = cast_ray_segments(
                origin_2d, directions, seg_start, seg_end, self.range_max
            )

        if self.noise:
            self.range_data[:] = ranges + rng.normal(0, self.std, self.number)
        else:
            self.range_data[:] = ranges

    def _step_fast(self, ox: float, oy: float, world_theta: float) -> None:
        """Zero-allocation inner step using precomputed f32 SoA geometry.

        Runs fully allocation-free on cache-hit steps:

        * Direction rotation uses 6 in-place NumPy ops on pre-allocated buffers.
        * The range prefilter is skipped when the sensor position is within
          ``_filter_margin`` metres of the last filter run (pose-change cache).
          Filtered segments already live in ``_work_seg_*``; the kernel reuses them.
        * On a cache miss, the prefilter executes entirely in-place using
          pre-allocated ``_pf_*`` temporaries (no heap allocations).
        * The AVX2 float32 kernel writes results into pre-allocated output buffers.

        The only remaining allocation on a cache-miss step is ``np.nonzero(mask)[0]``
        (the filtered index array, typically a few KB).
        """
        rmax_f = np.float32(self.range_max)
        ox_f = np.float32(ox)
        oy_f = np.float32(oy)

        # ── Directions: in-place rotation of precomputed local-frame trig ───
        cw_f = np.float32(cos(world_theta))
        sw_f = np.float32(sin(world_theta))
        # dir_dx = local_cos * cw - local_sin * sw
        np.multiply(self._local_dir_cos_f32, cw_f, out=self._dir_dx_f32)
        np.multiply(self._local_dir_sin_f32, sw_f, out=self._tmp_f32)
        np.subtract(self._dir_dx_f32, self._tmp_f32, out=self._dir_dx_f32)
        # dir_dy = local_cos * sw + local_sin * cw
        np.multiply(self._local_dir_cos_f32, sw_f, out=self._dir_dy_f32)
        np.multiply(self._local_dir_sin_f32, cw_f, out=self._tmp_f32)
        np.add(self._dir_dy_f32, self._tmp_f32, out=self._dir_dy_f32)

        self._origin_f32[0] = ox_f
        self._origin_f32[1] = oy_f

        M_all = len(self._all_seg_sx)  # type: ignore[arg-type]
        if M_all == 0:
            self._out_ranges_f32[:] = rmax_f
        else:
            # ── Pose-change cache ─────────────────────────────────────────────
            # The filtered segment set (which segments lie within range_max)
            # changes only when the robot moves.  Cache the last filtered set
            # and skip refiltering when position delta < filter_margin.
            # The filter uses range_max + margin so the cached set is always a
            # superset of the true in-range segments (triangle inequality), and
            # the kernel correctly treats out-of-range segment hits as misses.
            dx_c = ox_f - np.float32(self._filter_cache_x)
            dy_c = oy_f - np.float32(self._filter_cache_y)
            margin_f = np.float32(self._filter_margin)

            if dx_c * dx_c + dy_c * dy_c > margin_f * margin_f:
                # ── Cache miss: recompute prefilter in-place (no heap alloc) ─
                # Closest-point-on-segment test, fully vectorised.
                # t* = clip(-dot(O-A, B-A) / |B-A|^2, 0, 1)
                # P  = A + t*(B-A);  keep when |O-P|^2 <= (rmax+margin)^2
                rmax_m = rmax_f + margin_f
                rmax_m_sq = rmax_m * rmax_m

                # ax = sx - ox;  ay = sy - oy
                np.subtract(self._all_seg_sx, ox_f, out=self._pf_ax)  # type: ignore[arg-type]
                np.subtract(self._all_seg_sy, oy_f, out=self._pf_ay)  # type: ignore[arg-type]
                # t_num = ax*dvx + ay*dvy  (reuse _pf_px as scratch)
                np.multiply(self._pf_ax, self._seg_dvx, out=self._pf_t)  # type: ignore[arg-type]
                np.multiply(self._pf_ay, self._seg_dvy, out=self._pf_px)  # type: ignore[arg-type]
                np.add(self._pf_t, self._pf_px, out=self._pf_t)
                np.negative(self._pf_t, out=self._pf_t)
                np.divide(self._pf_t, self._seg_len2_safe, out=self._pf_t)  # type: ignore[arg-type]
                np.clip(self._pf_t, np.float32(0.0), np.float32(1.0), out=self._pf_t)
                # px = ax + t*dvx
                np.multiply(self._pf_t, self._seg_dvx, out=self._pf_px)  # type: ignore[arg-type]
                np.add(self._pf_ax, self._pf_px, out=self._pf_px)
                # py = ay + t*dvy
                np.multiply(self._pf_t, self._seg_dvy, out=self._pf_py)  # type: ignore[arg-type]
                np.add(self._pf_ay, self._pf_py, out=self._pf_py)
                # dist2 = px^2 + py^2  (reuse _pf_t and _pf_ay as scratch)
                np.multiply(self._pf_px, self._pf_px, out=self._pf_t)
                np.multiply(self._pf_py, self._pf_py, out=self._pf_ay)
                np.add(self._pf_t, self._pf_ay, out=self._pf_t)
                np.less_equal(self._pf_t, rmax_m_sq, out=self._pf_mask)  # type: ignore[arg-type]

                idx = np.nonzero(self._pf_mask)[0]
                M_filt = len(idx)
                if M_filt > 0:
                    np.take(self._all_seg_sx, idx, out=self._work_seg_sx[:M_filt])  # type: ignore[index]
                    np.take(self._all_seg_sy, idx, out=self._work_seg_sy[:M_filt])  # type: ignore[index]
                    np.take(self._all_seg_ex, idx, out=self._work_seg_ex[:M_filt])  # type: ignore[index]
                    np.take(self._all_seg_ey, idx, out=self._work_seg_ey[:M_filt])  # type: ignore[index]

                self._filter_cache_x = float(ox_f)
                self._filter_cache_y = float(oy_f)
                self._filter_cache_M = M_filt
            else:
                # ── Cache hit: reuse last filtered segment set ────────────────
                M_filt = self._filter_cache_M

            if M_filt == 0:
                self._out_ranges_f32[:] = rmax_f
            else:
                self._cast_inplace(  # type: ignore[misc]
                    self._origin_f32,
                    self._dir_dx_f32,
                    self._dir_dy_f32,
                    self._work_seg_sx[:M_filt],
                    self._work_seg_sy[:M_filt],
                    self._work_seg_ex[:M_filt],
                    self._work_seg_ey[:M_filt],
                    float(rmax_f),
                    self._out_ranges_f32,
                    self._out_hit_i32,
                )

        if self.noise:
            self.range_data[:] = self._out_ranges_f32 + rng.normal(
                0, self.std, self.number
            )
        else:
            self.range_data[:] = self._out_ranges_f32

    def _cast_rays_cached(
        self,
        lidar_geometry,
        detected_objects: list,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Ray-cast with disk-based map query and positional segment cache.

        Separates static map objects from dynamic obstacles so that the
        expensive spatial-tree query over map walls runs only when the sensor
        has moved more than ``_map_cache_thresh`` metres.  Dynamic obstacle
        segments are always recomputed.  Falls back to the plain ``cast_rays``
        path when no map objects are present so the common obstacle-only case
        carries no overhead.
        """
        # Partition detected objects into static (map) and dynamic
        map_objs: list[tuple[int, object]] = []
        dyn_objs: list[tuple[int, object]] = []
        for i, obj in enumerate(detected_objects):
            if getattr(obj, "shape", None) == "map":
                map_objs.append((i, obj))
            else:
                dyn_objs.append((i, obj))

        # Fast path: no map objects → standard pipeline (already fast)
        if not map_objs:
            return cast_rays(lidar_geometry, detected_objects, self.range_max)

        _shapely.prepare(lidar_geometry)
        origin, directions = _ray_parameters(lidar_geometry, self.range_max)
        origin_2d = origin[:2]

        # ── Map segments: use positional cache ───────────────────────────────
        cache_hit = (
            self._map_seg_cache is not None
            and np.linalg.norm(origin_2d - self._map_cache_origin)
            <= self._map_cache_thresh
        )
        if cache_hit:
            map_ss, map_se, map_owners = self._map_seg_cache  # type: ignore[misc]
        else:
            disk = _shapely.buffer(_shapely.points(origin_2d), self.range_max)
            ss_list, se_list, ow_list = [], [], []
            for obj_idx, obj in map_objs:
                hits = obj.geometry_tree.query(disk, predicate="intersects")
                if len(hits) == 0:
                    continue
                geoms = [obj.linestrings[h] for h in hits]
                s, e = boundary_segments(geoms)
                if len(s):
                    ss_list.append(s)
                    se_list.append(e)
                    ow_list.append(np.full(len(s), obj_idx, dtype=int))
            if ss_list:
                map_ss = np.concatenate(ss_list)
                map_se = np.concatenate(se_list)
                map_owners = np.concatenate(ow_list)
            else:
                map_ss, map_se, map_owners = _empty_segments()
            self._map_seg_cache = (map_ss, map_se, map_owners)
            self._map_cache_origin = origin_2d.copy()

        # ── Dynamic segments: always fresh ───────────────────────────────────
        dyn_ss_list, dyn_se_list, dyn_ow_list = [], [], []
        for obj_idx, obj in dyn_objs:
            if not lidar_geometry.intersects(obj._geometry):
                continue
            s, e = boundary_segments([obj._geometry])
            if len(s):
                dyn_ss_list.append(s)
                dyn_se_list.append(e)
                dyn_ow_list.append(np.full(len(s), obj_idx, dtype=int))
        if dyn_ss_list:
            dyn_ss = np.concatenate(dyn_ss_list)
            dyn_se = np.concatenate(dyn_se_list)
            dyn_owners = np.concatenate(dyn_ow_list)
        else:
            dyn_ss, dyn_se, dyn_owners = _empty_segments()

        # ── Merge ────────────────────────────────────────────────────────────
        parts_s = [a for a in (map_ss, dyn_ss) if len(a)]
        parts_e = [a for a in (map_se, dyn_se) if len(a)]
        parts_o = [a for a in (map_owners, dyn_owners) if len(a)]
        if parts_s:
            seg_start = np.concatenate(parts_s)
            seg_end = np.concatenate(parts_e)
            seg_owner = np.concatenate(parts_o)
        else:
            seg_start, seg_end, seg_owner = _empty_segments()

        # ── Cast (prefer OMP kernel) ──────────────────────────────────────────
        if self._omp_cast is not None:
            ranges, hit_segs = self._omp_cast(
                origin, directions, seg_start, seg_end, self.range_max
            )
        else:
            ranges, hit_segs = cast_ray_segments(
                origin, directions, seg_start, seg_end, self.range_max
            )

        hit_object_indices = np.full(len(directions), -1, dtype=int)
        has_hit = hit_segs >= 0
        if has_hit.any() and len(seg_owner):
            hit_object_indices[has_hit] = seg_owner[hit_segs[has_hit]]

        return ranges, hit_object_indices, origin, directions

    def _get_detected_objects(self, lidar_geometry) -> list:
        """Select objects that may produce a return for this lidar geometry.

        This is the environment-facing broad-phase operation. It owns access to
        the scene's complete object list and geometry tree, and filters objects
        that sensors must ignore. Exact boundary intersections remain in the
        geometry-only ray-casting operation.
        """
        env_p = self._env_param
        if env_p is None:
            return []
        objects = env_p.objects
        geometry_tree = env_p.GeometryTree
        if geometry_tree is None:
            return []

        detected_objects = []
        for object_index in geometry_tree.query(lidar_geometry):
            obj = objects[object_index]
            if obj._id == self.obj_id or not obj._geometry_valid or obj.unobstructed:
                continue
            detected_objects.append(obj)
        return detected_objects

    def _world_geometry(self, state: np.ndarray) -> MultiLineString:
        """Build the max-range beam geometry in world coordinates."""
        world_geometry = geometry_transform(self._original_geometry, state)
        self.lidar_origin = transform_point_with_state(self.offset, state)
        # Use the beam geometry's exact start coordinate. Computing the same
        # point through a separate transform can differ by one floating-point
        # step across platforms, which breaks exact GEOS origin predicates.
        self.lidar_origin[:2, 0] = shapely.get_coordinates(world_geometry)[0]
        return world_geometry

    def _rebuild_scan_geometry(
        self, origin: np.ndarray, directions: np.ndarray
    ) -> None:
        """Rebuild each clipped beam from its origin and measured range."""
        endpoints = origin + self.range_data[:, None] * directions
        origins = np.broadcast_to(origin, endpoints.shape)
        beam_coordinates = np.stack([origins, endpoints], axis=1)
        self._geometry = shapely.multilinestrings(
            shapely.linestrings(beam_coordinates),
        )

    def _assign_velocities(
        self,
        hit_object_indices: np.ndarray,
        detected_objects,
    ) -> None:
        """Assign velocity when ray casting reports an actual object hit.

        ``hit_object_indices`` refers to ``detected_objects`` and distinguishes
        a hit from a max-range miss, so no range margin is needed near
        ``range_max``.
        """
        self.velocity[:] = 0.0
        for beam_index in np.flatnonzero(hit_object_indices >= 0):
            object_velocity = detected_objects[
                hit_object_indices[beam_index]
            ].velocity_xy
            self.velocity[:, beam_index : beam_index + 1] = object_velocity

    def get_scan(self):
        """
        Get the 2D lidar scan data. refer to the ros topic scan: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

        Returns:
            dict: Scan data including angles, ranges, and velocities.
        """
        scan_data = {}
        scan_data["angle_min"] = self.angle_min
        scan_data["angle_max"] = self.angle_max
        scan_data["angle_increment"] = self.angle_inc
        scan_data["time_increment"] = self.time_inc
        scan_data["scan_time"] = self.scan_time
        scan_data["range_min"] = self.range_min
        scan_data["range_max"] = self.range_max
        scan_data["ranges"] = self.range_data
        scan_data["intensities"] = None
        scan_data["velocity"] = self.velocity

        return scan_data

    def get_points(self):
        """
        Convert scan data to a point cloud.

        Returns:
            np.ndarray: Point cloud (2xN).
        """
        return self.scan_to_pointcloud()

    def get_offset(self):
        """
        Get the sensor's offset.

        Returns:
            list: Offset as a list.
        """
        return np.squeeze(self.offset).tolist()

    def plot(self, ax, state: np.ndarray | None = None, **kwargs):
        """
        Plot the Lidar's detected lines on a given axis.
        """
        if state is None:
            state = self.state

        self._plot(ax, state, **kwargs)

    def _init_plot(self, ax, **kwargs):
        """
        Initialize the plot for the Lidar.
        """
        self._plot(ax, self.origin_state, **kwargs)

    @property
    def state(self) -> np.ndarray:
        """
        Get the current state of the lidar sensor.

        Returns:
            np.ndarray: Current state of the sensor.
        """
        return self._state

    def _plot(self, ax, state, **kwargs):
        """
        Plot the Lidar's detected lines using the specified state for positioning.
        Creates line segments in local coordinates and applies transforms to position them.

        Args:
            ax: Matplotlib axis.
            state: State vector [x, y, theta, ...] defining lidar position and orientation.
            **kwargs: Plotting options.
        """
        lines = []

        if isinstance(ax, Axes3D):
            # For 3D plotting, calculate actual world coordinates since transforms don't work the same way
            if state is not None and len(state) > 0:
                # Calculate lidar position based on object state and sensor offset
                lidar_x = self.lidar_origin[0, 0]
                lidar_y = self.lidar_origin[1, 0]
                lidar_theta = (
                    self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
                )
            else:
                lidar_x, lidar_y, lidar_theta = 0, 0, 0

            # Create line segments in world coordinates for 3D
            for i in range(self.number):
                x_local = self.range_data[i] * cos(self.angle_list[i])
                y_local = self.range_data[i] * sin(self.angle_list[i])

                # Transform to world coordinates
                x_world = (
                    lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta)
                )
                y_world = (
                    lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta)
                )

                start_point = np.array([lidar_x, lidar_y, 0])
                end_point = np.array([x_world, y_world, 0])
                segment = [start_point, end_point]
                lines.append(segment)

            self.laser_LineCollection = Line3DCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=2
            )
            ax.add_collection3d(self.laser_LineCollection)
        else:
            # For 2D plotting, create line segments in local coordinates and use transforms
            for i in range(self.number):
                x = self.range_data[i] * cos(self.angle_list[i])
                y = self.range_data[i] * sin(self.angle_list[i])
                segment = [np.array([0, 0]), np.array([x, y])]
                lines.append(segment)

            self.laser_LineCollection = LineCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=2
            )
            ax.add_collection(self.laser_LineCollection)

            # Apply transform for 2D case - use provided state for positioning
            if state is not None and len(state) > 0:
                lidar_x = self.lidar_origin[0, 0]
                lidar_y = self.lidar_origin[1, 0]
                lidar_theta = (
                    self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
                )

                # Create transform: rotate by lidar orientation, then translate to lidar position
                trans = (
                    mtransforms.Affine2D()
                    .rotate(lidar_theta)
                    .translate(lidar_x, lidar_y)
                    + ax.transData
                )
                self.laser_LineCollection.set_transform(trans)

        self.plot_patch_list.append(self.laser_LineCollection)

    def _step_plot(self):
        """
        Update the lidar visualization using matplotlib transforms based on current state.
        Creates line segments in local coordinates and applies transform to position them.
        """
        if not hasattr(self, "laser_LineCollection"):
            return

        ax = self.laser_LineCollection.axes
        lines = []

        if ax is None:
            return

        if isinstance(ax, Axes3D):
            # For 3D plotting, calculate actual world coordinates
            lidar_x = self.lidar_origin[0, 0]
            lidar_y = self.lidar_origin[1, 0]
            lidar_theta = (
                self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
            )

            # Create line segments in world coordinates for 3D
            for i in range(self.number):
                x_local = self.range_data[i] * cos(self.angle_list[i])
                y_local = self.range_data[i] * sin(self.angle_list[i])

                # Transform to world coordinates
                x_world = (
                    lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta)
                )
                y_world = (
                    lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta)
                )

                start_point = np.array([lidar_x, lidar_y, 0])
                end_point = np.array([x_world, y_world, 0])
                segment = [start_point, end_point]
                lines.append(segment)
        else:
            # For 2D plotting, create line segments in local coordinates
            for i in range(self.number):
                x = self.range_data[i] * cos(self.angle_list[i])
                y = self.range_data[i] * sin(self.angle_list[i])
                segment = [np.array([0, 0]), np.array([x, y])]
                lines.append(segment)

        # Update line segments
        self.laser_LineCollection.set_segments(lines)

        # Apply transform to position the LineCollection based on current lidar origin (2D only)
        if not isinstance(ax, Axes3D):  # 2D case
            lidar_x = self.lidar_origin[0, 0]
            lidar_y = self.lidar_origin[1, 0]
            lidar_theta = (
                self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
            )

            # Create transform: rotate by lidar orientation, then translate to lidar position
            trans = (
                mtransforms.Affine2D().rotate(lidar_theta).translate(lidar_x, lidar_y)
                + ax.transData
            )
            self.laser_LineCollection.set_transform(trans)

    def step_plot(self):
        """
        Public method to update the lidar visualization, calls _step_plot.
        """
        self._step_plot()

    def set_laser_color(
        self, laser_indices, laser_color: str = "blue", alpha: float = 0.3
    ):
        """
        Set a specific color of the selected lasers.

        Args:
            laser_indices (list): The indices of the lasers to set the color.
            laser_color (str): The color to set the selected lasers. Default is 'blue'.
            alpha (float): The transparency of the lasers. Default is 0.3.
        """

        current_color = [self.color] * self.number
        current_alpha = [self.alpha] * self.number

        for index in laser_indices:
            if index < self.number:
                current_color[index] = laser_color
                current_alpha[index] = alpha

        self.laser_LineCollection.set_color(current_color)
        self.laser_LineCollection.set_alpha(current_alpha)

    def plot_clear(self):
        """
        Clear the plot elements from the axis.
        """
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    def scan_to_pointcloud(self):
        """
        Convert the Lidar scan data to a point cloud.

        Returns:
            np.ndarray: Point cloud (2xN).
        """
        point_cloud = []

        ranges = self.range_data
        angles = np.linspace(self.angle_min, self.angle_max, len(ranges))

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (self.range_max - 0.02):
                point = np.array([[scan_range * cos(angle)], [scan_range * sin(angle)]])
                point_cloud.append(point)

        if len(point_cloud) == 0:
            return None

        return np.hstack(point_cloud)
