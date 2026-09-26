"""Example 04 — Simulate and publish LiDAR 2D/3D, IMU, encoder, and odometry via shmbridge.

Both the 2-D and 3-D LiDAR raycast against a real URDF world model (loaded
via `irsim_devices.models.load_urdf`, which uses trimesh -- no shapely, no
ir-sim dependency). Ray casting itself uses Embree4 (irsim_devices' native
`lidar_embree` extension) when it has been built, and falls back
automatically to open3d's `RaycastingScene` otherwise -- so this runs with
either backend, whichever is available, and never touches shapely.

The robot drives a fixed circle sized to the loaded world's footprint; IMU
and encoder/motor readings come from `irsim_devices`' standalone device
models (noise model + DC-motor chassis), not analytic formulas.

Requires shmbridge and irsim_devices installed:
    cd ../shmbridge     && pip install -e .
    cd ../irsim_devices  && pip install -e ".[urdf]"       # trimesh, for the URDF loader
    cd ../irsim_devices  && pip install open3d              # only if Embree wasn't built

Usage:
    python 04_pub_sensors.py                        # default 20 Hz, 1080 beams
    python 04_pub_sensors.py --rate 10 --beams 360
    python 04_pub_sensors.py --shm /my_seg
    python 04_pub_sensors.py --world models/warehouse_world.urdf
    python 04_pub_sensors.py --lidar3d-profile vlp32c
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from irsim_devices.actuators import MotorDiffChassis
from irsim_devices.models import load_urdf
from irsim_devices.sensors import IMU as DevImu
from irsim_devices.sensors.lidar3d_embree import EmbreeLidar3D

from urdf_tools.pubsub import EncoderState, Imu, LaserScan, Odometry, SensorPublisher

SHM_NAME = "/urdf_tools_sensors"
RATE_HZ = 20.0
BEAMS = 1080
RMAX = 20.0
DEFAULT_WORLD = Path(__file__).parent / "models" / "warehouse_world.urdf"

# Trajectory: a circle sized relative to the loaded world, at a constant
# forward speed. Radius/centre are computed from the mesh bounding box once
# it's loaded (see main()); the module-level default only applies if the
# world has no geometry at all (degenerate fallback).
V_LINEAR = 0.5


class RaycastScene:
    """URDF-mesh raycasting: Embree4 (native, fast) if built, else open3d.

    Neither path uses shapely. Exposes two primitives:

    * :meth:`cast_rays` — arbitrary ray batch, returns ranges only (used for
      the 2-D scan, which needs ranges in the sensor's own angular frame).
    * :meth:`cast_3d_lidar` — full spinning-LiDAR pattern, returns hit
      points (used for the 3-D cloud).
    """

    def __init__(self, vertices: np.ndarray, triangles: np.ndarray) -> None:
        vertices = np.ascontiguousarray(vertices, dtype=np.float32)
        triangles = np.ascontiguousarray(triangles, dtype=np.int32)
        self._embree = self._try_embree(vertices, triangles)
        self._o3d_scene = (
            None
            if self._embree is not None
            else self._build_open3d(vertices, triangles)
        )
        self.backend = "embree" if self._embree is not None else "open3d"

    @staticmethod
    def _try_embree(vertices: np.ndarray, triangles: np.ndarray):
        try:
            from irsim_devices.sensors.lidar3d_embree import _load_lidar_embree

            lidar_embree = _load_lidar_embree()
        except ImportError:
            return None
        scene = lidar_embree.EmbreeScene3D()
        scene.build(vertices, triangles)
        return scene

    @staticmethod
    def _build_open3d(vertices: np.ndarray, triangles: np.ndarray):
        import open3d as o3d

        mesh = o3d.t.geometry.TriangleMesh()
        mesh.vertex.positions = o3d.core.Tensor(vertices)
        mesh.triangle.indices = o3d.core.Tensor(triangles.astype(np.uint32))
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(mesh)
        return scene

    def cast_rays(
        self, origin: np.ndarray, directions: np.ndarray, range_max: float
    ) -> np.ndarray:
        """directions: (N, 3) unit vectors. Returns (N,) ranges (range_max on miss)."""
        directions = np.ascontiguousarray(directions, dtype=np.float32)
        origins = np.tile(np.asarray(origin, dtype=np.float32), (len(directions), 1))
        if self._embree is not None:
            return self._embree.cast_rays(origins, directions, float(range_max))

        import open3d as o3d

        rays = o3d.core.Tensor(np.hstack([origins, directions]))
        t_hit = self._o3d_scene.cast_rays(rays)["t_hit"].numpy()
        t_hit[np.isinf(t_hit)] = range_max
        return np.clip(t_hit, 0.0, range_max).astype(np.float32)

    def cast_3d_lidar(
        self,
        origin: np.ndarray,
        n_vertical: int,
        n_horizontal: int,
        elev_min_deg: float,
        elev_max_deg: float,
        range_max: float,
    ) -> np.ndarray:
        """Full 360 deg spinning-LiDAR pattern. Returns (N_hits, 4) [x, y, z, range]."""
        if self._embree is not None:
            return self._embree.cast_3d_lidar(
                np.asarray(origin, dtype=np.float32),
                n_vertical,
                n_horizontal,
                float(elev_min_deg),
                float(elev_max_deg),
                float(range_max),
            )

        elevations = np.radians(
            np.linspace(elev_min_deg, elev_max_deg, n_vertical, dtype=np.float32)
        )
        azimuths = np.linspace(
            -math.pi, math.pi, n_horizontal, endpoint=False, dtype=np.float32
        )
        elev_grid, az_grid = np.meshgrid(elevations, azimuths, indexing="ij")
        cos_e = np.cos(elev_grid)
        dirs = np.stack(
            [cos_e * np.cos(az_grid), cos_e * np.sin(az_grid), np.sin(elev_grid)],
            axis=-1,
        ).reshape(-1, 3)
        ranges = self.cast_rays(origin, dirs, range_max)
        hit = ranges < range_max * 0.999
        origin_f = np.asarray(origin, dtype=np.float32)
        pts = origin_f + ranges[hit, None] * dirs[hit]
        return np.column_stack([pts, ranges[hit]]).astype(np.float32)


def make_circle_trajectory(
    center_x: float, center_y: float, radius: float, speed: float
):
    """Circular trajectory of the given radius/speed. Returns (pose_fn, omega)."""
    omega = speed / radius

    def trajectory(t: float) -> tuple[float, float, float]:
        phi = omega * t
        x = center_x + radius * math.cos(phi)
        y = center_y + radius * math.sin(phi)
        theta = phi + math.pi / 2.0  # heading tangent to the circle (CCW travel)
        return x, y, theta

    return trajectory, omega


def main() -> None:
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    import argparse

    ap = argparse.ArgumentParser(description="Publish simulated sensors via shmbridge")
    ap.add_argument("--shm", default=SHM_NAME)
    ap.add_argument("--rate", type=float, default=RATE_HZ)
    ap.add_argument("--beams", type=int, default=BEAMS)
    ap.add_argument("--rmax", type=float, default=RMAX)
    ap.add_argument(
        "--world",
        default=str(DEFAULT_WORLD),
        metavar="URDF",
        help="World URDF raycast against (default: bundled warehouse model)",
    )
    ap.add_argument(
        "--lidar3d-profile",
        default="vlp16",
        metavar="NAME",
        help=f"EmbreeLidar3D.PROFILES entry (default vlp16); one of "
        f"{list(EmbreeLidar3D.PROFILES)}",
    )
    ap.add_argument("--lidar-height", type=float, default=0.30, metavar="M")
    ap.add_argument("--wheel-radius", type=float, default=0.05, metavar="M")
    ap.add_argument("--wheel-base", type=float, default=0.30, metavar="M")
    ap.add_argument("--motor-profile", default="small_dc", metavar="NAME")
    args = ap.parse_args()

    world = load_urdf(args.world, use_collision=False)
    print(f"[world] {world}")
    scene = RaycastScene(world.vertices, world.triangles)
    print(f"[world] raycast backend: {scene.backend}")

    vmin = world.vertices.min(axis=0)
    vmax = world.vertices.max(axis=0)
    center_x, center_y = (vmin[0] + vmax[0]) / 2.0, (vmin[1] + vmax[1]) / 2.0
    radius = float(min(vmax[0] - vmin[0], vmax[1] - vmin[1]) * 0.3) or 5.0
    trajectory, omega_yaw = make_circle_trajectory(center_x, center_y, radius, V_LINEAR)
    print(
        f"[world] circling ({center_x:.1f},{center_y:.1f}) r={radius:.1f}m "
        f"v={V_LINEAR}m/s"
    )

    angles = np.linspace(
        -math.pi, math.pi, args.beams, endpoint=False, dtype=np.float32
    )
    a_inc = float(2 * math.pi / args.beams)

    n_vert, n_horiz, elev_min, elev_max = EmbreeLidar3D.PROFILES[args.lidar3d_profile]

    step_time = 1.0 / args.rate
    half_wheel_base = args.wheel_base / 2.0
    x0, y0, theta0 = trajectory(0.0)

    dev_imu = DevImu(
        state=np.array([x0, y0, theta0], dtype=np.float64),
        step_time=step_time,
        noise_model="ieee517",
    )
    chassis = MotorDiffChassis(
        wheel_radius=args.wheel_radius,
        wheel_base=args.wheel_base,
        motor_profile=args.motor_profile,
        initial_state=[x0, y0, theta0],
    )
    chassis.left_motor.set_mode("velocity")
    chassis.right_motor.set_mode("velocity")
    omega_l_cmd = (V_LINEAR - omega_yaw * half_wheel_base) / args.wheel_radius
    omega_r_cmd = (V_LINEAR + omega_yaw * half_wheel_base) / args.wheel_radius

    print(f"Publishing on shm={args.shm!r}  rate={args.rate}Hz  beams={args.beams}")
    print(
        f"3-D lidar: {args.lidar3d_profile!r}  ({n_vert}x{n_horiz} beams, "
        f"{elev_min:.1f}..{elev_max:.1f} deg)"
    )
    print("Ctrl+C to stop.")
    t0 = time.time()

    with SensorPublisher(args.shm) as pub:
        try:
            while True:
                t = time.time() - t0
                x, y, theta = trajectory(t)
                origin2d = np.array([x, y, args.lidar_height], dtype=np.float32)

                # ── 2-D LiDAR (horizontal ring against the URDF mesh) ───────
                beam_a = angles + theta
                dirs2d = np.column_stack(
                    [np.cos(beam_a), np.sin(beam_a), np.zeros_like(beam_a)]
                )
                rng = scene.cast_rays(origin2d, dirs2d, args.rmax)
                hits2d = int(np.sum(rng < args.rmax * 0.999))
                pub.publish_scan(
                    LaserScan(
                        stamp=t,
                        angle_min=float(angles[0]),
                        angle_max=float(angles[-1]),
                        angle_increment=a_inc,
                        range_max=args.rmax,
                        ranges=rng.tolist(),
                    )
                )

                # ── 3-D LiDAR (full spin against the same URDF mesh) ────────
                cloud = scene.cast_3d_lidar(
                    origin2d, n_vert, n_horiz, elev_min, elev_max, args.rmax
                )
                if len(cloud):
                    pub.publish_cloud3d(cloud, stamp=t)

                # ── IMU (device noise model, derived from true motion) ─────
                dev_imu.step(np.array([x, y, theta], dtype=np.float64))
                pub.publish_imu(
                    Imu(
                        stamp=t,
                        linear_acceleration=dev_imu.linear_acceleration.tolist(),
                        angular_velocity=dev_imu.angular_velocity.tolist(),
                    )
                )

                # ── Odometry ─────────────────────────────────────────────
                pub.publish_odom(
                    Odometry(
                        stamp=t,
                        x=float(x),
                        y=float(y),
                        theta=float(theta),
                        vx=V_LINEAR,
                        omega=omega_yaw,
                    )
                )

                # ── Encoder + motor (chassis driven by the same v/omega) ───
                chassis.step([omega_l_cmd, omega_r_cmd], dt=step_time)
                enc = chassis.encoder_readings
                pub.publish_encoder(
                    EncoderState(
                        stamp=t,
                        ticks=[enc["left"]["ticks"], enc["right"]["ticks"], 0, 0],
                        speed=[
                            enc["left"]["omega_output"],
                            enc["right"]["omega_output"],
                            0.0,
                            0.0,
                        ],
                    )
                )

                print(
                    f"\r  t={t:6.1f}s  ({x:5.2f},{y:5.2f})  θ={theta:.2f}"
                    f"  hits2d={hits2d}/{args.beams}  pts3d={len(cloud):5d}",
                    end="",
                    flush=True,
                )
                time.sleep(step_time)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
