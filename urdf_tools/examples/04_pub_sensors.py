"""Example 04 — Simulate and publish LiDAR 2D/3D, IMU, encoder, and odometry via shmbridge.

Everything here is fully synthetic: a robot drives a fixed circle around
three cylindrical obstacles, and every sensor is derived analytically from
that trajectory. No URDF world, no external simulator, no native/compiled
extensions required -- just numpy plus the standalone `irsim_devices`
sensor/actuator models (IMU noise model, DC-motor + encoder chassis).

Requires shmbridge and irsim_devices installed:
    cd ../shmbridge     && pip install -e .
    cd ../irsim_devices  && pip install -e .

Usage:
    python 04_pub_sensors.py               # default 20 Hz, 1080 beams
    python 04_pub_sensors.py --rate 10 --beams 360
    python 04_pub_sensors.py --shm /my_seg
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
from irsim_devices.sensors import IMU as DevImu
from irsim_devices.sensors.lidar3d_embree import EmbreeLidar3D

from urdf_tools.pubsub import EncoderState, Imu, LaserScan, Odometry, SensorPublisher

SHM_NAME = "/urdf_tools_sensors"
RATE_HZ = 20.0
BEAMS = 1080
RMAX = 20.0
OBSTACLE_HEIGHT = 2.0
LIDAR_HEIGHT = 0.20

# Constant forward speed / yaw rate driving a circle of radius v/omega = 5 m.
V_LINEAR = 0.5
OMEGA_YAW = 0.1

OBSTACLES = [(3.0, 3.0, 0.5), (-4.0, 2.0, 0.8), (1.0, -5.0, 1.2)]


def trajectory(t: float) -> tuple[float, float, float]:
    """Pose [x, y, theta] at time t on the fixed test circle."""
    x = 5.0 * math.cos(0.1 * t)
    y = 5.0 * math.sin(0.1 * t)
    theta = math.atan2(-math.sin(0.1 * t), -math.cos(0.1 * t)) + math.pi
    return x, y, theta


def sim_scan(
    x: float, y: float, theta: float, angles: np.ndarray, rmax: float
) -> np.ndarray:
    """Analytic 2-D LiDAR scan: ray/circle intersection against OBSTACLES."""
    beams = len(angles)
    rng = np.full(beams, rmax, dtype=np.float32)
    beam_a = angles + theta
    ca, sa = np.cos(beam_a), np.sin(beam_a)
    for cx, cy, r in OBSTACLES:
        dx, dy = cx - x, cy - y
        b_q = -2 * (dx * ca + dy * sa)
        c_q = dx**2 + dy**2 - r**2
        disc = b_q**2 - 4 * c_q
        hit = disc >= 0
        t_hit = (-b_q[hit] - np.sqrt(np.maximum(disc[hit], 0))) / 2.0
        valid = t_hit > 0.05
        rng[hit] = np.where(valid, np.minimum(rng[hit], t_hit), rng[hit])
    return np.clip(rng, 0.05, rmax)


def sim_cloud3d(
    x: float,
    y: float,
    theta: float,
    sensor_height: float,
    n_vertical: int,
    n_horizontal: int,
    elev_min_deg: float,
    elev_max_deg: float,
    rmax: float,
    obstacle_height: float,
) -> np.ndarray:
    """Analytic spinning-LiDAR point cloud: ray/cylinder intersection.

    Extends :func:`sim_scan`'s 2-D ray/circle math to 3-D: OBSTACLES become
    vertical cylinders of height ``obstacle_height``, and each beam direction
    is a unit vector ``(cos(elev)*cos(az), cos(elev)*sin(az), sin(elev))``.
    Because that direction is already unit-length, the quadratic's scalar
    solution is directly the Euclidean range (no separate 2-D/3-D distance
    conversion needed).

    Returns:
        (N, 4) float32 array of world-frame ``[x, y, z, range]`` hits.
    """
    elevations = np.radians(
        np.linspace(elev_min_deg, elev_max_deg, n_vertical, dtype=np.float32)
    )
    azimuths = (
        np.linspace(-math.pi, math.pi, n_horizontal, endpoint=False, dtype=np.float32)
        + theta
    )
    elev_grid, az_grid = np.meshgrid(elevations, azimuths, indexing="ij")

    cos_e = np.cos(elev_grid)
    ux = cos_e * np.cos(az_grid)
    uy = cos_e * np.sin(az_grid)
    uz = np.sin(elev_grid)
    a_q = np.maximum(ux**2 + uy**2, 1e-9)  # cos^2(elev); ~0 at zenith/nadir

    best_t = np.full(elev_grid.shape, np.inf, dtype=np.float32)
    for cx, cy, r in OBSTACLES:
        dx, dy = cx - x, cy - y
        b_q = -2 * (dx * ux + dy * uy)
        c_q = dx**2 + dy**2 - r**2
        disc = b_q**2 - 4 * a_q * c_q
        with np.errstate(invalid="ignore"):
            t_hit = (-b_q - np.sqrt(np.maximum(disc, 0))) / (2 * a_q)
        z_hit = sensor_height + t_hit * uz
        ok = (
            (disc >= 0)
            & (t_hit > 0.05)
            & (t_hit < rmax)
            & (z_hit >= 0.0)
            & (z_hit <= obstacle_height)
        )
        best_t = np.where(ok, np.minimum(best_t, t_hit), best_t)

    finite = np.isfinite(best_t)
    if not np.any(finite):
        return np.empty((0, 4), dtype=np.float32)

    t = best_t[finite]
    xs = x + t * ux[finite]
    ys = y + t * uy[finite]
    zs = sensor_height + t * uz[finite]
    return np.column_stack([xs, ys, zs, t]).astype(np.float32)


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
        "--lidar3d-profile",
        default="vlp16",
        metavar="NAME",
        help=f"EmbreeLidar3D.PROFILES entry (default vlp16); one of "
        f"{list(EmbreeLidar3D.PROFILES)}",
    )
    ap.add_argument("--obstacle-height", type=float, default=OBSTACLE_HEIGHT)
    ap.add_argument("--wheel-radius", type=float, default=0.05, metavar="M")
    ap.add_argument("--wheel-base", type=float, default=0.30, metavar="M")
    ap.add_argument("--motor-profile", default="small_dc", metavar="NAME")
    args = ap.parse_args()

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
    omega_l_cmd = (V_LINEAR - OMEGA_YAW * half_wheel_base) / args.wheel_radius
    omega_r_cmd = (V_LINEAR + OMEGA_YAW * half_wheel_base) / args.wheel_radius

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

                # ── 2-D LiDAR ──────────────────────────────────────────────
                rng = sim_scan(x, y, theta, angles, args.rmax)
                hits2d = int(np.sum(rng < args.rmax))
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

                # ── 3-D LiDAR ──────────────────────────────────────────────
                cloud = sim_cloud3d(
                    x,
                    y,
                    theta,
                    LIDAR_HEIGHT,
                    n_vert,
                    n_horiz,
                    elev_min,
                    elev_max,
                    args.rmax,
                    args.obstacle_height,
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
                        omega=OMEGA_YAW,
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
