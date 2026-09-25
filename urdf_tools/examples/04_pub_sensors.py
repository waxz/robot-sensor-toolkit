"""Example 04 — Simulate and publish LiDAR / IMU / odometry via shmbridge.

Requires shmbridge installed:
    cd ../shmbridge && pip install -e .

Usage:
    python 04_pub_sensors.py               # default 20 Hz, 1080 beams
    python 04_pub_sensors.py --rate 10 --beams 360
    python 04_pub_sensors.py --shm /my_seg
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from urdf_tools.pubsub import Imu, LaserScan, Odometry, SensorPublisher

SHM_NAME = "/urdf_tools_sensors"
RATE_HZ = 20.0
BEAMS = 1080
RMAX = 20.0

OBSTACLES = [(3.0, 3.0, 0.5), (-4.0, 2.0, 0.8), (1.0, -5.0, 1.2)]


def sim_scan(
    x: float, y: float, theta: float, angles: np.ndarray, rmax: float
) -> np.ndarray:
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


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="Publish simulated sensors via shmbridge")
    ap.add_argument("--shm", default=SHM_NAME)
    ap.add_argument("--rate", type=float, default=RATE_HZ)
    ap.add_argument("--beams", type=int, default=BEAMS)
    ap.add_argument("--rmax", type=float, default=RMAX)
    args = ap.parse_args()

    angles = np.linspace(
        -math.pi, math.pi, args.beams, endpoint=False, dtype=np.float32
    )
    a_inc = float(2 * math.pi / args.beams)

    print(f"Publishing on shm={args.shm!r}  rate={args.rate}Hz  beams={args.beams}")
    print("Ctrl+C to stop.")
    t0 = time.time()

    with SensorPublisher(args.shm) as pub:
        try:
            while True:
                t = time.time() - t0
                x = 5.0 * math.cos(0.1 * t)
                y = 5.0 * math.sin(0.1 * t)
                theta = math.atan2(-math.sin(0.1 * t), -math.cos(0.1 * t)) + math.pi
                rng = sim_scan(x, y, theta, angles, args.rmax)
                hits = int(np.sum(rng < args.rmax))

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
                pub.publish_imu(
                    Imu(
                        stamp=t,
                        linear_acceleration=[0.0, 0.0, 9.81],
                        angular_velocity=[0.0, 0.0, float(0.1 * math.cos(0.1 * t))],
                    )
                )
                pub.publish_odom(
                    Odometry(
                        stamp=t,
                        x=float(x),
                        y=float(y),
                        theta=float(theta),
                        vx=0.5,
                        omega=0.1,
                    )
                )
                print(
                    f"\r  t={t:6.1f}s  ({x:5.2f},{y:5.2f})  θ={theta:.2f}"
                    f"  hits={hits}/{args.beams}",
                    end="",
                    flush=True,
                )
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
