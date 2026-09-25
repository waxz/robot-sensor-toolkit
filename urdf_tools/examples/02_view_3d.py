"""Example 02 — 3D wireframe viewer for a URDF file.

Usage:
    python 02_view_3d.py
    python 02_view_3d.py models/warehouse_world.urdf --save world_3d.png

    # Overlay a robot model at a world pose (x y z roll pitch yaw)
    python 02_view_3d.py models/warehouse_world.urdf \\
        --robot models/robot_diff.urdf --pose 15 10 0 0 0 1.57

    # With a point cloud (N×3 .npy)
    python 02_view_3d.py models/warehouse_world.urdf \\
        --robot models/robot_diff.urdf --pose 15 10 0 --cloud scan.npy
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from urdf_tools.parser import parse_urdf
from urdf_tools.viz import plot_3d

URDF = Path(__file__).parent / "models" / "warehouse_world.urdf"


def main() -> None:
    ap = argparse.ArgumentParser(description="3D URDF wireframe viewer")
    ap.add_argument("urdf", nargs="?", default=str(URDF), help="World/primary URDF")
    ap.add_argument(
        "--robot", default=None, metavar="URDF", help="Robot URDF to overlay"
    )
    ap.add_argument(
        "--pose",
        nargs="+",
        type=float,
        default=None,
        metavar="VAL",
        help="Robot world pose: x y z [roll pitch yaw]  (default: 0 0 0)",
    )
    ap.add_argument(
        "--cloud", default=None, metavar="NPY", help=".npy file with (N,3) point cloud"
    )
    ap.add_argument("--save", default=None, metavar="FILE")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    world = parse_urdf(args.urdf)
    print(f"World: {world.name!r}  links={len(world.links)}")

    overlay = []
    if args.robot:
        robot = parse_urdf(args.robot)
        pose = list(args.pose) if args.pose else [0.0, 0.0, 0.0]
        # Pad to 6 values (xyzrpy)
        pose = (pose + [0.0, 0.0, 0.0])[:6]
        print(f"Robot: {robot.name!r}  pose={pose}")
        overlay.append((robot, pose))

    cloud = None
    if args.cloud:
        cloud = np.load(args.cloud)
        if cloud.ndim == 1:
            cloud = cloud.reshape(-1, 3)
        print(f"Cloud: {cloud.shape}")

    plot_3d(world, cloud=cloud, overlay=overlay, save=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
