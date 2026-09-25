"""Example 03 — Kinematic tree diagram for a URDF file.

Usage:
    python 03_kinematic_tree.py
    python 03_kinematic_tree.py models/robot_diff.urdf --save tree.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from urdf_tools.parser import parse_urdf
from urdf_tools.viz import plot_kinematic_tree

URDF = Path(__file__).parent / "models" / "robot_diff.urdf"


def main() -> None:
    ap = argparse.ArgumentParser(description="URDF kinematic tree diagram")
    ap.add_argument("urdf", nargs="?", default=str(URDF), help="URDF file path")
    ap.add_argument("--save", default=None, metavar="FILE")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    robot = parse_urdf(args.urdf)
    print(f"Robot: {robot.name!r}")
    for j in robot.joints:
        print(f"  {j.parent:25s} --[{j.type}]--> {j.child}")
    plot_kinematic_tree(robot, save=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
