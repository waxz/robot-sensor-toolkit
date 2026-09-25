"""Example 01 — 2D floor plan from a URDF file.

Usage:
    python 01_view_2d.py
    python 01_view_2d.py --save floor_plan.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from urdf_tools.parser import parse_urdf
from urdf_tools.viz import plot_floor_plan

URDF = Path(__file__).parent / "models" / "warehouse_world.urdf"


def main() -> None:
    ap = argparse.ArgumentParser(description="2D URDF floor plan viewer")
    ap.add_argument("urdf", nargs="?", default=str(URDF), help="URDF file path")
    ap.add_argument("--save", default=None, metavar="FILE", help="Save to PNG/PDF")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    robot = parse_urdf(args.urdf)
    print(
        f"Robot: {robot.name!r}  links={len(robot.links)}  joints={len(robot.joints)}"
    )
    plot_floor_plan(robot, save=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
