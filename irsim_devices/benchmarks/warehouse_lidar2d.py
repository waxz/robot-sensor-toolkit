"""Warehouse LiDAR benchmark: correctness + performance evaluation.

Builds a 50 × 30 m warehouse (walls, shelf racks, pillars) using
Open3DScene2D, runs Lidar2D at multiple beam counts, checks range
accuracy against analytical geometry, and serialises results to JSON.

Usage::

    cd irsim_devices
    python benchmarks/warehouse_lidar2d.py [--out-dir /path/to/output]

The output directory defaults to the current working directory.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

# ── path setup ────────────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from irsim_devices.core.open3d_scene_2d import Open3DScene2D  # noqa: E402
from irsim_devices.sensors import Lidar2D  # noqa: E402

# ── warehouse dimensions ──────────────────────────────────────────────────────
W, H = 50.0, 30.0  # outer floor dimensions (m)
WALL_T = 0.2  # wall thickness

# Pillar grid: 4 columns × 3 rows, radius 0.3 m
PILLAR_R = 0.3
PILLAR_COLS = [10.0, 20.0, 30.0, 40.0]
PILLAR_ROWS = [7.5, 15.0, 22.5]

# Shelf racks: 2 columns × 5 rows, 14 m × 1 m each
RACK_LENGTH = 14.0
RACK_WIDTH = 1.0
RACK_MARGIN = 3.0  # gap between rack end and side wall
RACK_XS = [RACK_MARGIN, W - RACK_MARGIN - RACK_LENGTH]  # left / right column
RACK_YS = [5.0, 10.0, 15.0, 20.0, 25.0]


def build_warehouse_scene() -> tuple[Open3DScene2D, dict]:
    """Return (scene, geometry_info) for the synthetic warehouse."""
    scene = Open3DScene2D()
    geoms: dict[str, list] = {"walls": [], "pillars": [], "racks": []}

    hw, hh = W / 2, H / 2

    # Outer walls
    scene.add_box(center=(hw, WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    geoms["walls"].append(("south", 0.0))
    scene.add_box(center=(hw, H - WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    geoms["walls"].append(("north", H))
    scene.add_box(center=(WALL_T / 2, hh), width=WALL_T / 2, height=hh + WALL_T)
    geoms["walls"].append(("west", 0.0))
    scene.add_box(center=(W - WALL_T / 2, hh), width=WALL_T / 2, height=hh + WALL_T)
    geoms["walls"].append(("east", W))

    # Pillars
    for px in PILLAR_COLS:
        for py in PILLAR_ROWS:
            scene.add_circle(center=(px, py), radius=PILLAR_R, resolution=24)
            geoms["pillars"].append((px, py, PILLAR_R))

    # Shelf racks
    for rx in RACK_XS:
        for ry in RACK_YS:
            cx = rx + RACK_LENGTH / 2
            scene.add_box(center=(cx, ry), width=RACK_LENGTH / 2, height=RACK_WIDTH / 2)
            geoms["racks"].append((cx, ry, RACK_LENGTH, RACK_WIDTH))

    return scene, geoms


# ── correctness evaluation ────────────────────────────────────────────────────


def _make_lidar(ox, oy, scene, number=3600, range_max=55.0):
    lidar = Lidar2D(
        state=[ox, oy, 0.0],
        obj_id=0,
        range_max=range_max,
        angle_range=2 * math.pi,
        number=number,
    )
    lidar.scene = scene
    lidar.step([ox, oy, 0.0])
    angles = np.linspace(-math.pi, math.pi, number, endpoint=False)

    def beam(a):
        return float(lidar.range_data[int(np.argmin(np.abs(angles - a)))])

    return beam


def check_correctness(scene: Open3DScene2D, tol: float = 0.05) -> list[dict]:
    """Check LiDAR range accuracy against analytically known distances.

    Three test scenarios with analytically known nearest obstacles:
      1. Open aisle (no pillars/racks on cardinal beams) → walls
      2. Pillar row alignment → pillar surface distances
      3. Rack face alignment → rack surface distances
    """

    def check(measured, expected, label):
        err = abs(float(measured) - float(expected))
        return {
            "label": label,
            "measured": round(float(measured), 4),
            "expected": round(float(expected), 4),
            "error": round(float(err), 4),
            "pass": bool(err < tol),
        }

    results = []

    # ── scenario 1: open aisle — clear sight to all four walls ───────────────
    # Position (25, 3): south of first rack row (y=4.5), no pillars in path.
    # S/N/W/E beams reach the walls without obstruction.
    ox1, oy1 = 25.0, 3.0
    b1 = _make_lidar(ox1, oy1, scene)
    results.append(
        {
            "origin": [ox1, oy1],
            "scenario": "open aisle — clear line-of-sight to all four walls",
            "checks": [
                check(b1(-math.pi / 2), oy1 - WALL_T, "south wall (y = 0.2)"),
                check(b1(math.pi / 2), H - oy1 - WALL_T, "north wall (y = 29.8)"),
                check(b1(math.pi), ox1 - WALL_T, "west wall  (x = 0.2)"),
                check(b1(0.0), W - ox1 - WALL_T, "east wall  (x = 49.8)"),
            ],
        }
    )

    # ── scenario 2: pillar row — west/east beams hit pillars ─────────────────
    # Origin (25, 7.5): collinear with pillar row at y=7.5.
    # West beam hits pillar at (20, 7.5):  25 - 20 - 0.3 = 4.7
    # East beam hits pillar at (30, 7.5):  30 - 25 - 0.3 = 4.7
    # South beam reaches south wall:       7.5 - 0.2 = 7.3
    ox2, oy2 = 25.0, 7.5
    b2 = _make_lidar(ox2, oy2, scene)
    results.append(
        {
            "origin": [ox2, oy2],
            "scenario": "pillar row — west/east beams should hit pillar surfaces",
            "checks": [
                check(
                    b2(math.pi),
                    (ox2 - 20.0) - PILLAR_R,
                    "pillar (20, 7.5) via west beam",
                ),
                check(
                    b2(0.0), (30.0 - ox2) - PILLAR_R, "pillar (30, 7.5) via east beam"
                ),
                check(b2(-math.pi / 2), oy2 - WALL_T, "south wall via south beam"),
            ],
        }
    )

    # ── scenario 3: inside left rack corridor ─────────────────────────────────
    # Origin (10, 12.5): x=10 is inside the left rack zone (x ∈ [3, 17]);
    # y=12.5 is in the open gap between racks at y=10 and y=15.
    # North beam: hits bottom face of rack at y=15 → 14.5 − 12.5 = 2.0
    # South beam: hits top face of rack at y=10  → 12.5 − 10.5 = 2.0
    # West beam:  hits west wall inner face       → 10.0 − 0.2  = 9.8
    ox3, oy3 = 10.0, 12.5
    b3 = _make_lidar(ox3, oy3, scene)
    results.append(
        {
            "origin": [ox3, oy3],
            "scenario": "inside left rack corridor — north/south beams hit rack faces",
            "checks": [
                check(
                    b3(math.pi / 2),
                    (15.0 - RACK_WIDTH / 2) - oy3,
                    "rack at y=15 south face",
                ),
                check(
                    b3(-math.pi / 2),
                    oy3 - (10.0 + RACK_WIDTH / 2),
                    "rack at y=10 north face",
                ),
                check(b3(math.pi), ox3 - WALL_T, "west wall"),
            ],
        }
    )

    return results


# ── performance benchmark ─────────────────────────────────────────────────────


def benchmark_lidar2d(
    scene: Open3DScene2D,
    origin: tuple[float, float] = (25.0, 15.0),
    beam_counts: list[int] | None = None,
    repeats: int = 30,
) -> list[dict]:
    """Time Lidar2D.step() at various beam counts."""
    if beam_counts is None:
        beam_counts = [72, 180, 360, 720, 1080, 1800, 3600]
    ox, oy = origin
    results = []
    for n in beam_counts:
        lidar = Lidar2D(
            state=[ox, oy, 0.0],
            obj_id=0,
            range_max=40.0,
            angle_range=2 * math.pi,
            number=n,
        )
        lidar.scene = scene
        lidar.step([ox, oy, 0.0])  # warm-up

        times = []
        for _ in range(repeats):
            t0 = time.perf_counter()
            lidar.step([ox, oy, 0.0])
            times.append(time.perf_counter() - t0)

        arr = np.array(times) * 1000  # → ms
        results.append(
            {
                "beams": n,
                "mean_ms": round(float(arr.mean()), 3),
                "min_ms": round(float(arr.min()), 3),
                "max_ms": round(float(arr.max()), 3),
                "p50_ms": round(float(np.percentile(arr, 50)), 3),
                "p95_ms": round(float(np.percentile(arr, 95)), 3),
                "hz": round(float(1000.0 / arr.mean()), 1),
            }
        )
        print(f"  {n:5d} beams → {arr.mean():.2f} ms  ({1000 / arr.mean():.0f} Hz)")
    return results


# ── scan collection ───────────────────────────────────────────────────────────


def collect_multi_scans(scene, number=360, range_max=40.0):
    positions = [
        (25.0, 15.0, "warehouse centre"),
        (25.0, 3.0, "south aisle"),
        (10.0, 8.0, "west rack zone"),
    ]
    scans = []
    for ox, oy, label in positions:
        lidar = Lidar2D(
            state=[ox, oy, 0.0],
            obj_id=0,
            range_max=range_max,
            angle_range=2 * math.pi,
            number=number,
        )
        lidar.scene = scene
        lidar.step([ox, oy, 0.0])
        angles_deg = np.degrees(np.linspace(-math.pi, math.pi, number, endpoint=False))
        ranges = lidar.range_data
        angles_rad = np.radians(angles_deg)
        px = (ox + ranges * np.cos(angles_rad)).tolist()
        py = (oy + ranges * np.sin(angles_rad)).tolist()
        scans.append(
            {
                "origin": [ox, oy],
                "label": label,
                "point_x": px,
                "point_y": py,
            }
        )
    return scans


# ── main ──────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="Warehouse LiDAR 2D benchmark")
    parser.add_argument(
        "--out-dir", default=".", help="Output directory for JSON results"
    )
    args = parser.parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Building warehouse scene …")
    scene, geom_info = build_warehouse_scene()
    print(f"  {scene!r}")

    print("\nCorrectness checks …")
    correctness = check_correctness(scene)
    for scenario in correctness:
        print(f"  [{scenario['origin']}] {scenario['scenario']}")
        for c in scenario["checks"]:
            status = "PASS" if c["pass"] else "FAIL"
            print(
                f"    [{status}] {c['label']}: "
                f"measured={c['measured']:.4f}  expected={c['expected']:.4f}  "
                f"err={c['error']:.4f}"
            )

    print("\nPerformance benchmark (centre of warehouse) …")
    perf = benchmark_lidar2d(scene, origin=(25.0, 15.0))

    print("\nCollecting scans for visualisation …")
    scans = collect_multi_scans(scene)

    floor_plan = {
        "width": W,
        "height": H,
        "wall_t": WALL_T,
        "pillars": [{"cx": p[0], "cy": p[1], "r": p[2]} for p in geom_info["pillars"]],
        "racks": [
            {"cx": r[0], "cy": r[1], "w": r[2], "h": r[3]} for r in geom_info["racks"]
        ],
    }

    payload = {
        "floor_plan": floor_plan,
        "correctness": correctness,
        "performance": perf,
        "scans": scans,
    }

    out_path = out_dir / "warehouse_results.json"
    out_path.write_text(json.dumps(payload, indent=2))
    print(f"\nResults saved → {out_path}")
    return payload


if __name__ == "__main__":
    main()
