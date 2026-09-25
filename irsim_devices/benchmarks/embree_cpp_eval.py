"""
embree_cpp_eval.py  —  Evaluate a direct C++ Embree4 LiDAR 2D implementation.

Motivation
----------
The Open3D Python call chain (numpy pack → Tensor() → cast_rays() → .numpy())
adds ~145-165 µs of overhead per step on top of the raw Embree BVH traversal.
This script builds a minimal pybind11 C++ extension that calls Embree4 directly,
bypassing Open3D's tensor layer entirely, and benchmarks it against O3D Python.

Prerequisites
-------------
  apt-get install -y libembree-dev
  pip install pybind11

Build the extension (run once)
-------------------------------
  python embree_cpp_eval.py --build

Then benchmark
--------------
  python embree_cpp_eval.py --bench
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import open3d.t.geometry as otg

_HERE = Path(__file__).parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "src"))

# ── Scene params (same as open3d_presets.py warehouse scene) ──────────────────
W, H = 50.0, 30.0
WALL_T = 0.2
PILLAR_R = 0.3
PILLAR_COLS = [10.0, 20.0, 30.0, 40.0]
PILLAR_ROWS = [7.5, 15.0, 22.5]
RACK_LENGTH, RACK_WIDTH = 14.0, 1.0
RACK_MARGIN = 3.0
RACK_XS = [RACK_MARGIN, W - RACK_MARGIN - RACK_LENGTH]
RACK_YS = [5.0, 10.0, 15.0, 20.0, 25.0]
EXTRUDE_H = 3.0

N_BEAMS = 1500
FOV_DEG = 360.0
RANGE_MAX = 30.0
OX, OY, THETA = 25.0, 15.0, 0.0

REPEATS = 1000
WARMUP = 200


# ── Geometry builders ─────────────────────────────────────────────────────────
def _box_mesh(cx, cy, hw, hh):
    m = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
    m.translate([cx - hw, cy - hh, 0.0])
    return m


def _cylinder_mesh(cx, cy, r, n=360):
    m = o3d.geometry.TriangleMesh.create_cylinder(
        radius=r, height=EXTRUDE_H, resolution=n, split=1
    )
    m.translate([cx, cy, EXTRUDE_H / 2])
    return m


def _all_meshes(circle_res=32):
    hw = W / 2
    meshes = [
        _box_mesh(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2),
        _box_mesh(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2),
        _box_mesh(WALL_T / 2, H / 2, WALL_T / 2, H / 2),
        _box_mesh(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2),
    ]
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            meshes.append(_cylinder_mesh(cx, cy, PILLAR_R, circle_res))
    for rx in RACK_XS:
        for ry in RACK_YS:
            hw2, hh2 = RACK_LENGTH / 2, RACK_WIDTH / 2
            meshes.append(_box_mesh(rx + hw2, ry, hw2, hh2))
    return meshes


def build_o3d_scene(circle_res=32):
    rc = otg.RaycastingScene()
    for m in _all_meshes(circle_res):
        v = o3d.core.Tensor(np.asarray(m.vertices, np.float32))
        t = o3d.core.Tensor(np.asarray(m.triangles, np.uint32))
        rc.add_triangles(otg.TriangleMesh(v, t))
    return rc


def build_triangle_soup(circle_res=32):
    """Collect all triangles into [N, 3, 3] float32 for the C++ extension."""
    all_v, all_t = [], []
    offset = 0
    for m in _all_meshes(circle_res):
        V = np.asarray(m.vertices, np.float32)
        T = np.asarray(m.triangles, np.int32) + offset
        all_v.append(V)
        all_t.append(T)
        offset += len(V)
    V = np.concatenate(all_v, 0)
    T = np.concatenate(all_t, 0)
    return V[T].astype(np.float32)  # [N_tris, 3, 3]


# ── O3D step (matches O3DLidar2D in open3d_presets.py) ───────────────────────
def make_o3d_step(scene):
    fov_rad = math.radians(FOV_DEG)
    az = np.linspace(
        -fov_rad / 2, fov_rad / 2, N_BEAMS, endpoint=False, dtype=np.float32
    )
    base_dx = np.cos(az)
    base_dy = np.sin(az)
    origs = np.zeros((N_BEAMS, 3), np.float32)
    dirs = np.zeros((N_BEAMS, 3), np.float32)

    def step(ox, oy, theta):
        ct, st = np.cos(theta), np.sin(theta)
        dirs[:, 0] = ct * base_dx - st * base_dy
        dirs[:, 1] = st * base_dx + ct * base_dy
        origs[:, 0] = ox
        origs[:, 1] = oy
        origs[:, 2] = 1.5
        rays = o3d.core.Tensor(
            np.concatenate([origs, dirs], axis=1),
            dtype=o3d.core.Dtype.Float32,
        )
        result = scene.cast_rays(rays)
        return np.minimum(result["t_hit"].numpy(), RANGE_MAX)

    return step


# ── C++ Embree step ───────────────────────────────────────────────────────────
def make_embree_step(soup, packet8=False):
    import lidar2d_embree as _ext

    lidar = _ext.Lidar2DEmbree()
    lidar.build_scene(soup)
    fov_rad = math.radians(FOV_DEG)
    lidar.set_beams(-fov_rad / 2, fov_rad / 2, N_BEAMS, RANGE_MAX)
    fn = lidar.cast_packet8 if packet8 else lidar.cast

    def step(ox, oy, theta):
        return fn(ox, oy, theta)

    return step


# ── Build extension ───────────────────────────────────────────────────────────
def build_extension():
    try:
        import pybind11

        pb_inc = pybind11.get_include()
    except ImportError:
        print("ERROR: pybind11 not installed.  pip install pybind11")
        sys.exit(1)

    import sysconfig

    py_inc = sysconfig.get_path("include")
    suffix = sysconfig.get_config_var("EXT_SUFFIX")
    src = str(_HERE / "lidar2d_embree.cpp")
    out = str(_HERE / f"lidar2d_embree{suffix}")

    cmd = [
        "g++",
        "-O3",
        "-march=native",
        "-ffast-math",
        "-shared",
        "-fPIC",
        f"-I{pb_inc}",
        f"-I{py_inc}",
        "-I/usr/include/embree4",
        src,
        "-L/usr/lib/x86_64-linux-gnu",
        "-lembree4",
        "-ltbb",
        "-o",
        out,
    ]
    print("Building lidar2d_embree extension …")
    print(" ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("STDERR:", result.stderr)
        sys.exit(1)
    print(f"Built: {out}")


# ── Benchmark ─────────────────────────────────────────────────────────────────
def bench(name, fn, warmup=WARMUP, n=REPEATS):
    for _ in range(warmup):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    us = (time.perf_counter() - t0) / n * 1e6
    print(f"  {name:28s}: {us:7.1f} µs/step")
    return us


def run_bench():
    sys.path.insert(0, str(_HERE))

    print("Building scenes (circle_res=32) …")
    o3d_scene = build_o3d_scene(circle_res=32)
    soup = build_triangle_soup(circle_res=32)
    n_tris = len(soup)
    print(f"  Triangles: {n_tris}")

    o3d_step = make_o3d_step(o3d_scene)
    cpp1_step = make_embree_step(soup, packet8=False)
    cpp8_step = make_embree_step(soup, packet8=True)

    print(f"\n{'─' * 55}")
    print(
        f"  {N_BEAMS} beams | {FOV_DEG}° FOV | {RANGE_MAX} m max | {n_tris} triangles"
    )
    print(f"{'─' * 55}")
    us_o3d = bench("O3D Python (baseline)", lambda: o3d_step(OX, OY, THETA))
    us_c1 = bench("C++ Embree scalar", lambda: cpp1_step(OX, OY, THETA))
    us_c8 = bench("C++ Embree packet8", lambda: cpp8_step(OX, OY, THETA))
    print(f"{'─' * 55}")

    print("\nSpeedup vs O3D Python:")
    print(f"  scalar : {us_o3d / us_c1:.1f}×  ({us_o3d - us_c1:.0f} µs eliminated)")
    print(f"  packet8: {us_o3d / us_c8:.1f}×  ({us_o3d - us_c8:.0f} µs eliminated)")

    # Anatomy breakdown
    print("\nO3D overhead anatomy (warehouse scene):")
    az = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False, dtype=np.float32)
    base_dx = np.cos(az)
    base_dy = np.sin(az)
    origs = np.zeros((N_BEAMS, 3), np.float32)
    dirs = np.zeros((N_BEAMS, 3), np.float32)
    origs[:, 2] = 1.5
    ct, st = math.cos(THETA), math.sin(THETA)
    dirs[:, 0] = ct * base_dx - st * base_dy
    dirs[:, 1] = st * base_dx + ct * base_dy
    ray_array = np.concatenate([origs, dirs], axis=1)
    ray_tensor = o3d.core.Tensor(ray_array, dtype=o3d.core.Dtype.Float32)
    result = o3d_scene.cast_rays(ray_tensor)

    t_cc = bench("  np.concatenate", lambda: np.concatenate([origs, dirs], axis=1))
    t_tc = bench(
        "  Tensor() construct",
        lambda: o3d.core.Tensor(ray_array, dtype=o3d.core.Dtype.Float32),
    )
    t_cr = bench("  cast_rays()", lambda: o3d_scene.cast_rays(ray_tensor))
    t_np = bench("  .numpy()", lambda: result["t_hit"].numpy())
    t_py = t_cc + t_tc + t_np
    t_internal = t_cr - us_c1  # O3D internal overhead above pure Embree
    print(f"\n  Pure Python layer (concat+Tensor+numpy) : {t_py:.0f} µs")
    print(f"  cast_rays() O3D-internal overhead       : {t_internal:.0f} µs")
    print(f"  Pure Embree BVH traversal               : {us_c1:.0f} µs")
    print(
        f"  Sum                                     : {t_py + t_internal + us_c1:.0f} µs"
    )

    results = {
        "scene": {"n_tris": n_tris, "circle_res": 32},
        "n_beams": N_BEAMS,
        "fov_deg": FOV_DEG,
        "timing_us": {
            "o3d_python": round(us_o3d, 1),
            "cpp_embree_scalar": round(us_c1, 1),
            "cpp_embree_packet8": round(us_c8, 1),
        },
        "speedup": {
            "scalar_vs_o3d": round(us_o3d / us_c1, 1),
            "packet8_vs_o3d": round(us_o3d / us_c8, 1),
        },
        "overhead_anatomy_us": {
            "np_concat": round(t_cc, 1),
            "tensor_construct": round(t_tc, 1),
            "cast_rays_internal_overhead": round(t_internal, 0),
            "numpy_unpack": round(t_np, 1),
            "pure_embree_bvh": round(us_c1, 0),
        },
        "interpretation": {
            "o3d_python_overhead_us": round(us_o3d - us_c1, 0),
            "cpp_practical_floor_us": round(us_c1, 0),
            "vs_v3_baseline_us": 1205,
            "cpp_vs_v3_speedup_x": round(1205 / us_c1, 1),
        },
    }
    out = _HERE / "embree_cpp_eval_results.json"
    with open(out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults → {out}")


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true", help="Build the C++ extension")
    parser.add_argument("--bench", action="store_true", help="Run the benchmark")
    args = parser.parse_args()

    if not args.build and not args.bench:
        parser.print_help()
        sys.exit(0)

    if args.build:
        build_extension()

    if args.bench:
        run_bench()
