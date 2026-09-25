"""
open3d_comparison.py — LiDAR 2D: custom AVX2 kernel vs Open3D Embree backend.

Measures for 1 500 beams / 824 segments / 30 m range:
  1. Build time   — BVH construction (Open3D) vs SoA precompute (v3)
  2. Step latency — p50 / p95 for hot path at stationary + moving poses
  3. Output accuracy — range-array comparison
  4. Per-call overhead anatomy — ray-tensor packing, cast_rays(), unpack
  5. CPU at 30 Hz

Results written to open3d_comparison_results.json.
"""

from __future__ import annotations

import json
import math
import sys
import time
from math import pi
from pathlib import Path
from time import perf_counter

import numpy as np
import open3d as o3d
import open3d.t.geometry as otg
import psutil

_HERE = Path(__file__).parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "src"))

from irsim_devices.core.open3d_scene_2d import Open3DScene2D  # noqa: E402
from irsim_devices.sensors.lidar2d import Lidar2D  # noqa: E402

# ── Scene geometry (identical to bottleneck_analysis.py) ─────────────────────
W, H = 50.0, 30.0
WALL_T = 0.2
PILLAR_R = 0.3
PILLAR_COLS = [10.0, 20.0, 30.0, 40.0]
PILLAR_ROWS = [7.5, 15.0, 22.5]
RACK_LENGTH, RACK_WIDTH = 14.0, 1.0
RACK_MARGIN = 3.0
RACK_XS = [RACK_MARGIN, W - RACK_MARGIN - RACK_LENGTH]
RACK_YS = [5.0, 10.0, 15.0, 20.0, 25.0]


def _make_open3d_scene_2d() -> Open3DScene2D:
    """Build the Open3DScene2D used by the existing v3 lidar2d."""
    scene = Open3DScene2D()
    hw = W / 2
    scene.add_box(center=(hw, WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(hw, H - WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    scene.add_box(center=(W - WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            scene.add_circle(center=(cx, cy), radius=PILLAR_R, resolution=16)
    for rx in RACK_XS:
        for ry in RACK_YS:
            scene.add_box(
                center=(rx + RACK_LENGTH / 2, ry),
                width=RACK_LENGTH / 2,
                height=RACK_WIDTH / 2,
            )
    return scene


def _make_scene3d() -> otg.RaycastingScene:
    """
    Build the equivalent scene as a 3D Open3D RaycastingScene.

    Each 2D shape is extruded to height = 3 m so horizontal rays at z = 1.5 m
    always intersect them.  This mirrors what scene3d.py does for a real 3D env.
    """
    EXTRUDE_H = 3.0
    Z_BOT = 0.0

    def _box_mesh(cx, cy, half_w, half_h, extrude=EXTRUDE_H):
        mesh = o3d.geometry.TriangleMesh.create_box(
            width=half_w * 2, height=half_h * 2, depth=extrude
        )
        mesh.translate([cx - half_w, cy - half_h, Z_BOT])
        return mesh

    def _circle_mesh(cx, cy, r, n=32, extrude=EXTRUDE_H):
        angles = np.linspace(0, 2 * pi, n, endpoint=False)
        verts_xy = np.stack([cx + r * np.cos(angles), cy + r * np.sin(angles)], axis=1)
        verts_bot = np.column_stack([verts_xy, np.zeros(n)])
        verts_top = np.column_stack([verts_xy, np.full(n, extrude)])
        center_bot = np.array([[cx, cy, 0.0]])
        center_top = np.array([[cx, cy, extrude]])
        verts = np.vstack([verts_bot, verts_top, center_bot, center_top])
        tris = []
        cbi = 2 * n
        cti = 2 * n + 1
        for i in range(n):
            j = (i + 1) % n
            # side quad
            tris += [[i, j, i + n], [j, j + n, i + n]]
            # bottom cap
            tris.append([cbi, j, i])
            # top cap
            tris.append([cti, i + n, j + n])
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts.astype(np.float32))
        mesh.triangles = o3d.utility.Vector3iVector(np.array(tris, dtype=np.int32))
        return mesh

    rc = otg.RaycastingScene()

    def _add(mesh):
        v = o3d.core.Tensor(np.asarray(mesh.vertices, dtype=np.float32))
        t = o3d.core.Tensor(np.asarray(mesh.triangles, dtype=np.uint32))
        rc.add_triangles(otg.TriangleMesh(v, t))

    hw = W / 2
    _add(_box_mesh(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2))
    _add(_box_mesh(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2))
    _add(_box_mesh(WALL_T / 2, H / 2, WALL_T / 2, H / 2))
    _add(_box_mesh(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2))
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            _add(_circle_mesh(cx, cy, PILLAR_R))
    for rx in RACK_XS:
        for ry in RACK_YS:
            _add(_box_mesh(rx + RACK_LENGTH / 2, ry, RACK_LENGTH / 2, RACK_WIDTH / 2))

    return rc


def _make_v3_lidar(n_beams=1500, range_max=30.0, ox=25.0, oy=15.0) -> Lidar2D:
    lidar = Lidar2D(
        state=np.array([[ox], [oy], [0.0]]),
        range_max=range_max,
        angle_range=2 * pi,
        number=n_beams,
        noise=False,
    )
    return lidar


def _pct(arr, p):
    return float(np.percentile(arr, p))


# ── 1. Build time ─────────────────────────────────────────────────────────────


def bench_build(n_reps=30):
    t_2d, t_3d = [], []

    for _ in range(n_reps):
        t0 = perf_counter()
        _make_open3d_scene_2d()
        t_2d.append((perf_counter() - t0) * 1e3)

    for _ in range(n_reps):
        t0 = perf_counter()
        _make_scene3d()
        t_3d.append((perf_counter() - t0) * 1e3)

    # Also time the set_scene (SoA precompute) for v3
    sc2d_warm = _make_open3d_scene_2d()
    lidar = _make_v3_lidar()
    t_attach = []
    for _ in range(n_reps):
        t0 = perf_counter()
        lidar.set_scene(sc2d_warm, n_omp_threads=2, filter_margin=1.0)
        t_attach.append((perf_counter() - t0) * 1e3)

    return {
        "scene2d_build_ms": {"p50": _pct(t_2d, 50), "p95": _pct(t_2d, 95)},
        "scene3d_bvh_build_ms": {"p50": _pct(t_3d, 50), "p95": _pct(t_3d, 95)},
        "v3_set_scene_ms": {"p50": _pct(t_attach, 50), "p95": _pct(t_attach, 95)},
    }


# ── 2. Step latency ───────────────────────────────────────────────────────────

Z_HEIGHT = 1.5  # ray height for Open3D cast_2d_lidar


def _o3d_step(rc: otg.RaycastingScene, ox, oy, n_beams, range_max) -> np.ndarray:
    """Minimal hot-path for Open3D 2D lidar step (matches scene3d.cast_2d_lidar)."""
    origin = np.array([ox, oy, Z_HEIGHT], dtype=np.float32)
    az = np.linspace(-pi, pi, n_beams, endpoint=False, dtype=np.float32)
    dx = np.cos(az)
    dy = np.sin(az)
    dz = np.zeros(n_beams, dtype=np.float32)
    dirs = np.stack([dx, dy, dz], axis=1)
    origs = np.tile(origin, (n_beams, 1))
    rays = o3d.core.Tensor(
        np.concatenate([origs, dirs], axis=1), dtype=o3d.core.Dtype.Float32
    )
    result = rc.cast_rays(rays)
    t_hit = result["t_hit"].numpy()
    t_hit = np.where(np.isfinite(t_hit) & (t_hit < range_max), t_hit, range_max)
    return t_hit.astype(np.float32)


def _o3d_step_prealloc(
    rc: otg.RaycastingScene,
    ox: float,
    oy: float,
    n_beams: int,
    range_max: float,
    az_precomp: np.ndarray,
    dirs_prealloc: np.ndarray,
) -> np.ndarray:
    """Open3D step with pre-computed angles (saves np.linspace + trig cost)."""
    origin = np.array([ox, oy, Z_HEIGHT], dtype=np.float32)
    origs = np.tile(origin, (n_beams, 1))
    rays = o3d.core.Tensor(
        np.concatenate([origs, dirs_prealloc], axis=1), dtype=o3d.core.Dtype.Float32
    )
    result = rc.cast_rays(rays)
    t_hit = result["t_hit"].numpy()
    t_hit = np.where(np.isfinite(t_hit) & (t_hit < range_max), t_hit, range_max)
    return t_hit.astype(np.float32)


def bench_step(n_beams=1500, range_max=30.0, n_runs=2000):
    N = n_beams
    rmax = range_max

    # Build scenes
    sc2d = _make_open3d_scene_2d()
    rc = _make_scene3d()

    # Prepare v3 lidar
    lidar = _make_v3_lidar(n_beams=N, range_max=rmax)
    lidar.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)

    # Pre-computed directions for the optimised Open3D variant
    az = np.linspace(-pi, pi, N, endpoint=False, dtype=np.float32)
    o3d_dirs = np.stack([np.cos(az), np.sin(az), np.zeros(N, dtype=np.float32)], axis=1)

    ox, oy, theta = 25.0, 15.0, 0.0
    state = np.array([[ox], [oy], [theta]])

    # Warm up both
    for _ in range(50):
        lidar.step(state)
        _o3d_step(rc, ox, oy, N, rmax)

    t_v3, t_o3d_naive, t_o3d_prealloc = [], [], []

    for _ in range(n_runs):
        t0 = perf_counter()
        lidar.step(state)
        t_v3.append((perf_counter() - t0) * 1e6)

        t0 = perf_counter()
        _o3d_step(rc, ox, oy, N, rmax)
        t_o3d_naive.append((perf_counter() - t0) * 1e6)

        t0 = perf_counter()
        _o3d_step_prealloc(rc, ox, oy, N, rmax, az, o3d_dirs)
        t_o3d_prealloc.append((perf_counter() - t0) * 1e6)

    return {
        "v3_us": {"p50": _pct(t_v3, 50), "p95": _pct(t_v3, 95)},
        "o3d_naive_us": {"p50": _pct(t_o3d_naive, 50), "p95": _pct(t_o3d_naive, 95)},
        "o3d_prealloc_us": {
            "p50": _pct(t_o3d_prealloc, 50),
            "p95": _pct(t_o3d_prealloc, 95),
        },
    }


# ── 3. Anatomy: per-phase cost inside Open3D step ─────────────────────────────


def bench_o3d_anatomy(n_beams=1500, range_max=30.0, n_runs=2000):
    N = n_beams
    rc = _make_scene3d()
    ox, oy = 25.0, 15.0
    origin = np.array([ox, oy, Z_HEIGHT], dtype=np.float32)

    # Pre-compute
    az = np.linspace(-pi, pi, N, endpoint=False, dtype=np.float32)
    dirs = np.stack([np.cos(az), np.sin(az), np.zeros(N, dtype=np.float32)], axis=1)
    origs = np.tile(origin, (N, 1))
    ray_np = np.concatenate([origs, dirs], axis=1)

    # Warm up
    rays = o3d.core.Tensor(ray_np, dtype=o3d.core.Dtype.Float32)
    for _ in range(30):
        rc.cast_rays(rays)

    t_pack, t_cast, t_unpack = [], [], []

    for _ in range(n_runs):
        # Phase A: numpy → o3d.core.Tensor (always regenerate tile+concat)
        t0 = perf_counter()
        origs2 = np.tile(origin, (N, 1))
        _rays_local = o3d.core.Tensor(
            np.concatenate([origs2, dirs], axis=1), dtype=o3d.core.Dtype.Float32
        )
        t_pack.append((perf_counter() - t0) * 1e6)

        # Phase B: cast_rays (Embree BVH traversal)
        t0 = perf_counter()
        result = rc.cast_rays(_rays_local)
        t_cast.append((perf_counter() - t0) * 1e6)

        # Phase C: t_hit tensor → numpy + isfinite filter
        t0 = perf_counter()
        t_hit = result["t_hit"].numpy()
        np.where(np.isfinite(t_hit) & (t_hit < range_max), t_hit, range_max)
        t_unpack.append((perf_counter() - t0) * 1e6)

    return {
        "pack_tile_concat_us": {"p50": _pct(t_pack, 50), "p95": _pct(t_pack, 95)},
        "cast_rays_us": {"p50": _pct(t_cast, 50), "p95": _pct(t_cast, 95)},
        "unpack_us": {"p50": _pct(t_unpack, 50), "p95": _pct(t_unpack, 95)},
    }


# ── 4. Accuracy: do both return the same ranges? ─────────────────────────────


def check_accuracy(n_beams=1500, range_max=30.0):
    sc2d = _make_open3d_scene_2d()
    rc = _make_scene3d()

    lidar = _make_v3_lidar(n_beams=n_beams, range_max=range_max)
    lidar.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)
    lidar._filter_cache_x = math.inf  # force cache miss

    ox, oy, theta = 25.0, 15.0, 0.0
    state = np.array([[ox], [oy], [theta]])
    lidar.step(state)
    v3_ranges = np.array(lidar.range_data, dtype=np.float32)

    o3d_t_hit = _o3d_step(rc, ox, oy, n_beams, range_max)

    # Compare: v3 returns range_data (n,); o3d returns t_hit which equals range
    diff = np.abs(v3_ranges - o3d_t_hit)
    finite_mask = (v3_ranges < range_max) & (o3d_t_hit < range_max)

    return {
        "max_diff_m": float(np.max(diff)),
        "mean_diff_hit_m": float(np.mean(diff[finite_mask]))
        if finite_mask.any()
        else 0.0,
        "v3_hits": int(np.sum(v3_ranges < range_max)),
        "o3d_hits": int(np.sum(o3d_t_hit < range_max)),
        "hit_agreement_pct": float(100.0 * np.sum(finite_mask) / n_beams),
    }


# ── 5. Scale: latency vs segment count ───────────────────────────────────────


def bench_scale(n_beams=1500, n_runs=500):
    """Compare how each backend scales as the scene grows."""
    segment_counts = [100, 400, 824, 2000, 5000]
    results = {}

    for M_target in segment_counts:
        # Build a scene with ~M_target segments (circles with varying resolution)
        sc2d = Open3DScene2D()
        rc2 = otg.RaycastingScene()
        n_added = 0
        r = 0.3
        for i in range(200):
            cx = 5.0 + (i % 20) * 2.0
            cy = 2.0 + (i // 20) * 2.0
            res = max(4, min(32, M_target // max(1, i + 1)))
            sc2d.add_circle(center=(cx, cy), radius=r, resolution=res)

            # Extrude circle for o3d
            angles = np.linspace(0, 2 * pi, res, endpoint=False)
            vx = cx + r * np.cos(angles)
            vy = cy + r * np.sin(angles)
            verts_b = np.column_stack([vx, vy, np.zeros(res)])
            verts_t = np.column_stack([vx, vy, np.full(res, 3.0)])
            cb = np.array([[cx, cy, 0.0]])
            ct = np.array([[cx, cy, 3.0]])
            vs = np.vstack([verts_b, verts_t, cb, ct])
            cbi, cti = 2 * res, 2 * res + 1
            tris = []
            for k in range(res):
                j = (k + 1) % res
                tris += [[k, j, k + res], [j, j + res, k + res]]
                tris.append([cbi, j, k])
                tris.append([cti, k + res, j + res])
            vt = o3d.core.Tensor(vs.astype(np.float32))
            tt = o3d.core.Tensor(np.array(tris, dtype=np.uint32))
            rc2.add_triangles(otg.TriangleMesh(vt, tt))

            # Count actual segments
            M_now = sum(len(obj.linestrings[0].coords) - 1 for obj in sc2d.objects)
            n_added = M_now
            if n_added >= M_target:
                break

        lidar = _make_v3_lidar(n_beams=n_beams, range_max=30.0)
        lidar.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)
        M_actual = len(lidar._all_seg_sx)

        # Pre-compute o3d dirs
        az = np.linspace(-pi, pi, n_beams, endpoint=False, dtype=np.float32)
        dirs = np.stack([np.cos(az), np.sin(az), np.zeros(n_beams, np.float32)], axis=1)

        ox, oy = 25.0, 15.0
        state = np.array([[ox], [oy], [0.0]])

        # Warm up
        for _ in range(20):
            lidar.step(state)
            _o3d_step_prealloc(rc2, ox, oy, n_beams, 30.0, az, dirs)

        tv3, to3d = [], []
        for _ in range(n_runs):
            lidar._filter_cache_x = math.inf  # force cache miss to measure full cost
            t0 = perf_counter()
            lidar.step(state)
            tv3.append((perf_counter() - t0) * 1e6)

            t0 = perf_counter()
            _o3d_step_prealloc(rc2, ox, oy, n_beams, 30.0, az, dirs)
            to3d.append((perf_counter() - t0) * 1e6)

        results[str(M_actual)] = {
            "M": M_actual,
            "v3_p50_us": _pct(tv3, 50),
            "o3d_p50_us": _pct(to3d, 50),
        }

    return results


# ── 6. CPU at 30 Hz ──────────────────────────────────────────────────────────


def bench_cpu_30hz(n_beams=1500, range_max=30.0, duration=5.0):
    sc2d = _make_open3d_scene_2d()
    rc = _make_scene3d()

    lidar = _make_v3_lidar(n_beams=n_beams, range_max=range_max)
    lidar.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)

    az = np.linspace(-pi, pi, n_beams, endpoint=False, dtype=np.float32)
    o3d_dirs = np.stack([np.cos(az), np.sin(az), np.zeros(n_beams, np.float32)], axis=1)

    ox, oy = 25.0, 15.0
    state = np.array([[ox], [oy], [0.0]])
    interval = 1.0 / 30.0

    proc = psutil.Process()

    def _run(step_fn, label):
        proc.cpu_percent()
        time.sleep(0.2)
        proc.cpu_percent()
        t_end = time.monotonic() + duration
        n = 0
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            step_fn()
            elapsed = time.monotonic() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)
            n += 1
        return proc.cpu_percent(), n

    cpu_v3, n_v3 = _run(lambda: lidar.step(state), "v3")
    time.sleep(1.0)
    cpu_o3d, n_o3d = _run(
        lambda: _o3d_step_prealloc(rc, ox, oy, n_beams, range_max, az, o3d_dirs), "o3d"
    )

    return {
        "v3_cpu_pct": cpu_v3,
        "v3_steps": n_v3,
        "o3d_cpu_pct": cpu_o3d,
        "o3d_steps": n_o3d,
    }


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    N_BEAMS = 1500
    RANGE_MAX = 30.0
    print("=== LiDAR 2D: v3 (AVX2 + pose cache) vs Open3D Embree comparison ===\n")

    print("[1/6] Build times …")
    build = bench_build()
    print(f"  Open3DScene2D build : {build['scene2d_build_ms']['p50']:.1f} ms p50")
    print(f"  Embree BVH build    : {build['scene3d_bvh_build_ms']['p50']:.1f} ms p50")
    print(f"  v3 set_scene        : {build['v3_set_scene_ms']['p50']:.3f} ms p50")

    print("\n[2/6] Step latency (stationary, 2 000 runs) …")
    step = bench_step(N_BEAMS, RANGE_MAX, n_runs=2000)
    print(
        f"  v3 (AVX2 + cache)   : {step['v3_us']['p50']:.0f} µs p50"
        f" / {step['v3_us']['p95']:.0f} µs p95"
    )
    print(
        f"  Open3D naive        : {step['o3d_naive_us']['p50']:.0f} µs p50"
        f" / {step['o3d_naive_us']['p95']:.0f} µs p95"
    )
    print(
        f"  Open3D pre-alloc    : {step['o3d_prealloc_us']['p50']:.0f} µs p50"
        f" / {step['o3d_prealloc_us']['p95']:.0f} µs p95"
    )

    print("\n[3/6] Open3D per-phase anatomy …")
    anatomy = bench_o3d_anatomy(N_BEAMS, RANGE_MAX, n_runs=2000)
    total_us = (
        anatomy["pack_tile_concat_us"]["p50"]
        + anatomy["cast_rays_us"]["p50"]
        + anatomy["unpack_us"]["p50"]
    )
    print(
        f"  pack (tile+concat→Tensor): {anatomy['pack_tile_concat_us']['p50']:.0f} µs"
        f" ({100 * anatomy['pack_tile_concat_us']['p50'] / total_us:.0f}%)"
    )
    print(
        f"  cast_rays (Embree BVH)  : {anatomy['cast_rays_us']['p50']:.0f} µs"
        f" ({100 * anatomy['cast_rays_us']['p50'] / total_us:.0f}%)"
    )
    print(
        f"  unpack (.numpy+filter)  : {anatomy['unpack_us']['p50']:.0f} µs"
        f" ({100 * anatomy['unpack_us']['p50'] / total_us:.0f}%)"
    )

    print("\n[4/6] Accuracy check …")
    acc = check_accuracy(N_BEAMS, RANGE_MAX)
    print(f"  v3 hits / o3d hits : {acc['v3_hits']} / {acc['o3d_hits']}")
    print(f"  max |Δrange|       : {acc['max_diff_m']:.4f} m")
    print(f"  mean |Δrange| (hit): {acc['mean_diff_hit_m']:.4f} m")
    print(f"  hit agreement      : {acc['hit_agreement_pct']:.1f}%")

    print("\n[5/6] Scale benchmark (M = 100 … 5 000 segments) …")
    scale = bench_scale(N_BEAMS, n_runs=300)
    for _key, row in scale.items():
        print(
            f"  M={row['M']:>5d}  v3 {row['v3_p50_us']:>6.0f} µs"
            f"  o3d {row['o3d_p50_us']:>6.0f} µs"
            f"  {'v3 faster' if row['v3_p50_us'] < row['o3d_p50_us'] else 'o3d faster'}"
        )

    print("\n[6/6] CPU at 30 Hz …")
    cpu = bench_cpu_30hz(N_BEAMS, RANGE_MAX, duration=5.0)
    print(f"  v3 CPU%  : {cpu['v3_cpu_pct']:.1f}%  ({cpu['v3_steps']} steps)")
    print(f"  o3d CPU% : {cpu['o3d_cpu_pct']:.1f}%  ({cpu['o3d_steps']} steps)")

    results = {
        "config": {
            "n_beams": N_BEAMS,
            "range_max": RANGE_MAX,
            "n_objects": 26,
            "M_all": 824,
        },
        "build": build,
        "step": step,
        "anatomy": anatomy,
        "accuracy": acc,
        "scale": scale,
        "cpu": cpu,
    }

    out_path = _HERE / "open3d_comparison_results.json"
    out_path.write_text(json.dumps(results, indent=2))
    print(f"\nResults → {out_path}")


if __name__ == "__main__":
    main()
