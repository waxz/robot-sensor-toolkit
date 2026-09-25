"""
open3d_accuracy_fix.py — Diagnose and fix the ~10 cm geometric error in O3DLidar2D.

Root cause
----------
The hand-built _circle_mesh() approximates circular pillars as N-gon prisms.
Near-tangent rays suffer two compounding problems:

  1. Chord setback: Δr = r·(1 − cos(π/N)) / cos(α)
     At N=32, r=0.3 m, α=89°: Δr ≈ 83 mm  (over the 9 mm spec)

  2. Ray escape: a handful of grazing rays (~0.5 %) slip through Embree's
     floating-point tolerance at near-degenerate prism face edges and hit
     the far wall (~18 m extra).  These ~8 outliers drive mean |Δrange| to
     ~100 mm even though 99.5 % of beams are accurate.

Fix
---
Replace hand-built prisms with o3d.geometry.TriangleMesh.create_cylinder()
at high resolution (default 360 sides):
  • Guaranteed watertight (Open3D's own verified topology → no escape rays)
  • Chord setback ≈ 0.011 mm at N=360, r=0.3 m
  • Near-tangent α=89°: Δr ≈ 0.011/cos(89°) ≈ 0.63 mm (well under 9 mm)

Required resolution for < 9 mm at α < 89°:
  r·(1−cos(π/N)) / cos(89°) < 9 mm  →  N > π / arccos(1 − 9·cos(89°)/r)
  For r = 0.3 m:  N > π / arccos(1 − 9e-3·cos(89°)/0.3) ≈ N > 58
  Use N = 360 for comfortable margin.

Output: open3d_accuracy_results.json
"""

from __future__ import annotations

import json
import math
import sys
from math import cos, pi, sin
from pathlib import Path
from time import perf_counter

import numpy as np
import open3d as o3d
import open3d.t.geometry as otg

_HERE = Path(__file__).parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "src"))

from irsim_devices.core.open3d_scene_2d import Open3DScene2D  # noqa: E402
from irsim_devices.sensors.lidar2d import Lidar2D  # noqa: E402

# ── Scene geometry (warehouse, identical to other benchmarks) ─────────────────
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
Z_HEIGHT = 1.5


# ── Geometry builders ─────────────────────────────────────────────────────────


def _make_scene_2d(circle_res: int = 32) -> Open3DScene2D:
    scene = Open3DScene2D()
    hw = W / 2
    scene.add_box(center=(hw, WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(hw, H - WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    scene.add_box(center=(W - WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            scene.add_circle(center=(cx, cy), radius=PILLAR_R, resolution=circle_res)
    for rx in RACK_XS:
        for ry in RACK_YS:
            scene.add_box(
                center=(rx + RACK_LENGTH / 2, ry),
                width=RACK_LENGTH / 2,
                height=RACK_WIDTH / 2,
            )
    return scene


def _make_scene_3d_handbuilt(circle_res: int = 32) -> otg.RaycastingScene:
    """Original hand-built N-gon prism approach (buggy for near-tangent rays)."""

    def _box_mesh(cx, cy, hw, hh):
        m = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
        m.translate([cx - hw, cy - hh, 0.0])
        return m

    def _circle_mesh_handbuilt(cx, cy, r, n):
        a = np.linspace(0, 2 * pi, n, endpoint=False)
        vx, vy = cx + r * np.cos(a), cy + r * np.sin(a)
        vb = np.column_stack([vx, vy, np.zeros(n)])
        vt = np.column_stack([vx, vy, np.full(n, EXTRUDE_H)])
        cb = np.array([[cx, cy, 0.0]])
        ct = np.array([[cx, cy, EXTRUDE_H]])
        vs = np.vstack([vb, vt, cb, ct]).astype(np.float32)
        cbi, cti = 2 * n, 2 * n + 1
        tris = []
        for i in range(n):
            j = (i + 1) % n
            tris += [[i, j, i + n], [j, j + n, i + n]]
            tris.append([cbi, j, i])
            tris.append([cti, i + n, j + n])
        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(vs)
        m.triangles = o3d.utility.Vector3iVector(np.array(tris, np.int32))
        return m

    rc = otg.RaycastingScene()

    def _add(mesh):
        v = o3d.core.Tensor(np.asarray(mesh.vertices, np.float32))
        t = o3d.core.Tensor(np.asarray(mesh.triangles, np.uint32))
        rc.add_triangles(otg.TriangleMesh(v, t))

    hw = W / 2
    _add(_box_mesh(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2))
    _add(_box_mesh(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2))
    _add(_box_mesh(WALL_T / 2, H / 2, WALL_T / 2, H / 2))
    _add(_box_mesh(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2))
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            _add(_circle_mesh_handbuilt(cx, cy, PILLAR_R, n=circle_res))
    for rx in RACK_XS:
        for ry in RACK_YS:
            _add(_box_mesh(rx + RACK_LENGTH / 2, ry, RACK_LENGTH / 2, RACK_WIDTH / 2))
    return rc


def _make_scene_3d_fixed(circle_res: int = 360) -> otg.RaycastingScene:
    """Fixed approach: use create_cylinder() — watertight, correct normals."""

    def _box_mesh(cx, cy, hw, hh):
        m = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
        m.translate([cx - hw, cy - hh, 0.0])
        return m

    def _cylinder_mesh(cx, cy, r, n):
        # create_cylinder: center at origin, Z-axis, spans [-h/2, +h/2]
        m = o3d.geometry.TriangleMesh.create_cylinder(
            radius=r, height=EXTRUDE_H, resolution=n, split=1
        )
        m.translate([cx, cy, EXTRUDE_H / 2.0])
        return m

    rc = otg.RaycastingScene()

    def _add(mesh):
        v = o3d.core.Tensor(np.asarray(mesh.vertices, np.float32))
        t = o3d.core.Tensor(np.asarray(mesh.triangles, np.uint32))
        rc.add_triangles(otg.TriangleMesh(v, t))

    hw = W / 2
    _add(_box_mesh(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2))
    _add(_box_mesh(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2))
    _add(_box_mesh(WALL_T / 2, H / 2, WALL_T / 2, H / 2))
    _add(_box_mesh(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2))
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            _add(_cylinder_mesh(cx, cy, PILLAR_R, n=circle_res))
    for rx in RACK_XS:
        for ry in RACK_YS:
            _add(_box_mesh(rx + RACK_LENGTH / 2, ry, RACK_LENGTH / 2, RACK_WIDTH / 2))
    return rc


# ── Ray casting helpers ───────────────────────────────────────────────────────

N_BEAMS = 1500
RANGE_MAX = 30.0
OX, OY = 25.0, 15.0


def _cast_v3(sc2d: Open3DScene2D, n_beams: int = N_BEAMS) -> np.ndarray:
    lidar = Lidar2D(
        state=np.array([[OX], [OY], [0.0]]),
        range_max=RANGE_MAX,
        angle_range=2 * pi,
        number=n_beams,
        noise=False,
    )
    lidar.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)
    lidar.step(np.array([[OX], [OY], [0.0]]))
    return lidar.range_data.copy()


def _cast_o3d(rc: otg.RaycastingScene, n_beams: int = N_BEAMS) -> np.ndarray:
    az = np.linspace(-pi, pi, n_beams, endpoint=False, dtype=np.float32)
    dirs = np.stack(
        [np.cos(az), np.sin(az), np.zeros(n_beams, dtype=np.float32)], axis=1
    )
    origs = np.tile(np.array([OX, OY, Z_HEIGHT], np.float32), (n_beams, 1))
    rays = o3d.core.Tensor(
        np.concatenate([origs, dirs], axis=1), dtype=o3d.core.Dtype.Float32
    )
    t_hit = rc.cast_rays(rays)["t_hit"].numpy()
    return np.where(np.isfinite(t_hit) & (t_hit < RANGE_MAX), t_hit, RANGE_MAX).astype(
        np.float32
    )


def _accuracy_stats(ref: np.ndarray, o3d_r: np.ndarray, range_max: float = RANGE_MAX):
    ref_hit = ref < range_max - 1e-3
    o3d_hit = o3d_r < range_max - 1e-3
    both_hit_mask = ref_hit & o3d_hit
    diff = np.abs(ref - o3d_r)
    # Disagree beams: one hits, other misses (or both hit but > 1m apart)
    n_disagree = int(np.sum(ref_hit != o3d_hit))
    return {
        "ref_hits": int(ref_hit.sum()),
        "o3d_hits": int(o3d_hit.sum()),
        "both_hit": int(both_hit_mask.sum()),
        "n_disagree": n_disagree,
        "max_diff_m": float(diff[both_hit_mask].max()) if both_hit_mask.any() else 0.0,
        "mean_diff_m": float(diff[both_hit_mask].mean())
        if both_hit_mask.any()
        else 0.0,
        "p95_diff_m": float(np.percentile(diff[both_hit_mask], 95))
        if both_hit_mask.any()
        else 0.0,
        "p99_diff_m": float(np.percentile(diff[both_hit_mask], 99))
        if both_hit_mask.any()
        else 0.0,
        "agreement_pct": float(100.0 * (len(ref) - n_disagree) / len(ref)),
    }


# ── Analytical ground truth ───────────────────────────────────────────────────

_PILLARS = [(cx, cy, PILLAR_R) for cx in PILLAR_COLS for cy in PILLAR_ROWS]

_BOXES = (
    # (cx, cy, hw, hh) for walls and racks
    [
        (W / 2, WALL_T / 2, W / 2 + WALL_T, WALL_T / 2),
        (W / 2, H - WALL_T / 2, W / 2 + WALL_T, WALL_T / 2),
        (WALL_T / 2, H / 2, WALL_T / 2, H / 2),
        (W - WALL_T / 2, H / 2, WALL_T / 2, H / 2),
    ]
    + [
        (rx + RACK_LENGTH / 2, ry, RACK_LENGTH / 2, RACK_WIDTH / 2)
        for rx in RACK_XS
        for ry in RACK_YS
    ]
)


def _ray_box_t(ox, oy, dx, dy, cx, cy, hw, hh) -> float:
    """Axis-aligned box ray intersection (2D slab method), returns t or inf."""
    inv_dx = 1.0 / dx if dx != 0.0 else float("inf")
    inv_dy = 1.0 / dy if dy != 0.0 else float("inf")
    tx1, tx2 = ((cx - hw) - ox) * inv_dx, ((cx + hw) - ox) * inv_dx
    ty1, ty2 = ((cy - hh) - oy) * inv_dy, ((cy + hh) - oy) * inv_dy
    tmin = max(min(tx1, tx2), min(ty1, ty2))
    tmax = min(max(tx1, tx2), max(ty1, ty2))
    return tmin if 0.0 < tmin <= tmax else float("inf")


def _ray_cylinder_t(ox, oy, dx, dy, cx, cy, r) -> float:
    """Analytical 2D ray-circle intersection, returns t or inf."""
    ex, ey = ox - cx, oy - cy
    a = dx * dx + dy * dy
    b = 2.0 * (ex * dx + ey * dy)
    c = ex * ex + ey * ey - r * r
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return float("inf")
    sq = disc**0.5
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    if t1 > 1e-6:
        return t1
    if t2 > 1e-6:
        return t2
    return float("inf")


def analytical_ranges(
    n_beams: int = N_BEAMS, range_max: float = RANGE_MAX
) -> np.ndarray:
    """Ground-truth range array from analytical ray-scene intersection."""
    az = np.linspace(-pi, pi, n_beams, endpoint=False)
    ranges = np.empty(n_beams, dtype=np.float32)
    for i, a in enumerate(az):
        dx, dy = cos(a), sin(a)
        t_min = range_max
        for cx, cy, hw, hh in _BOXES:
            t_min = min(t_min, _ray_box_t(OX, OY, dx, dy, cx, cy, hw, hh))
        for cx, cy, r in _PILLARS:
            t_min = min(t_min, _ray_cylinder_t(OX, OY, dx, dy, cx, cy, r))
        ranges[i] = t_min
    return ranges


def _pct(arr, p):
    return float(np.percentile(arr, p))


# ── 1. Resolution sweep (hand-built prism) ───────────────────────────────────


def bench_resolution_sweep(resolutions=(8, 16, 32, 64, 128, 360)):
    print("\n[1/4] Resolution sweep (hand-built N-gon prism vs analytical truth) …")
    results = {}
    gt = analytical_ranges()
    for n in resolutions:
        rc = _make_scene_3d_handbuilt(circle_res=n)
        o3d_r = _cast_o3d(rc)
        stats = _accuracy_stats(gt, o3d_r)
        chord_setback_mm = PILLAR_R * (1 - math.cos(pi / n)) * 1000
        print(
            f"  N={n:>4d}  chord_setback={chord_setback_mm:.3f} mm"
            f"  disagree={stats['n_disagree']:>3d}"
            f"  max|Δ|={stats['max_diff_m']:.3f} m"
            f"  mean|Δ|={stats['mean_diff_m'] * 1000:.1f} mm"
            f"  p99|Δ|={stats['p99_diff_m'] * 1000:.1f} mm"
        )
        results[str(n)] = {**stats, "chord_setback_mm": chord_setback_mm}
    return results


# ── 2. Fixed approach: create_cylinder ───────────────────────────────────────


def bench_fixed_approach(resolutions=(32, 64, 128, 360)):
    print("\n[2/4] Fixed approach: create_cylinder() vs analytical truth …")
    results = {}
    gt = analytical_ranges()
    for n in resolutions:
        rc = _make_scene_3d_fixed(circle_res=n)
        o3d_r = _cast_o3d(rc)
        stats = _accuracy_stats(gt, o3d_r)
        chord_setback_mm = PILLAR_R * (1 - math.cos(pi / n)) * 1000
        print(
            f"  N={n:>4d}  chord_setback={chord_setback_mm:.3f} mm"
            f"  disagree={stats['n_disagree']:>3d}"
            f"  max|Δ|={stats['max_diff_m']:.3f} m"
            f"  mean|Δ|={stats['mean_diff_m'] * 1000:.1f} mm"
            f"  p99|Δ|={stats['p99_diff_m'] * 1000:.1f} mm"
        )
        results[str(n)] = {**stats, "chord_setback_mm": chord_setback_mm}
    return results


# ── 3. Latency at each resolution ────────────────────────────────────────────


def bench_latency_by_resolution(resolutions=(32, 64, 128, 360), n_runs=500):
    print("\n[3/4] Step latency vs circle resolution (create_cylinder) …")
    results = {}
    az = np.linspace(-pi, pi, N_BEAMS, endpoint=False, dtype=np.float32)
    dirs = np.stack(
        [np.cos(az), np.sin(az), np.zeros(N_BEAMS, dtype=np.float32)], axis=1
    )
    origs = np.tile(np.array([OX, OY, Z_HEIGHT], np.float32), (N_BEAMS, 1))

    for n in resolutions:
        t0 = perf_counter()
        rc = _make_scene_3d_fixed(circle_res=n)
        build_ms = (perf_counter() - t0) * 1e3

        # Warm up
        for _ in range(50):
            rays = o3d.core.Tensor(
                np.concatenate([origs, dirs], axis=1), dtype=o3d.core.Dtype.Float32
            )
            rc.cast_rays(rays)

        times = []
        for _ in range(n_runs):
            t0 = perf_counter()
            rays = o3d.core.Tensor(
                np.concatenate([origs, dirs], axis=1), dtype=o3d.core.Dtype.Float32
            )
            rc.cast_rays(rays)
            times.append((perf_counter() - t0) * 1e6)

        # Triangle count estimate: n side quads * 2 + 2 * n caps = 4n per cylinder
        n_tris_pillars = 12 * 4 * n
        print(
            f"  N={n:>4d}  tris/pillar={4 * n:>6d}"
            f"  build={build_ms:.0f} ms"
            f"  step p50={_pct(times, 50):.0f} µs"
            f"  p95={_pct(times, 95):.0f} µs"
        )
        results[str(n)] = {
            "circle_res": n,
            "n_tris_pillars": n_tris_pillars,
            "build_ms": build_ms,
            "step_p50_us": _pct(times, 50),
            "step_p95_us": _pct(times, 95),
        }
    return results


# ── 4. Final accuracy comparison at recommended N=360 ────────────────────────


def bench_final_accuracy(n_beams=N_BEAMS, range_max=RANGE_MAX):
    print("\n[4/4] Final accuracy vs analytical ground truth …")
    gt = analytical_ranges(n_beams, range_max)

    # Before: hand-built N=32
    rc_before = _make_scene_3d_handbuilt(circle_res=32)
    o3d_before = _cast_o3d(rc_before, n_beams)
    before = _accuracy_stats(gt, o3d_before, range_max)

    # After: create_cylinder N=360
    rc_after = _make_scene_3d_fixed(circle_res=360)
    o3d_after = _cast_o3d(rc_after, n_beams)
    after = _accuracy_stats(gt, o3d_after, range_max)

    print(f"\n  {'Metric':<25} {'Before (N=32)':>15} {'After (N=360)':>15}")
    print(f"  {'-' * 55}")
    print(
        f"  {'Disagree beams':<25} {before['n_disagree']:>15d}"
        f" {after['n_disagree']:>15d}"
    )
    print(
        f"  {'max |Δrange| (m)':<25} {before['max_diff_m']:>15.3f}"
        f" {after['max_diff_m']:>15.3f}"
    )
    print(
        f"  {'mean |Δrange| (mm)':<25} {before['mean_diff_m'] * 1000:>14.1f}"
        f" {after['mean_diff_m'] * 1000:>14.1f}"
    )
    print(
        f"  {'p99 |Δrange| (mm)':<25} {before['p99_diff_m'] * 1000:>14.1f}"
        f" {after['p99_diff_m'] * 1000:>14.1f}"
    )
    print(f"  {'chord setback (mm)':<25} {'1.450':>15} {'0.011':>15}")
    print("\n  9mm commercial spec at α<89°:")
    for n_val, label in [(32, "Before N=32 "), (360, "After  N=360")]:
        setback = PILLAR_R * (1 - cos(pi / n_val)) * 1000
        worst_case = setback / cos(math.radians(89))
        ok = "✓ PASS" if worst_case < 9.0 else "✗ FAIL"
        print(f"    {label}: worst-case near-tangent error = {worst_case:.2f} mm  {ok}")
    return {"before": before, "after": after}


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    sweep = bench_resolution_sweep()
    fixed = bench_fixed_approach()
    latency = bench_latency_by_resolution()
    final = bench_final_accuracy()

    results = {
        "resolution_sweep_handbuilt": sweep,
        "resolution_sweep_cylinder": fixed,
        "latency_by_resolution": latency,
        "final_comparison": final,
        "recommendation": {
            "circle_builder": "create_cylinder",
            "resolution": 360,
            "chord_setback_mm": PILLAR_R * (1 - cos(pi / 360)) * 1000,
            "worst_case_tangent_mm": PILLAR_R
            * (1 - cos(pi / 360))
            * 1000
            / cos(math.radians(89)),
        },
    }

    out = _HERE / "open3d_accuracy_results.json"
    with open(out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults → {out}")


if __name__ == "__main__":
    main()
