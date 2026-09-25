"""
open3d_presets.py — Evaluate Open3D Embree backend across LiDAR 2D sensor presets.

Covers:
  A. Beam-count sweep: 360→3600 beams (common 2D lidar models)
  B. Direction-rotation overhead: cost of rotating pre-built ray dirs each step
  C. Hybrid approach: precompute directions once, rotate and cast per step
  D. Dynamic-scene cost: BVH rebuild when obstacles move
  E. Accuracy: same geometry, v3 vs Open3D (aligned resolution)

Results → open3d_presets_results.json
"""

from __future__ import annotations

import json
import math
import sys
import time
from math import cos, pi, sin
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

# ── Standard 2D LiDAR presets ─────────────────────────────────────────────────
#   (name, n_beams, fov_deg, range_max_m, hz)
PRESETS = [
    ("SICK TiM571", 811, 270, 25.0, 15),
    ("Hokuyo UTM-30LX", 1080, 270, 30.0, 40),
    ("Hokuyo URG-04LX", 683, 240, 4.0, 10),
    ("LiDAR 2D default", 1500, 360, 30.0, 30),
    ("RPLIDAR A2", 1147, 360, 12.0, 10),
    ("Velodyne VLP-16 H", 1800, 360, 100.0, 20),
]

# ── Scene (same as bottleneck_analysis) ──────────────────────────────────────
W, H = 50.0, 30.0
WALL_T = 0.2
PILLAR_R = 0.3
PILLAR_COLS = [10.0, 20.0, 30.0, 40.0]
PILLAR_ROWS = [7.5, 15.0, 22.5]
RACK_LENGTH, RACK_WIDTH = 14.0, 1.0
RACK_MARGIN = 3.0
RACK_XS = [RACK_MARGIN, W - RACK_MARGIN - RACK_LENGTH]
RACK_YS = [5.0, 10.0, 15.0, 20.0, 25.0]


def _make_scene_2d():
    scene = Open3DScene2D()
    hw = W / 2
    scene.add_box(center=(hw, WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(hw, H - WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    scene.add_box(center=(W - WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            scene.add_circle(center=(cx, cy), radius=PILLAR_R, resolution=32)
    for rx in RACK_XS:
        for ry in RACK_YS:
            scene.add_box(
                center=(rx + RACK_LENGTH / 2, ry),
                width=RACK_LENGTH / 2,
                height=RACK_WIDTH / 2,
            )
    return scene


def _make_scene_3d(circle_res=360):
    """Build the Open3D RaycastingScene.

    Uses create_cylinder() for circular obstacles so the mesh is watertight
    with correct normals.  circle_res=360 gives chord setback ≈ 0.011 mm
    (sub-mm vs analytical ground truth) with no measurable step-latency cost.
    """
    EXTRUDE_H = 3.0

    def _box_mesh(cx, cy, hw, hh):
        mesh = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
        mesh.translate([cx - hw, cy - hh, 0.0])
        return mesh

    def _cylinder_mesh(cx, cy, r, n):
        # create_cylinder centers at origin, spans [-h/2, +h/2] along Z.
        # Translate so the cylinder sits at z=0..EXTRUDE_H.
        mesh = o3d.geometry.TriangleMesh.create_cylinder(
            radius=r, height=EXTRUDE_H, resolution=n, split=1
        )
        mesh.translate([cx, cy, EXTRUDE_H / 2.0])
        return mesh

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


def _pct(arr, p):
    return float(np.percentile(arr, p))


Z_HEIGHT = 1.5  # ray z for all Open3D 2D steps


# ── Optimised Open3D step: pre-allocated dirs, rotation per step ─────────────


class O3DLidar2D:
    """
    Drop-in Open3D backend for 2D LiDAR.
    Pre-allocates base direction array; rotates by world_theta each step.
    """

    def __init__(self, n_beams: int, range_max: float, fov_deg: float = 360.0):
        self.n_beams = n_beams
        self.range_max = range_max
        self._rc: otg.RaycastingScene | None = None
        self.range_data = np.full(n_beams, range_max, dtype=np.float32)

        fov_rad = math.radians(fov_deg)
        az_start = -fov_rad / 2.0
        az_end = fov_rad / 2.0
        az = np.linspace(az_start, az_end, n_beams, endpoint=False, dtype=np.float32)
        # Base directions (no rotation): unrotated beam angles
        self._base_cos = np.cos(az)
        self._base_sin = np.sin(az)
        # Working arrays (reused each step)
        self._dirs = np.zeros((n_beams, 3), dtype=np.float32)
        self._origs = np.zeros((n_beams, 3), dtype=np.float32)
        self._dirs[:, 2] = 0.0  # dz always 0

    def set_scene(self, rc: otg.RaycastingScene) -> None:
        self._rc = rc

    def step(self, ox: float, oy: float, world_theta: float) -> None:
        # Rotate base directions by world_theta in-place
        cw = np.float32(cos(world_theta))
        sw = np.float32(sin(world_theta))
        np.multiply(self._base_cos, cw, out=self._dirs[:, 0])
        np.multiply(self._base_sin, sw, out=self._origs[:, 0])  # tmp
        np.subtract(self._dirs[:, 0], self._origs[:, 0], out=self._dirs[:, 0])
        np.multiply(self._base_cos, sw, out=self._dirs[:, 1])
        np.multiply(self._base_sin, cw, out=self._origs[:, 0])  # tmp
        np.add(self._dirs[:, 1], self._origs[:, 0], out=self._dirs[:, 1])
        # Fill origins
        self._origs[:] = [ox, oy, Z_HEIGHT]
        # Pack rays (still needs one concatenation per step)
        rays = o3d.core.Tensor(
            np.concatenate([self._origs, self._dirs], axis=1),
            dtype=o3d.core.Dtype.Float32,
        )
        result = self._rc.cast_rays(rays)
        t_hit = result["t_hit"].numpy()
        np.copyto(
            self.range_data,
            np.where(
                np.isfinite(t_hit) & (t_hit < self.range_max), t_hit, self.range_max
            ),
        )


# ── A. Beam-count sweep ───────────────────────────────────────────────────────


def bench_presets(sc2d, rc, n_runs=1000):
    results = {}
    for name, n_beams, fov_deg, range_max, hz in PRESETS:
        # v3 lidar
        lidar_v3 = Lidar2D(
            state=np.array([[25.0], [15.0], [0.0]]),
            range_max=range_max,
            angle_range=math.radians(fov_deg),
            number=n_beams,
            noise=False,
        )
        lidar_v3.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)

        # O3D lidar
        lidar_o3d = O3DLidar2D(n_beams, range_max, fov_deg)
        lidar_o3d.set_scene(rc)

        state = np.array([[25.0], [15.0], [0.0]])
        ox, oy, theta = 25.0, 15.0, 0.0

        # Warm up
        for _ in range(50):
            lidar_v3.step(state)
            lidar_o3d.step(ox, oy, theta)

        tv3, to3d = [], []
        for _ in range(n_runs):
            t0 = perf_counter()
            lidar_v3.step(state)
            tv3.append((perf_counter() - t0) * 1e6)

            t0 = perf_counter()
            lidar_o3d.step(ox, oy, theta)
            to3d.append((perf_counter() - t0) * 1e6)

        speedup = _pct(tv3, 50) / _pct(to3d, 50)
        results[name] = {
            "n_beams": n_beams,
            "fov_deg": fov_deg,
            "range_max": range_max,
            "hz": hz,
            "v3_p50_us": _pct(tv3, 50),
            "v3_p95_us": _pct(tv3, 95),
            "o3d_p50_us": _pct(to3d, 50),
            "o3d_p95_us": _pct(to3d, 95),
            "speedup_x": round(speedup, 2),
        }
        print(
            f"  {name:25s}: v3={_pct(tv3, 50):5.0f}µs  o3d={_pct(to3d, 50):5.0f}µs  "
            f"{speedup:.1f}× faster"
        )
    return results


# ── B. Rotation overhead ─────────────────────────────────────────────────────


def bench_rotation_overhead(rc, n_beams=1500, n_runs=2000):
    """Measure per-step direction rotation cost inside O3DLidar2D."""
    lidar_o3d = O3DLidar2D(n_beams, 30.0)
    lidar_o3d.set_scene(rc)

    # Warm up
    for _ in range(50):
        lidar_o3d.step(25.0, 15.0, 0.0)

    t_zero, t_rot = [], []
    for i in range(n_runs):
        theta = (i * 0.01) % (2 * pi)  # varying angle

        # No rotation (theta=0)
        t0 = perf_counter()
        lidar_o3d.step(25.0, 15.0, 0.0)
        t_zero.append((perf_counter() - t0) * 1e6)

        # With rotation
        t0 = perf_counter()
        lidar_o3d.step(25.0, 15.0, theta)
        t_rot.append((perf_counter() - t0) * 1e6)

    return {
        "no_rotation_us": {"p50": _pct(t_zero, 50), "p95": _pct(t_zero, 95)},
        "with_rotation_us": {"p50": _pct(t_rot, 50), "p95": _pct(t_rot, 95)},
        "rotation_overhead_us": _pct(t_rot, 50) - _pct(t_zero, 50),
    }


# ── C. Dynamic scene: BVH rebuild cost ───────────────────────────────────────


def bench_dynamic_cost(sc2d, n_runs=100):
    """Measure how expensive adding a dynamic object (moving robot) is."""

    # Simulate a scene with N_dyn dynamic circles (robots) that move each step
    N_DYN = [0, 1, 5, 10, 20]
    results = {}

    for n_dyn in N_DYN:
        # Build base Open3D scene
        rc = _make_scene_3d()

        # For v3: add dynamic circles via Open3DScene2D
        sc_dyn = Open3DScene2D()
        hw = W / 2
        sc_dyn.add_box(center=(hw, WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
        sc_dyn.add_box(
            center=(hw, H - WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2
        )
        sc_dyn.add_box(center=(WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
        sc_dyn.add_box(center=(W - WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
        for cx in PILLAR_COLS:
            for cy in PILLAR_ROWS:
                sc_dyn.add_circle(center=(cx, cy), radius=PILLAR_R, resolution=32)
        for rx in RACK_XS:
            for ry in RACK_YS:
                sc_dyn.add_box(
                    center=(rx + RACK_LENGTH / 2, ry),
                    width=RACK_LENGTH / 2,
                    height=RACK_WIDTH / 2,
                )

        # v3 lidar with set_scene (static only — v3 rebuilds SoA when scene changes)
        lidar_v3 = Lidar2D(
            state=np.array([[25.0], [15.0], [0.0]]),
            range_max=30.0,
            angle_range=2 * pi,
            number=1500,
            noise=False,
        )
        lidar_v3.set_scene(sc_dyn, n_omp_threads=2, filter_margin=1.0)

        state = np.array([[25.0], [15.0], [0.0]])
        ox, oy = 25.0, 15.0

        # Pre-compute o3d dirs
        az = np.linspace(-pi, pi, 1500, endpoint=False, dtype=np.float32)
        o3d_dirs = np.stack(
            [np.cos(az), np.sin(az), np.zeros(1500, np.float32)], axis=1
        )
        origin = np.array([ox, oy, Z_HEIGHT], np.float32)
        origs = np.tile(origin, (1500, 1))

        _origs_cap = origs
        _dirs_cap = o3d_dirs

        def _o3d_cast(rc_local, _o=_origs_cap, _d=_dirs_cap):
            rays = o3d.core.Tensor(
                np.concatenate([_o, _d], axis=1),
                dtype=o3d.core.Dtype.Float32,
            )
            return rc_local.cast_rays(rays)["t_hit"].numpy()

        # Warm up
        for _ in range(20):
            lidar_v3.step(state)
            _o3d_cast(rc)

        t_v3_step, t_o3d_step = [], []

        # Note: for dynamic objects, v3 needs to call set_scene again to rebuild SoA.
        # Open3D needs to rebuild the RaycastingScene. We measure the rebuild + cast.
        for k in range(n_runs):
            # v3: rebuild set_scene with n_dyn extra circles (simulate update)
            t0 = perf_counter()
            if n_dyn > 0:
                sc2_local = Open3DScene2D()
                sc2_local.add_box(
                    center=(W / 2, WALL_T / 2), width=W / 2 + WALL_T, height=WALL_T / 2
                )
                sc2_local.add_box(
                    center=(W / 2, H - WALL_T / 2),
                    width=W / 2 + WALL_T,
                    height=WALL_T / 2,
                )
                sc2_local.add_box(
                    center=(WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2
                )
                sc2_local.add_box(
                    center=(W - WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2
                )
                for cx in PILLAR_COLS:
                    for cy in PILLAR_ROWS:
                        sc2_local.add_circle(
                            center=(cx, cy), radius=PILLAR_R, resolution=32
                        )
                for rx in RACK_XS:
                    for ry in RACK_YS:
                        sc2_local.add_box(
                            center=(rx + RACK_LENGTH / 2, ry),
                            width=RACK_LENGTH / 2,
                            height=RACK_WIDTH / 2,
                        )
                for i in range(n_dyn):
                    sc2_local.add_circle(
                        center=(10.0 + i, 10.0 + k * 0.01), radius=0.3, resolution=16
                    )
                lidar_v3.set_scene(sc2_local, n_omp_threads=2, filter_margin=1.0)
            lidar_v3.step(state)
            t_v3_step.append((perf_counter() - t0) * 1e6)

            # Open3D: full rebuild with dynamic circles
            t0 = perf_counter()
            rc_new = _make_scene_3d()
            for i in range(n_dyn):
                cx_dyn = 10.0 + i
                cy_dyn = 10.0 + k * 0.01
                r_dyn = 0.3
                n_r = 16
                a = np.linspace(0, 2 * pi, n_r, endpoint=False)
                vx, vy = cx_dyn + r_dyn * np.cos(a), cy_dyn + r_dyn * np.sin(a)
                vb = np.column_stack([vx, vy, np.zeros(n_r)])
                vt = np.column_stack([vx, vy, np.full(n_r, 3.0)])
                cb, ct = (
                    np.array([[cx_dyn, cy_dyn, 0.0]]),
                    np.array([[cx_dyn, cy_dyn, 3.0]]),
                )
                vs_dyn = np.vstack([vb, vt, cb, ct]).astype(np.float32)
                cbi, cti = 2 * n_r, 2 * n_r + 1
                tris_dyn = []
                for ii in range(n_r):
                    j = (ii + 1) % n_r
                    tris_dyn += [[ii, j, ii + n_r], [j, j + n_r, ii + n_r]]
                    tris_dyn.append([cbi, j, ii])
                    tris_dyn.append([cti, ii + n_r, j + n_r])
                vt2 = o3d.core.Tensor(vs_dyn)
                tt2 = o3d.core.Tensor(np.array(tris_dyn, np.uint32))
                rc_new.add_triangles(otg.TriangleMesh(vt2, tt2))
            _o3d_cast(rc_new)
            t_o3d_step.append((perf_counter() - t0) * 1e6)

        results[str(n_dyn)] = {
            "n_dynamic": n_dyn,
            "v3_rebuild_step_p50_us": _pct(t_v3_step, 50),
            "o3d_rebuild_step_p50_us": _pct(t_o3d_step, 50),
        }
        print(
            f"  n_dyn={n_dyn:2d}: v3 rebuild+step={_pct(t_v3_step, 50):6.0f}µs  "
            f"o3d rebuild+step={_pct(t_o3d_step, 50):6.0f}µs"
        )

    return results


# ── D. Accuracy with aligned geometry ────────────────────────────────────────


def check_accuracy_aligned(n_beams=1500, range_max=30.0):
    """Both backends use resolution=32 circles; compare range arrays."""
    sc2d = _make_scene_2d()  # already uses resolution=32
    rc = _make_scene_3d(circle_res=32)

    lidar_v3 = Lidar2D(
        state=np.array([[25.0], [15.0], [0.0]]),
        range_max=range_max,
        angle_range=2 * pi,
        number=n_beams,
        noise=False,
    )
    lidar_v3.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)
    lidar_v3._filter_cache_x = math.inf
    lidar_v3.step(np.array([[25.0], [15.0], [0.0]]))
    v3_ranges = np.array(lidar_v3.range_data, dtype=np.float32)

    lidar_o3d = O3DLidar2D(n_beams, range_max)
    lidar_o3d.set_scene(rc)
    lidar_o3d.step(25.0, 15.0, 0.0)
    o3d_ranges = lidar_o3d.range_data.astype(np.float32)

    diff = np.abs(v3_ranges - o3d_ranges)
    both_hit = (v3_ranges < range_max) & (o3d_ranges < range_max)
    both_miss = (v3_ranges >= range_max) & (o3d_ranges >= range_max)

    return {
        "n_beams": n_beams,
        "v3_hits": int(np.sum(v3_ranges < range_max)),
        "o3d_hits": int(np.sum(o3d_ranges < range_max)),
        "both_hit": int(np.sum(both_hit)),
        "both_miss": int(np.sum(both_miss)),
        "agreement_pct": float(
            100.0 * (np.sum(both_hit) + np.sum(both_miss)) / n_beams
        ),
        "max_diff_m": float(np.max(diff)),
        "mean_diff_hit_m": float(np.mean(diff[both_hit])) if both_hit.any() else 0.0,
        "p95_diff_hit_m": float(_pct(diff[both_hit], 95)) if both_hit.any() else 0.0,
    }


# ── E. CPU at 30 Hz across presets ───────────────────────────────────────────


def bench_cpu_presets(sc2d, rc, duration=4.0):
    results = {}
    proc = psutil.Process()
    ox, oy, theta = 25.0, 15.0, 0.0

    for name, n_beams, fov_deg, range_max, hz in PRESETS:
        lidar_v3 = Lidar2D(
            state=np.array([[ox], [oy], [0.0]]),
            range_max=range_max,
            angle_range=math.radians(fov_deg),
            number=n_beams,
            noise=False,
        )
        lidar_v3.set_scene(sc2d, n_omp_threads=2, filter_margin=1.0)
        lidar_o3d = O3DLidar2D(n_beams, range_max, fov_deg)
        lidar_o3d.set_scene(rc)

        state = np.array([[ox], [oy], [0.0]])
        _interval = 1.0 / hz

        def _run_hz(step_fn, _ivl=_interval):
            proc.cpu_percent()
            time.sleep(0.2)
            proc.cpu_percent()
            t_end = time.monotonic() + duration
            n = 0
            while time.monotonic() < t_end:
                t0 = time.monotonic()
                step_fn()
                rem = _ivl - (time.monotonic() - t0)
                if rem > 0:
                    time.sleep(rem)
                n += 1
            return proc.cpu_percent(), n

        _lv3 = lidar_v3
        _st = state
        _lo3d = lidar_o3d
        _ox, _oy, _th = ox, oy, theta
        cpu_v3, _ = _run_hz(lambda _l=_lv3, _s=_st: _l.step(_s))
        time.sleep(0.5)
        cpu_o3d, _ = _run_hz(
            lambda _l=_lo3d, _x=_ox, _y=_oy, _t=_th: _l.step(_x, _y, _t)
        )
        time.sleep(0.5)

        results[name] = {
            "n_beams": n_beams,
            "hz": hz,
            "v3_cpu_pct": cpu_v3,
            "o3d_cpu_pct": cpu_o3d,
            "cpu_reduction_pct": round(cpu_v3 - cpu_o3d, 1),
        }
        print(
            f"  {name:25s} @{hz:2d}Hz: v3={cpu_v3:4.1f}%  o3d={cpu_o3d:4.1f}%  "
            f"saved={cpu_v3 - cpu_o3d:+.1f}pp"
        )

    return results


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    print("=== Open3D Embree backend: LiDAR 2D preset evaluation ===\n")
    print("Building scenes …")
    sc2d = _make_scene_2d()
    rc = _make_scene_3d(circle_res=32)
    print(
        f"  Open3DScene2D: {sum(len(o.linestrings[0].coords) - 1 for o in sc2d.objects)} segs"
    )

    print("\n[A] Beam-count sweep across LiDAR 2D presets (1 000 runs each) …")
    presets = bench_presets(sc2d, rc, n_runs=1000)

    print("\n[B] Direction-rotation overhead in O3DLidar2D (1 500 beams) …")
    rot = bench_rotation_overhead(rc, n_beams=1500, n_runs=2000)
    print(f"  No rotation   : {rot['no_rotation_us']['p50']:.0f} µs p50")
    print(f"  With rotation : {rot['with_rotation_us']['p50']:.0f} µs p50")
    print(f"  Overhead      : {rot['rotation_overhead_us']:.0f} µs")

    print("\n[C] Dynamic scene rebuild + step cost (1 500 beams) …")
    dynamic = bench_dynamic_cost(sc2d, n_runs=50)

    print("\n[D] Accuracy with aligned geometry (resolution=32) …")
    acc = check_accuracy_aligned()
    print(f"  Both-hit agreement: {acc['agreement_pct']:.1f}%")
    print(f"  max |Δrange|      : {acc['max_diff_m']:.4f} m")
    print(f"  mean |Δrange| hit : {acc['mean_diff_hit_m']:.4f} m")
    print(f"  p95  |Δrange| hit : {acc['p95_diff_hit_m']:.4f} m")

    print("\n[E] CPU at rated Hz per preset …")
    cpu = bench_cpu_presets(sc2d, rc, duration=4.0)

    results = {
        "config": {"M_all": 824, "M_segs_aligned": 824},
        "presets": presets,
        "rotation": rot,
        "dynamic": dynamic,
        "accuracy": acc,
        "cpu": cpu,
    }

    out_path = _HERE / "open3d_presets_results.json"
    out_path.write_text(json.dumps(results, indent=2))
    print(f"\nResults → {out_path}")


if __name__ == "__main__":
    main()
