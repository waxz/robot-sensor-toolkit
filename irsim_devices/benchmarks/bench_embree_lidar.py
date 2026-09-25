"""bench_embree_lidar.py — Benchmark Embree LiDAR 2D and 3D backends vs Open3D.

Saves results to benchmarks/embree_lidar_results.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

BENCH_DIR = os.path.dirname(os.path.abspath(__file__))
CPP_DIR = os.path.join(BENCH_DIR, "..", "cpp")
SRC_DIR = os.path.join(BENCH_DIR, "..", "src")
sys.path.insert(0, CPP_DIR)
sys.path.insert(0, SRC_DIR)

import lidar_embree as _le  # noqa: E402

# ── Scene constants ────────────────────────────────────────────────────────────
W, H = 50.0, 30.0
WALL_T = 0.2
PILLAR_R = 0.3
PILLAR_COLS = [10.0, 20.0, 30.0, 40.0]
PILLAR_ROWS = [7.5, 15.0, 22.5]
RACK_LEN, RACK_W = 14.0, 1.0
RACK_XS = [3.0, W - 3.0 - RACK_LEN]
RACK_YS = [5.0, 10.0, 15.0, 20.0, 25.0]
EXTRUDE_H = 3.0

N_BEAMS = 1500
FOV_RAD = 2 * math.pi
RMAX = 30.0
OX, OY = 25.0, 15.0
THETA = 0.0
REPEATS = 2000
WARMUP = 300


# ── Geometry builders ──────────────────────────────────────────────────────────
def _circle_segs(cx, cy, r, n=32):
    pts = [
        (cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]
    return [
        (pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
        for i in range(n)
    ]


def _box_segs(cx, cy, hw, hh):
    c = [(cx - hw, cy - hh), (cx + hw, cy - hh), (cx + hw, cy + hh), (cx - hw, cy + hh)]
    return [(c[i][0], c[i][1], c[(i + 1) % 4][0], c[(i + 1) % 4][1]) for i in range(4)]


def build_segments(circle_res=32):
    s = []
    hw = W / 2
    s += _box_segs(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2)
    s += _box_segs(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2)
    s += _box_segs(WALL_T / 2, H / 2, WALL_T / 2, H / 2)
    s += _box_segs(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2)
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            s += _circle_segs(cx, cy, PILLAR_R, circle_res)
    for rx in RACK_XS:
        for ry in RACK_YS:
            s += _box_segs(rx + RACK_LEN / 2, ry, RACK_LEN / 2, RACK_W / 2)
    return np.array(s, dtype=np.float32)


def _build_meshes_o3d(circle_res=32):
    """Return list of open3d TriangleMesh objects for the warehouse scene."""
    import open3d as o3d

    def bm(cx, cy, hw, hh):
        m = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
        m.translate([cx - hw, cy - hh, 0])
        return m

    def cm(cx, cy, r, n=circle_res):
        m = o3d.geometry.TriangleMesh.create_cylinder(
            radius=r, height=EXTRUDE_H, resolution=n, split=1
        )
        m.translate([cx, cy, EXTRUDE_H / 2])
        return m

    hw = W / 2
    ms = [
        bm(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2),
        bm(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2),
        bm(WALL_T / 2, H / 2, WALL_T / 2, H / 2),
        bm(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2),
    ]
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            ms.append(cm(cx, cy, PILLAR_R))
    for rx in RACK_XS:
        for ry in RACK_YS:
            ms.append(bm(rx + RACK_LEN / 2, ry, RACK_LEN / 2, RACK_W / 2))
    return ms


def build_3d_mesh(circle_res=32):
    """Return float32 triangle soup [T,3,3] from extruded 2D scene."""
    try:
        import open3d as o3d

        def _bm(cx, cy, hw, hh):
            m = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
            m.translate([cx - hw, cy - hh, 0])
            return m

        def _cm(cx, cy, r, n=32):
            m = o3d.geometry.TriangleMesh.create_cylinder(
                radius=r, height=EXTRUDE_H, resolution=n, split=1
            )
            m.translate([cx, cy, EXTRUDE_H / 2])
            return m

        hw = W / 2
        ms = [
            _bm(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2),
            _bm(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2),
            _bm(WALL_T / 2, H / 2, WALL_T / 2, H / 2),
            _bm(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2),
        ]
        for cx in PILLAR_COLS:
            for cy in PILLAR_ROWS:
                ms.append(_cm(cx, cy, PILLAR_R, circle_res))
        for rx in RACK_XS:
            for ry in RACK_YS:
                ms.append(_bm(rx + RACK_LEN / 2, ry, RACK_LEN / 2, RACK_W / 2))
        all_v, all_t, off = [], [], 0
        for m in ms:
            V = np.asarray(m.vertices, np.float32)
            T = np.asarray(m.triangles, np.int32) + off
            all_v.append(V)
            all_t.append(T)
            off += len(V)
        V = np.concatenate(all_v)
        T = np.concatenate(all_t)
        return V[T].astype(np.float32)  # [T,3,3]
    except ImportError:
        return None


# ── Beam direction precompute ──────────────────────────────────────────────────
def _make_beams():
    az = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False).astype(np.float32)
    ct, st = math.cos(THETA), math.sin(THETA)
    cos_az = np.cos(az)
    sin_az = np.sin(az)
    dx = (ct * cos_az - st * sin_az).astype(np.float32)
    dy = (st * cos_az + ct * sin_az).astype(np.float32)
    return dx, dy


def _bench(name, fn, n=REPEATS, w=WARMUP):
    for _ in range(w):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    ms = (time.perf_counter() - t0) / n * 1e3
    print(f"  {name:30s}: {ms:8.3f} ms/step")
    return ms


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    print("Building scenes …")
    segs32 = build_segments(32)
    segs360 = build_segments(360)
    n_segs32 = len(segs32)
    n_segs360 = len(segs360)
    print(f"  2D segments (res=32) : {n_segs32}")
    print(f"  2D segments (res=360): {n_segs360}")

    dx, dy = _make_beams()
    origin_f = np.array([OX, OY], dtype=np.float32)
    out_r = np.empty(N_BEAMS, dtype=np.float32)
    out_h = np.empty(N_BEAMS, dtype=np.int32)

    # ── EmbreeScene2D builds ─────────────────────────────────────────────────
    s2d_32 = _le.EmbreeScene2D()
    s2d_32.build(segs32)

    def fn_e2d_s32():
        return s2d_32.cast_inplace(origin_f, dx, dy, RMAX, out_r, out_h)

    def fn_e2d_8_32():
        return s2d_32.cast8_inplace(origin_f, dx, dy, RMAX, out_r, out_h)

    s2d_360 = _le.EmbreeScene2D()
    s2d_360.build(segs360)

    def fn_e2d_s360():
        return s2d_360.cast_inplace(origin_f, dx, dy, RMAX, out_r, out_h)

    # Correctness check
    fn_e2d_s32()
    hits_e2d = int(np.sum(out_r < RMAX))
    print(f"\nHit counts (Embree2D scalar, res=32): {hits_e2d}/{N_BEAMS}")

    # ── O3D Python baseline ───────────────────────────────────────────────────
    ms_o3d = None
    try:
        import open3d as o3d
        import open3d.t.geometry as otg

        def _bm(cx, cy, hw, hh):
            m = o3d.geometry.TriangleMesh.create_box(hw * 2, hh * 2, EXTRUDE_H)
            m.translate([cx - hw, cy - hh, 0])
            return m

        def _cm(cx, cy, r, n=32):
            m = o3d.geometry.TriangleMesh.create_cylinder(
                radius=r, height=EXTRUDE_H, resolution=n, split=1
            )
            m.translate([cx, cy, EXTRUDE_H / 2])
            return m

        hw = W / 2
        rc = otg.RaycastingScene()
        meshes = (
            [
                _bm(hw, WALL_T / 2, hw + WALL_T, WALL_T / 2),
                _bm(hw, H - WALL_T / 2, hw + WALL_T, WALL_T / 2),
                _bm(WALL_T / 2, H / 2, WALL_T / 2, H / 2),
                _bm(W - WALL_T / 2, H / 2, WALL_T / 2, H / 2),
            ]
            + [_cm(cx, cy, PILLAR_R, 32) for cx in PILLAR_COLS for cy in PILLAR_ROWS]
            + [
                _bm(rx + RACK_LEN / 2, ry, RACK_LEN / 2, RACK_W / 2)
                for rx in RACK_XS
                for ry in RACK_YS
            ]
        )
        for m in meshes:
            v = o3d.core.Tensor(np.asarray(m.vertices, np.float32))
            t = o3d.core.Tensor(np.asarray(m.triangles, np.uint32))
            rc.add_triangles(otg.TriangleMesh(v, t))
        az = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False, dtype=np.float32)
        origs = np.zeros((N_BEAMS, 3), np.float32)
        dirs = np.zeros((N_BEAMS, 3), np.float32)
        dirs[:, 0] = np.cos(az)
        dirs[:, 1] = np.sin(az)
        origs[:, 0] = OX
        origs[:, 1] = OY
        origs[:, 2] = 1.5
        rays = o3d.core.Tensor(
            np.concatenate([origs, dirs], 1), dtype=o3d.core.Dtype.Float32
        )

        def fn_o3d():
            return np.minimum(rc.cast_rays(rays)["t_hit"].numpy(), RMAX)

        print("\nBenchmark (1500 beams, warehouse scene, 2000 reps):")
        ms_o3d = _bench("Open3D Python", fn_o3d)
    except ImportError:
        print("  Open3D not available — skipping O3D baseline")

    # ── AVX2 scalar kernel ────────────────────────────────────────────────────
    ms_avx2_32 = None
    try:
        sys.path.insert(0, os.path.join(SRC_DIR, "irsim_devices", "core"))
        from irsim_devices.core.ray_casting_2d import cast_ray_segments  # noqa: E402

        seg_start = segs32[:, :2].copy()
        seg_end = segs32[:, 2:].copy()
        az = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False, dtype=np.float64)
        origin64 = np.array([OX, OY, 0.0], dtype=np.float64)
        dirs64 = np.column_stack([np.cos(az), np.sin(az), np.zeros(N_BEAMS)]).astype(
            np.float64
        )

        def fn_avx2():
            return cast_ray_segments(
                origin64,
                dirs64,
                seg_start.astype(np.float64),
                seg_end.astype(np.float64),
                RMAX,
            )

        if ms_o3d is None:
            print("\nBenchmark (1500 beams, warehouse scene, 2000 reps):")
        ms_avx2_32 = _bench("AVX2 scalar (Python)", fn_avx2)
    except Exception as e:
        print(f"  AVX2 kernel unavailable: {e}")

    # ── Embree 2D ─────────────────────────────────────────────────────────────
    if ms_o3d is None and ms_avx2_32 is None:
        print("\nBenchmark (1500 beams, warehouse scene, 2000 reps):")
    ms_e2d_s32 = _bench("Embree2D scalar  (res=32)", fn_e2d_s32)
    ms_e2d_8_32 = _bench("Embree2D packet8 (res=32)", fn_e2d_8_32)
    ms_e2d_s360 = _bench("Embree2D scalar  (res=360)", fn_e2d_s360)

    # ── Embree 3D vs O3D 3D (fair: same VLP-16 ray count) ────────────────────
    soup = build_3d_mesh(32)
    ms_e3d = None
    ms_o3d_3d = None
    n_tris = 0
    vlp16_rays = 16 * 1800  # 28800

    if soup is not None:
        n_tris = len(soup)
        s3d = _le.EmbreeScene3D()
        s3d.build_soup(soup)
        vlp16 = (16, 1800, -15.0, 15.0)
        origin3d = np.array([OX, OY, 0.3], dtype=np.float32)

        def fn_e3d():
            return s3d.cast_3d_lidar(origin3d, *vlp16, RMAX)

        warmup_scan = fn_e3d()
        print(
            f"  Embree3D VLP-16 ({vlp16_rays} rays, {n_tris} tris): {len(warmup_scan)} hits"
        )
        ms_e3d = _bench("Embree3D VLP-16 (OMP)", fn_e3d)

        # O3D with same VLP-16 rays for a fair comparison
        try:
            import open3d as o3d
            import open3d.t.geometry as otg

            rc = otg.RaycastingScene()
            all_v2, all_t2, off2 = [], [], 0
            for m in _build_meshes_o3d():
                V2 = np.asarray(m.vertices, np.float32)
                T2 = np.asarray(m.triangles, np.int32) + off2
                all_v2.append(V2)
                all_t2.append(T2)
                off2 += len(V2)
                v2 = o3d.core.Tensor(np.asarray(m.vertices, np.float32))
                t2 = o3d.core.Tensor(np.asarray(m.triangles, np.uint32))
                rc.add_triangles(otg.TriangleMesh(v2, t2))

            n_v, n_h, el_mn, el_mx = vlp16
            origs3 = np.zeros((vlp16_rays, 3), np.float32)
            dirs3 = np.zeros((vlp16_rays, 3), np.float32)
            idx = 0
            for h in range(n_h):
                az = 2 * math.pi * h / n_h
                caz, saz = math.cos(az), math.sin(az)
                for v in range(n_v):
                    el = math.radians(el_mn + (el_mx - el_mn) * v / (n_v - 1))
                    ce, se = math.cos(el), math.sin(el)
                    origs3[idx] = [OX, OY, 0.3]
                    dirs3[idx] = [ce * caz, ce * saz, se]
                    idx += 1
            rays3 = o3d.core.Tensor(
                np.concatenate([origs3, dirs3], 1), dtype=o3d.core.Dtype.Float32
            )

            def fn_o3d_3d():
                return rc.cast_rays(rays3)["t_hit"].numpy()

            ms_o3d_3d = _bench("O3D  VLP-16 (threaded)", fn_o3d_3d)
        except Exception as e:
            print(f"  O3D 3D benchmark skipped: {e}")

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'─' * 60}")
    print(
        f"Summary — LiDAR 2D ({N_BEAMS} beams, 440 segs) + 3D ({vlp16_rays} rays, {n_tris} tris)"
    )
    if ms_o3d:
        print(f"  O3D  2D Python         : {ms_o3d:8.3f} ms")
    if ms_avx2_32:
        print(f"  AVX2 scalar kernel     : {ms_avx2_32:8.3f} ms")
    if ms_o3d:
        print(
            f"  Embree2D scalar  N=32  : {ms_e2d_s32:8.3f} ms  ({ms_o3d / ms_e2d_s32:.1f}× vs O3D)"
        )
        print(
            f"  Embree2D packet8 N=32  : {ms_e2d_8_32:8.3f} ms  ({ms_o3d / ms_e2d_8_32:.1f}× vs O3D)"
        )
    else:
        print(f"  Embree2D scalar  N=32  : {ms_e2d_s32:8.3f} ms")
        print(f"  Embree2D packet8 N=32  : {ms_e2d_8_32:8.3f} ms")
    if ms_o3d_3d:
        print(f"  O3D  3D VLP-16         : {ms_o3d_3d:8.3f} ms")
    if ms_e3d:
        sp3 = f"  ({ms_o3d_3d / ms_e3d:.1f}× vs O3D)" if ms_o3d_3d else ""
        print(f"  Embree3D VLP-16        : {ms_e3d:8.3f} ms{sp3}")

    results = {
        "n_beams_2d": N_BEAMS,
        "n_segs_32": n_segs32,
        "n_segs_360": n_segs360,
        "n_rays_vlp16": vlp16_rays,
        "n_tris": n_tris,
        "hits_embree2d_scalar": hits_e2d,
        "timing_ms": {
            k: round(v, 3)
            for k, v in [
                ("o3d_2d_python", ms_o3d),
                ("avx2_scalar_2d", ms_avx2_32),
                ("embree2d_scalar_32", ms_e2d_s32),
                ("embree2d_packet8_32", ms_e2d_8_32),
                ("embree2d_scalar_360", ms_e2d_s360),
                ("o3d_3d_vlp16", ms_o3d_3d),
                ("embree3d_vlp16_omp", ms_e3d),
            ]
            if v is not None
        },
        "speedup_vs_o3d_2d": (
            {
                k: round(ms_o3d / v, 1)
                for k, v in [
                    ("embree2d_scalar_32", ms_e2d_s32),
                    ("embree2d_packet8_32", ms_e2d_8_32),
                ]
                if v is not None
            }
            if ms_o3d
            else {}
        ),
        "speedup_vs_o3d_3d": (
            {"embree3d_vlp16": round(ms_o3d_3d / ms_e3d, 1)}
            if ms_o3d_3d and ms_e3d
            else {}
        ),
    }

    out = os.path.join(BENCH_DIR, "embree_lidar_results.json")
    with open(out, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults → {out}")


if __name__ == "__main__":
    main()
