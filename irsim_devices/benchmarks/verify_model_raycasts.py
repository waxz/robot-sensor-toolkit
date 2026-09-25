"""verify_model_raycasts.py — Load warehouse model, cast 2D/3D rays, benchmark.

Produces:
  fig_2d_scan.png       — top-down floor plan + 2D LiDAR scan
  fig_3d_cloud.png      — 3D VLP-16 point cloud coloured by height
  fig_polar.png         — polar range profile
  model_bench_results.json
"""

from __future__ import annotations

import base64
import io
import json
import math
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# ── Path setup ────────────────────────────────────────────────────────────────
BENCH_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(BENCH_DIR, "..")
CPP_DIR = os.path.join(PKG_DIR, "cpp")
MODEL_DIR = os.path.join(PKG_DIR, "models")
sys.path.insert(0, CPP_DIR)
sys.path.insert(0, os.path.join(PKG_DIR, "src"))

import lidar_embree as _le  # noqa: E402

sys.path.insert(0, MODEL_DIR)
from warehouse_model import build_warehouse  # noqa: E402

# ── Constants ─────────────────────────────────────────────────────────────────
OX, OY = 12.0, 15.0  # sensor position (centre aisle)
SENSOR_Z = 1.5  # sensor height
N_BEAMS = 1500  # 2D beams
RMAX_2D = 35.0
RMAX_3D = 50.0
REPEATS = 2000
WARMUP = 300

# VLP-16 profile: 16 rings × 1800 horizontal steps = 28 800 rays
VLP16 = (16, 1800, -15.0, 15.0)

PALETTE = {
    "floor": "#e8eaed",
    "wall": "#8892a4",
    "rack": "#c0cde0",
    "scan": "#ef4444",
    "beam": "#fca5a5",
}


# ── OBJ loader ─────────────────────────────────────────────────────────────────
def load_obj(path: str):
    """Parse OBJ → (vertices float32[V,3], triangles int32[T,3])."""
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            tok = line.split()
            if not tok:
                continue
            if tok[0] == "v":
                verts.append([float(x) for x in tok[1:4]])
            elif tok[0] == "f":
                idxs = [int(t.split("/")[0]) - 1 for t in tok[1:]]
                for i in range(1, len(idxs) - 1):
                    tris.append((idxs[0], idxs[i], idxs[i + 1]))
    return np.array(verts, dtype=np.float32), np.array(tris, dtype=np.int32)


# ── Timing helper ──────────────────────────────────────────────────────────────
def bench(name, fn, n=REPEATS, w=WARMUP):
    for _ in range(w):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    ms = (time.perf_counter() - t0) / n * 1e3
    print(f"  {name:35s}: {ms:8.3f} ms/step")
    return ms


# ── Build scenes ───────────────────────────────────────────────────────────────
def build_scenes(model):
    print("Building Embree scenes …")
    segs = model.segments_2d.copy()
    soup = model.triangle_soup()

    s2d = _le.EmbreeScene2D()
    s2d.build(np.ascontiguousarray(segs, np.float32))

    s3d = _le.EmbreeScene3D()
    s3d.build_soup(np.ascontiguousarray(soup, np.float32))

    print(f"  2D: {len(segs)} segments,  3D: {len(soup)} triangles")
    return s2d, s3d, segs, soup


# ── 2D scan ────────────────────────────────────────────────────────────────────
def cast_2d(s2d, segs):
    az = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False, dtype=np.float32)
    dx = np.cos(az)
    dy = np.sin(az)
    origin = np.array([OX, OY], dtype=np.float32)
    out_r = np.empty(N_BEAMS, dtype=np.float32)
    out_h = np.empty(N_BEAMS, dtype=np.int32)

    # packet-8 cast
    s2d.cast8_inplace(origin, dx, dy, float(RMAX_2D), out_r, out_h)

    hits = int(np.sum(out_r < RMAX_2D))
    print(f"  2D: {hits}/{N_BEAMS} beams hit  (range_max={RMAX_2D} m)")

    # Endpoint coords
    ex = OX + out_r * dx
    ey = OY + out_r * dy

    return az, out_r, dx, dy, ex, ey


# ── 3D scan ────────────────────────────────────────────────────────────────────
def cast_3d(s3d):
    origin = np.array([OX, OY, SENSOR_Z], dtype=np.float32)
    nv, nh, el_min, el_max = VLP16
    scan = s3d.cast_3d_lidar(
        origin, nv, nh, float(el_min), float(el_max), float(RMAX_3D)
    )
    print(f"  3D VLP-16: {len(scan)} hits  ({nv * nh} rays, range_max={RMAX_3D} m)")
    return scan  # [N_hits, 4] x y z intensity


# ── Visualisations ─────────────────────────────────────────────────────────────


def fig_2d_scan(segs, az, out_r, dx, dy, ex, ey) -> bytes:
    fig, ax = plt.subplots(figsize=(12, 7.5), facecolor="#f8f9fb")
    ax.set_facecolor("#f0f2f5")
    ax.set_aspect("equal")

    # Draw geometry segments
    for seg in segs[::2]:  # thin out for speed
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]], color="#8892a4", lw=0.4, alpha=0.5)

    # Draw scan beams (thin, clipped to hit)
    hit_mask = out_r < RMAX_2D
    for i in range(0, N_BEAMS, 5):
        if hit_mask[i]:
            ax.plot([OX, ex[i]], [OY, ey[i]], color="#fca5a5", lw=0.2, alpha=0.35)

    # Sensor position
    ax.scatter([OX], [OY], s=60, color="#2563eb", zorder=5, label="Sensor")

    # Hit points
    ax.scatter(
        ex[hit_mask],
        ey[hit_mask],
        s=2,
        color="#ef4444",
        zorder=4,
        label=f"Hits ({hit_mask.sum()})",
    )

    # Range ring
    theta = np.linspace(0, 2 * math.pi, 300)
    ax.plot(
        OX + RMAX_2D * np.cos(theta),
        OY + RMAX_2D * np.sin(theta),
        color="#2563eb",
        lw=0.6,
        ls="--",
        alpha=0.3,
        label=f"R_max={RMAX_2D} m",
    )

    ax.set_xlim(-2, 53)
    ax.set_ylim(-2, 33)
    ax.set_xlabel("X (m)", fontsize=10)
    ax.set_ylabel("Y (m)", fontsize=10)
    ax.set_title(
        f"2D LiDAR — {N_BEAMS} beams · {hit_mask.sum()} hits · "
        f"Embree2D packet-8 · warehouse 50×30 m",
        fontsize=11,
        fontweight="bold",
    )
    ax.legend(loc="upper right", fontsize=9, framealpha=0.8)
    ax.grid(True, lw=0.3, alpha=0.4)

    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


def fig_3d_cloud(scan: np.ndarray) -> bytes:
    if len(scan) == 0:
        return b""
    x, y, z = scan[:, 0], scan[:, 1], scan[:, 2]

    fig = plt.figure(figsize=(12, 7), facecolor="#0d1117")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0d1117")
    ax.set_facecolor("#0d1117")

    # Colour by height
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-6)
    sc = ax.scatter(x, y, z, c=z_norm, cmap="plasma", s=0.8, alpha=0.7, linewidths=0)

    cbar = fig.colorbar(sc, ax=ax, pad=0.02, shrink=0.6)
    cbar.set_label("Height (normalised)", color="#c9d1d9", fontsize=8)
    cbar.ax.yaxis.set_tick_params(color="#c9d1d9")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="#c9d1d9")

    ax.set_xlabel("X (m)", color="#c9d1d9", fontsize=8, labelpad=4)
    ax.set_ylabel("Y (m)", color="#c9d1d9", fontsize=8, labelpad=4)
    ax.set_zlabel("Z (m)", color="#c9d1d9", fontsize=8, labelpad=4)
    ax.tick_params(colors="#6b7280", labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor("#3d4460")
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    ax.grid(True, color="#2d3248", lw=0.4)

    n_hits = len(scan)
    n_rays = VLP16[0] * VLP16[1]
    ax.set_title(
        f"VLP-16 Point Cloud — {n_hits} hits / {n_rays} rays\n"
        f"Embree3D + OpenMP · sensor @ ({OX:.0f}, {OY:.0f}, {SENSOR_Z} m)",
        color="#e8ecf4",
        fontsize=10,
        fontweight="bold",
        pad=12,
    )
    ax.view_init(elev=28, azim=-55)

    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(
        buf, format="png", dpi=130, bbox_inches="tight", facecolor=fig.get_facecolor()
    )
    plt.close(fig)
    return buf.getvalue()


def fig_polar(az: np.ndarray, out_r: np.ndarray) -> bytes:
    fig, ax = plt.subplots(
        figsize=(7, 7), subplot_kw={"projection": "polar"}, facecolor="#f8f9fb"
    )
    ax.set_facecolor("#f0f2f5")

    hit_mask = out_r < RMAX_2D
    r_plot = np.where(hit_mask, out_r, RMAX_2D)

    ax.plot(az, r_plot, color="#2563eb", lw=0.5, alpha=0.6)
    ax.fill(az, r_plot, color="#2563eb", alpha=0.15)
    ax.scatter(
        az[hit_mask], out_r[hit_mask], s=0.6, color="#ef4444", alpha=0.6, zorder=3
    )

    ax.set_rmax(RMAX_2D)
    ax.set_rticks([5, 10, 20, 30])
    ax.set_rlabel_position(45)
    ax.grid(True, lw=0.4, alpha=0.5)
    ax.set_title(
        f"2D LiDAR Range Profile\n{N_BEAMS} beams, {hit_mask.sum()} hits",
        fontsize=10,
        fontweight="bold",
        pad=16,
    )

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


def b64(data: bytes) -> str:
    return base64.b64encode(data).decode()


# ── Main ───────────────────────────────────────────────────────────────────────


def main():
    print("=" * 60)
    print("Warehouse model — Embree ray-cast verification + benchmark")
    print("=" * 60)

    # ── Build model ──────────────────────────────────────────────────────
    print("\n[1] Building model …")
    model = build_warehouse()
    v, t = model.vertices, model.triangles
    segs = model.segments_2d
    print(f"  {len(v)} verts · {len(t)} triangles · {len(segs)} 2D segments")

    # ── Build Embree scenes ──────────────────────────────────────────────
    print("\n[2] Building Embree scenes …")
    s2d, s3d, segs, soup = build_scenes(model)

    # ── Ray casts ────────────────────────────────────────────────────────
    print("\n[3] Casting rays …")
    az, out_r, dx, dy, ex, ey = cast_2d(s2d, segs)
    scan3d = cast_3d(s3d)

    # ── Visualise ────────────────────────────────────────────────────────
    print("\n[4] Rendering visualisations …")
    png_2d = fig_2d_scan(segs, az, out_r, dx, dy, ex, ey)
    png_3d = fig_3d_cloud(scan3d)
    png_polar = fig_polar(az, out_r)
    print(f"  fig_2d_scan  : {len(png_2d) // 1024} KB")
    print(f"  fig_3d_cloud : {len(png_3d) // 1024} KB")
    print(f"  fig_polar    : {len(png_polar) // 1024} KB")

    # ── Benchmarks ────────────────────────────────────────────────────────
    print("\n[5] Benchmarking …")
    origin_f = np.array([OX, OY], dtype=np.float32)
    out_r2 = np.empty(N_BEAMS, dtype=np.float32)
    out_h2 = np.empty(N_BEAMS, dtype=np.int32)

    def fn_scalar():
        return s2d.cast_inplace(
            origin_f,
            dx.astype(np.float32),
            dy.astype(np.float32),
            float(RMAX_2D),
            out_r2,
            out_h2,
        )

    def fn_packet8():
        return s2d.cast8_inplace(
            origin_f,
            dx.astype(np.float32),
            dy.astype(np.float32),
            float(RMAX_2D),
            out_r2,
            out_h2,
        )

    def fn_packet16():
        return s2d.cast16_inplace(
            origin_f,
            dx.astype(np.float32),
            dy.astype(np.float32),
            float(RMAX_2D),
            out_r2,
            out_h2,
        )

    origin3d = np.array([OX, OY, SENSOR_Z], dtype=np.float32)

    def fn_3d():
        return s3d.cast_3d_lidar(origin3d, *VLP16, float(RMAX_3D))

    def fn_3d_p16():
        return s3d.cast_3d_lidar_packet16(origin3d, *VLP16, float(RMAX_3D))

    ms_scalar = bench("Embree2D scalar      (1548 segs)", fn_scalar)
    ms_packet8 = bench("Embree2D packet-8    (1548 segs)", fn_packet8)
    ms_packet16 = bench("Embree2D packet-16   (1548 segs)", fn_packet16)
    ms_3d = bench("Embree3D VLP-16 scalar+OMP (4692 tris)", fn_3d)
    ms_3d_p16 = bench("Embree3D VLP-16 pkt16+OMP (4692 tris)", fn_3d_p16)

    print(f"\n  2D  packet8  vs scalar  : {ms_scalar / ms_packet8:.2f}×")
    print(f"  2D  packet16 vs scalar  : {ms_scalar / ms_packet16:.2f}×")
    print(f"  3D  pkt16+OMP vs scalar : {ms_3d / ms_3d_p16:.2f}×")

    # ── Try O3D 3D for fair comparison ────────────────────────────────────
    ms_o3d_3d = None
    try:
        import open3d as o3d
        import open3d.t.geometry as otg

        rc = otg.RaycastingScene()
        for i in range(0, len(t), 100):
            chunk = t[i : i + 100]
            vt = o3d.core.Tensor(v[chunk.flatten()].reshape(-1, 3, 3).reshape(-1, 3))
            tt = o3d.core.Tensor(
                np.arange(len(chunk) * 3, dtype=np.uint32).reshape(-1, 3)
            )
            rc.add_triangles(otg.TriangleMesh(vt, tt))
        nv2, nh2, el_mn, el_mx = VLP16
        n_rays = nv2 * nh2
        origs3 = np.zeros((n_rays, 3), np.float32)
        dirs3 = np.zeros((n_rays, 3), np.float32)
        origs3[:, 0] = OX
        origs3[:, 1] = OY
        origs3[:, 2] = SENSOR_Z
        idx = 0
        for h in range(nh2):
            az_v = 2 * math.pi * h / nh2
            caz, saz = math.cos(az_v), math.sin(az_v)
            for vv in range(nv2):
                el = math.radians(el_mn + (el_mx - el_mn) * vv / (nv2 - 1))
                ce, se = math.cos(el), math.sin(el)
                dirs3[idx] = [ce * caz, ce * saz, se]
                idx += 1
        rays3 = o3d.core.Tensor(
            np.concatenate([origs3, dirs3], 1), dtype=o3d.core.Dtype.Float32
        )

        def fn_o3d():
            return rc.cast_rays(rays3)["t_hit"].numpy()

        ms_o3d_3d = bench("Open3D 3D VLP-16 (reference)", fn_o3d)
    except Exception as e:
        print(f"  O3D 3D skipped: {e}")

    # ── Save results ──────────────────────────────────────────────────────
    results = {
        "model": {
            "vertices": int(len(v)),
            "triangles": int(len(t)),
            "segments_2d": int(len(segs)),
        },
        "raycast": {
            "n_beams_2d": N_BEAMS,
            "hits_2d": int(np.sum(out_r < RMAX_2D)),
            "n_rays_3d": VLP16[0] * VLP16[1],
            "hits_3d": int(len(scan3d)),
            "sensor_pos": [OX, OY, SENSOR_Z],
        },
        "timing_ms": {
            k: round(v2, 3)
            for k, v2 in [
                ("embree2d_scalar", ms_scalar),
                ("embree2d_packet8", ms_packet8),
                ("embree2d_packet16", ms_packet16),
                ("embree3d_vlp16_omp", ms_3d),
                ("embree3d_vlp16_pkt16_omp", ms_3d_p16),
                ("o3d_3d_vlp16", ms_o3d_3d),
            ]
            if v2 is not None
        },
        "speedup_2d": {
            "packet8_vs_scalar": round(ms_scalar / ms_packet8, 2),
            "packet16_vs_scalar": round(ms_scalar / ms_packet16, 2),
            "packet16_vs_packet8": round(ms_packet8 / ms_packet16, 2),
        },
        "speedup_3d_pkt16_vs_scalar_omp": round(ms_3d / ms_3d_p16, 2),
    }
    if ms_o3d_3d:
        results["speedup_3d_vs_o3d"] = {
            "scalar_omp": round(ms_o3d_3d / ms_3d, 2),
            "pkt16_omp": round(ms_o3d_3d / ms_3d_p16, 2),
        }

    out_json = os.path.join(BENCH_DIR, "model_bench_results.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  Results → {out_json}")

    # ── Save PNGs ─────────────────────────────────────────────────────────
    for fname, data in [
        ("fig_2d_scan.png", png_2d),
        ("fig_3d_cloud.png", png_3d),
        ("fig_polar.png", png_polar),
    ]:
        path = os.path.join(BENCH_DIR, fname)
        with open(path, "wb") as f:
            f.write(data)
        print(f"  PNG  → {path}")

    return results, png_2d, png_3d, png_polar


if __name__ == "__main__":
    main()
