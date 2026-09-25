"""urdf_demo.py — Load world and robot from URDF, cast rays, compare vs programmatic model.

Produces
--------
urdf_demo_results.json   — model stats + timing comparison
urdf_fig_2d.png          — 2D LiDAR scan from URDF-loaded world
urdf_fig_robot.png       — robot URDF link/joint diagram
"""

from __future__ import annotations

import io
import json
import math
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BENCH_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(BENCH_DIR, "..")
CPP_DIR = os.path.join(PKG_DIR, "cpp")
MODEL_DIR = os.path.join(PKG_DIR, "models")

sys.path.insert(0, CPP_DIR)
sys.path.insert(0, MODEL_DIR)
sys.path.insert(0, os.path.join(PKG_DIR, "src"))

import lidar_embree as _le  # noqa: E402
from urdf_loader import describe_urdf, load_urdf  # noqa: E402
from warehouse_model import build_warehouse  # noqa: E402

# ── Constants ──────────────────────────────────────────────────────────────────
OX, OY = 12.0, 15.0
SENSOR_Z = 1.5
N_BEAMS = 1500
RMAX = 35.0
REPEATS = 1000
WARMUP = 200
WORLD_URDF = os.path.join(MODEL_DIR, "warehouse_world.urdf")
ROBOT_URDF = os.path.join(MODEL_DIR, "robot_diff.urdf")


# ── Timing ─────────────────────────────────────────────────────────────────────
def bench(name: str, fn, n: int = REPEATS, w: int = WARMUP) -> float:
    for _ in range(w):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    ms = (time.perf_counter() - t0) / n * 1e3
    print(f"  {name:45s}: {ms:8.3f} ms/step")
    return ms


# ── Build Embree 2D scene from any model ───────────────────────────────────────
def build_2d_scene(segments_2d: np.ndarray) -> _le.EmbreeScene2D:
    s = _le.EmbreeScene2D()
    s.build(np.ascontiguousarray(segments_2d, np.float32))
    return s


# ── Cast 2D ────────────────────────────────────────────────────────────────────
def cast_2d(scene: _le.EmbreeScene2D):
    az = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False, dtype=np.float32)
    dx, dy = np.cos(az), np.sin(az)
    origin = np.array([OX, OY], dtype=np.float32)
    out_r = np.empty(N_BEAMS, dtype=np.float32)
    out_h = np.empty(N_BEAMS, dtype=np.int32)
    scene.cast8_inplace(origin, dx, dy, float(RMAX), out_r, out_h)
    return az, out_r, dx, dy


# ── Figures ────────────────────────────────────────────────────────────────────
def fig_2d_comparison(
    segs_prog: np.ndarray,
    segs_urdf: np.ndarray,
    r_prog: np.ndarray,
    r_urdf: np.ndarray,
    dx: np.ndarray,
    dy: np.ndarray,
) -> bytes:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), facecolor="#f8f9fb")

    for ax, segs, out_r, title, col in zip(
        axes,
        [segs_prog, segs_urdf],
        [r_prog, r_urdf],
        ["Programmatic model\n(warehouse_model.py)", "URDF-loaded model\n(warehouse_world.urdf)"],
        ["#ef4444", "#2563eb"],
        strict=True,
    ):
        ax.set_facecolor("#f0f2f5")
        ax.set_aspect("equal")
        for seg in segs[::2]:
            ax.plot([seg[0], seg[2]], [seg[1], seg[3]], color="#8892a4", lw=0.4, alpha=0.4)
        hit = out_r < RMAX
        ex = OX + out_r * dx
        ey = OY + out_r * dy
        for i in range(0, N_BEAMS, 6):
            if hit[i]:
                ax.plot([OX, ex[i]], [OY, ey[i]], color=col, lw=0.18, alpha=0.3)
        ax.scatter(ex[hit], ey[hit], s=1.5, color=col, zorder=4, label=f"Hits {hit.sum()}")
        ax.scatter([OX], [OY], s=60, color="#1d4ed8", zorder=5)
        ax.set_xlim(-2, 53)
        ax.set_ylim(-2, 33)
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.set_xlabel("X (m)", fontsize=9)
        ax.set_ylabel("Y (m)", fontsize=9)
        ax.grid(True, lw=0.3, alpha=0.4)
        ax.legend(fontsize=8, loc="upper right")

    fig.suptitle(f"2D LiDAR — {N_BEAMS} beams, sensor @ ({OX},{OY})", fontsize=12, fontweight="bold")
    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


def fig_robot_urdf(robot_info: dict) -> bytes:
    """Draw the robot kinematic tree as a box-and-arrow diagram."""
    links = robot_info["links"]
    joints = robot_info["joints"]

    # Build parent→children map
    children: dict[str, list[str]] = {lnk: [] for lnk in links}
    joint_types: dict[str, str] = {}
    for j in joints:
        children[j["parent"]].append(j["child"])
        joint_types[j["child"]] = j["type"]

    # BFS layout: x = depth, y = sibling order
    pos: dict[str, tuple[float, float]] = {}
    depth_counts: dict[int, int] = {}

    def place(name, depth, order):
        if name in pos:
            return
        pos[name] = (depth, order)
        kids = children.get(name, [])
        for _i, kid in enumerate(kids):
            d2 = depth + 1
            cnt = depth_counts.get(d2, 0)
            depth_counts[d2] = cnt + 1
            place(kid, d2, cnt)

    depth_counts[0] = 0
    place("base_link", 0, 0)
    # Fill any disconnected links
    for lnk in links:
        if lnk not in pos:
            pos[lnk] = (0, len(pos))

    # Normalize y positions by depth
    by_depth: dict[int, list[str]] = {}
    for name, (d, _) in pos.items():
        by_depth.setdefault(d, []).append(name)
    final_pos: dict[str, tuple[float, float]] = {}
    max_d = max(by_depth.keys()) if by_depth else 0
    for d, names in by_depth.items():
        n = len(names)
        for i, name in enumerate(names):
            final_pos[name] = (d, i - (n - 1) / 2.0)

    fig, ax = plt.subplots(figsize=(max(8, (max_d + 1) * 2.5), max(5, len(links) * 0.7 + 2)),
                           facecolor="#f8f9fb")
    ax.set_facecolor("#f8f9fb")
    ax.set_aspect("equal")
    ax.axis("off")

    # Color per joint type
    type_color = {
        "fixed": "#6366f1",
        "continuous": "#10b981",
        "revolute": "#f59e0b",
        "prismatic": "#ef4444",
        "floating": "#8b5cf6",
        "planar": "#ec4899",
    }

    # Draw edges
    for j in joints:
        p, c = j["parent"], j["child"]
        if p not in final_pos or c not in final_pos:
            continue
        xp, yp = final_pos[p]
        xc, yc = final_pos[c]
        jtype = j.get("type", "fixed")
        col = type_color.get(jtype, "#888")
        ax.annotate(
            "",
            xy=(xc - 0.42, yc),
            xytext=(xp + 0.42, yp),
            arrowprops=dict(arrowstyle="-|>", color=col, lw=1.4),
        )
        mx, my = (xp + xc) / 2, (yp + yc) / 2
        ax.text(mx, my + 0.08, j["name"], fontsize=6.5, color=col,
                ha="center", va="bottom", style="italic")

    # Draw link boxes
    for name, (x, y) in final_pos.items():
        is_base = name == "base_link"
        facecolor = "#1d4ed8" if is_base else "#e2e8f0"
        textcolor = "white" if is_base else "#1e293b"
        rect = mpatches.FancyBboxPatch(
            (x - 0.40, y - 0.22), 0.80, 0.44,
            boxstyle="round,pad=0.05",
            linewidth=1.2,
            edgecolor="#94a3b8",
            facecolor=facecolor,
        )
        ax.add_patch(rect)
        ax.text(x, y, name, ha="center", va="center", fontsize=7.5,
                color=textcolor, fontweight="bold" if is_base else "normal")

    # Legend
    legend_entries = [
        mpatches.Patch(color=c, label=t) for t, c in type_color.items()
        if any(j.get("type") == t for j in joints)
    ]
    if legend_entries:
        ax.legend(handles=legend_entries, fontsize=7, loc="upper right",
                  title="Joint type", title_fontsize=7.5,
                  framealpha=0.85, borderpad=0.6)

    ax.set_xlim(-0.6, max_d + 0.6)
    yvals = [y for _, y in final_pos.values()]
    ax.set_ylim(min(yvals) - 0.6, max(yvals) + 0.6)
    ax.set_title(
        f"URDF Kinematic Tree — {robot_info['robot_name']}\n"
        f"{len(links)} links · {len(joints)} joints",
        fontsize=11, fontweight="bold",
    )
    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


# ── Main ───────────────────────────────────────────────────────────────────────


def main():
    print("=" * 65)
    print("URDF Demo — warehouse world + diff-drive robot")
    print("=" * 65)

    # ── Load URDF world ────────────────────────────────────────────────────
    print(f"\n[1] Loading URDF world: {WORLD_URDF}")
    world_info = describe_urdf(WORLD_URDF)
    print(f"     robot_name : {world_info['robot_name']}")
    print(f"     links      : {world_info['links']}")
    print(f"     geometry   : {world_info['geometry_counts']}")

    urdf_model = load_urdf(WORLD_URDF, use_collision=True, slice_z=1.0)
    print(f"     {urdf_model}")

    # ── Describe robot URDF ────────────────────────────────────────────────
    print(f"\n[2] Describing robot URDF: {ROBOT_URDF}")
    robot_info = describe_urdf(ROBOT_URDF)
    print(f"     robot_name : {robot_info['robot_name']}")
    for j in robot_info["joints"]:
        print(f"     joint  {j['name']:30s}  [{j['type']}]  {j['parent']} → {j['child']}")

    # ── Programmatic model for comparison ─────────────────────────────────
    print("\n[3] Building programmatic model (warehouse_model.py) …")
    prog_model = build_warehouse()
    segs_prog = prog_model.segments_2d
    print(
        f"     prog: {len(prog_model.vertices)} verts, "
        f"{len(prog_model.triangles)} tris, {len(segs_prog)} segs"
    )
    segs_urdf = urdf_model.segments_2d
    print(
        f"     urdf: {len(urdf_model.vertices)} verts, "
        f"{len(urdf_model.triangles)} tris, {len(segs_urdf)} segs"
    )

    # ── Build Embree scenes & cast ─────────────────────────────────────────
    print("\n[4] Building Embree scenes and casting rays …")
    s2d_prog = build_2d_scene(segs_prog)
    s2d_urdf = build_2d_scene(segs_urdf)

    az, r_prog, dx, dy = cast_2d(s2d_prog)
    _, r_urdf, _, _ = cast_2d(s2d_urdf)

    hits_prog = int(np.sum(r_prog < RMAX))
    hits_urdf = int(np.sum(r_urdf < RMAX))
    print(f"     prog hits: {hits_prog}/{N_BEAMS}   urdf hits: {hits_urdf}/{N_BEAMS}")

    # ── Benchmark ──────────────────────────────────────────────────────────
    print("\n[5] Benchmarking …")

    def fn_prog():
        out = np.empty(N_BEAMS, dtype=np.float32)
        oh = np.empty(N_BEAMS, dtype=np.int32)
        s2d_prog.cast8_inplace(
            np.array([OX, OY], np.float32), dx.astype(np.float32),
            dy.astype(np.float32), float(RMAX), out, oh,
        )

    def fn_urdf():
        out = np.empty(N_BEAMS, dtype=np.float32)
        oh = np.empty(N_BEAMS, dtype=np.int32)
        s2d_urdf.cast8_inplace(
            np.array([OX, OY], np.float32), dx.astype(np.float32),
            dy.astype(np.float32), float(RMAX), out, oh,
        )

    ms_prog = bench("Embree2D packet-8 programmatic model", fn_prog)
    ms_urdf = bench("Embree2D packet-8 URDF-loaded model  ", fn_urdf)

    # ── Figures ────────────────────────────────────────────────────────────
    print("\n[6] Rendering figures …")
    png_2d = fig_2d_comparison(segs_prog, segs_urdf, r_prog, r_urdf, dx, dy)
    png_robot = fig_robot_urdf(robot_info)

    for fname, data in [("urdf_fig_2d.png", png_2d), ("urdf_fig_robot.png", png_robot)]:
        path = os.path.join(BENCH_DIR, fname)
        with open(path, "wb") as f:
            f.write(data)
        print(f"     PNG → {path}  ({len(data) // 1024} KB)")

    # ── Save results ───────────────────────────────────────────────────────
    results = {
        "world_urdf": {
            "robot_name": world_info["robot_name"],
            "links": world_info["links"],
            "geometry_counts": world_info["geometry_counts"],
        },
        "robot_urdf": {
            "robot_name": robot_info["robot_name"],
            "links": robot_info["links"],
            "joints": robot_info["joints"],
        },
        "model_comparison": {
            "programmatic": {
                "vertices": int(len(prog_model.vertices)),
                "triangles": int(len(prog_model.triangles)),
                "segments_2d": int(len(segs_prog)),
                "hits_2d": hits_prog,
            },
            "urdf_loaded": {
                "vertices": int(len(urdf_model.vertices)),
                "triangles": int(len(urdf_model.triangles)),
                "segments_2d": int(len(segs_urdf)),
                "hits_2d": hits_urdf,
            },
        },
        "timing_ms": {
            "embree2d_packet8_prog": round(ms_prog, 3),
            "embree2d_packet8_urdf": round(ms_urdf, 3),
        },
    }

    out_json = os.path.join(BENCH_DIR, "urdf_demo_results.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  Results → {out_json}")
    return results, png_2d, png_robot


if __name__ == "__main__":
    main()
