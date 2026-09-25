"""Visualization: 2D floor plan, 3D wireframe, kinematic tree."""

from __future__ import annotations

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from .geometry import (
    box_wireframe,
    cylinder_wireframe,
    pose_to_matrix,
    sphere_wireframe,
    world_transform,
)
from .parser import Geometry, Robot

# ── helpers ───────────────────────────────────────────────────────────────────


_ALPHA_CUTOFF = 0.1  # skip near-transparent visuals (e.g. scene_bounds phantoms)


def _link_world_blocks(
    robot: Robot,
    use_collision: bool = False,
    root_T: np.ndarray | None = None,
) -> list[tuple[np.ndarray, Geometry, list[float]]]:
    """Return (world_T 4×4, Geometry, rgba) for every visible geometry block."""
    parent_map = robot.parent_map()
    joint_T = {j.child: pose_to_matrix(j.pose.xyz, j.pose.rpy) for j in robot.joints}
    if root_T is None:
        root_T = np.eye(4)

    results: list[tuple[np.ndarray, Geometry, list[float]]] = []
    for link in robot.links:
        T_world = root_T @ world_transform(link.name, parent_map, joint_T)
        # Prefer visuals (they carry color); fall back to collisions for display
        blocks = link.visuals if not use_collision else link.collisions
        if not blocks:
            blocks = link.collisions if not use_collision else link.visuals
        for block in blocks:
            if block.geometry is None or block.geometry.type == "mesh":
                continue
            if block.color[3] < _ALPHA_CUTOFF:
                continue  # skip phantom / near-transparent geometry
            T_local = pose_to_matrix(block.pose.xyz, block.pose.rpy)
            results.append((T_world @ T_local, block.geometry, block.color))
    return results


# ── 2D floor plan ─────────────────────────────────────────────────────────────


def _xy_footprint(
    T: np.ndarray, geom: Geometry, color: list[float]
) -> list[tuple[str, object, list[float]]]:
    """Top-down XY footprint of a geometry in world frame."""
    patches: list[tuple[str, object, list[float]]] = []
    if geom.type == "box":
        sx, sy = geom.size[0] / 2, geom.size[1] / 2
        local = np.array(
            [
                [-sx, -sy, 0, 1],
                [sx, -sy, 0, 1],
                [sx, sy, 0, 1],
                [-sx, sy, 0, 1],
            ]
        ).T
        world_xy = (T @ local)[:2, :].T  # (4, 2)
        patches.append(("polygon", world_xy, color))
    elif geom.type in ("cylinder", "sphere"):
        cx, cy = T[0, 3], T[1, 3]
        patches.append(("circle", (cx, cy, geom.radius), color))
    return patches


def plot_floor_plan(
    robot: Robot,
    *,
    scan_ranges: np.ndarray | None = None,
    scan_angles: np.ndarray | None = None,
    sensor_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ax: plt.Axes | None = None,
    save: str | None = None,
    show: bool = True,
) -> plt.Figure:
    """Draw top-down 2D floor plan with optional LiDAR scan overlay."""
    fig, ax = plt.subplots(figsize=(10, 7)) if ax is None else (ax.get_figure(), ax)
    ax.set_facecolor("#f0f2f5")
    ax.set_aspect("equal")

    # Draw phantom bounding-box links (near-transparent in URDF) as dashed outlines
    parent_map = robot.parent_map()
    joint_T = {j.child: pose_to_matrix(j.pose.xyz, j.pose.rpy) for j in robot.joints}
    for link in robot.links:
        T_world = world_transform(link.name, parent_map, joint_T)
        for block in link.visuals:
            if block.geometry is None or block.geometry.type == "mesh":
                continue
            if block.color[3] >= _ALPHA_CUTOFF:
                continue  # handled below in the solid pass
            T_local = pose_to_matrix(block.pose.xyz, block.pose.rpy)
            T = T_world @ T_local
            for kind, params, _ in _xy_footprint(T, block.geometry, block.color):
                if kind == "polygon":
                    ax.add_patch(
                        plt.Polygon(
                            params,
                            fc="none",
                            ec="#94a3b8",
                            lw=1.0,
                            ls="--",
                            alpha=0.5,
                            zorder=0,
                        )
                    )
                elif kind == "circle":
                    cx, cy, r = params
                    ax.add_patch(
                        plt.Circle(
                            (cx, cy),
                            r,
                            fc="none",
                            ec="#94a3b8",
                            lw=1.0,
                            ls="--",
                            alpha=0.5,
                            zorder=0,
                        )
                    )

    # Compute scene scale for minimum-size clamping (tiny objects stay visible)
    all_blocks = _link_world_blocks(robot)
    if all_blocks:
        all_x = [T[0, 3] for T, _, _ in all_blocks]
        all_y = [T[1, 3] for T, _, _ in all_blocks]
        scene_scale = max(max(all_x) - min(all_x), max(all_y) - min(all_y), 1.0)
    else:
        scene_scale = 1.0
    min_vis = scene_scale * 0.012  # minimum visible half-extent

    for T, geom, rgba in all_blocks:
        fc = rgba[:3]
        alpha = rgba[3] * 0.80
        ec = [max(0, c - 0.25) for c in fc]
        for kind, params, _ in _xy_footprint(T, geom, rgba):
            if kind == "polygon":
                pts = np.asarray(params)
                # Pad very thin boxes so they stay visible
                ext = pts.max(axis=0) - pts.min(axis=0)
                if ext[0] < min_vis * 2 or ext[1] < min_vis * 2:
                    cx, cy = pts.mean(axis=0)
                    hw = max(ext[0] / 2, min_vis)
                    hh = max(ext[1] / 2, min_vis)
                    pts = np.array(
                        [
                            [cx - hw, cy - hh],
                            [cx + hw, cy - hh],
                            [cx + hw, cy + hh],
                            [cx - hw, cy + hh],
                        ]
                    )
                ax.add_patch(plt.Polygon(pts, fc=fc, ec=ec, lw=0.8, alpha=alpha))
            elif kind == "circle":
                cx, cy, r = params
                r = max(r, min_vis)
                ax.add_patch(plt.Circle((cx, cy), r, fc=fc, ec=ec, lw=0.8, alpha=alpha))

    if scan_ranges is not None and scan_angles is not None:
        sx, sy, sth = sensor_pose
        rmax = float(np.max(scan_ranges)) * 1.05
        hit = scan_ranges < rmax
        ex = sx + scan_ranges * np.cos(sth + scan_angles)
        ey = sy + scan_ranges * np.sin(sth + scan_angles)
        for i in range(0, len(scan_angles), 4):
            if hit[i]:
                ax.plot([sx, ex[i]], [sy, ey[i]], color="#ef4444", lw=0.15, alpha=0.2)
        ax.scatter(
            ex[hit], ey[hit], s=2, color="#ef4444", zorder=4, label=f"hits {hit.sum()}"
        )
        ax.scatter([sx], [sy], s=70, color="#1d4ed8", zorder=5, label="sensor")
        ax.legend(fontsize=8, loc="upper right")

    ax.autoscale_view()
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Floor Plan — {robot.name}", fontweight="bold")
    ax.grid(True, lw=0.3, alpha=0.4)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130, bbox_inches="tight")
        print(f"  Saved → {save}")
    if show:
        plt.show()
    return fig


# ── 3D wireframe ──────────────────────────────────────────────────────────────


def _draw_robot_wireframe(
    ax3,
    robot: Robot,
    root_T: np.ndarray | None = None,
    all_pts: list | None = None,
) -> None:
    """Draw wireframe edges for one robot into ax3; append world-space corners to all_pts."""
    for T, geom, rgba in _link_world_blocks(robot, root_T=root_T):
        if geom.type == "box":
            corners, edges = box_wireframe(geom.size)
        elif geom.type == "cylinder":
            corners, edges = cylinder_wireframe(geom.radius, geom.length)
        elif geom.type == "sphere":
            corners, edges = sphere_wireframe(geom.radius)
        else:
            continue
        h = np.column_stack([corners, np.ones(len(corners))])
        w = (T @ h.T).T[:, :3]
        if all_pts is not None:
            all_pts.append(w)
        edge_color = [max(0, c - 0.15) for c in rgba[:3]]
        for i, j in edges:
            p0, p1 = w[i], w[j]
            ax3.plot(
                [p0[0], p1[0]],
                [p0[1], p1[1]],
                [p0[2], p1[2]],
                color=edge_color,
                lw=0.8,
                alpha=0.85,
            )


def plot_3d(
    robot: Robot,
    *,
    cloud: np.ndarray | None = None,
    overlay: list[tuple[Robot, list[float]]] | None = None,
    ax=None,
    save: str | None = None,
    show: bool = True,
) -> plt.Figure:
    """Draw 3D wireframe for all link geometries.

    Parameters
    ----------
    robot:
        Primary robot/world URDF (rendered at origin).
    cloud:
        Optional (N, 3) or (N, 4) point cloud to scatter-plot.
    overlay:
        List of ``(Robot, [x, y, z, roll, pitch, yaw])`` tuples to render at
        arbitrary world poses — use this to place a robot model inside a world scene.
    """
    fig = plt.figure(figsize=(10, 8)) if ax is None else ax.get_figure()
    ax3 = fig.add_subplot(111, projection="3d") if ax is None else ax
    ax3.set_facecolor("#f0f2f5")

    all_pts: list[np.ndarray] = []
    _draw_robot_wireframe(ax3, robot, all_pts=all_pts)

    for ov_robot, pose_xyzrpy in overlay or []:
        xyz = pose_xyzrpy[:3]
        rpy = pose_xyzrpy[3:6] if len(pose_xyzrpy) >= 6 else [0.0, 0.0, 0.0]
        root_T = pose_to_matrix(xyz, rpy)
        _draw_robot_wireframe(ax3, ov_robot, root_T=root_T, all_pts=all_pts)

    # Equal-aspect 3D scaling (include cloud)
    if cloud is not None and len(cloud):
        all_pts.append(cloud[:, :3])
    if all_pts:
        pts = np.vstack(all_pts)
        mins, maxs = pts.min(axis=0), pts.max(axis=0)
        ranges = maxs - mins
        max_range = max(ranges.max() * 0.5, 1.0)
        mids = (mins + maxs) * 0.5
        ax3.set_xlim(mids[0] - max_range, mids[0] + max_range)
        ax3.set_ylim(mids[1] - max_range, mids[1] + max_range)
        ax3.set_zlim(mids[2] - max_range, mids[2] + max_range)

    if cloud is not None and len(cloud):
        z_col = cloud[:, 2] if cloud.shape[1] > 2 else np.zeros(len(cloud))
        ax3.scatter(
            cloud[:, 0],
            cloud[:, 1],
            z_col,
            s=1.0,
            c=z_col,
            cmap="plasma",
            alpha=0.6,
            zorder=5,
        )

    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_zlabel("Z (m)")
    names = robot.name
    if overlay:
        names += " + " + " + ".join(r.name for r, _ in overlay)
    ax3.set_title(f"3D View — {names}", fontweight="bold")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130, bbox_inches="tight")
        print(f"  Saved → {save}")
    if show:
        plt.show()
    return fig


# ── kinematic tree ────────────────────────────────────────────────────────────

_JOINT_COLORS = {
    "fixed": "#6366f1",
    "continuous": "#10b981",
    "revolute": "#f59e0b",
    "prismatic": "#ef4444",
    "floating": "#8b5cf6",
    "planar": "#ec4899",
}


def plot_kinematic_tree(
    robot: Robot,
    *,
    ax: plt.Axes | None = None,
    save: str | None = None,
    show: bool = True,
) -> plt.Figure:
    """Draw kinematic tree as a BFS box-and-arrow diagram."""
    children = robot.children_map()
    root = robot.root_link() or robot.links[0].name

    # BFS layout
    by_depth: dict[int, list[str]] = {}
    queue = [(root, 0)]
    visited: set[str] = set()
    while queue:
        name, depth = queue.pop(0)
        if name in visited:
            continue
        visited.add(name)
        by_depth.setdefault(depth, []).append(name)
        for kid in children.get(name, []):
            queue.append((kid, depth + 1))
    for lnk in robot.links:
        if lnk.name not in visited:
            by_depth.setdefault(0, []).append(lnk.name)

    final_pos: dict[str, tuple[float, float]] = {}
    for d, names in by_depth.items():
        n = len(names)
        for i, name in enumerate(names):
            final_pos[name] = (d, i - (n - 1) / 2.0)

    max_d = max(by_depth.keys()) if by_depth else 0
    w = max(8.0, (max_d + 1) * 2.5)
    h = max(5.0, len(robot.links) * 0.7 + 2)
    fig, ax_t = plt.subplots(figsize=(w, h)) if ax is None else (ax.get_figure(), ax)
    ax_t.set_facecolor("#f8f9fb")
    ax_t.set_aspect("equal")
    ax_t.axis("off")

    for j in robot.joints:
        if j.parent not in final_pos or j.child not in final_pos:
            continue
        xp, yp = final_pos[j.parent]
        xc, yc = final_pos[j.child]
        col = _JOINT_COLORS.get(j.type, "#888")
        ax_t.annotate(
            "",
            xy=(xc - 0.42, yc),
            xytext=(xp + 0.42, yp),
            arrowprops=dict(arrowstyle="-|>", color=col, lw=1.4),
        )
        ax_t.text(
            (xp + xc) / 2,
            (yp + yc) / 2 + 0.08,
            j.name,
            fontsize=6.5,
            color=col,
            ha="center",
            va="bottom",
            style="italic",
        )

    for name, (x, y) in final_pos.items():
        is_root = name == root
        ax_t.add_patch(
            mpatches.FancyBboxPatch(
                (x - 0.40, y - 0.22),
                0.80,
                0.44,
                boxstyle="round,pad=0.05",
                lw=1.2,
                edgecolor="#94a3b8",
                facecolor="#1d4ed8" if is_root else "#e2e8f0",
            )
        )
        ax_t.text(
            x,
            y,
            name,
            ha="center",
            va="center",
            fontsize=7.5,
            color="white" if is_root else "#1e293b",
            fontweight="bold" if is_root else "normal",
        )

    legend_entries = [
        mpatches.Patch(color=c, label=t)
        for t, c in _JOINT_COLORS.items()
        if any(j.type == t for j in robot.joints)
    ]
    if legend_entries:
        ax_t.legend(
            handles=legend_entries,
            fontsize=7,
            loc="upper right",
            title="Joint type",
            title_fontsize=7.5,
            framealpha=0.85,
        )

    yvals = [y for _, y in final_pos.values()]
    ax_t.set_xlim(-0.6, max_d + 0.6)
    ax_t.set_ylim(min(yvals) - 0.6, max(yvals) + 0.6)
    ax_t.set_title(
        f"Kinematic Tree — {robot.name}\n"
        f"{len(robot.links)} links · {len(robot.joints)} joints",
        fontsize=11,
        fontweight="bold",
    )
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130, bbox_inches="tight")
        print(f"  Saved → {save}")
    if show:
        plt.show()
    return fig
