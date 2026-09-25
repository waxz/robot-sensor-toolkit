"""Transform utilities and primitive wireframe generators."""

from __future__ import annotations

import math

import numpy as np


def rpy_to_matrix(rpy: list[float]) -> np.ndarray:
    """Intrinsic XYZ roll-pitch-yaw → 3×3 rotation matrix."""
    cr, cp, cy = math.cos(rpy[0]), math.cos(rpy[1]), math.cos(rpy[2])
    sr, sp, sy = math.sin(rpy[0]), math.sin(rpy[1]), math.sin(rpy[2])
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def pose_to_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """XYZ + RPY → 4×4 homogeneous transform."""
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = rpy_to_matrix(rpy)
    return T


def world_transform(
    link_name: str,
    parent_map: dict[str, str],
    joint_T: dict[str, np.ndarray],
) -> np.ndarray:
    """Walk kinematic chain; return link-to-world 4×4 matrix."""
    T, cur, seen = np.eye(4), link_name, set()
    while cur in parent_map and cur not in seen:
        seen.add(cur)
        T = joint_T[cur] @ T
        cur = parent_map[cur]
    return T


def box_wireframe(size: list[float]) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """8 corners + 12 edges for a box centered at origin."""
    sx, sy, sz = size[0] / 2, size[1] / 2, size[2] / 2
    corners = np.array(
        [
            [-sx, -sy, -sz],
            [sx, -sy, -sz],
            [sx, sy, -sz],
            [-sx, sy, -sz],
            [-sx, -sy, sz],
            [sx, -sy, sz],
            [sx, sy, sz],
            [-sx, sy, sz],
        ],
        dtype=np.float32,
    )
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    return corners, edges


def cylinder_wireframe(
    radius: float, height: float, n: int = 16
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """n-gon prism: bottom ring + top ring + vertical edges."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    xs, ys = radius * np.cos(a), radius * np.sin(a)
    bottom = np.column_stack([xs, ys, np.full(n, -height / 2)])
    top = np.column_stack([xs, ys, np.full(n, height / 2)])
    corners = np.vstack([bottom, top]).astype(np.float32)
    edges = []
    for i in range(n):
        edges += [(i, (i + 1) % n), (n + i, n + (i + 1) % n), (i, n + i)]
    return corners, edges


def sphere_wireframe(
    radius: float, n: int = 12
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """Three great circles (XY, XZ, YZ) each with n segments."""
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    all_pts: list[np.ndarray] = []
    edges: list[tuple[int, int]] = []
    for a0, a1 in [(0, 1), (0, 2), (1, 2)]:  # XY, XZ, YZ planes
        base = len(all_pts)
        pts = np.zeros((n, 3), dtype=np.float32)
        pts[:, a0] = radius * np.cos(t)
        pts[:, a1] = radius * np.sin(t)
        all_pts.extend(pts.tolist())
        for i in range(n):
            edges.append((base + i, base + (i + 1) % n))
    return np.array(all_pts, dtype=np.float32), edges
