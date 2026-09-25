"""URDF model loader for Embree ray-cast benchmarks.

Loads a URDF file, resolves all link geometries (mesh / box / cylinder /
sphere), applies joint transforms to place everything in the world frame,
and returns a combined mesh plus 2D floor-plan segments suitable for the
Embree 2D and 3D ray-casters.

Usage
-----
    from urdf_loader import load_urdf, describe_urdf

    model = load_urdf("warehouse_world.urdf", slice_z=1.0)
    # model.vertices   -> float32 [V, 3]
    # model.triangles  -> int32   [T, 3]
    # model.segments_2d -> float32 [N, 4]  (ax, ay, bx, by)
    # model.triangle_soup() -> float32 [T, 3, 3]

    info = describe_urdf("robot_diff.urdf")
    # {'robot_name': ..., 'links': [...], 'joints': [...]}
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import trimesh.creation  # noqa: F401

# ── URDF parse helpers ────────────────────────────────────────────────────────


def _floats(s: str) -> list[float]:
    return [float(x) for x in s.strip().split()]


def _rpy_matrix(rpy: list[float]) -> np.ndarray:
    """Intrinsic XYZ (roll-pitch-yaw) → 3×3 rotation matrix."""
    cr, cp, cy = math.cos(rpy[0]), math.cos(rpy[1]), math.cos(rpy[2])
    sr, sp, sy = math.sin(rpy[0]), math.sin(rpy[1]), math.sin(rpy[2])
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _origin_transform(elem) -> np.ndarray:
    """Parse a URDF ``<origin>`` element → 4×4 homogeneous matrix."""
    T = np.eye(4)
    if elem is None:
        return T
    T[:3, 3] = _floats(elem.get("xyz", "0 0 0"))
    T[:3, :3] = _rpy_matrix(_floats(elem.get("rpy", "0 0 0")))
    return T


# ── Primitive / mesh factory ──────────────────────────────────────────────────


def _geom_to_trimesh(geom_elem, base_dir: str) -> trimesh.Trimesh | None:
    """Convert a URDF ``<geometry>`` element to a trimesh.Trimesh."""
    if geom_elem is None:
        return None
    child = None
    for _tag in ("box", "cylinder", "sphere", "mesh"):
        _found = geom_elem.find(_tag)
        if _found is not None:
            child = _found
            break
    if child is None:
        return None

    tag = child.tag
    try:
        if tag == "box":
            extents = _floats(child.get("size", "1 1 1"))
            return trimesh.creation.box(extents=extents)

        elif tag == "cylinder":
            r = float(child.get("radius", "0.1"))
            h = float(child.get("length", "0.1"))
            return trimesh.creation.cylinder(radius=r, height=h, sections=20)

        elif tag == "sphere":
            r = float(child.get("radius", "0.1"))
            return trimesh.creation.icosphere(subdivisions=2, radius=r)

        elif tag == "mesh":
            fn = child.get("filename", "")
            if not os.path.isabs(fn):
                fn = os.path.join(base_dir, fn)
            fn = os.path.normpath(fn)
            if not os.path.exists(fn):
                return None
            loaded = trimesh.load(fn, force="mesh", process=False)
            if isinstance(loaded, trimesh.Scene):
                meshes = [g for g in loaded.dump() if isinstance(g, trimesh.Trimesh)]
                loaded = trimesh.util.concatenate(meshes) if meshes else None
            if not isinstance(loaded, trimesh.Trimesh):
                return None
            scale_s = child.get("scale", "1 1 1")
            scale = _floats(scale_s)
            if scale != [1.0, 1.0, 1.0]:
                loaded.apply_scale(scale)
            return loaded
    except Exception:
        return None

    return None


# ── 2D segment extraction ─────────────────────────────────────────────────────


def _segments_from_section(mesh: trimesh.Trimesh, slice_z: float) -> np.ndarray:
    """Slice mesh at height *slice_z* and return [N, 4] float32 segments."""
    try:
        section = mesh.section(
            plane_origin=[0.0, 0.0, slice_z],
            plane_normal=[0.0, 0.0, 1.0],
        )
        if section is None:
            return np.empty((0, 4), dtype=np.float32)
        path2d, _ = section.to_planar()
        segs: list[list[float]] = []
        for entity in path2d.entities:
            pts = path2d.vertices[entity.points]
            n = len(pts)
            for i in range(n - 1):
                segs.append([pts[i, 0], pts[i, 1], pts[i + 1, 0], pts[i + 1, 1]])
            closed = getattr(entity, "closed", False)
            if closed and n > 1:
                segs.append([pts[-1, 0], pts[-1, 1], pts[0, 0], pts[0, 1]])
        return np.array(segs, dtype=np.float32) if segs else np.empty((0, 4), dtype=np.float32)
    except Exception:
        return np.empty((0, 4), dtype=np.float32)


def _segments_from_vertical_faces(mesh: trimesh.Trimesh, slice_z: float) -> np.ndarray:
    """Fallback: project vertical triangle edges that straddle slice_z to XY."""
    verts = mesh.vertices
    faces = mesh.faces
    segs: list[list[float]] = []

    for tri in faces:
        pts = verts[tri]  # (3, 3)
        zs = pts[:, 2]
        # Does this triangle straddle slice_z?
        if zs.min() > slice_z or zs.max() < slice_z:
            continue
        # Normal must be approximately horizontal (|nz| < 0.3)
        e1 = pts[1] - pts[0]
        e2 = pts[2] - pts[0]
        n = np.cross(e1, e2)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if abs(n[2]) > 0.3:
            continue
        # Project each edge that crosses slice_z
        for i in range(3):
            p0, p1 = pts[i], pts[(i + 1) % 3]
            z0, z1 = p0[2], p1[2]
            if min(z0, z1) <= slice_z <= max(z0, z1) and abs(z1 - z0) > 1e-6:
                t = (slice_z - z0) / (z1 - z0)
                px = p0[0] + t * (p1[0] - p0[0])
                py = p0[1] + t * (p1[1] - p0[1])
                segs.append([px, py, px, py])

    if not segs:
        return np.empty((0, 4), dtype=np.float32)

    raw = np.array(segs, dtype=np.float32)
    # Pair up intersection points (per-triangle they come in pairs → form segments)
    if len(raw) % 2 == 0:
        pairs = raw.reshape(-1, 2, 4)
        out = np.column_stack([pairs[:, 0, :2], pairs[:, 1, :2]])
        return out.astype(np.float32)
    return np.empty((0, 4), dtype=np.float32)


def _extract_2d_segments(mesh: trimesh.Trimesh, slice_z: float) -> np.ndarray:
    segs = _segments_from_section(mesh, slice_z)
    if len(segs) < 10:
        segs = _segments_from_vertical_faces(mesh, slice_z)
    return segs


# ── Public model class ────────────────────────────────────────────────────────


class URDFModel:
    """Combined mesh produced by :func:`load_urdf`, with the same interface
    as the ``MeshBuilder`` in ``warehouse_model.py``."""

    def __init__(
        self,
        vertices: np.ndarray,
        triangles: np.ndarray,
        segments_2d: np.ndarray,
        robot_name: str = "",
    ) -> None:
        self._vertices = vertices
        self._triangles = triangles
        self._segments_2d = segments_2d
        self.robot_name = robot_name

    @property
    def vertices(self) -> np.ndarray:
        return self._vertices

    @property
    def triangles(self) -> np.ndarray:
        return self._triangles

    @property
    def segments_2d(self) -> np.ndarray:
        return self._segments_2d

    def triangle_soup(self) -> np.ndarray:
        """Return [T, 3, 3] float32 triangle soup (no index array)."""
        return self._vertices[self._triangles]

    def __repr__(self) -> str:
        return (
            f"URDFModel({self.robot_name!r}: "
            f"{len(self._vertices)} verts, {len(self._triangles)} tris, "
            f"{len(self._segments_2d)} 2D segs)"
        )


# ── Loader ────────────────────────────────────────────────────────────────────


def load_urdf(
    urdf_path: str,
    *,
    use_collision: bool = True,
    slice_z: float = 1.0,
) -> URDFModel:
    """Load a URDF and return a :class:`URDFModel`.

    Parameters
    ----------
    urdf_path:
        Path to the ``.urdf`` file.
    use_collision:
        Use ``<collision>`` geometry when True (default), ``<visual>`` otherwise.
    slice_z:
        Height in metres at which to slice the mesh for 2D segment extraction.
    """
    urdf_path = os.path.abspath(urdf_path)
    base_dir = os.path.dirname(urdf_path)
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_name = root.get("name", "")

    # ── Collect per-link meshes in their local link frame ─────────────────
    link_meshes: dict[str, list[trimesh.Trimesh]] = {}
    geom_tag = "collision" if use_collision else "visual"

    for link_el in root.findall("link"):
        name = link_el.get("name", "")
        meshes: list[trimesh.Trimesh] = []
        for block in link_el.findall(geom_tag):
            m = _geom_to_trimesh(block.find("geometry"), base_dir)
            if m is None or len(m.faces) == 0:
                continue
            T_local = _origin_transform(block.find("origin"))
            mc = m.copy()
            mc.apply_transform(T_local)
            meshes.append(mc)
        link_meshes[name] = meshes

    # ── Build kinematic tree ───────────────────────────────────────────────
    parent_of: dict[str, str] = {}
    joint_T: dict[str, np.ndarray] = {}
    for joint_el in root.findall("joint"):
        parent = joint_el.find("parent").get("link")
        child = joint_el.find("child").get("link")
        parent_of[child] = parent
        joint_T[child] = _origin_transform(joint_el.find("origin"))

    def world_T(link_name: str) -> np.ndarray:
        T, cur, seen = np.eye(4), link_name, set()
        while cur in parent_of and cur not in seen:
            seen.add(cur)
            T = joint_T[cur] @ T
            cur = parent_of[cur]
        return T

    # ── Compose all meshes in world frame ──────────────────────────────────
    world_meshes: list[trimesh.Trimesh] = []
    for link_name, meshes in link_meshes.items():
        T = world_T(link_name)
        for m in meshes:
            mc = m.copy()
            mc.apply_transform(T)
            world_meshes.append(mc)

    if not world_meshes:
        empty_v = np.zeros((0, 3), dtype=np.float32)
        empty_t = np.zeros((0, 3), dtype=np.int32)
        empty_s = np.zeros((0, 4), dtype=np.float32)
        return URDFModel(empty_v, empty_t, empty_s, robot_name)

    combined = trimesh.util.concatenate(world_meshes)
    vertices = np.array(combined.vertices, dtype=np.float32)
    triangles = np.array(combined.faces, dtype=np.int32)
    segments_2d = _extract_2d_segments(combined, slice_z)

    return URDFModel(vertices, triangles, segments_2d, robot_name)


# ── Describe (no mesh loading) ────────────────────────────────────────────────


def describe_urdf(urdf_path: str) -> dict:
    """Return a lightweight summary of the URDF structure."""
    tree = ET.parse(os.path.abspath(urdf_path))
    root = tree.getroot()
    links = [el.get("name") for el in root.findall("link")]
    joints = [
        {
            "name": jel.get("name"),
            "type": jel.get("type"),
            "parent": jel.find("parent").get("link"),
            "child": jel.find("child").get("link"),
        }
        for jel in root.findall("joint")
    ]
    # Count geometry types
    geom_counts: dict[str, int] = {}
    for el in root.iter("geometry"):
        child = list(el)
        if child:
            t = child[0].tag
            geom_counts[t] = geom_counts.get(t, 0) + 1
    return {
        "robot_name": root.get("name", ""),
        "links": links,
        "joints": joints,
        "geometry_counts": geom_counts,
    }
