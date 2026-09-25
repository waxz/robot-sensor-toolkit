"""Open3D-backed 2D scene for Lidar2D.

Loads 3D mesh files (or pre-built ``open3d.geometry.TriangleMesh`` objects)
and produces a scene that :class:`~irsim_devices.sensors.lidar2d.Lidar2D`
can query without any ir-sim dependency.

The scene is built by slicing each mesh at a horizontal plane ``z = slice_z``
using triangle–plane intersection.  The resulting 2-D line segments are
packaged as *map* objects (``shape = "map"``) so that Lidar2D's two-level
spatial cache (scene-level STRtree → per-object segment STRtree) is used
automatically.

Usage::

    from irsim_devices.core.open3d_scene_2d import Open3DScene2D
    from irsim_devices.sensors import Lidar2D

    scene = Open3DScene2D.from_files(
        ["wall.obj", "pillar.ply"],
        slice_z=0.5,   # height of the scan plane in metres
    )
    lidar = Lidar2D(state=[0, 0, 0], obj_id=0)
    lidar.scene = scene

    lidar.step([0, 0, 0])   # scan from origin facing +x

Dynamic obstacles can be added as simple shapely geometries via
:meth:`add_dynamic`; they are treated as non-cached objects whose segments
are rebuilt every step.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import shapely
from shapely import STRtree
from shapely.geometry import LineString, MultiLineString, Point, Polygon

# ──────────────────────────────────────────────────────────────────────────────
# Triangle-plane slice
# ──────────────────────────────────────────────────────────────────────────────

_SLICE_EPS = 1e-9


def slice_mesh_at_z(
    mesh: Any,  # open3d.geometry.TriangleMesh
    z: float,
    eps: float = _SLICE_EPS,
) -> list[LineString]:
    """Slice a triangle mesh with a horizontal plane at height *z*.

    Computes the intersection of every triangle with the plane ``z = z``
    using exact edge–plane intersection arithmetic.  Triangles that lie fully
    on the plane are ignored (degenerate cross-section).

    Args:
        mesh: An ``open3d.geometry.TriangleMesh`` with vertices and triangles.
        z: Height of the cutting plane in metres.
        eps: Tolerance for treating a vertex as exactly on the plane.

    Returns:
        List of 2-D :class:`~shapely.geometry.LineString` segments.
    """
    verts = np.asarray(mesh.vertices)  # (V, 3)
    tris = np.asarray(mesh.triangles)  # (T, 3)

    segments: list[LineString] = []
    for tri in tris:
        v = verts[tri]  # (3, 3)
        dz = v[:, 2] - z  # signed distance of each vertex from plane

        on_plane = np.abs(dz) < eps
        pts: list[np.ndarray] = []

        # Vertices exactly on the plane
        for i in range(3):
            if on_plane[i]:
                pts.append(v[i, :2])

        # Edges that strictly cross the plane (both vertices off-plane on
        # opposite sides)
        for i in range(3):
            a, b = i, (i + 1) % 3
            if on_plane[a] or on_plane[b]:
                continue
            if dz[a] * dz[b] < 0:
                t = dz[a] / (dz[a] - dz[b])
                pts.append(v[a, :2] + t * (v[b, :2] - v[a, :2]))

        # Deduplicate within eps
        unique: list[np.ndarray] = []
        for p in pts:
            if not any(np.allclose(p, q, atol=eps) for q in unique):
                unique.append(p)

        if len(unique) == 2:
            segments.append(LineString(unique))

    return segments


# ──────────────────────────────────────────────────────────────────────────────
# Scene object wrappers
# ──────────────────────────────────────────────────────────────────────────────


@dataclass
class _MapObject:
    """Static scene object built from sliced mesh segments.

    The ``shape = "map"`` sentinel tells Lidar2D to use the disk-based
    two-level cache path for this object.
    """

    _id: int
    linestrings: list[LineString]
    geometry: Any  # union geometry used for scene-level broad-phase
    geometry_tree: STRtree  # fine-grained index over *linestrings*
    shape: str = "map"
    _geometry_valid: bool = True
    unobstructed: bool = False


@dataclass
class _DynamicObject:
    """Dynamic obstacle as a Shapely polygon or geometry.

    Uses the dynamic path in Lidar2D (no caching; rebuilt every step).
    """

    _id: int
    _geometry: Any  # shapely geometry
    geometry: Any  # same reference; used for scene-level broad-phase
    shape: str = "dynamic"
    _geometry_valid: bool = True
    unobstructed: bool = False


# ──────────────────────────────────────────────────────────────────────────────
# Open3DScene2D
# ──────────────────────────────────────────────────────────────────────────────


class Open3DScene2D:
    """2-D scene derived from Open3D meshes, compatible with Lidar2D.

    Build the scene once (static mesh geometry does not change at runtime),
    then assign it to :attr:`Lidar2D.scene` before calling
    :meth:`~irsim_devices.sensors.lidar2d.Lidar2D.step`.

    Args:
        slice_z: Height of the horizontal slice plane in metres.  All meshes
            added to this scene are sliced at this height.

    Attributes:
        objects (list): Scene objects consumed by
            :meth:`~irsim_devices.sensors.lidar2d.Lidar2D._get_detected_objects`.
        GeometryTree (STRtree | None): Scene-level spatial index over object
            bounding geometries.  ``None`` when the scene is empty.
    """

    def __init__(self, slice_z: float = 0.0) -> None:
        self.slice_z: float = slice_z
        self.objects: list[_MapObject | _DynamicObject] = []
        self._geometries: list[Any] = []  # parallel to objects, for STRtree
        self.GeometryTree: STRtree | None = None
        # Use negative IDs so scene objects never match a robot's obj_id
        # (robot IDs in irsim are always non-negative integers).
        self._next_id: int = -1

    # ── Builders ──────────────────────────────────────────────────────────────

    @classmethod
    def from_files(
        cls,
        paths: list[str],
        slice_z: float = 0.0,
        *,
        transforms: list[np.ndarray | None] | None = None,
    ) -> Open3DScene2D:
        """Create a scene by loading mesh files and slicing them at *slice_z*.

        Args:
            paths: File paths to 3D meshes accepted by Open3D
                (``".obj"``, ``".ply"``, ``".stl"``, …).
            slice_z: Cutting plane height.
            transforms: Optional per-mesh 4×4 homogeneous transform matrices
                applied before slicing.  ``None`` entries mean identity.

        Returns:
            A ready-to-use :class:`Open3DScene2D`.
        """
        try:
            import open3d as o3d
        except ImportError as exc:
            raise ImportError(
                "open3d is required for Open3DScene2D.from_files().\n"
                "Install it with:  pip install open3d"
            ) from exc

        scene = cls(slice_z=slice_z)
        for i, path in enumerate(paths):
            mesh = o3d.io.read_triangle_mesh(path)
            tf = (transforms or [None] * len(paths))[i]
            scene.add_mesh(mesh, transform=tf)
        return scene

    @classmethod
    def from_meshes(
        cls,
        meshes: list[Any],  # list[open3d.geometry.TriangleMesh]
        slice_z: float = 0.0,
        *,
        transforms: list[np.ndarray | None] | None = None,
    ) -> Open3DScene2D:
        """Create a scene from pre-built ``open3d.geometry.TriangleMesh`` objects.

        Args:
            meshes: Pre-loaded Open3D triangle meshes.
            slice_z: Cutting plane height.
            transforms: Optional per-mesh 4×4 homogeneous transforms.

        Returns:
            A ready-to-use :class:`Open3DScene2D`.
        """
        scene = cls(slice_z=slice_z)
        for i, mesh in enumerate(meshes):
            tf = (transforms or [None] * len(meshes))[i]
            scene.add_mesh(mesh, transform=tf)
        return scene

    # ── Primitive helpers ──────────────────────────────────────────────────────

    @classmethod
    def from_shapely_geometries(
        cls,
        geometries: list[Any],
        slice_z: float = 0.0,
    ) -> Open3DScene2D:
        """Create a scene directly from Shapely geometries (no Open3D needed).

        Each geometry is treated as a static map object.  Use this when the
        2-D floor plan is already available as Shapely polygons or linestrings.

        Args:
            geometries: Shapely geometries (``Polygon``, ``LineString``,
                ``MultiLineString``, …).  Polygons are converted to their
                exterior linestrings.
            slice_z: Stored for reference; has no effect here.

        Returns:
            A ready-to-use :class:`Open3DScene2D`.
        """
        scene = cls(slice_z=slice_z)
        for geom in geometries:
            scene._add_shapely(geom)
        return scene

    # ── Mutation helpers ───────────────────────────────────────────────────────

    def add_mesh(
        self,
        mesh: Any,  # open3d.geometry.TriangleMesh
        *,
        transform: np.ndarray | None = None,
    ) -> None:
        """Slice *mesh* at :attr:`slice_z` and add it as a static map object.

        Args:
            mesh: Open3D ``TriangleMesh``.
            transform: Optional 4×4 homogeneous matrix to apply to the mesh
                before slicing (e.g. to place it in the world frame).
        """
        if transform is not None:
            mesh = mesh.transform(transform)
        segments = slice_mesh_at_z(mesh, self.slice_z)
        if not segments:
            return
        self._add_linestrings(segments)

    def add_box(
        self,
        center: tuple[float, float],
        width: float,
        height: float,
    ) -> None:
        """Add an axis-aligned rectangular obstacle (2-D convenience).

        Args:
            center: (x, y) of the box centre in metres.
            width: Half-width along x.
            height: Half-height along y.
        """
        cx, cy = center
        poly = Polygon(
            [
                (cx - width, cy - height),
                (cx + width, cy - height),
                (cx + width, cy + height),
                (cx - width, cy + height),
            ]
        )
        self._add_shapely(poly)

    def add_circle(
        self,
        center: tuple[float, float],
        radius: float,
        resolution: int = 32,
    ) -> None:
        """Add a circular obstacle (2-D convenience).

        Args:
            center: (x, y) in metres.
            radius: Radius in metres.
            resolution: Number of polygon vertices approximating the circle.
        """
        poly = Point(center).buffer(radius, resolution=resolution)
        self._add_shapely(poly)

    def add_dynamic(
        self,
        geometry: Any,  # shapely geometry
    ) -> None:
        """Add a dynamic obstacle as a Shapely geometry.

        Dynamic objects bypass the map cache and are rebuilt every step.
        Call this method again (with new geometry) to update a moving obstacle;
        then call :meth:`rebuild_tree` before the next :meth:`Lidar2D.step`.

        Args:
            geometry: Shapely polygon or geometry in the world frame.
        """
        obj_id = self._next_id
        self._next_id -= 1
        obj = _DynamicObject(_id=obj_id, _geometry=geometry, geometry=geometry)
        self.objects.append(obj)
        self._geometries.append(geometry)
        self._rebuild_tree()

    def rebuild_tree(self) -> None:
        """Rebuild the scene-level :attr:`GeometryTree` after mutations.

        Call this after updating dynamic obstacles in-place (mutating their
        ``_geometry`` attribute directly) so the scene-level index reflects
        the current positions.
        """
        self._geometries = [obj.geometry for obj in self.objects]
        self._rebuild_tree()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _add_linestrings(self, segments: list[LineString]) -> None:
        """Package *segments* as a single map object and add to the scene."""
        obj_id = self._next_id
        self._next_id -= 1
        seg_tree = STRtree(segments)
        union_geom = MultiLineString(segments) if len(segments) > 1 else segments[0]
        obj = _MapObject(
            _id=obj_id,
            linestrings=segments,
            geometry=union_geom,
            geometry_tree=seg_tree,
        )
        self.objects.append(obj)
        self._geometries.append(union_geom)
        self._rebuild_tree()

    def _add_shapely(self, geom: Any) -> None:
        """Convert a Shapely geometry to linestrings and add as a map object."""
        if isinstance(geom, Polygon):
            exterior = geom.exterior
            rings = [exterior, *list(geom.interiors)]
            coords_list = [np.array(r.coords) for r in rings]
            segs: list[LineString] = []
            for coords in coords_list:
                for i in range(len(coords) - 1):
                    segs.append(LineString([coords[i], coords[i + 1]]))
        elif isinstance(geom, LineString):
            coords = np.array(geom.coords)
            segs = [
                LineString([coords[i], coords[i + 1]]) for i in range(len(coords) - 1)
            ]
        else:
            # MultiLineString, GeometryCollection, etc. — extract linestrings
            segs = []
            for sub in shapely.get_parts(geom):
                self._add_shapely(sub)
            return
        if segs:
            self._add_linestrings(segs)

    def _rebuild_tree(self) -> None:
        """Rebuild the scene-level STRtree from current _geometries."""
        if self._geometries:
            self.GeometryTree = STRtree(self._geometries)
        else:
            self.GeometryTree = None

    # ── Utility ───────────────────────────────────────────────────────────────

    def __len__(self) -> int:
        return len(self.objects)

    def __repr__(self) -> str:
        n_map = sum(1 for o in self.objects if isinstance(o, _MapObject))
        n_dyn = len(self.objects) - n_map
        total_segs = sum(
            len(o.linestrings) for o in self.objects if isinstance(o, _MapObject)
        )
        return (
            f"Open3DScene2D(slice_z={self.slice_z}, "
            f"map_objects={n_map}, dynamic_objects={n_dyn}, "
            f"total_segments={total_segs})"
        )
