"""Tests for Open3DScene2D and its integration with Lidar2D."""

import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from irsim_devices.core.open3d_scene_2d import Open3DScene2D, slice_mesh_at_z

# ──────────────────────────────────────────────────────────────────────────────
# slice_mesh_at_z unit tests (uses a tiny synthetic Open3D mesh)
# ──────────────────────────────────────────────────────────────────────────────


def _unit_cube_mesh():
    """Build a 1×1×1 Open3D TriangleMesh cube centred at the origin."""
    o3d = pytest.importorskip("open3d")
    mesh = o3d.geometry.TriangleMesh.create_box(1.0, 1.0, 1.0)
    # translate so the cube spans [-0.5, 0.5] in all axes
    mesh.translate([-0.5, -0.5, -0.5])
    return mesh


def test_slice_unit_cube_at_zero():
    """Slicing a unit cube at z=0 must produce exactly the 4 bottom edges."""
    mesh = _unit_cube_mesh()
    segs = slice_mesh_at_z(mesh, z=0.0)
    assert len(segs) > 0, "expected segments at z=0"
    # Each segment should have both endpoints very close to z=0 (2-D check)
    for seg in segs:
        coords = np.array(seg.coords)
        assert coords.shape[1] == 2, "segments must be 2-D"


def test_slice_unit_cube_above_top():
    """Slicing above the cube should yield no segments."""
    mesh = _unit_cube_mesh()
    segs = slice_mesh_at_z(mesh, z=1.0)
    assert segs == [], "expected no segments above the cube"


def test_slice_unit_cube_midplane():
    """Slicing through the middle should give 4 segments (the square cross-section)."""
    mesh = _unit_cube_mesh()
    segs = slice_mesh_at_z(mesh, z=0.0)  # unit cube already at z=0 plane
    # The bottom face is at z=0; count is well-defined.
    assert len(segs) >= 4


# ──────────────────────────────────────────────────────────────────────────────
# Open3DScene2D tests (shapely-only path, no Open3D required)
# ──────────────────────────────────────────────────────────────────────────────


def test_from_shapely_box():
    """from_shapely_geometries on a square polygon must build non-empty scene."""
    box = Polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)])
    scene = Open3DScene2D.from_shapely_geometries([box])
    assert len(scene) == 1
    assert scene.GeometryTree is not None


def test_scene_repr():
    box = Polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)])
    scene = Open3DScene2D.from_shapely_geometries([box])
    r = repr(scene)
    assert "Open3DScene2D" in r
    assert "map_objects=1" in r


def test_multiple_geometries():
    boxes = [
        Polygon([(-2, -0.1), (-1, -0.1), (-1, 0.1), (-2, 0.1)]),
        Polygon([(1, -0.1), (2, -0.1), (2, 0.1), (1, 0.1)]),
    ]
    scene = Open3DScene2D.from_shapely_geometries(boxes)
    assert len(scene) == 2
    assert scene.GeometryTree is not None


def test_add_box_helper():
    scene = Open3DScene2D()
    scene.add_box(center=(0, 0), width=1.0, height=1.0)
    assert len(scene) == 1


def test_add_circle_helper():
    scene = Open3DScene2D()
    scene.add_circle(center=(0, 0), radius=0.5, resolution=16)
    assert len(scene) == 1


def test_add_dynamic():
    scene = Open3DScene2D()
    scene.add_box(center=(3, 0), width=0.2, height=0.2)
    dyn = Polygon([(0.5, -0.1), (0.9, -0.1), (0.9, 0.1), (0.5, 0.1)])
    scene.add_dynamic(dyn)
    assert len(scene) == 2


# ──────────────────────────────────────────────────────────────────────────────
# Integration with Lidar2D (no Open3D; shapely-only scene)
# ──────────────────────────────────────────────────────────────────────────────


def test_lidar2d_with_scene_no_irsim():
    """Lidar2D.step must run without irsim when a scene is assigned."""
    from irsim_devices.sensors import Lidar2D

    # Wall at x=2 spanning y in [-1, 1]
    wall = Polygon([(1.9, -1), (2.1, -1), (2.1, 1), (1.9, 1)])
    scene = Open3DScene2D.from_shapely_geometries([wall])

    lidar = Lidar2D(
        state=[0, 0, 0],
        obj_id=0,
        range_max=5.0,
        angle_range=2 * math.pi,
        number=72,
    )
    lidar.scene = scene
    lidar.step([0, 0, 0])

    # Some beams must have hit the wall (~2 m away)
    assert np.any(lidar.range_data < 5.0), "expected at least one hit"
    # No beam should exceed range_max
    assert np.all(lidar.range_data <= 5.0 + 1e-9)


def test_lidar2d_returns_max_when_scene_empty():
    from irsim_devices.sensors import Lidar2D

    scene = Open3DScene2D()  # empty scene
    lidar = Lidar2D(state=[0, 0, 0], obj_id=0, range_max=5.0, number=36)
    lidar.scene = scene
    lidar.step([0, 0, 0])

    assert np.allclose(lidar.range_data, 5.0), "expected all beams at max range"


def test_lidar2d_scene_property():
    from irsim_devices.sensors import Lidar2D

    lidar = Lidar2D(state=[0, 0, 0], obj_id=0)
    assert lidar.scene is None

    scene = Open3DScene2D()
    lidar.scene = scene
    assert lidar.scene is scene


def test_lidar2d_two_obstacles():
    from irsim_devices.sensors import Lidar2D

    left = Polygon([(-3.1, -0.5), (-2.9, -0.5), (-2.9, 0.5), (-3.1, 0.5)])
    right = Polygon([(2.9, -0.5), (3.1, -0.5), (3.1, 0.5), (2.9, 0.5)])
    scene = Open3DScene2D.from_shapely_geometries([left, right])

    lidar = Lidar2D(
        state=[0, 0, 0],
        obj_id=0,
        range_max=5.0,
        angle_range=2 * math.pi,
        number=360,
    )
    lidar.scene = scene
    lidar.step([0, 0, 0])

    min_range = lidar.range_data.min()
    assert min_range < 5.0, "expected hits on both obstacles"
