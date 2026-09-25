"""Convert URDF geometry into irsim obstacle objects or irsim_devices scenes.

Two entry points are provided:

``urdf_to_irsim_obstacles`` — wraps URDF primitives as ``ObjectStatic``
obstacles for irsim's built-in lidar raycaster::

    import irsim
    from urdf_tools.parser import parse_urdf
    from urdf_tools.irsim_compat import urdf_to_irsim_obstacles

    env = irsim.make("world.yaml")
    world_robot = parse_urdf("models/warehouse.urdf")
    env.add_objects(urdf_to_irsim_obstacles(world_robot))

``urdf_to_scene_2d`` — builds an ``Open3DScene2D`` for
``irsim_devices.sensors.Lidar2D`` (standalone, no irsim required)::

    from irsim_devices.sensors import Lidar2D
    from urdf_tools.parser import parse_urdf
    from urdf_tools.irsim_compat import urdf_to_scene_2d

    world_robot = parse_urdf("models/warehouse.urdf")
    scene = urdf_to_scene_2d(world_robot, lidar_height=0.30)
    lidar = Lidar2D(state=[0, 0, 0], range_max=20.0, number=360,
                    angle_range=6.2832)
    lidar.set_scene(scene)
    lidar.step([x, y, theta])
    scan = lidar.get_scan()
"""

from __future__ import annotations

import numpy as np

# Lidar scan-plane height used by the 3-D viewer and the default bridge.
LIDAR_HEIGHT_DEFAULT: float = 0.30  # metres


def urdf_to_irsim_obstacles(
    robot,
    name_prefix: str = "urdf",
    lidar_height: float = LIDAR_HEIGHT_DEFAULT,
) -> list:
    """Convert URDF primitives to irsim static obstacles for 2-D lidar raycasting.

    Each URDF geometry element whose world-frame Z extent contains *lidar_height*
    is projected onto the XY plane and wrapped in an ``ObjectStatic``.

    Geometry that is purely horizontal (floors, ceilings, beams) is excluded
    automatically: if a box's full Z range is thinner than 0.15 m, or lies
    entirely above *lidar_height*, it produces no obstacle.

    Parameters
    ----------
    robot:
        Parsed URDF ``Robot`` (from ``urdf_tools.parser.parse_urdf``).
    name_prefix:
        Prefix for auto-generated obstacle names.
    lidar_height:
        Height of the horizontal scan plane in metres.

    Returns
    -------
    list[ObjectStatic]
        Pass directly to ``env.add_objects()``.
    """
    from irsim.world.obstacles.obstacle_static import ObjectStatic
    from shapely.geometry import MultiPoint

    from urdf_tools.geometry import box_wireframe
    from urdf_tools.viz import _link_world_blocks

    obstacles: list = []

    for idx, (T, geom, _rgba) in enumerate(_link_world_blocks(robot)):
        name = f"{name_prefix}_{idx:04d}"

        if geom.type == "box":
            # Transform all 8 box corners to world frame.
            corners_local, _ = box_wireframe(geom.size)
            h = np.column_stack([corners_local, np.ones(len(corners_local))])
            corners_w = (T @ h.T).T[:, :3]

            z_vals = corners_w[:, 2]
            z_lo, z_hi = float(z_vals.min()), float(z_vals.max())

            # Skip slabs whose Z extent doesn't reach the scan plane.
            if z_lo > lidar_height or z_hi < lidar_height:
                continue
            # Skip near-horizontal geometry (floors, thin roof panels, beams).
            if (z_hi - z_lo) < 0.15:
                continue

            # 2-D footprint: convex hull of all 8 corner XY projections.
            hull = MultiPoint(corners_w[:, :2]).convex_hull
            if hull.is_empty or hull.geom_type in ("Point", "LineString"):
                continue
            verts = list(hull.exterior.coords[:-1])  # drop repeated closing vertex
            if len(verts) < 3:
                continue

            obstacles.append(
                ObjectStatic(
                    name=name,
                    shape={"name": "polygon", "vertices": verts},
                    state=[0.0, 0.0, 0.0],
                    static=True,
                    role="obstacle",
                )
            )

        elif geom.type == "cylinder":
            # Cylinder axis is local Z; after transform, check world Z extent.
            z_axis_w = T[:3, 2]
            half_len = geom.length / 2.0
            z_center = float(T[2, 3])
            z_span = abs(float(z_axis_w[2])) * half_len
            z_lo = z_center - max(z_span, geom.radius)
            z_hi = z_center + max(z_span, geom.radius)

            if z_lo > lidar_height or z_hi < lidar_height:
                continue

            obstacles.append(
                ObjectStatic(
                    name=name,
                    shape={"name": "circle", "radius": geom.radius},
                    state=[float(T[0, 3]), float(T[1, 3]), 0.0],
                    static=True,
                    role="obstacle",
                )
            )

        elif geom.type == "sphere":
            z_center = float(T[2, 3])
            r = geom.radius
            if z_center - r > lidar_height or z_center + r < lidar_height:
                continue

            obstacles.append(
                ObjectStatic(
                    name=name,
                    shape={"name": "circle", "radius": r},
                    state=[float(T[0, 3]), float(T[1, 3]), 0.0],
                    static=True,
                    role="obstacle",
                )
            )

    return obstacles


def urdf_to_scene_2d(
    robot,
    lidar_height: float = LIDAR_HEIGHT_DEFAULT,
) -> object:
    """Build an ``Open3DScene2D`` from URDF geometry for ``irsim_devices.Lidar2D``.

    Applies the same Z-height filter as :func:`urdf_to_irsim_obstacles` but
    returns the footprints as Shapely polygons packed into an
    ``Open3DScene2D`` instead of wrapping them as irsim ``ObjectStatic``
    obstacles.  Assign the returned scene to ``lidar.scene`` (or call
    ``lidar.set_scene(scene)``) before stepping the sensor.

    Parameters
    ----------
    robot:
        Parsed URDF ``Robot`` (from ``urdf_tools.parser.parse_urdf``).
    lidar_height:
        Height of the horizontal scan plane in metres.

    Returns
    -------
    Open3DScene2D
        Ready to attach to an ``irsim_devices.sensors.Lidar2D``.
    """
    from irsim_devices.core.open3d_scene_2d import Open3DScene2D
    from shapely.geometry import MultiPoint, Point

    from urdf_tools.geometry import box_wireframe
    from urdf_tools.viz import _link_world_blocks

    geometries = []

    for T, geom, _rgba in _link_world_blocks(robot):
        if geom.type == "box":
            corners_local, _ = box_wireframe(geom.size)
            h = np.column_stack([corners_local, np.ones(len(corners_local))])
            corners_w = (T @ h.T).T[:, :3]

            z_vals = corners_w[:, 2]
            z_lo, z_hi = float(z_vals.min()), float(z_vals.max())
            if z_lo > lidar_height or z_hi < lidar_height:
                continue
            if (z_hi - z_lo) < 0.15:
                continue

            hull = MultiPoint(corners_w[:, :2]).convex_hull
            if hull.is_empty or hull.geom_type in ("Point", "LineString"):
                continue
            geometries.append(hull)

        elif geom.type == "cylinder":
            z_axis_w = T[:3, 2]
            half_len = geom.length / 2.0
            z_center = float(T[2, 3])
            z_span = abs(float(z_axis_w[2])) * half_len
            z_lo = z_center - max(z_span, geom.radius)
            z_hi = z_center + max(z_span, geom.radius)
            if z_lo > lidar_height or z_hi < lidar_height:
                continue
            geometries.append(Point(float(T[0, 3]), float(T[1, 3])).buffer(geom.radius))

        elif geom.type == "sphere":
            z_center = float(T[2, 3])
            r = geom.radius
            if z_center - r > lidar_height or z_center + r < lidar_height:
                continue
            geometries.append(Point(float(T[0, 3]), float(T[1, 3])).buffer(r))

    return Open3DScene2D.from_shapely_geometries(geometries, slice_z=lidar_height)
