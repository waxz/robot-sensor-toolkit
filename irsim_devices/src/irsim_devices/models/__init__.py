"""URDF -> triangle-mesh loader for standalone ray casting (Embree or open3d).

Parses a URDF file (box/cylinder/sphere primitives and/or ``<mesh>``
references) into a combined world-frame triangle mesh via ``trimesh``.
No shapely, no ir-sim dependency.

    from irsim_devices.models import load_urdf

    model = load_urdf("world.urdf", use_collision=False)
    # model.vertices  -> float32 [V, 3]
    # model.triangles -> int32   [T, 3]
"""

from irsim_devices.models.urdf_loader import URDFModel, describe_urdf, load_urdf

__all__ = ["URDFModel", "describe_urdf", "load_urdf"]
