"""Core utilities: world-model protocols, geometry helpers, RNG, ray-casting."""

from irsim_devices.core.geo_utils import (
    ClipTo2Pi,
    geometry_transform,
    transform_point_with_state,
)
from irsim_devices.core.random_utils import rng, set_seed
from irsim_devices.core.world_model import GeometryObject2D, Scene3DProtocol

__all__ = [
    "ClipTo2Pi",
    "GeometryObject2D",
    "Open3DScene2D",
    "Scene3DProtocol",
    "geometry_transform",
    "rng",
    "set_seed",
    "transform_point_with_state",
]

try:
    from irsim_devices.core.open3d_scene_2d import Open3DScene2D
except ImportError:
    pass
