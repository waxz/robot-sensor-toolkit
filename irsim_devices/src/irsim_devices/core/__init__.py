"""Core utilities: world-model protocols, geometry helpers, RNG, ray-casting.

``Open3DScene2D`` is loaded lazily (PEP 562 ``__getattr__``): it pulls in
``shapely``, which nothing else in this module needs. Importing e.g.
``irsim_devices.core.rng`` must not pay for that.
"""

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


def __getattr__(name: str):
    if name == "Open3DScene2D":
        import importlib

        module = importlib.import_module("irsim_devices.core.open3d_scene_2d")
        value = module.Open3DScene2D
        globals()[name] = value  # cache: subsequent access skips __getattr__
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
