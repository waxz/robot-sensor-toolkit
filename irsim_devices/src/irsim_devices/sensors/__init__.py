"""Sensor modules: IMU, 2-D LiDAR, 3-D LiDAR, wheel encoder.

``Encoder`` and ``IMU`` are dependency-free and imported eagerly. The LiDAR
classes are loaded lazily (PEP 562 ``__getattr__``) since ``Lidar2D`` pulls
in ``shapely`` and ``Lidar3D`` pulls in ``open3d`` — code that only needs
``IMU``/``Encoder`` (or the Embree-only raycasting primitives) should never
pay for those imports just by doing ``from irsim_devices.sensors import ...``.
"""

from irsim_devices.sensors.encoder import Encoder
from irsim_devices.sensors.imu import IMU

__all__ = [
    "EmbreeLidar2D",
    "EmbreeLidar3D",
    "Encoder",
    "IMU",
    "Lidar2D",
    "Lidar3D",
]

_LAZY = {
    "Lidar2D": ("irsim_devices.sensors.lidar2d", "Lidar2D"),
    "Lidar3D": ("irsim_devices.sensors.lidar3d", "Lidar3D"),
    "EmbreeLidar2D": ("irsim_devices.sensors.lidar2d_embree", "EmbreeLidar2D"),
    "EmbreeLidar3D": ("irsim_devices.sensors.lidar3d_embree", "EmbreeLidar3D"),
}


def __getattr__(name: str):
    target = _LAZY.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = target
    import importlib

    module = importlib.import_module(module_name)
    value = getattr(module, attr_name)
    globals()[name] = value  # cache: subsequent access skips __getattr__
    return value
