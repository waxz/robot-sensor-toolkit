"""Sensor modules: IMU, 2-D LiDAR, 3-D LiDAR, wheel encoder."""

from irsim_devices.sensors.encoder import Encoder
from irsim_devices.sensors.imu import IMU

__all__ = [
    "Encoder",
    "IMU",
]

try:
    from irsim_devices.sensors.lidar2d import Lidar2D

    __all__ += ["Lidar2D"]
except ImportError:
    pass

try:
    from irsim_devices.sensors.lidar3d import Lidar3D

    __all__ += ["Lidar3D"]
except ImportError:
    pass

try:
    from irsim_devices.sensors.lidar2d_embree import EmbreeLidar2D

    __all__ += ["EmbreeLidar2D"]
except ImportError:
    pass

try:
    from irsim_devices.sensors.lidar3d_embree import EmbreeLidar3D

    __all__ += ["EmbreeLidar3D"]
except ImportError:
    pass
