# irsim-devices

Standalone sensor and actuator simulation models — IMU, 2D/3D LiDAR, wheel
encoders, and DC motors. Each module is self-contained: no simulator
framework, `ObjectBase`, or kinematics handler required.

## Modules

| Module | Description | Extra |
|--------|-------------|-------|
| `sensors.IMU` | 3-DOF IMU with IEEE 517 / Gaussian noise models | — |
| `sensors.Lidar2D` | 2D LiDAR with hardware profiles, AVX2 ray casting | `lidar2d` |
| `sensors.Lidar3D` | 3D spinning LiDAR via open3d Embree BVH | `lidar3d` |
| `sensors.Encoder` | Wheel encoder with named motor profiles | — |
| `actuators.Motor` | DC motor physics with built-in PID & encoder | — |
| `actuators.MotorDiffChassis` | Dual-motor differential drive chassis | — |

## Install

```bash
pip install irsim-devices           # core (IMU, Encoder, Motor)
pip install "irsim-devices[lidar2d]" # adds 2D LiDAR (shapely + matplotlib)
pip install "irsim-devices[lidar3d]" # adds 3D LiDAR (open3d)
pip install "irsim-devices[all]"     # everything
```

A plain install never downloads anything beyond PyPI wheels — the optional
`lidar_embree` C++ extension's ~35 MB Embree4 SDK is fetched only when you
explicitly opt in (see [C extensions](#c-extensions) below).

## Quick start

```python
from irsim_devices.actuators import Motor, MotorDiffChassis

chassis = MotorDiffChassis(wheel_radius=0.05, wheel_base=0.30,
                            motor_profile="small_dc")
for _ in range(1000):
    state = chassis.step([0.8, 0.8], dt=0.001)

from irsim_devices.sensors import IMU
imu = IMU(noise_model="ieee517")
imu.step(state_3dof, dt=0.01)
print(imu.angular_velocity, imu.linear_acceleration)

# Encoder reads straight off the chassis — no simulator, no attach step
from irsim_devices.sensors import Encoder
encoder = Encoder(profile="small_dc")
encoder.parent = chassis
encoder.step(chassis.state)
print(encoder.get_measurement())  # {"left": {...}, "right": {...}}
```

## World model interface

`Lidar2D` and `Lidar3D` accept any object satisfying the lightweight
protocols in `irsim_devices.core.world_model`:

```python
from irsim_devices.core.world_model import GeometryObject2D, Scene3DProtocol
```

These protocols make it straightforward to use these sensors with any
simulation framework that provides 2D Shapely geometry or an open3d
`RaycastingScene` — no specific simulator required.

## C extensions

Three optional native accelerators exist; each falls back cleanly when
unavailable:

| Extension | Accelerates | Falls back to |
|---|---|---|
| `cpp/lidar_embree.cpp` (pybind11 + Embree4) | `sensors.EmbreeLidar2D` / `EmbreeLidar3D` ray casting | `open3d`-backed `Lidar2D`/`Lidar3D`, or pure Shapely |
| `csrc/ray_casting_omp.c` (C + OpenMP + AVX2) | `core.ray_casting_2d_omp` — used by `Lidar2D`'s standalone-scene fast path | pure NumPy |

`lidar_embree` needs the Embree4 SDK (~35 MB), which is **not** downloaded by
a plain install — `pip install -e .` finishes in ~1 second and simply skips
that extension. Opt in explicitly to build it:

```bash
IRSIM_DEVICES_BUILD_EMBREE=1 pip install -e ".[embree]"
# or, with an Embree4 SDK already installed locally:
EMBREE_ROOT=/path/to/embree4 pip install -e ".[embree]"
```

`pip install -e .` (or `python setup.py build_ext`) prints a
**build summary** at the end listing exactly what compiled — the
`lidar_embree` status, which `ray_casting_2d_omp` kernel is active
(AVX2 float32 / AVX2 float64 / scalar OpenMP / NumPy fallback), and
whether `pybind11`/`open3d` are installed. Run with `pip install -v -e .`
if you don't see it.

## Testing

```bash
pip install -e ".[dev]"
pytest
```

## License

MIT — see [LICENSE](../LICENSE).
