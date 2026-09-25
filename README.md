# robot-sensor-toolkit

A standalone collection of three independent Python/C++ packages for
robotics simulation and control, originally developed as part of
[ir-sim](https://github.com/hanruihua/ir-sim) and extracted here so they can
be used, versioned, and released on their own — with no dependency on any
particular simulator.

| Package | What it is |
|---|---|
| [`shmbridge/`](shmbridge/) | POSIX shared-memory publish/subscribe for Python ↔ C++ robot control — sub-microsecond writes, zero copies in the critical path |
| [`irsim_devices/`](irsim_devices/) | Standalone sensor/actuator simulation models: IMU, 2D/3D LiDAR, wheel encoders, DC motors |
| [`urdf_tools/`](urdf_tools/) | stdlib-only URDF parser + CLI for visualization, kinematic-tree inspection, and shmbridge sensor pub/sub |

None of the three depends on the others' being installed, and none depends
on ir-sim. `urdf_tools` optionally uses `shmbridge` for its pub/sub examples
and `irsim_devices` for standalone sensor simulation; both are regular
dependencies you install like any other package, not a coupling to a
specific simulator.

## Install

Each package is installed independently, from its own subdirectory:

```bash
pip install -e ./shmbridge          # C++ extension, built via scikit-build-core
pip install -e ./irsim_devices       # optional extras: [lidar2d] [lidar3d] [embree] [all]
pip install -e ./urdf_tools          # CLI: `urdf-tools`
```

Or build all three at once:

```bash
python build_extensions.py
```

See each package's own README for details:
[shmbridge](shmbridge/README.md) ·
[irsim_devices](irsim_devices/README.md) ·
[urdf_tools](urdf_tools/README.md)

## Repository layout

```
robot-sensor-toolkit/
├── shmbridge/        # C++ header-only + Python ctypes shared-memory bridge
├── irsim_devices/     # Python sensor/actuator models (+ optional C++ Embree lidar)
├── urdf_tools/        # Python URDF parser, viz, CLI
├── build_extensions.py
└── LICENSE
```

## Testing

```bash
cd shmbridge && pytest
cd irsim_devices && pytest
```

## License

MIT — see [LICENSE](LICENSE). Portions of this code originate from
[ir-sim](https://github.com/hanruihua/ir-sim) (Copyright © 2022 Ruihua Han),
also MIT licensed.
