# urdf-tools

Standalone, stdlib-only URDF parser plus CLI tools for visualization,
kinematic-tree inspection, and shared-memory sensor pub/sub. No ROS,
no simulator framework required.

## Modules

| Module | Description |
|--------|-------------|
| `urdf_tools.parser` | stdlib-only URDF parser → typed dataclasses (`Robot`, `Link`, `Joint`, `Geometry`) |
| `urdf_tools.geometry` | Transform utilities and primitive wireframe generators (box/cylinder/sphere) |
| `urdf_tools.viz` | 2D floor plan, 3D wireframe, and kinematic-tree plotting (matplotlib) |
| `urdf_tools.pubsub` | `SensorPublisher`/`SensorSubscriber` — scan/IMU/odometry/encoder over [shmbridge](../shmbridge) shared memory |

## Install

```bash
pip install -e .                 # core (parser, viz, CLI)
pip install -e ".[mesh]"         # adds trimesh-based mesh geometry support
pip install -e ../shmbridge       # required for `urdf-tools publish`/`subscribe`
```

## CLI

```bash
urdf-tools info   robot.urdf              # link/joint/geometry summary
urdf-tools view   robot.urdf              # 2D top-down floor plan
urdf-tools view3d robot.urdf              # 3D wireframe viewer
urdf-tools tree   robot.urdf              # kinematic-tree diagram
urdf-tools publish   robot.urdf --shm /urdf_tools_sensors   # synthetic 2D scan → shmbridge
urdf-tools subscribe --shm /urdf_tools_sensors --live        # live viewer for the above
```

## Examples

The `examples/` directory has runnable scripts, roughly in order of
sophistication:

| Script | Shows |
|--------|-------|
| `01_view_2d.py` / `02_view_3d.py` / `03_kinematic_tree.py` | Direct use of `urdf_tools.viz` without the CLI |
| `04_pub_sensors.py` | Publishing a synthetic scan over shmbridge |
| `05_sub_viewer.py` | Subscribing and rendering (matplotlib 2D/3D, or a three.js web viewer via `--live3d`) |

## License

MIT — see [LICENSE](../LICENSE).
