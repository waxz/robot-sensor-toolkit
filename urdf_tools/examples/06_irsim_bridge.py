"""Example 06 — IR-SIM → shmbridge publisher bridge with optional live visualisation.

Runs an IR-SIM environment with a LiDAR-equipped robot and publishes real
sensor data (scan, odometry, IMU) via shmbridge shared memory so that
05_sub_viewer.py can visualise it in real-time.

Optionally renders 2-D scan + URDF world and/or 3-D point cloud + URDF mesh
inline using matplotlib (``--viz``), so no second terminal is needed.

Requires:
    pip install ir-sim        (external simulator, not part of this repo)
    pip install shmbridge     (or: cd ../../shmbridge && pip install -e .)
    pip install irsim-devices (or: cd ../../irsim_devices && pip install -e .)

Usage:
    # Warehouse — irsim_devices raycaster against warehouse URDF geometry
    python 06_irsim_bridge.py \\
        --yaml irsim_warehouse.yaml \\
        --world models/warehouse_world.urdf

    # With inline 2-D matplotlib visualisation (headless irsim)
    python 06_irsim_bridge.py \\
        --yaml irsim_warehouse.yaml \\
        --world models/warehouse_world.urdf \\
        --no-render --viz 2d

    # With inline 2-D + 3-D side-by-side (requires Embree build)
    python 06_irsim_bridge.py \\
        --yaml irsim_warehouse.yaml \\
        --world models/warehouse_world.urdf \\
        --no-render --viz both

    # Subscribe to the bridge from a second terminal
    python 05_sub_viewer.py --live2d \\
        --world models/warehouse_world.urdf \\
        --robot models/robot_diff.urdf

    # Run headless for N steps then exit
    python 06_irsim_bridge.py --steps 500 --no-render
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from urdf_tools.pubsub import EncoderState, Imu, LaserScan, Odometry, SensorPublisher

SHM_NAME = "/urdf_tools_sensors"
WORLD_YAML = Path(__file__).parent / "irsim_world.yaml"


# ── helpers ───────────────────────────────────────────────────────────────────


def _robot_xytheta(robot) -> tuple[float, float, float]:
    st = robot.state
    return float(st[0, 0]), float(st[1, 0]), float(st[2, 0])


def _robot_vel(robot) -> tuple[float, float]:
    vel = robot.velocity
    v = float(vel[0, 0]) if vel.size > 0 else 0.0
    omega = float(vel[-1, 0]) if vel.size > 1 else 0.0
    return v, omega


# ── dark-theme palette (matches 05_sub_viewer.py) ─────────────────────────────

_BG = "#0f172a"  # slate-900 — figure / pane background
_PANE = "#0d1526"  # 3-D pane (slightly darker)
_GRID = "#1e293b"  # slate-800 — grid lines
_TEXT = "#94a3b8"  # slate-400 — axis labels
_WFC = "#28517e"  # world geometry fill  (brighter for contrast against _BG)
_WEC = "#7dd3fc"  # world geometry edge  (sky-300 — pops against dark navy)
_RC = "#3b82f6"  # robot colour  (blue-500)
_HC = "#facc15"  # heading arrow (amber-400 — contrasts with blue world/robot)
_RAY = "#f97316"  # scan ray      (orange-500)
_INFO_BG = "#0d1526"  # sensor-info textbox background
_INFO_EC = "#1e293b"  # sensor-info textbox border


def _draw_sensor_info(ax, title: str, info: dict, *, is_3d: bool = False) -> None:
    """Draw a monospace sensor-parameter readout in the top-left of *ax*."""
    text = title + "\n" + "\n".join(f"{k}: {v}" for k, v in info.items())
    kwargs = dict(
        fontsize=7.5,
        color=_TEXT,
        family="monospace",
        va="top",
        ha="left",
        transform=ax.transAxes,
        bbox=dict(fc=_INFO_BG, ec=_INFO_EC, lw=0.8, alpha=0.85, pad=4),
        zorder=20,
    )
    if is_3d:
        ax.text2D(0.02, 0.98, text, **kwargs)
    else:
        ax.text(0.02, 0.98, text, **kwargs)


# ── inline matplotlib visualiser ─────────────────────────────────────────────


class _LiveViz:
    """Real-time matplotlib visualiser embedded inside the bridge loop.

    Draws:
      2-D panel — URDF world polygons, LiDAR hit scatter + ray lines,
                  robot circle with heading arrow.
      3-D panel — URDF triangulated mesh (Poly3DCollection, downsampled),
                  VLP-16 point cloud (scatter), robot pyramid marker.

    Call ``update(x, y, theta, scan_dict, pts3d)`` every N steps.
    """

    def __init__(
        self,
        mode: str,  # "2d" | "3d" | "both"
        world_robot=None,
        urdf_model=None,
        rmax: float = 12.0,
        lidar2d_info: dict | None = None,
        lidar3d_info: dict | None = None,
    ) -> None:
        import matplotlib.pyplot as plt

        self._plt = plt
        self._mode = mode
        self._rmax = rmax
        self._ax2 = None
        self._ax3 = None
        self._motor_text = None
        self._motor_text3 = None
        self._dynamic_xlim = True
        self._ray_step = 6

        # ── build figure ──────────────────────────────────────────────────────
        if mode == "both":
            self._fig = plt.figure(figsize=(18, 8), facecolor=_BG)
            self._fig.canvas.manager.set_window_title("06_irsim_bridge — live viz")
            self._ax2 = self._fig.add_subplot(121)
            self._ax3 = self._fig.add_subplot(122, projection="3d")
        elif mode == "2d":
            self._fig, self._ax2 = plt.subplots(figsize=(9, 8))
            self._fig.patch.set_facecolor(_BG)
            self._fig.canvas.manager.set_window_title("06_irsim_bridge — 2-D LiDAR")
        else:  # "3d"
            self._fig = plt.figure(figsize=(10, 8), facecolor=_BG)
            self._fig.canvas.manager.set_window_title("06_irsim_bridge — 3-D cloud")
            self._ax3 = self._fig.add_subplot(111, projection="3d")

        if self._ax2 is not None:
            self._setup_2d(world_robot, rmax, lidar2d_info)
        if self._ax3 is not None:
            self._setup_3d(urdf_model, lidar3d_info)

        plt.tight_layout(pad=1.5)
        plt.ion()
        plt.show(block=False)
        plt.pause(0.05)  # flush initial draw

    # ─────────────────────────────────────────── 2-D panel ────────────────────

    def _setup_2d(self, world_robot, rmax: float, lidar2d_info: dict | None) -> None:
        import matplotlib.pyplot as plt

        ax = self._ax2
        ax.set_facecolor(_BG)
        ax.tick_params(colors=_TEXT, which="both", labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor(_GRID)
        ax.set_title("2-D LiDAR + URDF world", color=_TEXT, fontsize=10, pad=6)
        ax.set_xlabel("x  [m]", color=_TEXT, fontsize=8)
        ax.set_ylabel("y  [m]", color=_TEXT, fontsize=8)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, color=_GRID, lw=0.5, alpha=0.5)

        if lidar2d_info:
            _draw_sensor_info(ax, "LiDAR 2D", lidar2d_info)

        # ── static URDF world overlay ──────────────────────────────────────
        bounds = None
        if world_robot is not None:
            bounds = self._draw_world_2d(ax, world_robot)

        if bounds:
            xmin, xmax, ymin, ymax = bounds
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)
            self._rmax = min(max(xmax - xmin, ymax - ymin) * 0.4, 30.0)
            self._dynamic_xlim = False
        else:
            ax.set_xlim(-rmax, rmax)
            ax.set_ylim(-rmax, rmax)
            self._dynamic_xlim = True

        # Faint range rings
        ring = max(1, int(self._rmax / 4))
        for r in range(ring, int(self._rmax) + 1, ring):
            ax.add_patch(
                plt.Circle((0, 0), r, fc="none", ec=_GRID, lw=0.4, ls="--", alpha=0.35)
            )

        # ── animated artists ───────────────────────────────────────────────
        self._scan_sc = ax.scatter(
            [],
            [],
            s=10,
            c=[],
            cmap="plasma",
            vmin=0,
            vmax=self._rmax,
            zorder=5,
            edgecolors="none",
        )
        self._robot_circ = plt.Circle(
            (0, 0), 0.25, fc=_RC, ec="white", lw=1.2, zorder=7
        )
        ax.add_patch(self._robot_circ)
        (self._hdg_line,) = ax.plot([], [], color=_HC, lw=2.2, zorder=8)

        n_rays = 360 // self._ray_step + 10
        self._ray_lines = [
            ax.plot([], [], color=_RAY, lw=0.6, alpha=0.35, zorder=3)[0]
            for _ in range(n_rays)
        ]

        self._motor_text = ax.text(
            0.02,
            0.02,
            "",
            transform=ax.transAxes,
            fontsize=7.5,
            color=_TEXT,
            family="monospace",
            va="bottom",
            ha="left",
            bbox=dict(fc=_INFO_BG, ec=_INFO_EC, lw=0.8, alpha=0.85, pad=4),
            zorder=20,
        )

    def _draw_world_2d(self, ax, world_robot):
        """Draw URDF world as filled 2-D polygons (top-down XY footprint).

        Returns (xmin, xmax, ymin, ymax) bounding box, or None if the URDF
        has no groundable geometry.
        """
        import matplotlib.pyplot as plt

        from urdf_tools.viz import _link_world_blocks, _xy_footprint

        xs: list = []
        ys: list = []
        for T, geom, rgba in _link_world_blocks(world_robot):
            for kind, params, _ in _xy_footprint(T, geom, rgba):
                if kind == "polygon":
                    ax.add_patch(
                        plt.Polygon(
                            params, fc=_WFC, ec=_WEC, lw=0.8, alpha=0.75, zorder=2
                        )
                    )
                    xs.extend(params[:, 0])
                    ys.extend(params[:, 1])
                elif kind == "circle":
                    cx, cy, r = params
                    ax.add_patch(
                        plt.Circle(
                            (cx, cy), r, fc=_WFC, ec=_WEC, lw=0.8, alpha=0.75, zorder=2
                        )
                    )
                    xs += [cx - r, cx + r]
                    ys += [cy - r, cy + r]

        if not xs:
            return None
        pad = 1.5
        return (min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad)

    # ─────────────────────────────────────────── 3-D panel ────────────────────

    def _setup_3d(self, urdf_model, lidar3d_info: dict | None) -> None:
        ax = self._ax3
        ax.set_facecolor(_PANE)
        ax.tick_params(colors=_TEXT, which="both", labelsize=7)
        ax.set_title("3-D LiDAR + URDF mesh", color=_TEXT, fontsize=10, pad=6)
        ax.set_xlabel("x [m]", color=_TEXT, fontsize=7, labelpad=3)
        ax.set_ylabel("y [m]", color=_TEXT, fontsize=7, labelpad=3)
        ax.set_zlabel("z [m]", color=_TEXT, fontsize=7, labelpad=3)
        for attr in ("xaxis", "yaxis", "zaxis"):
            pane = getattr(ax, attr).pane
            pane.fill = False
            pane.set_edgecolor(_GRID)

        if lidar3d_info:
            _draw_sensor_info(ax, "LiDAR 3D", lidar3d_info, is_3d=True)

        # Motor/encoder text lives here only in 3D-only mode; "both" mode
        # shows it on the 2-D panel instead (see _setup_2d).
        if self._ax2 is None:
            self._motor_text3 = ax.text2D(
                0.02,
                0.02,
                "",
                transform=ax.transAxes,
                fontsize=7.5,
                color=_TEXT,
                family="monospace",
                va="bottom",
                ha="left",
                bbox=dict(fc=_INFO_BG, ec=_INFO_EC, lw=0.8, alpha=0.85, pad=4),
                zorder=20,
            )

        # Static URDF mesh
        xlim = ylim = zlim = (-12.0, 12.0)
        if urdf_model is not None:
            xlim, ylim, zlim = self._draw_urdf_mesh(ax, urdf_model)

        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_zlim(*zlim)
        ax.view_init(elev=28, azim=-55)

        # Animated cloud scatter (Nx4 — x,y,z,intensity)
        self._cloud_sc = ax.scatter(
            [],
            [],
            [],
            s=4,
            c=[],
            cmap="plasma",
            vmin=0.0,
            vmax=1.0,
            alpha=0.95,
            depthshade=False,
        )
        # Robot pyramid marker
        self._robot_sc3 = ax.scatter(
            [0],
            [0],
            [0.3],
            s=90,
            c=_RC,
            edgecolors="white",
            linewidths=0.6,
            marker="^",
            depthshade=False,
            zorder=10,
        )

    def _draw_urdf_mesh(self, ax, urdf_model):
        """Render triangulated URDF mesh as translucent Poly3DCollection.

        Returns (xlim, ylim, zlim) tuples for axis limits.
        """
        try:
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection

            verts = np.asarray(urdf_model.vertices, dtype=np.float32)
            tris = np.asarray(urdf_model.triangles, dtype=np.int32)

            # Downsample: at most ~600 triangles for speed
            step = max(1, len(tris) // 600)
            polys = verts[tris[::step]]  # (N, 3, 3)

            mesh = Poly3DCollection(
                polys,
                alpha=0.35,
                facecolor=_WFC,
                edgecolor=_WEC,
                lw=0.4,
            )
            ax.add_collection3d(mesh)

            xr, yr, zr = verts[:, 0], verts[:, 1], verts[:, 2]
            pad = 1.0
            return (
                (float(xr.min()) - pad, float(xr.max()) + pad),
                (float(yr.min()) - pad, float(yr.max()) + pad),
                (max(float(zr.min()) - 0.5, -1.0), min(float(zr.max()) + 0.5, 6.0)),
            )
        except Exception as exc:
            print(f"[viz] mesh draw failed: {exc}", file=sys.stderr)
            return (-12, 12), (-12, 12), (-1, 5)

    # ─────────────────────────────────────────── per-step update ──────────────

    def update(
        self,
        x: float,
        y: float,
        theta: float,
        scan_dict: dict | None = None,
        pts3d=None,
        motor_state: dict | None = None,
    ) -> None:
        """Refresh all animated artists and flush the figure."""
        if self._ax2 is not None:
            self._update_2d(x, y, theta, scan_dict)
        if self._ax3 is not None:
            self._update_3d(x, y, pts3d)
        if motor_state is not None:
            self._update_motor_text(motor_state)
        self._plt.pause(0.001)

    def _update_motor_text(self, motor_state: dict) -> None:
        left, right = motor_state["left"], motor_state["right"]
        text = (
            "Motor  I(A)  V(V)  wheel(rad/s)\n"
            f"L    {left['current']:6.2f} {left['voltage_est']:6.2f} {left['omega_output']:7.2f}\n"
            f"R    {right['current']:6.2f} {right['voltage_est']:6.2f} {right['omega_output']:7.2f}"
        )
        if self._motor_text is not None:
            self._motor_text.set_text(text)
        if self._motor_text3 is not None:
            self._motor_text3.set_text(text)

    def _update_2d(self, x, y, theta, scan_dict) -> None:
        # Robot pose
        self._robot_circ.center = (x, y)
        self._hdg_line.set_data(
            [x, x + 0.7 * math.cos(theta)],
            [y, y + 0.7 * math.sin(theta)],
        )
        if self._dynamic_xlim:
            rm = self._rmax
            self._ax2.set_xlim(x - rm, x + rm)
            self._ax2.set_ylim(y - rm, y + rm)

        if not scan_dict:
            self._scan_sc.set_offsets(np.empty((0, 2)))
            for li in self._ray_lines:
                li.set_data([], [])
            return

        rng = np.asarray(scan_dict.get("ranges", []), dtype=np.float32).ravel()
        if not len(rng):
            return

        amin = float(scan_dict.get("angle_min", -math.pi))
        inc = float(scan_dict.get("angle_increment", 2 * math.pi / max(len(rng), 1)))
        rmax_v = float(scan_dict.get("range_max", 8.0))
        a = amin + np.arange(len(rng), dtype=np.float32) * inc
        hit = rng < rmax_v * 0.999
        hx = x + rng[hit] * np.cos(theta + a[hit])
        hy = y + rng[hit] * np.sin(theta + a[hit])
        pts = np.column_stack([hx, hy]) if len(hx) else np.empty((0, 2))

        self._scan_sc.set_offsets(pts)
        self._scan_sc.set_array(rng[hit])

        # Ray lines (every ray_step-th beam)
        ray_idx = np.arange(0, len(rng), self._ray_step)
        for k, li in enumerate(self._ray_lines):
            if k < len(ray_idx):
                i = ray_idx[k]
                li.set_data(
                    [x, x + rng[i] * math.cos(theta + float(a[i]))],
                    [y, y + rng[i] * math.sin(theta + float(a[i]))],
                )
            else:
                li.set_data([], [])

    def _update_3d(self, x, y, pts3d) -> None:
        # Robot marker
        self._robot_sc3._offsets3d = ([x], [y], [0.3])

        if pts3d is None or not len(pts3d):
            return

        arr = np.asarray(pts3d, dtype=np.float32)
        # Downsample to ≤4 000 points for interactive speed
        step = max(1, len(arr) // 4000)
        arr = arr[::step]
        xs, ys, zs = arr[:, 0], arr[:, 1], arr[:, 2]

        # Colour by intensity (col 3) or fall back to height
        intens = arr[:, 3] if arr.shape[1] > 3 else zs - zs.min()

        self._cloud_sc._offsets3d = (xs, ys, zs)
        self._cloud_sc.set_array(intens)
        if len(intens):
            self._cloud_sc.set_clim(float(intens.min()), float(intens.max()))

    def close(self) -> None:
        try:
            self._plt.close(self._fig)
        except Exception:
            pass


# ── main ──────────────────────────────────────────────────────────────────────


def main() -> None:
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    ap = argparse.ArgumentParser(description="IR-SIM shmbridge publisher")
    ap.add_argument("--yaml", default=str(WORLD_YAML), help="World YAML path")
    ap.add_argument("--shm", default=SHM_NAME, help="Shared memory segment name")
    ap.add_argument("--steps", type=int, default=0, help="Max steps (0 = infinite)")
    ap.add_argument(
        "--no-render", action="store_true", help="Run headless (no irsim window)"
    )
    ap.add_argument(
        "--world",
        default=None,
        metavar="URDF",
        help="World URDF — geometry injected as static obstacles for lidar raycasting",
    )
    ap.add_argument(
        "--lidar-height",
        type=float,
        default=0.30,
        metavar="M",
        help="Scan-plane height for URDF Z filter (default 0.30 m)",
    )
    ap.add_argument(
        "--loop", action="store_true", help="Loop simulation when goal is reached"
    )
    ap.add_argument(
        "--hz", type=float, default=30.0, help="Reported sensor rate (Hz, display only)"
    )
    ap.add_argument(
        "--wheel-radius",
        type=float,
        default=0.05,
        metavar="M",
        help="Shadow chassis wheel radius for encoder/motor telemetry (default 0.05 m)",
    )
    ap.add_argument(
        "--wheel-base",
        type=float,
        default=0.30,
        metavar="M",
        help="Shadow chassis wheel base for encoder/motor telemetry (default 0.30 m)",
    )
    ap.add_argument(
        "--motor-profile",
        default="small_dc",
        metavar="NAME",
        help="Motor.PROFILES name for the shadow chassis (default 'small_dc')",
    )
    ap.add_argument(
        "--viz",
        choices=["2d", "3d", "both"],
        default=None,
        metavar="MODE",
        help="Inline matplotlib visualisation: 2d | 3d | both",
    )
    ap.add_argument(
        "--viz-rate",
        type=int,
        default=5,
        metavar="N",
        help="Render matplotlib every N steps (default 5)",
    )
    args = ap.parse_args()

    import irsim

    env = irsim.make(args.yaml, headless=args.no_render)

    # ── irsim_devices sensors ────────────────────────────────────────────────
    dev_lidar = None
    dev_lidar3d = None
    world_robot = None
    urdf_model = None

    if args.world:
        from urdf_tools.irsim_compat import urdf_to_scene_2d
        from urdf_tools.parser import parse_urdf

        world_robot = parse_urdf(args.world)
        scene = urdf_to_scene_2d(world_robot, lidar_height=args.lidar_height)

        from irsim_devices.sensors import Lidar2D as DevLidar2D

        robot_tmp = env.robot_list[0]
        irsim_lidar = getattr(robot_tmp, "lidar", None)
        x0, y0, th0 = _robot_xytheta(robot_tmp)
        dev_lidar = DevLidar2D(
            state=np.array([x0, y0, th0], dtype=np.float64),
            range_min=irsim_lidar.range_min if irsim_lidar else 0.1,
            range_max=irsim_lidar.range_max if irsim_lidar else 20.0,
            angle_range=irsim_lidar.angle_range if irsim_lidar else 6.2832,
            number=irsim_lidar.number if irsim_lidar else 360,
        )
        dev_lidar.set_scene(scene)
        print(
            f"[bridge] irsim_devices lidar — {len(scene)} scene objects"
            f" from {args.world!r}  (lidar_height={args.lidar_height} m)"
        )

        # 3-D Embree lidar
        try:
            from irsim_devices.models.urdf_loader import load_urdf
            from irsim_devices.sensors.lidar3d_embree import EmbreeLidar3D

            urdf_model = load_urdf(args.world, use_collision=False)
            dev_lidar3d = EmbreeLidar3D(
                state=np.array([x0, y0, th0], dtype=np.float64),
                obj_id=-2,
                profile="vlp16",
                sensor_height=args.lidar_height,
            )
            dev_lidar3d.build_embree_scene(urdf_model.vertices, urdf_model.triangles)
            print(
                f"[bridge] EmbreeLidar3D vlp16 — {len(urdf_model.triangles)} tris"
                f" ({len(urdf_model.vertices)} verts)"
            )
        except Exception as exc:
            print(f"[warn] EmbreeLidar3D unavailable: {exc}", file=sys.stderr)
            dev_lidar3d = None

    robot = env.robot_list[0]
    active_lidar = dev_lidar or getattr(robot, "lidar", None)
    has_lidar = active_lidar is not None
    if not has_lidar:
        print(
            "[warn] Robot has no lidar sensor — only odometry will be published.",
            file=sys.stderr,
        )

    # ── IMU device model + shadow motor/encoder chassis ─────────────────────
    # The IMU derives realistic noisy accel/gyro from the robot's true motion.
    # The chassis is a "shadow" model: it is not what drives the robot (irsim
    # owns the ground-truth pose) — it is fed the same [v, omega] command each
    # step so its per-wheel Motor + encoder dynamics give realistic telemetry.
    from irsim_devices.actuators import MotorDiffChassis
    from irsim_devices.sensors import IMU as DevImu

    x0, y0, th0 = _robot_xytheta(robot)
    step_time = getattr(env, "step_time", 0.05)

    dev_imu = DevImu(
        state=np.array([x0, y0, th0], dtype=np.float64),
        step_time=step_time,
        noise_model="ieee517",
    )

    chassis = MotorDiffChassis(
        wheel_radius=args.wheel_radius,
        wheel_base=args.wheel_base,
        motor_profile=args.motor_profile,
        initial_state=[x0, y0, th0],
    )
    chassis.left_motor.set_mode("velocity")
    chassis.right_motor.set_mode("velocity")
    half_wheel_base = args.wheel_base / 2.0

    rate_hz = args.hz
    lidar2d_info = None
    if active_lidar is not None:
        res_deg = math.degrees(active_lidar.angle_range) / max(
            active_lidar.number - 1, 1
        )
        lidar2d_info = {
            "beams": active_lidar.number,
            "resolution": f"{res_deg:.2f} deg",
            "range": f"{active_lidar.range_min:.2f}-{active_lidar.range_max:.1f} m",
            "frequency": f"{rate_hz:.1f} Hz",
        }
    lidar3d_info = None
    if dev_lidar3d is not None:
        n_vert, n_horiz, elev_min, elev_max = dev_lidar3d.PROFILES[dev_lidar3d.profile]
        lidar3d_info = {
            "profile": dev_lidar3d.profile,
            "resolution": f"{n_vert}x{n_horiz}",
            "elevation": f"{elev_min:.1f} to {elev_max:.1f} deg",
            "range": f"0-{dev_lidar3d.range_max:.1f} m",
            "frequency": f"{rate_hz:.1f} Hz",
        }

    # ── inline matplotlib visualiser ────────────────────────────────────────
    viz: _LiveViz | None = None
    if args.viz:
        try:
            viz = _LiveViz(
                mode=args.viz,
                world_robot=world_robot,
                urdf_model=urdf_model if args.viz in ("3d", "both") else None,
                rmax=12.0,
                lidar2d_info=lidar2d_info,
                lidar3d_info=lidar3d_info,
            )
            print(
                f"[bridge] inline viz={args.viz!r}  update every {args.viz_rate} steps"
            )
        except Exception as exc:
            print(f"[warn] viz init failed: {exc}", file=sys.stderr)
            viz = None

    print(f"[bridge] shm={args.shm!r}  render={not args.no_render}")
    print("[bridge] Ctrl+C to stop.\n")

    step = 0
    t0 = time.time()

    with SensorPublisher(args.shm) as pub:
        try:
            while True:
                env.step()
                if not args.no_render:
                    env.render(0.001)
                else:
                    time.sleep(getattr(env, "step_time", 0.05))

                sim_time = time.time() - t0
                x, y, theta = _robot_xytheta(robot)
                v, omega = _robot_vel(robot)

                # ── Odometry ──────────────────────────────────────────────
                pub.publish_odom(
                    Odometry(
                        stamp=sim_time,
                        x=x,
                        y=y,
                        theta=theta,
                        vx=v * math.cos(theta),
                        vy=v * math.sin(theta),
                        omega=omega,
                    )
                )

                # ── 2-D LiDAR scan ────────────────────────────────────────
                scan_dict: dict | None = None
                if has_lidar:
                    if dev_lidar is not None:
                        dev_lidar.step(np.array([x, y, theta], dtype=np.float64))
                        scan_dict = dev_lidar.get_scan()
                    else:
                        scan_dict = robot.get_lidar_scan()

                    ranges = scan_dict.get("ranges") if scan_dict else None
                    if ranges is not None and len(ranges):
                        rng = np.asarray(ranges, dtype=np.float32).ravel()
                        pub.publish_scan(
                            LaserScan(
                                stamp=sim_time,
                                angle_min=float(scan_dict.get("angle_min", -math.pi)),
                                angle_max=float(scan_dict.get("angle_max", math.pi)),
                                angle_increment=float(
                                    scan_dict.get(
                                        "angle_increment",
                                        2 * math.pi / max(len(rng), 1),
                                    )
                                ),
                                range_min=float(scan_dict.get("range_min", 0.1)),
                                range_max=float(scan_dict.get("range_max", 8.0)),
                                ranges=rng.tolist(),
                            )
                        )

                # ── 3-D LiDAR cloud ───────────────────────────────────────
                pts3d = None
                if dev_lidar3d is not None:
                    dev_lidar3d.step(np.array([x, y, theta], dtype=np.float64))
                    pts3d = dev_lidar3d.scan
                    if pts3d is not None and len(pts3d):
                        pub.publish_cloud3d(pts3d, stamp=sim_time)

                # ── IMU (device model, derived from true motion) ──────────
                dev_imu.step(np.array([x, y, theta], dtype=np.float64))
                pub.publish_imu(
                    Imu(
                        stamp=sim_time,
                        linear_acceleration=dev_imu.linear_acceleration.tolist(),
                        angular_velocity=dev_imu.angular_velocity.tolist(),
                        orientation_rpy=[0.0, 0.0, theta],
                    )
                )

                # ── Encoder + motor (shadow chassis driven by [v, omega]) ──
                omega_l_cmd = (v - omega * half_wheel_base) / args.wheel_radius
                omega_r_cmd = (v + omega * half_wheel_base) / args.wheel_radius
                chassis.step([omega_l_cmd, omega_r_cmd], dt=step_time)
                enc = chassis.encoder_readings
                pub.publish_encoder(
                    EncoderState(
                        stamp=sim_time,
                        ticks=[enc["left"]["ticks"], enc["right"]["ticks"], 0, 0],
                        speed=[
                            enc["left"]["omega_output"],
                            enc["right"]["omega_output"],
                            0.0,
                            0.0,
                        ],
                    )
                )
                motor_state = chassis.motor_state

                # ── inline visualisation ───────────────────────────────────
                if viz is not None and step % args.viz_rate == 0:
                    viz.update(
                        x,
                        y,
                        theta,
                        scan_dict=scan_dict,
                        pts3d=pts3d,
                        motor_state=motor_state,
                    )

                step += 1
                cl, cr = motor_state["left"]["current"], motor_state["right"]["current"]
                print(
                    f"\r  step={step:5d}  t={sim_time:6.1f}s"
                    f"  ({x:5.2f},{y:5.2f})  θ={theta:.2f}  ω={omega:.3f}"
                    f"  motor(A)=[{cl:5.2f},{cr:5.2f}]",
                    end="",
                    flush=True,
                )

                if env.done():
                    print(f"\n[bridge] goal reached at step={step}")
                    if args.loop:
                        env.reset()
                        step = 0
                    else:
                        break
                if args.steps and step >= args.steps:
                    print(f"\n[bridge] max steps ({args.steps}) reached")
                    break

        except KeyboardInterrupt:
            print("\n[bridge] stopped.")
        finally:
            if viz is not None:
                viz.close()
            env.end()


if __name__ == "__main__":
    main()
