"""CLI entry points for urdf-tools."""

from __future__ import annotations

import math
import sys
import time

import click
import numpy as np

from .parser import parse_urdf

SHM_DEFAULT = "/urdf_tools_sensors"


@click.group()
@click.version_option()
def main() -> None:
    """URDF visualization and sensor pub/sub tools."""


# ── info ──────────────────────────────────────────────────────────────────────


@main.command()
@click.argument("urdf_file", type=click.Path(exists=True))
def info(urdf_file: str) -> None:
    """Print a summary of links, joints, and geometry counts."""
    robot = parse_urdf(urdf_file)
    click.echo(f"Robot  : {robot.name}")
    click.echo(f"Links  : {len(robot.links)}")
    click.echo(f"Joints : {len(robot.joints)}")
    geom_counts: dict[str, int] = {}
    for lnk in robot.links:
        for block in lnk.visuals + lnk.collisions:
            if block.geometry:
                t = block.geometry.type
                geom_counts[t] = geom_counts.get(t, 0) + 1
    click.echo(f"Geoms  : {geom_counts}")
    click.echo("\nJoints:")
    for j in robot.joints:
        click.echo(f"  [{j.type:10s}] {j.name:30s}  {j.parent} → {j.child}")


# ── view (2D) ─────────────────────────────────────────────────────────────────


@main.command()
@click.argument("urdf_file", type=click.Path(exists=True))
@click.option("--save", default=None, metavar="FILE", help="Save figure to PNG/PDF.")
@click.option("--no-show", is_flag=True, default=False)
def view(urdf_file: str, save: str | None, no_show: bool) -> None:
    """2D top-down floor plan."""
    import matplotlib

    if no_show:
        matplotlib.use("Agg")
    from .viz import plot_floor_plan

    robot = parse_urdf(urdf_file)
    click.echo(
        f"  {robot.name!r}: {len(robot.links)} links, {len(robot.joints)} joints"
    )
    plot_floor_plan(robot, save=save, show=not no_show)


# ── view3d ────────────────────────────────────────────────────────────────────


@main.command()
@click.argument("urdf_file", type=click.Path(exists=True))
@click.option("--save", default=None, metavar="FILE")
@click.option("--no-show", is_flag=True, default=False)
def view3d(urdf_file: str, save: str | None, no_show: bool) -> None:
    """3D wireframe viewer."""
    import matplotlib

    if no_show:
        matplotlib.use("Agg")
    from .viz import plot_3d

    robot = parse_urdf(urdf_file)
    click.echo(f"  {robot.name!r}: {len(robot.links)} links")
    plot_3d(robot, save=save, show=not no_show)


# ── tree ──────────────────────────────────────────────────────────────────────


@main.command()
@click.argument("urdf_file", type=click.Path(exists=True))
@click.option("--save", default=None, metavar="FILE")
@click.option("--no-show", is_flag=True, default=False)
def tree(urdf_file: str, save: str | None, no_show: bool) -> None:
    """Kinematic tree diagram."""
    import matplotlib

    if no_show:
        matplotlib.use("Agg")
    from .viz import plot_kinematic_tree

    robot = parse_urdf(urdf_file)
    for j in robot.joints:
        click.echo(f"  {j.parent:20s} --[{j.type}]--> {j.child}")
    plot_kinematic_tree(robot, save=save, show=not no_show)


# ── publish ───────────────────────────────────────────────────────────────────


@main.command()
@click.argument("urdf_file", type=click.Path(exists=True))
@click.option("--shm", default=SHM_DEFAULT, show_default=True, help="Shm segment name.")
@click.option("--rate", default=20.0, show_default=True, help="Publish rate (Hz).")
@click.option("--beams", default=1080, show_default=True, help="LiDAR beam count.")
@click.option("--rmax", default=20.0, show_default=True, help="Max range (m).")
def publish(urdf_file: str, shm: str, rate: float, beams: int, rmax: float) -> None:
    """Simulate LiDAR + IMU + odometry and publish via shmbridge."""
    from .pubsub import Imu, LaserScan, Odometry, SensorPublisher

    robot = parse_urdf(urdf_file)
    click.echo(
        f"  Publishing for {robot.name!r} on shm={shm!r}  rate={rate}Hz  beams={beams}"
    )
    click.echo("  Ctrl+C to stop.")

    OBSTACLES = [(3.0, 3.0, 0.5), (-4.0, 2.0, 0.8), (1.0, -5.0, 1.2)]
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False, dtype=np.float32)
    a_inc = float(2 * math.pi / beams)

    def sim_scan(x: float, y: float, theta: float) -> np.ndarray:
        rng = np.full(beams, rmax, dtype=np.float32)
        beam_a = angles + theta
        ca, sa = np.cos(beam_a), np.sin(beam_a)
        for cx, cy, r in OBSTACLES:
            dx, dy = cx - x, cy - y
            b_q = -2 * (dx * ca + dy * sa)
            c_q = dx**2 + dy**2 - r**2
            disc = b_q**2 - 4 * c_q
            hit = disc >= 0
            t_hit = (-b_q[hit] - np.sqrt(np.maximum(disc[hit], 0))) / 2.0
            valid = t_hit > 0.05
            rng[hit] = np.where(valid, np.minimum(rng[hit], t_hit), rng[hit])
        return np.clip(rng, 0.05, rmax)

    t0 = time.time()
    with SensorPublisher(shm) as pub:
        try:
            while True:
                t = time.time() - t0
                x = 5.0 * math.cos(0.1 * t)
                y = 5.0 * math.sin(0.1 * t)
                theta = math.atan2(-math.sin(0.1 * t), -math.cos(0.1 * t)) + math.pi

                rng = sim_scan(x, y, theta)
                hits = int(np.sum(rng < rmax))

                pub.publish_scan(
                    LaserScan(
                        stamp=t,
                        angle_min=float(angles[0]),
                        angle_max=float(angles[-1]),
                        angle_increment=a_inc,
                        range_max=rmax,
                        ranges=rng.tolist(),
                    )
                )
                pub.publish_imu(
                    Imu(
                        stamp=t,
                        linear_acceleration=[0.0, 0.0, 9.81],
                        angular_velocity=[0.0, 0.0, float(0.1 * math.cos(0.1 * t))],
                    )
                )
                pub.publish_odom(
                    Odometry(
                        stamp=t,
                        x=float(x),
                        y=float(y),
                        theta=float(theta),
                        vx=0.5,
                        omega=0.1,
                    )
                )

                click.echo(
                    f"\r  t={t:6.1f}s  ({x:5.2f},{y:5.2f})  θ={theta:.2f}"
                    f"  hits={hits}/{beams}",
                    nl=False,
                )
                time.sleep(1.0 / rate)
        except KeyboardInterrupt:
            click.echo("\n  Stopped.")


# ── subscribe ─────────────────────────────────────────────────────────────────


@main.command()
@click.option("--shm", default=SHM_DEFAULT, show_default=True)
@click.option(
    "--count", default=0, show_default=True, help="Stop after N polls (0=forever)."
)
@click.option(
    "--live", is_flag=True, default=False, help="Show live matplotlib scan viewer."
)
@click.option(
    "--timeout",
    default=10000.0,
    show_default=True,
    help="Attach timeout in ms before giving up.",
)
def subscribe(shm: str, count: int, live: bool, timeout: float) -> None:
    """Subscribe to sensor topics; print or display live scan."""
    from .pubsub import SensorSubscriber

    click.echo(f"  Attaching to shm={shm!r} (timeout={timeout:.0f}ms) …")
    sub = SensorSubscriber(shm, timeout_ms=timeout)
    try:
        sub.attach()
    except TimeoutError as e:
        click.echo(f"  Error: {e}", err=True)
        sys.exit(1)
    click.echo("  Attached.  Ctrl+C to stop.\n")

    if live:
        _live_viewer(sub)
        sub.detach()
        return

    n = 0
    try:
        while True:
            scan = sub.read_scan()
            imu = sub.read_imu()
            odom = sub.read_odom()
            if scan:
                hits = sum(1 for r in scan.ranges if r < scan.range_max)
                click.echo(
                    f"  [scan] t={scan.stamp:.3f}  beams={len(scan.ranges)}  hits={hits}"
                )
            if imu:
                click.echo(
                    f"  [imu]  t={imu.stamp:.3f}  acc={[f'{v:.2f}' for v in imu.linear_acceleration]}"
                )
            if odom:
                click.echo(
                    f"  [odom] t={odom.stamp:.3f}  x={odom.x:.2f}  y={odom.y:.2f}  θ={odom.theta:.2f}"
                )
            n += 1
            if count and n >= count:
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        click.echo("\n  Stopped.")
    finally:
        sub.detach()


def _live_viewer(sub) -> None:
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    RMAX = 25.0
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(-RMAX, RMAX)
    ax.set_ylim(-RMAX, RMAX)
    ax.set_facecolor("#0f172a")
    fig.patch.set_facecolor("#0f172a")
    ax.set_title("Live LiDAR — shmbridge", fontweight="bold", color="white")
    ax.tick_params(colors="#475569")
    for sp in ax.spines.values():
        sp.set_edgecolor("#1e293b")

    scat = ax.scatter([], [], s=1.5, c=[], cmap="plasma", vmin=0, vmax=RMAX, zorder=3)
    robot_dot = ax.scatter([0], [0], s=100, c="#3b82f6", zorder=5, marker="D")
    (hdg,) = ax.plot([], [], color="#60a5fa", lw=1.5, zorder=4)

    state = {"pose": (0.0, 0.0, 0.0)}

    def update(_frame):
        scan = sub.read_scan()
        odom = sub.read_odom()
        if odom:
            state["pose"] = (odom.x, odom.y, odom.theta)
        x0, y0, th = state["pose"]
        robot_dot.set_offsets([[x0, y0]])
        hdg.set_data([x0, x0 + 1.2 * math.cos(th)], [y0, y0 + 1.2 * math.sin(th)])
        if scan and scan.ranges:
            rng = np.array(scan.ranges, dtype=np.float32)
            a = scan.angle_min + np.arange(len(rng)) * scan.angle_increment
            hit = rng < scan.range_max
            xs = x0 + rng[hit] * np.cos(th + a[hit])
            ys = y0 + rng[hit] * np.sin(th + a[hit])
            scat.set_offsets(np.column_stack([xs, ys]) if len(xs) else np.empty((0, 2)))
            scat.set_array(rng[hit])
        return scat, robot_dot, hdg

    ani = animation.FuncAnimation(fig, update, interval=60, blit=True)
    _ = ani  # keep reference
    plt.tight_layout()
    plt.show()
