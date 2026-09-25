"""
cpu_reduction_30hz.py — LiDAR 2D CPU-reduction benchmark at 30 Hz / 1500 beams / 30 m.

Compares three configurations:
  v1  Positional-cache path (Shapely disk query per cache miss, old default)
  v2  Static precompute (no Shapely per step, zero-alloc direction SoA)
  v2t Static precompute + 2 OMP threads (coexistence mode for robot stack)

For each config, reports:
  - Max throughput (step/s) — sustained 2 s run
  - Step latency: median, p99
  - Throttled 30 Hz CPU% — 5 s wall-clock at exactly 30 Hz using time.sleep()
  - CPU per core at idle and under 30 Hz load

Results are written to a JSON file and to stdout.
"""

from __future__ import annotations

import json
import sys
import time
from math import pi
from pathlib import Path

import numpy as np
import psutil

# ── paths ────────────────────────────────────────────────────────────────────
_HERE = Path(__file__).parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "src"))

from irsim_devices.core.open3d_scene_2d import Open3DScene2D  # noqa: E402
from irsim_devices.sensors.lidar2d import Lidar2D  # noqa: E402

# ── scene (same synthetic warehouse as warehouse_lidar2d.py) ─────────────────
W, H = 50.0, 30.0
WALL_T = 0.2
PILLAR_R = 0.3
PILLAR_COLS = [10.0, 20.0, 30.0, 40.0]
PILLAR_ROWS = [7.5, 15.0, 22.5]
RACK_LENGTH, RACK_WIDTH = 14.0, 1.0
RACK_MARGIN = 3.0
RACK_XS = [RACK_MARGIN, W - RACK_MARGIN - RACK_LENGTH]
RACK_YS = [5.0, 10.0, 15.0, 20.0, 25.0]


def _make_scene() -> Open3DScene2D:
    scene = Open3DScene2D()
    hw = W / 2
    scene.add_box(center=(hw, WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(hw, H - WALL_T / 2), width=hw + WALL_T, height=WALL_T / 2)
    scene.add_box(center=(WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    scene.add_box(center=(W - WALL_T / 2, H / 2), width=WALL_T / 2, height=H / 2)
    for cx in PILLAR_COLS:
        for cy in PILLAR_ROWS:
            scene.add_circle(center=(cx, cy), radius=PILLAR_R, resolution=16)
    for rx in RACK_XS:
        for ry in RACK_YS:
            scene.add_box(center=(rx + RACK_LENGTH / 2, ry),
                          width=RACK_LENGTH / 2, height=RACK_WIDTH / 2)
    return scene


# ── sensor factory ────────────────────────────────────────────────────────────
def _make_lidar(state=(25.0, 15.0, 0.0)) -> Lidar2D:
    return Lidar2D(
        state=np.array(state),
        obj_id=0,
        range_min=0.1,
        range_max=30.0,
        angle_range=2 * pi,
        number=1500,
        scan_time=1 / 30,
    )


# ── CPU sampling helpers ─────────────────────────────────────────────────────

def _cpu_baseline(duration: float = 1.0) -> list[float]:
    """Sample per-core CPU% at idle for *duration* seconds."""
    proc = psutil.Process()
    cpu_count = psutil.cpu_count(logical=True)
    # Warm up the psutil counter
    psutil.cpu_percent(percpu=True)
    time.sleep(duration)
    return psutil.cpu_percent(interval=None, percpu=True)


def _cpu_under_load(
    step_fn,
    target_hz: float,
    duration: float = 5.0,
) -> tuple[list[float], int, list[float]]:
    """Run *step_fn* at *target_hz* for *duration* seconds.

    Returns:
        (per_core_cpu_pct, n_steps, step_latencies_ms)
    """
    psutil.cpu_percent(percpu=True)  # reset counter
    interval = 1.0 / target_hz
    t0 = time.perf_counter()
    t_next = t0
    latencies: list[float] = []
    n = 0

    while time.perf_counter() - t0 < duration:
        t_next += interval
        t_step = time.perf_counter()
        step_fn()
        latencies.append((time.perf_counter() - t_step) * 1e3)
        n += 1
        sleep = t_next - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)

    cpus = psutil.cpu_percent(interval=None, percpu=True)
    return cpus, n, latencies


def _throughput(step_fn, duration: float = 2.0) -> tuple[float, list[float]]:
    """Run *step_fn* as fast as possible for *duration* seconds."""
    t0 = time.perf_counter()
    latencies: list[float] = []
    while time.perf_counter() - t0 < duration:
        ts = time.perf_counter()
        step_fn()
        latencies.append((time.perf_counter() - ts) * 1e3)
    elapsed = time.perf_counter() - t0
    hz = len(latencies) / elapsed
    return hz, latencies


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    print("Loading warehouse scene …", flush=True)
    scene = _make_scene()
    n_segs_total = sum(len(o.linestrings) for o in scene.objects)
    print(f"  Scene: {len(scene.objects)} objects, {n_segs_total} linestrings")

    state = np.array([25.0, 15.0, 0.0])
    target_hz = 30.0

    # ── Warmup (JIT, OS page faults) ─────────────────────────────────────────
    sensor_w = _make_lidar()
    sensor_w.set_scene(scene, n_omp_threads=4)
    for _ in range(50):
        sensor_w.step(state)

    results: dict = {}

    # ═══════════════════════════════════════════════════════════════════════════
    #  v1 — positional cache (Shapely disk query path, no precompute)
    # ═══════════════════════════════════════════════════════════════════════════
    print("\n── v1: positional-cache (Shapely) ──")
    sensor_v1 = _make_lidar()
    # Attach scene via simple setter → triggers _attach_scene, but we want the
    # OLD Shapely path for comparison.  Monkeypatch: clear precomputed segs.
    sensor_v1._attach_scene(scene)
    # Force fallback by removing precomputed data
    sensor_v1._all_seg_sx = None
    sensor_v1._cast_inplace = None

    # Warm up positional cache at current position
    for _ in range(20):
        sensor_v1.step(state)

    hz_v1, lat_v1 = _throughput(lambda: sensor_v1.step(state))
    cpu_v1, n_v1, lat30_v1 = _cpu_under_load(
        lambda: sensor_v1.step(state), target_hz
    )
    print(f"  Throughput: {hz_v1:.0f} Hz")
    print(f"  Step p50/p99: {np.percentile(lat_v1, 50):.3f} / {np.percentile(lat_v1, 99):.3f} ms")
    print(f"  30 Hz CPU total: {sum(cpu_v1):.1f}%  mean-core: {np.mean(cpu_v1):.1f}%")
    results["v1_shapely"] = {
        "throughput_hz": hz_v1,
        "lat_p50_ms": float(np.percentile(lat_v1, 50)),
        "lat_p99_ms": float(np.percentile(lat_v1, 99)),
        "cpu_per_core_pct": cpu_v1,
        "cpu_total_pct": sum(cpu_v1),
        "n_steps_30hz": n_v1,
    }

    # ═══════════════════════════════════════════════════════════════════════════
    #  v2 — static precompute, all OMP threads
    # ═══════════════════════════════════════════════════════════════════════════
    print("\n── v2: static precompute, all OMP threads ──")
    n_cores = psutil.cpu_count(logical=False) or 4
    sensor_v2 = _make_lidar()
    sensor_v2.set_scene(scene, n_omp_threads=n_cores)
    for _ in range(20):
        sensor_v2.step(state)

    hz_v2, lat_v2 = _throughput(lambda: sensor_v2.step(state))
    cpu_v2, n_v2, lat30_v2 = _cpu_under_load(
        lambda: sensor_v2.step(state), target_hz
    )
    print(f"  Throughput: {hz_v2:.0f} Hz  ({hz_v2/hz_v1:.1f}× v1)")
    print(f"  Step p50/p99: {np.percentile(lat_v2, 50):.3f} / {np.percentile(lat_v2, 99):.3f} ms")
    print(f"  30 Hz CPU total: {sum(cpu_v2):.1f}%  mean-core: {np.mean(cpu_v2):.1f}%")
    results["v2_precompute_allcores"] = {
        "throughput_hz": hz_v2,
        "lat_p50_ms": float(np.percentile(lat_v2, 50)),
        "lat_p99_ms": float(np.percentile(lat_v2, 99)),
        "cpu_per_core_pct": cpu_v2,
        "cpu_total_pct": sum(cpu_v2),
        "n_steps_30hz": n_v2,
        "n_omp_threads": n_cores,
    }

    # ═══════════════════════════════════════════════════════════════════════════
    #  v2t — static precompute + 2 OMP threads (robot-stack coexistence mode)
    # ═══════════════════════════════════════════════════════════════════════════
    print("\n── v2t: static precompute, 2 OMP threads (coexistence) ──")
    sensor_v2t = _make_lidar()
    sensor_v2t.set_scene(scene, n_omp_threads=2)
    for _ in range(20):
        sensor_v2t.step(state)

    hz_v2t, lat_v2t = _throughput(lambda: sensor_v2t.step(state))
    cpu_v2t, n_v2t, lat30_v2t = _cpu_under_load(
        lambda: sensor_v2t.step(state), target_hz
    )
    print(f"  Throughput: {hz_v2t:.0f} Hz  ({hz_v2t/target_hz:.0f}× margin over 30 Hz)")
    print(f"  Step p50/p99: {np.percentile(lat_v2t, 50):.3f} / {np.percentile(lat_v2t, 99):.3f} ms")
    print(f"  30 Hz CPU total: {sum(cpu_v2t):.1f}%  mean-core: {np.mean(cpu_v2t):.1f}%")
    results["v2t_precompute_2threads"] = {
        "throughput_hz": hz_v2t,
        "lat_p50_ms": float(np.percentile(lat_v2t, 50)),
        "lat_p99_ms": float(np.percentile(lat_v2t, 99)),
        "cpu_per_core_pct": cpu_v2t,
        "cpu_total_pct": sum(cpu_v2t),
        "n_steps_30hz": n_v2t,
        "n_omp_threads": 2,
    }

    # Measure segment prefilter effect at various positions
    prefilter_data = []
    if sensor_v2t._all_seg_sx is not None:
        total_segs = len(sensor_v2t._all_seg_sx)
        rmax_f = np.float32(30.0)
        for (px_pos, py_pos, label) in [
            (25.0, 15.0, "centre"),
            (5.0, 5.0, "corner"),
            (25.0, 15.0, "centre@10m"),  # 10m range
        ]:
            test_rmax = np.float32(10.0) if "10m" in label else rmax_f
            ox_f, oy_f = np.float32(px_pos), np.float32(py_pos)
            ax = sensor_v2t._all_seg_sx - ox_f
            ay = sensor_v2t._all_seg_sy - oy_f
            t = np.clip(
                -(ax * sensor_v2t._seg_dvx + ay * sensor_v2t._seg_dvy)
                / sensor_v2t._seg_len2_safe,
                0.0, 1.0,
            )
            closex = ax + t * sensor_v2t._seg_dvx
            closey = ay + t * sensor_v2t._seg_dvy
            kept = int(np.sum(closex * closex + closey * closey <= test_rmax * test_rmax))
            print(f"  Prefilter @{label}: {kept}/{total_segs} segments kept")
            prefilter_data.append({"label": label, "total": total_segs, "kept": kept,
                                    "range_m": float(test_rmax),
                                    "position": [px_pos, py_pos]})
        results["prefilter"] = prefilter_data

    # ═══════════════════════════════════════════════════════════════════════════
    #  Moving robot: compare v1 vs v2t with position changes forcing cache miss
    # ═══════════════════════════════════════════════════════════════════════════
    print("\n── Moving robot: cache-miss cost ──")
    # Move sensor in 5m steps (well beyond cache_thresh=range_max*0.05=1.5m)
    n_positions = 10
    positions = [
        np.array([5.0 + i * 4.0, 7.5 + (i % 3) * 5.0, 0.0])
        for i in range(n_positions)
    ]

    def _step_cycling(sensor, positions):
        pos = positions[int(time.perf_counter() * 3) % len(positions)]
        sensor.step(pos)

    sensor_mv1 = _make_lidar()
    sensor_mv1._attach_scene(scene)
    sensor_mv1._all_seg_sx = None   # force Shapely path
    sensor_mv1._cast_inplace = None
    for p in positions:  # warm up
        sensor_mv1.step(p)

    sensor_mv2t = _make_lidar()
    sensor_mv2t.set_scene(scene, n_omp_threads=2)
    for p in positions:
        sensor_mv2t.step(p)

    hz_mv1, lat_mv1 = _throughput(lambda: _step_cycling(sensor_mv1, positions))
    hz_mv2t, lat_mv2t = _throughput(lambda: _step_cycling(sensor_mv2t, positions))
    print(f"  v1 moving:  {hz_mv1:.0f} Hz  p50={np.percentile(lat_mv1, 50):.3f} ms")
    print(f"  v2t moving: {hz_mv2t:.0f} Hz  p50={np.percentile(lat_mv2t, 50):.3f} ms  "
          f"({hz_mv2t/hz_mv1:.1f}× faster)")
    results["moving_robot"] = {
        "v1_hz": hz_mv1,
        "v1_lat_p50_ms": float(np.percentile(lat_mv1, 50)),
        "v1_lat_p99_ms": float(np.percentile(lat_mv1, 99)),
        "v2t_hz": hz_mv2t,
        "v2t_lat_p50_ms": float(np.percentile(lat_mv2t, 50)),
        "v2t_lat_p99_ms": float(np.percentile(lat_mv2t, 99)),
        "speedup": hz_mv2t / hz_mv1,
    }

    # ── Speedup summary ───────────────────────────────────────────────────────
    print("\n═══ Summary ═══")
    cpu_saved_pct = 100 * (1 - sum(cpu_v2t) / sum(cpu_v1))
    print(f"  CPU@30Hz:  {sum(cpu_v1):.1f}% → {sum(cpu_v2t):.1f}%  "
          f"({cpu_saved_pct:.0f}% reduction)")
    print(f"  Headroom (2 threads): {hz_v2t/target_hz:.0f}× required rate")
    print(f"  Moving robot speedup: {hz_mv2t/hz_mv1:.1f}×")

    out_path = _HERE / "cpu_reduction_30hz_results.json"
    results["scene"] = {
        "n_objects": len(scene.objects),
        "n_linestrings_total": n_segs_total,
    }
    results["config"] = {
        "beams": 1500,
        "range_m": 30.0,
        "target_hz": target_hz,
    }
    out_path.write_text(json.dumps(results, indent=2))
    print(f"\nResults → {out_path}")
    return results


if __name__ == "__main__":
    main()
