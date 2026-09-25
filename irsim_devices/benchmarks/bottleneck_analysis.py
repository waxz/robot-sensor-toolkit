"""
bottleneck_analysis.py — LiDAR 2D workflow bottleneck analysis.

Profiles each sub-component of _step_fast and evaluates:
  1. Per-component time breakdown (directions / prefilter / take / kernel / copy)
  2. Old allocating prefilter (v2t) vs new in-place prefilter with pose cache (v3)
  3. Pose-cache effectiveness at different robot speeds
  4. OMP thread count sweet spot
  5. CPU utilisation at 30 Hz for all variants

Results written to bottleneck_analysis_results.json.
"""

from __future__ import annotations

import json
import sys
import time
from math import cos, pi, sin
from pathlib import Path
from time import perf_counter

import numpy as np
import psutil

_HERE = Path(__file__).parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO / "src"))

from irsim_devices.core.open3d_scene_2d import Open3DScene2D  # noqa: E402
from irsim_devices.sensors.lidar2d import Lidar2D  # noqa: E402

# ── Scene ──────────────────────────────────────────────────────────────────────
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
            scene.add_box(
                center=(rx + RACK_LENGTH / 2, ry),
                width=RACK_LENGTH / 2,
                height=RACK_WIDTH / 2,
            )
    return scene


def _make_lidar(state=(25.0, 15.0, 0.0), n_beams=1500, range_max=30.0) -> Lidar2D:
    return Lidar2D(
        state=np.array([[state[0]], [state[1]], [state[2]]]),
        range_max=range_max,
        angle_range=2 * pi,
        number=n_beams,
        noise=False,
    )


# ── Helpers ───────────────────────────────────────────────────────────────────


def _pct(arr, p):
    return float(np.percentile(arr, p))


def _cpu_at_hz(lidar, states, hz=30.0, duration=5.0, desc="") -> float:
    """Return mean process CPU% when stepping lidar at exactly hz for duration seconds."""
    proc = psutil.Process()
    interval = 1.0 / hz
    proc.cpu_percent()  # prime
    time.sleep(0.2)
    proc.cpu_percent()
    t_end = time.monotonic() + duration
    n = 0
    while time.monotonic() < t_end:
        s = states[n % len(states)]
        t0 = time.monotonic()
        lidar.step(s)
        elapsed = time.monotonic() - t0
        remaining = interval - elapsed
        if remaining > 0:
            time.sleep(remaining)
        n += 1
    cpu = proc.cpu_percent()
    return cpu


# ══════════════════════════════════════════════════════════════════════════════
# 1. Per-component timing
# ══════════════════════════════════════════════════════════════════════════════


def profile_components(
    lidar: Lidar2D, ox=25.0, oy=15.0, theta=0.0, n_runs=2000
) -> dict:
    """Time each sub-component of _step_fast in isolation."""
    from irsim_devices.core.ray_casting_2d_omp import cast_ray_segments_avx2_f32_inplace

    rmax_f = np.float32(lidar.range_max)
    ox_f, oy_f = np.float32(ox), np.float32(oy)

    # Force a cache miss first to populate work buffers
    lidar._filter_cache_x = np.inf
    lidar._step_fast(ox, oy, theta)

    # Pre-determine M_filt (from what's cached after the above call)
    M_filt = lidar._filter_cache_M

    t_dir, t_filter_alloc, t_filter_inplace, t_nonzero_take, t_kernel, t_copy = (
        [],
        [],
        [],
        [],
        [],
        [],
    )

    for _ in range(n_runs):
        # ── Direction rotation (in-place) ─────────────────────────────────────
        t0 = perf_counter()
        cw_f = np.float32(cos(theta))
        sw_f = np.float32(sin(theta))
        np.multiply(lidar._local_dir_cos_f32, cw_f, out=lidar._dir_dx_f32)
        np.multiply(lidar._local_dir_sin_f32, sw_f, out=lidar._tmp_f32)
        np.subtract(lidar._dir_dx_f32, lidar._tmp_f32, out=lidar._dir_dx_f32)
        np.multiply(lidar._local_dir_cos_f32, sw_f, out=lidar._dir_dy_f32)
        np.multiply(lidar._local_dir_sin_f32, cw_f, out=lidar._tmp_f32)
        np.add(lidar._dir_dy_f32, lidar._tmp_f32, out=lidar._dir_dy_f32)
        lidar._origin_f32[0] = ox_f
        lidar._origin_f32[1] = oy_f
        t_dir.append((perf_counter() - t0) * 1e6)

        # ── Prefilter: OLD allocating version ────────────────────────────────
        t0 = perf_counter()
        ax = lidar._all_seg_sx - ox_f
        ay = lidar._all_seg_sy - oy_f
        t_param = np.clip(
            -(ax * lidar._seg_dvx + ay * lidar._seg_dvy) / lidar._seg_len2_safe,
            np.float32(0.0),
            np.float32(1.0),
        )
        px = ax + t_param * lidar._seg_dvx
        py = ay + t_param * lidar._seg_dvy
        _mask_alloc = px * px + py * py <= rmax_f * rmax_f
        t_filter_alloc.append((perf_counter() - t0) * 1e6)

        # ── Prefilter: NEW in-place version ──────────────────────────────────
        t0 = perf_counter()
        rmax_m = rmax_f + np.float32(lidar._filter_margin)
        rmax_m_sq = rmax_m * rmax_m
        np.subtract(lidar._all_seg_sx, ox_f, out=lidar._pf_ax)
        np.subtract(lidar._all_seg_sy, oy_f, out=lidar._pf_ay)
        np.multiply(lidar._pf_ax, lidar._seg_dvx, out=lidar._pf_t)
        np.multiply(lidar._pf_ay, lidar._seg_dvy, out=lidar._pf_px)
        np.add(lidar._pf_t, lidar._pf_px, out=lidar._pf_t)
        np.negative(lidar._pf_t, out=lidar._pf_t)
        np.divide(lidar._pf_t, lidar._seg_len2_safe, out=lidar._pf_t)
        np.clip(lidar._pf_t, np.float32(0.0), np.float32(1.0), out=lidar._pf_t)
        np.multiply(lidar._pf_t, lidar._seg_dvx, out=lidar._pf_px)
        np.add(lidar._pf_ax, lidar._pf_px, out=lidar._pf_px)
        np.multiply(lidar._pf_t, lidar._seg_dvy, out=lidar._pf_py)
        np.add(lidar._pf_ay, lidar._pf_py, out=lidar._pf_py)
        np.multiply(lidar._pf_px, lidar._pf_px, out=lidar._pf_t)
        np.multiply(lidar._pf_py, lidar._pf_py, out=lidar._pf_ay)
        np.add(lidar._pf_t, lidar._pf_ay, out=lidar._pf_t)
        np.less_equal(lidar._pf_t, rmax_m_sq, out=lidar._pf_mask)
        t_filter_inplace.append((perf_counter() - t0) * 1e6)

        # ── np.nonzero + np.take (cache miss cost) ────────────────────────────
        t0 = perf_counter()
        idx = np.nonzero(lidar._pf_mask)[0]
        Mf = len(idx)
        if Mf > 0:
            np.take(lidar._all_seg_sx, idx, out=lidar._work_seg_sx[:Mf])
            np.take(lidar._all_seg_sy, idx, out=lidar._work_seg_sy[:Mf])
            np.take(lidar._all_seg_ex, idx, out=lidar._work_seg_ex[:Mf])
            np.take(lidar._all_seg_ey, idx, out=lidar._work_seg_ey[:Mf])
        t_nonzero_take.append((perf_counter() - t0) * 1e6)

        # ── Kernel ─────────────────────────────────────────────────────────────
        t0 = perf_counter()
        cast_ray_segments_avx2_f32_inplace(
            lidar._origin_f32,
            lidar._dir_dx_f32,
            lidar._dir_dy_f32,
            lidar._work_seg_sx[:M_filt],
            lidar._work_seg_sy[:M_filt],
            lidar._work_seg_ex[:M_filt],
            lidar._work_seg_ey[:M_filt],
            float(rmax_f),
            lidar._out_ranges_f32,
            lidar._out_hit_i32,
        )
        t_kernel.append((perf_counter() - t0) * 1e6)

        # ── range_data copy ────────────────────────────────────────────────────
        t0 = perf_counter()
        lidar.range_data[:] = lidar._out_ranges_f32
        t_copy.append((perf_counter() - t0) * 1e6)

    return {
        "M_all": int(len(lidar._all_seg_sx)),
        "M_filt": int(M_filt),
        "directions_us": {"p50": _pct(t_dir, 50), "p95": _pct(t_dir, 95)},
        "filter_alloc_us": {
            "p50": _pct(t_filter_alloc, 50),
            "p95": _pct(t_filter_alloc, 95),
        },
        "filter_inplace_us": {
            "p50": _pct(t_filter_inplace, 50),
            "p95": _pct(t_filter_inplace, 95),
        },
        "nonzero_take_us": {
            "p50": _pct(t_nonzero_take, 50),
            "p95": _pct(t_nonzero_take, 95),
        },
        "kernel_us": {"p50": _pct(t_kernel, 50), "p95": _pct(t_kernel, 95)},
        "copy_us": {"p50": _pct(t_copy, 50), "p95": _pct(t_copy, 95)},
    }


# ══════════════════════════════════════════════════════════════════════════════
# 2. Thread-count sweet spot
# ══════════════════════════════════════════════════════════════════════════════


def benchmark_threads(scene, n_beams=1500, range_max=30.0, n_runs=500) -> dict:
    """Measure kernel+overhead time as OMP thread count varies 1–4."""
    from irsim_devices.core.ray_casting_2d_omp import set_omp_threads

    results = {}
    for n_threads in [1, 2, 3, 4]:
        set_omp_threads(n_threads)
        lidar = _make_lidar(n_beams=n_beams, range_max=range_max)
        lidar.set_scene(scene, n_omp_threads=n_threads, filter_margin=0.0)

        # Warm up
        s = np.array([[25.0], [15.0], [0.0]])
        for _ in range(50):
            lidar.step(s)

        times = []
        for _ in range(n_runs):
            t0 = perf_counter()
            lidar.step(s)
            times.append((perf_counter() - t0) * 1e6)

        results[f"t{n_threads}"] = {
            "p50_us": _pct(times, 50),
            "p95_us": _pct(times, 95),
            "throughput_hz": 1e6 / _pct(times, 50),
        }
    return results


# ══════════════════════════════════════════════════════════════════════════════
# 3. End-to-end benchmark: v2t (old) vs v3 at various robot speeds
# ══════════════════════════════════════════════════════════════════════════════


def benchmark_e2e(scene, n_beams=1500, range_max=30.0, n_steps=3000, hz=30.0) -> dict:
    """Benchmark v2t vs v3 for stationary and various movement speeds."""
    results = {}

    # ── Common trajectory helpers ─────────────────────────────────────────────
    def _straight_traj(speed_mps, n_steps):
        """States for straight-line motion at speed_mps (dt = 1/hz)."""
        dt = 1.0 / hz
        states = []
        x, y = 5.0, 15.0
        for _i in range(n_steps):
            theta = 0.0
            states.append(np.array([[x], [y], [theta]]))
            x += speed_mps * dt
            if x > W - 2.0:
                x, y = 5.0, 15.0
        return states

    speeds = {
        "stationary": 0.0,
        "slow_0.5mps": 0.5,
        "medium_1.0mps": 1.0,
        "fast_2.0mps": 2.0,
    }

    for label, speed in speeds.items():
        states = _straight_traj(speed, n_steps)

        # ── v2t: old allocating prefilter, no pose cache ──────────────────────
        # We simulate v2t by calling with filter_margin=0 (always refilter)
        lidar_v2t = _make_lidar(n_beams=n_beams, range_max=range_max)
        lidar_v2t.set_scene(scene, n_omp_threads=2, filter_margin=0.0)
        # warm up
        for s in states[:100]:
            lidar_v2t.step(s)
        times_v2t = []
        for s in states:
            t0 = perf_counter()
            lidar_v2t.step(s)
            times_v2t.append((perf_counter() - t0) * 1e6)

        # ── v3: in-place prefilter + 1m pose cache ────────────────────────────
        lidar_v3 = _make_lidar(n_beams=n_beams, range_max=range_max)
        lidar_v3.set_scene(scene, n_omp_threads=2, filter_margin=1.0)
        for s in states[:100]:
            lidar_v3.step(s)
        times_v3 = []
        cache_misses = 0
        for s in states:
            prev_fx = lidar_v3._filter_cache_x
            t0 = perf_counter()
            lidar_v3.step(s)
            times_v3.append((perf_counter() - t0) * 1e6)
            if lidar_v3._filter_cache_x != prev_fx:
                cache_misses += 1

        hit_rate = 1.0 - cache_misses / len(states)

        results[label] = {
            "speed_mps": speed,
            "v2t": {
                "p50_us": _pct(times_v2t, 50),
                "p95_us": _pct(times_v2t, 95),
                "throughput_hz": 1e6 / _pct(times_v2t, 50),
            },
            "v3": {
                "p50_us": _pct(times_v3, 50),
                "p95_us": _pct(times_v3, 95),
                "throughput_hz": 1e6 / _pct(times_v3, 50),
                "cache_hit_rate": hit_rate,
            },
        }

    return results


# ══════════════════════════════════════════════════════════════════════════════
# 4. CPU at 30 Hz
# ══════════════════════════════════════════════════════════════════════════════


def benchmark_cpu(scene, n_beams=1500, range_max=30.0, hz=30.0, duration=5.0) -> dict:
    speeds = {
        "stationary": 0.0,
        "medium_1.0mps": 1.0,
        "fast_2.0mps": 2.0,
    }
    dt = 1.0 / hz
    results = {}

    for label, speed in speeds.items():
        # Build trajectory
        x, y = 5.0, 15.0
        states_v2t, states_v3 = [], []
        for _ in range(int(duration * hz) + 10):
            s = np.array([[x], [y], [0.0]])
            states_v2t.append(s)
            states_v3.append(s)
            x += speed * dt
            if x > W - 2.0:
                x, y = 5.0, 15.0

        lidar_v2t = _make_lidar(n_beams=n_beams, range_max=range_max)
        lidar_v2t.set_scene(scene, n_omp_threads=2, filter_margin=0.0)
        cpu_v2t = _cpu_at_hz(lidar_v2t, states_v2t, hz=hz, duration=duration)

        lidar_v3 = _make_lidar(n_beams=n_beams, range_max=range_max)
        lidar_v3.set_scene(scene, n_omp_threads=2, filter_margin=1.0)
        cpu_v3 = _cpu_at_hz(lidar_v3, states_v3, hz=hz, duration=duration)

        results[label] = {
            "v2t_cpu_pct": cpu_v2t,
            "v3_cpu_pct": cpu_v3,
            "reduction_pct": cpu_v2t - cpu_v3,
        }

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════


def main() -> None:
    print("Building warehouse scene…")
    scene = _make_scene()
    n_segs = sum(len(getattr(obj, "linestrings", []) or []) for obj in scene.objects)
    print(f"  {len(scene.objects)} objects, ≈{n_segs} linestrings/polys")

    lidar_probe = _make_lidar()
    lidar_probe.set_scene(scene, n_omp_threads=2, filter_margin=1.0)
    s0 = np.array([[25.0], [15.0], [0.0]])
    lidar_probe.step(s0)
    M_all = int(len(lidar_probe._all_seg_sx))
    print(f"  {M_all} total segments after precompute")

    print("\n[1/4] Per-component profile (2000 iterations, centre, 2 OMP threads)…")
    lidar_comp = _make_lidar()
    lidar_comp.set_scene(scene, n_omp_threads=2, filter_margin=1.0)
    lidar_comp.step(s0)
    comp = profile_components(lidar_comp, ox=25.0, oy=15.0, theta=0.0)
    print(f"  directions:     {comp['directions_us']['p50']:.1f} µs p50")
    print(f"  filter alloc:   {comp['filter_alloc_us']['p50']:.1f} µs p50")
    print(f"  filter inplace: {comp['filter_inplace_us']['p50']:.1f} µs p50")
    print(f"  nonzero+take:   {comp['nonzero_take_us']['p50']:.1f} µs p50")
    print(f"  kernel (2T):    {comp['kernel_us']['p50']:.1f} µs p50")
    print(f"  copy:           {comp['copy_us']['p50']:.1f} µs p50")
    miss_total = (
        comp["directions_us"]["p50"]
        + comp["filter_inplace_us"]["p50"]
        + comp["nonzero_take_us"]["p50"]
        + comp["kernel_us"]["p50"]
        + comp["copy_us"]["p50"]
    )
    hit_total = (
        comp["directions_us"]["p50"] + comp["kernel_us"]["p50"] + comp["copy_us"]["p50"]
    )
    print(
        f"  ── total cache miss: {miss_total:.1f} µs  | cache hit: {hit_total:.1f} µs"
    )
    comp["estimated_step_miss_us"] = round(miss_total, 2)
    comp["estimated_step_hit_us"] = round(hit_total, 2)

    print("\n[2/4] OMP thread sweet spot (1–4 threads, 500 iters)…")
    threads = benchmark_threads(scene)
    for k, v in threads.items():
        print(
            f"  {k}: p50={v['p50_us']:.1f} µs  throughput={v['throughput_hz']:.0f} Hz"
        )

    print("\n[3/4] End-to-end: v2t vs v3 at various speeds (3000 steps each)…")
    e2e = benchmark_e2e(scene, n_steps=3000, hz=30.0)
    for label, r in e2e.items():
        print(
            f"  {label:20s}  v2t p50={r['v2t']['p50_us']:.1f}µs"
            f"  v3 p50={r['v3']['p50_us']:.1f}µs"
            f"  hit={r['v3']['cache_hit_rate'] * 100:.1f}%"
        )

    print("\n[4/4] CPU @ 30 Hz, 5 s each…")
    cpu = benchmark_cpu(scene, hz=30.0, duration=5.0)
    for label, r in cpu.items():
        print(
            f"  {label:22s}  v2t={r['v2t_cpu_pct']:.1f}%  v3={r['v3_cpu_pct']:.1f}%"
            f"  Δ={r['reduction_pct']:+.1f}%"
        )

    out = {
        "scene": {"n_objects": len(scene.objects), "M_all": M_all},
        "lidar": {"n_beams": 1500, "range_max": 30.0, "hz": 30.0},
        "components": comp,
        "threads": threads,
        "e2e": e2e,
        "cpu": cpu,
    }
    out_path = _HERE / "bottleneck_analysis_results.json"
    out_path.write_text(json.dumps(out, indent=2))
    print(f"\nResults → {out_path}")


if __name__ == "__main__":
    main()
