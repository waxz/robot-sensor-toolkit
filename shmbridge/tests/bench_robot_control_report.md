# shmbridge robot-control-loop benchmark report

Generated from `bench_robot_control.py`, using
`bench_robot_control_results.json` (raw per-run data). Reproduce with:

```bash
python bench_robot_control.py --rates 50,100,250,500,1000 --duration 3 \
    --warmup 0.5 --poll-sleep-us 0,50,200 --backend both \
    --json bench_robot_control_results.json
```

## What this measures

`bench_robot_control.py` runs the exact publish/subscribe pattern a real
robot stack uses on top of `shmbridge.bridge_ext.ExtShmBridge`: a **sim**
process writes `RobotState` at a fixed control-loop rate, a **controller**
process reads it, runs a trivial control law, and writes a `RobotCmd` back;
the sim closes the loop by reading that command. Both roles run as separate
OS processes (`multiprocessing`, spawn), so the numbers include real
cross-process IPC and scheduling cost, not just an in-process call.

Two independent axes are swept:

- **backend** — the pure-Python ctypes implementation (`_PyExtShmBridge`)
  vs. the compiled C++ extension (`_CppExtShmBridge`), both reachable
  through the same `ExtShmBridge` name.
- **poll strategy** — the controller either busy-spins on `read_state()`
  (`--poll-sleep-us 0`) or sleeps a fixed amount between polls when no new
  state has arrived (`50`/`200`), trading latency for CPU.

Latency is measured with `time.perf_counter_ns()`, which reads the same
OS-wide monotonic clock in every process, so a timestamp taken in the sim
process and read back in the controller process (or vice versa) is directly
comparable with no clock-sync step needed.

## Test machine

Windows-11-10.0.26200-SP0, AMD64 (AuthenticAMD, 16 physical / 32 logical
cores), Python 3.12.12. 3s measured per run + 0.5s discarded warmup.

## Headline results (busy-spin, `poll_sleep=0`)

| rate (Hz) | backend | 1-way p50 | 1-way p99 | round-trip p50 | round-trip p99 | missed deadlines | sim CPU% | ctrl CPU% |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 50   | python | 14.4 µs | 30.3 µs | 23.0 µs | 50.7 µs | 0   | 3.1  | 100.0 |
| 50   | cpp    | 13.4 µs | 23.9 µs | 20.0 µs | 33.9 µs | 0   | 2.2  | 100.5 |
| 250  | python |  9.3 µs | 30.7 µs | 14.2 µs | 41.1 µs | 0   | 4.5  |  98.2 |
| 250  | cpp    | 10.6 µs | 37.6 µs | 15.3 µs | 48.3 µs | 0   | 4.9  | 100.0 |
| 1000 | python | 10.7 µs | 25.5 µs | 16.2 µs | 40.4 µs | **177** (~7.1%) | 14.7 | 96.1 |
| 1000 | cpp    |  8.0 µs | 17.0 µs | 11.2 µs | 26.2 µs | **6** (~0.24%)  |  9.8 | 100.0 |

(Full 5-rate x 3-poll x 2-backend sweep is in
`bench_robot_control_results.json`.)

## Findings

1. **Busy-spin latency is already excellent and backend-independent at low
   rates.** At 50-500 Hz, one-way latency is 8-15 µs median / under 40 µs
   p99 for *both* backends — the ctypes struct read/write overhead is not
   the bottleneck at these rates. If your control loop runs at 500 Hz or
   below, the pure-Python bridge is not measurably slower than the C++ one.

2. **The compiled backend matters at 1 kHz, and it matters for reliability,
   not raw latency.** Median/p99 latency at 1 kHz is only modestly better
   for cpp (8.0/17.0 µs vs. 10.7/25.5 µs), but the **missed-deadline rate**
   is the real story: the pure-Python bridge's higher per-call ctypes
   overhead makes the sim's write loop unable to hold a strict 1 ms period
   under scheduler noise, missing ~7% of ticks by more than half a period.
   The C++ backend misses ~0.24%. For a hard-real-time 1 kHz+ control loop,
   build the C++ extension (`pip install -e .` from `shmbridge/` with a C++
   compiler available); for anything at or below a few hundred Hz, the pure
   Python fallback is sufficient.

3. **`time.sleep()`-based polling is a poor low-latency/low-CPU compromise
   on this machine.** Any nonzero `--poll-sleep-us` (even 50 µs) collapses
   controller CPU from ~100% to single digits, but inflates round-trip
   latency by roughly 20-30x (from ~15-20 µs to 350-500 µs), because Windows
   does not honor sub-millisecond `time.sleep()` durations precisely — the
   actual delay is dominated by OS timer/scheduler granularity, not the
   requested value. If low CPU matters more than latency, sleep-based
   polling works; if both matter, a hybrid strategy (short busy-spin window,
   then fall back to sleep) would be needed to get closer to the busy-spin
   latency at a fraction of its CPU cost — not implemented here, but a
   natural next benchmark.

4. **Busy-spin costs one full core per role, as expected.** Controller CPU
   is pinned at ~96-100% in every busy-spin run regardless of rate, since it
   never blocks. Sim-side CPU under busy-spin scales with rate (2-15%
   observed) because higher rates mean less idle time between ticks even
   with a bounded spin-wait for the cmd echo.

5. **Round-trip latency is consistently ~1.4-2x one-way latency**, as
   expected for a symmetric two-hop exchange (state write→read, cmd
   write→read) using the same seqlock mechanism in both directions.

## Caveats

- All runs are same-host, same-machine; this does not measure NUMA effects,
  container/VM overhead, or a genuinely loaded system (other processes
  competing for CPU).
- The sim process's cmd-wait loop is a bounded busy-spin (up to 4x the
  target period) even when the controller itself sleeps between polls, so
  "sim CPU%" under nonzero `--poll-sleep-us` still reflects some spin cost,
  not a fully blocking wait.
- `--poll-sleep-us` only affects the *controller's* read loop; a
  from-scratch low-CPU benchmark of the *sim* side's wait strategy would
  need the same knob added there.
- Jitter (`period_actual_ns` stdev, 55-380 µs across runs, worst at 1 kHz
  spin) is dominated by Windows' default scheduler quantum and was not the
  focus of this run; a real-time control application should combine this
  bridge with a proper OS-level scheduling policy (e.g. `timeBeginPeriod`,
  a high-priority thread, or a real-time kernel on Linux) rather than
  relying on `time.sleep()` alone for timing precision.
