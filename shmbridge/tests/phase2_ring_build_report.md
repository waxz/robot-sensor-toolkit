# Ring transport — Phase 2 compile, correctness, and performance report

Generated while completing `docs/design_ring_zero_copy.md` §15 phase 2
("Overwrite semantics, local cursors, config, type/schema checks,
producer-conflict detection, latest-only read" — F-1, F-2, F-6, F-7, F-8,
F-10, F-11, F-13; R-1, R-2, R-4, R-6, R-7, R-11, R-12, R-13, R-14, R-16,
R-17). Every number in this report comes from a `ctest` run executed in
this session against the actual, current working tree (not a prior or
cached run) — see "Reproduce" below for the exact commands used on each
platform.

## What this covers

Phase 2 rewrote `include/shmbridge/ring.hpp` to the new design: runtime
`RingConfig` (replacing the old compile-time-`N` template parameter), the
64-byte `RingHeader` layout (`magic`/`type_hash`/`schema_version`/
`start_idx`/`end_idx`/`notify_seq`/`closed`/`producer_active`/
`producer_pid`), `claim_producer_slot()`'s CAS-based stale-PID takeover
(F-11), `pop_latest()` (F-13), and `resolve_ring_config()` backed by a
newly vendored `third_party/toml.hpp` (toml++ v3.4.0) for TOML-based
per-topic configuration (F-8). `tests/test_ring.cpp` grew from phase 1's
11 cases to 27, covering every item in this phase's exit criterion plus
the carried-forward phase 1/zero-copy/resilience/latency suites.

| File | Change |
|---|---|
| `include/shmbridge/ring.hpp` | Rewritten: `RingConfig`, new `RingHeader`, `claim_producer_slot()`, `pop_latest()`, `resolve_ring_config()` |
| `include/shmbridge/platform.hpp` | + `current_pid()`, `process_alive()` (producer-conflict detection), `spawn_and_reap_process()` (spawns and reaps a real OS process so tests can simulate a crashed producer with a genuinely dead PID — a `std::thread`'s PID is always its live parent's) |
| `include/third_party/toml.hpp` | New — vendored toml++ v3.4.0 single header |
| `tests/test_ring.cpp` | 27 GTest cases across 12 suites (`RingBasic`, `RingZeroCopy`, `RingMisuseGuards`, `RingFreshness`, `RingConfigResolution`, `RingTypeSafety`, `RingResilience`, `RingLatestOnly`, `RingProducerConflict`, `RingBlockingWait`, `RingLatencyDistribution`) |
| `tests/test_ring_results.json` | Regenerated — this report's Windows/MSVC numbers come directly from this checked-in file |

## Correctness — was it built to the specification?

All 27 cases map onto §11.1's test table; none were skipped or disabled
on any platform.

| Suite | Cases | Verifies |
|---|---|---|
| `RingBasic` | 4 | Push/pop round trip, overflow/eviction resync, multi-consumer independence, `drain_ex()` |
| `RingZeroCopy` | 2 | `reserve()`/`commit()` and `borrow()`/`end_borrow()` match `push()`/`pop_ex()` byte-for-byte |
| `RingMisuseGuards` | 2 | Reserve/commit and borrow/end_borrow misuse (F-14) are safe no-ops, not corruption |
| `RingFreshness` | 1 | `is_stale()` boundary correctness |
| `RingConfigResolution` | 2 | `[ring."<topic>"]` > `[ring.default]` > built-in precedence; missing-file fallback |
| `RingTypeSafety` | 1 | Mismatched `type_hash` throws `std::invalid_argument`, not silently accepted (R-7) |
| `RingResilience` | 3 | `try_attach()` before any publisher, `TimeoutError` when none appears, reattach-after-close gets a fresh cursor (R-9) |
| `RingLatestOnly` | 1 | `pop_latest()` jumps to the newest entry, cursor lands exactly at `end_idx` |
| `RingProducerConflict` | 4 | Second `open()` by a live producer throws; `close()`+`open()` succeeds; a dead producer's stale PID is taken over; a genuine two-thread takeover race has exactly one winner (R-17) |
| `RingBlockingWait` | 4 | `pop_wait()` fast path, 5-call back-to-back fast path, realistic 10ms timeout, `close()` wake latency |
| `RingLatencyDistribution` | 3 | 200-trial p50/p90/p99/max distributions for push-to-wake, close-to-wake, and immediate-read latency |

**Result: 27/27 pass on every toolchain exercised**, run fresh in this
session against the current working tree:

| Toolchain | Build type | Result | Notes |
|---|---|---|---|
| Native Windows, MSVC 19.50.35725.0 (Visual Studio 2026, `cl.exe`/Ninja) | Release, plain | 27/27 pass, 5.90s | This project's canonical Windows verification toolchain (MinGW is no longer part of the verification loop — a standing correction applied earlier in this phase) |
| WSL Ubuntu 24.04, GCC 13.3.0 | Release, plain | 27/27 pass, 0.94s | Linux futex notify path |
| WSL Ubuntu 24.04, GCC 13.3.0 | AddressSanitizer+UndefinedBehaviorSanitizer | 27/27 pass, 1.00s | Zero memory-safety/UB findings |
| WSL Ubuntu 24.04, GCC 13.3.0 | ThreadSanitizer (`setarch $(uname -m) -R`) | 27/27 pass, 1.00s | Zero data races |

No regressions, no flaky reruns needed. (Two real test-design bugs — a
race-barrier timing issue in `ConcurrentTakeoverRaceHasExactlyOneWinner`
and an over-tight OS-jitter-sensitive bound in
`PopWaitTimesOutWhenNoDataArrives` — were found and fixed earlier in this
phase, before the runs in this report; both are stable here.)

## Performance — NFR-6 (wake-path latency)

NFR-6 requires p99 wake-path latency to leave at least 90% headroom
against a 1kHz control loop's 1ms period. Measured via the 200-trial
`RingLatencyDistribution` suite, read directly from each platform's
`--gtest_output=json` artifact (`tests/test_ring_results.json` for
Windows; `/tmp/wsl_test_ring_results.json` for WSL, not checked in since
the committed artifact is the Windows run per existing convention):

| Path | Platform | mean | p50 | p90 | p99 | max | n | p99 headroom vs. 1ms |
|---|---|---|---|---|---|---|---|---|
| Push-to-wake | Windows/MSVC | 14.9µs | 14.3µs | 23.7µs | 33.2µs | 49.1µs | 200 | 96.7% |
| Push-to-wake | WSL/GCC (futex) | 33.5µs | 34.1µs | 38.3µs | 63.7µs | 105.3µs | 200 | 93.6% |
| Close-to-wake | Windows/MSVC | 14.4µs | 12.7µs | 24.8µs | 36.5µs | 41.8µs | 200 | 96.4% |
| Close-to-wake | WSL/GCC (futex) | 29.8µs | 29.0µs | 36.1µs | 45.6µs | 62.4µs | 200 | 95.4% |
| Immediate read (fast path, no wait) | Windows/MSVC | ~0µs | 0.0µs | 0.1µs | 0.1µs | 0.1µs | 200 | n/a (no wait call) |
| Immediate read (fast path, no wait) | WSL/GCC | ~0µs | 0.0µs | 0.0µs | 0.0µs | 0.0µs | 200 | n/a (no wait call) |

**NFR-6 met on both platforms**, each with comfortable margin above the
90% headroom requirement (93.6%–96.7%). The Linux futex path is slower
than Windows' named-Event path at every percentile here (consistent with
the different underlying wake primitives — expected, not a regression),
but neither is close to the 1ms budget. As in phase 1 (0.27), this is a
soft-real-time, p99-class commitment, not a hard worst-case guarantee —
no general-purpose OS can promise the latter for a user-space wake path.

Other timing data points gathered in the same runs, for context rather
than against a specific NFR:

- The realistic 10ms `pop_wait()` timeout returns in 14.3ms on Windows
  (consistent with Windows' ~15.6ms timer-tick rounding, §8) and 10.1ms
  on WSL/Linux (no comparable rounding on this path).
- Immediate-read latency is sub-microsecond on both platforms — the fast
  path genuinely never reaches the OS wait primitive.

## Reproduce

```powershell
# Windows / MSVC (Visual Studio 2026) — this project's canonical Windows toolchain
& "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
git clone --depth 1 --branch v1.14.0 https://github.com/google/googletest.git
cmake -S googletest -B gtest_build -G Ninja -DCMAKE_BUILD_TYPE=Release -Dgtest_force_shared_crt=OFF
cmake --build gtest_build --config Release
cmake --install gtest_build --prefix gtest_install

cmake -S shmbridge -B build -G Ninja -DCMAKE_BUILD_TYPE=Release `
    -DSHMBRIDGE_BUILD_TESTS=ON -DGTEST_ROOT=gtest_install `
    -DCMAKE_PREFIX_PATH=gtest_install -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

```bash
# WSL / Linux (GCC) — plain, then with sanitizers
git clone --depth 1 --branch v1.14.0 https://github.com/google/googletest.git
cmake -S googletest -B gtest_build -DCMAKE_BUILD_TYPE=Release
cmake --build gtest_build -j"$(nproc)"
cmake --install gtest_build --prefix gtest_install

cmake -S shmbridge -B build -DCMAKE_BUILD_TYPE=Release -DSHMBRIDGE_BUILD_TESTS=ON \
    -DGTEST_ROOT=gtest_install -DCMAKE_PREFIX_PATH=gtest_install
cmake --build build -j"$(nproc)"
ctest --test-dir build -R test_ring --output-on-failure

# Repeat with -DSHMBRIDGE_SANITIZE=address,undefined and =thread
# (thread requires: setarch $(uname -m) -R ctest ...)
```

Note: this run built against a `rsync`'d copy of the live working tree
(`shmbridge`'s uncommitted phase 2 changes), not a fresh `git clone` of
HEAD — HEAD does not yet contain this phase's code. A plain
`git clone`/checkout will reproduce this report only once phase 2 is
committed.
