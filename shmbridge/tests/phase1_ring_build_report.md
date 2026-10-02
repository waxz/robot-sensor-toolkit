# Ring transport — Phase 1 compile and execution report

Generated while completing `docs/design_ring_zero_copy.md` §15 phase 1
("Build system + notify wiring"). Reproduce with:

```powershell
# One-time: a local GTest build, since neither CMake nor GTest ships with
# this host by default.
git clone --depth 1 --branch v1.14.0 https://github.com/google/googletest.git
cmake -S googletest -B gtest_build -G Ninja -DCMAKE_BUILD_TYPE=Release -Dgtest_force_shared_crt=ON
cmake --build gtest_build -j
cmake --install gtest_build --prefix gtest_install

# shmbridge itself
cmake -S shmbridge -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo `
    -DGTEST_ROOT=<path-to-gtest_install> -DSHMBRIDGE_BUILD_TESTS=ON `
    -DSHMBRIDGE_BUILD_BENCH=OFF -DSHMBRIDGE_NATIVE_ARCH=OFF -DSHMBRIDGE_LTO=OFF
cmake --build build --target test_ring
ctest --test-dir build --output-on-failure
```

`ctest` runs `test_ring` with `--gtest_output=json:shmbridge/tests/test_ring_results.json`
(wired into `CMakeLists.txt`'s `add_test()` call) — every number in this
report's tables comes directly from that file's per-test `properties`
(`RecordProperty()` calls in `test_ring.cpp`), not a hand-transcription
off the console. The file itself is checked in at
`shmbridge/tests/test_ring_results.json` as the most recent run's
artifact, the same convention `bench_pubsub_matrix_results.json` already
uses in this directory.

## What this covers

Phase 1 touched four files: `CMakeLists.txt` (test build gate),
`include/shmbridge/ring.hpp` (migrated off raw POSIX shm calls, added
`notify_seq`/`pop_wait()`/`drain_wait()`), `include/shmbridge/platform.hpp`
(added the Windows named-Event mechanism fixing R-18), and
`include/shmbridge/topic.hpp` (wired to the same `platform.hpp` fix), plus
the new `tests/test_ring.cpp`. This report is the first time all of that
has gone through the actual `cmake --build` + `ctest` pipeline rather than
a standalone compile of equivalent logic (see
`design_ring_zero_copy.md` revision 0.22).

| File | Change |
|---|---|
| `CMakeLists.txt` | +54/-23 — new platform-independent `test_ring` target (now passing `--gtest_output=json:...` so every `ctest` run regenerates `tests/test_ring_results.json`); existing POSIX/`fork()`-dependent targets moved under their own `if(UNIX)` guard |
| `include/shmbridge/ring.hpp` | +144/-58 — `platform.hpp`-based shm lifecycle, `notify_seq`, `pop_wait()`/`drain_wait()` |
| `include/shmbridge/platform.hpp` | +147/-4 — Windows named-Event notify mechanism (`notify_bind`/`notify_unbind`), fixing R-18 |
| `include/shmbridge/topic.hpp` | +49/-4 — wired to the same `platform.hpp` fix |
| `tests/test_ring.cpp` | 11 GTest cases (8 single-sample + 3 `RingLatencyDistribution` 200-trial p50/p90/p99/max cases); records real microsecond-precision latency as GTest test properties (`RecordProperty()`) so `--gtest_output=json` carries real numbers, not just console-rounded milliseconds |
| `tests/test_ring_results.json` | new — the most recent `ctest` run's machine-readable report, checked in the same way `bench_pubsub_matrix_results.json` already is |

## Test machine

Windows 10.0.26200, AMD Ryzen 9 8945HX (16 cores / 32 threads), MinGW
GCC 13.1.0 + Ninja 1.12.1 (CLion 2025.3.3's bundled toolchain), CMake
4.1.2, GoogleTest 1.14.0 (built locally — not preinstalled on this host).
No CI configured yet; phase 9 (§15) covers that. WSL Ubuntu has not yet
been exercised against this code — tracked as an open item in
`design_ring_zero_copy.md` §16.

## Compile

`cmake --build build --target test_ring -v` — full command lines:

```
G++ -DSB_ENABLE_NOTIFY=1 -I.../shmbridge/include -isystem .../gtest_install/include
    -O2 -g -DNDEBUG -std=c++17 -O2 -g -Wall -Wextra -O3 -funroll-loops
    -c tests/test_ring.cpp -o CMakeFiles/test_ring.dir/tests/test_ring.cpp.obj

G++ -O2 -g -DNDEBUG CMakeFiles/test_ring.dir/tests/test_ring.cpp.obj
    -o test_ring.exe -Wl,--out-implib,libtest_ring.dll.a
    .../gtest_install/lib/libgtest_main.a -lSynchronization
    .../gtest_install/lib/libgtest.a
    -lkernel32 -luser32 -lgdi32 -lwinspool -lshell32 -lole32 -loleaut32
    -luuid -lcomdlg32 -ladvapi32
```

**Result: clean compile, 0 errors.** One pre-existing warning, unrelated
to phase 1's changes (present in `platform.hpp` before this work):

```
include/shmbridge/platform.hpp:248:46: warning: unused parameter 'size'
  [-Wunused-parameter]
  inline void shm_unmap(void* ptr, std::size_t size) noexcept {
```

`-Wall -Wextra` was active for `test_ring.cpp` itself (`/W4` is configured
for MSVC builds; this build used MinGW/GCC) — no warnings from the new
test file or from `ring.hpp`'s/`platform.hpp`'s changed code.

## Execution

`test_ring.exe` run directly, and via `ctest`, both pass:

```
Running main() from .../gtest_main.cc
[==========] Running 11 tests from 3 test suites.
[----------] 2 tests from RingBasic
[ RUN      ] RingBasic.PushPopRoundTrip
[       OK ] RingBasic.PushPopRoundTrip (0 ms)
[ RUN      ] RingBasic.FullRingRejectsPush
[       OK ] RingBasic.FullRingRejectsPush (0 ms)
[----------] 2 tests from RingBasic (0 ms total)

[----------] 6 tests from RingBlockingWait
[ RUN      ] RingBlockingWait.PopWaitReturnsImmediatelyWhenDataAlreadyPresent
[       OK ] RingBlockingWait.PopWaitReturnsImmediatelyWhenDataAlreadyPresent (0 ms)
[ RUN      ] RingBlockingWait.PopWaitReturnsImmediatelyForEveryAlreadyPublishedItem
[       OK ] RingBlockingWait.PopWaitReturnsImmediatelyForEveryAlreadyPublishedItem (0 ms)
[ RUN      ] RingBlockingWait.PopWaitTimesOutWhenNoDataArrives
[       OK ] RingBlockingWait.PopWaitTimesOutWhenNoDataArrives (16 ms)
[ RUN      ] RingBlockingWait.PopWaitObservesConcurrentPublishPromptly
[       OK ] RingBlockingWait.PopWaitObservesConcurrentPublishPromptly (15 ms)
[ RUN      ] RingBlockingWait.CloseWakesBlockedReaderNearImmediately
[       OK ] RingBlockingWait.CloseWakesBlockedReaderNearImmediately (17 ms)
[ RUN      ] RingBlockingWait.DrainWaitConsumesEverythingAvailable
[       OK ] RingBlockingWait.DrainWaitConsumesEverythingAvailable (0 ms)
[----------] 6 tests from RingBlockingWait (49 ms total)

[----------] 3 tests from RingLatencyDistribution
[ RUN      ] RingLatencyDistribution.ImmediateReadDistribution
[       OK ] RingLatencyDistribution.ImmediateReadDistribution (4 ms)
[ RUN      ] RingLatencyDistribution.PushToWakeLatencyDistribution
[       OK ] RingLatencyDistribution.PushToWakeLatencyDistribution (3267 ms)
[ RUN      ] RingLatencyDistribution.CloseToWakeLatencyDistribution
[       OK ] RingLatencyDistribution.CloseToWakeLatencyDistribution (3172 ms)
[----------] 3 tests from RingLatencyDistribution (6445 ms total)

[==========] 11 tests from 3 test suites ran. (6503 ms total)
[  PASSED  ] 11 tests.
```

(`RingLatencyDistribution`'s 200-trial tests dominate runtime — ~3.2s
each, from 200 × ~15-16ms per trial including the thread-spawn/sleep/join
overhead each trial incurs, not from anything slow in the ring itself.)

```
      Start 1: test_ring
1/1 Test #1: test_ring ........................   Passed    6.55 sec
100% tests passed, 0 tests failed out of 1
```

(Numbers above are from the final revision of `test_ring.cpp`, after the
timeout was shrunk to 10ms, the R-3/R-5 cases were changed to measure
isolated wake latency, the cold-start warm-up fix below was added, and
the 3 `RingLatencyDistribution` 200-trial cases were added — superseding
the 7-test/254ms figures an earlier revision of this report showed.)

### Timing-sensitive case stability (7 runs, real measured values)

A robotics control loop can't tolerate the 100ms-class timeout and
50-67ms wake latency the first version of this report showed — those
numbers mixed test scaffolding delay (an artificial `sleep_for` before
the producer/closer thread acts) with actual wake latency, and were
rounded to GTest's own millisecond-granularity console timer rather than
measured directly. Both are fixed now: the timeout request shrunk to
10ms, R-3/R-5 measure latency from the *actual* `push()`/`close()` call
to `pop_wait()` returning (excluding each test's artificial 10ms
pre-action delay), and every value below is read directly out of
`test_ring_results.json`'s `RecordProperty()` fields — microsecond
precision, not a console-rounded guess.

| Run | Immediate read (`immediate_read_us`) | 5-call mean (`immediate_read_5_calls_mean_us`) | PopWaitTimesOut, 10ms target (`timeout_10ms_actual_us`) | push()-to-wake (R-3, `push_to_wake_latency_us`) | close()-to-wake (R-5, `close_to_wake_latency_us`) |
|---|---:|---:|---:|---:|---:|
| 1 | 1.0 µs | 0.2 µs | 16546.5 µs | 19.6 µs | 10.6 µs |
| 2 | 0.8 µs | 0.3 µs | 15239.3 µs | 10.9 µs | 6.1 µs |
| 3 | 0.8 µs | 0.2 µs | 15291.3 µs | 9.4 µs | 24.9 µs |
| 4 | 0.9 µs | 0.2 µs | 17677.3 µs | 17.6 µs | 21.8 µs |
| 5 | 0.5 µs | 0.1 µs | 17570.1 µs | 24.9 µs | 23.9 µs |
| 6 | 0.8 µs | 0.2 µs | 17109.3 µs | 3.4 µs | 17.9 µs |
| 7 (checked in) | 0.8 µs | 0.2 µs | 12992.1 µs | 16.4 µs | 5.0 µs |

The headline number for a robotics-facing read of this design:
**push()-to-wake and close()-to-wake latency are consistently in the
single-digit-to-low-20s of *microseconds*, not milliseconds** — three
orders of magnitude inside the `< 10ms` bound `test_ring.cpp` asserts on
them (a bound deliberately left loose to not be a flaky, hair-trigger
assertion; the real numbers are what should inform any capacity-planning
decision, not the bound itself). Immediate reads of already-available
data are sub-microsecond, confirming the `pop_wait()` fast path never
goes anywhere near the OS wait primitive when there's nothing to wait
for. `PopWaitTimesOutWhenNoDataArrives`'s 13-18ms for a 10ms request
matches Windows' default ~15.6ms timer-tick rounding (the wait generally
overshoots a short requested timeout rather than undershooting it; see
the cold-start finding below for the one case it doesn't).

### p99 and headroom (200-trial distributions)

Six or seven samples say nothing about the tail, and a realtime budget is
blown by the rare slow call, not the typical one. `RingLatencyDistribution`
(`test_ring.cpp`) runs 200 independent trials of each timing-sensitive
path and records the full distribution (mean, p50, p90, p99, max) via
`RecordProperty()`, so a capacity decision can be made from p99 — what a
control loop running at some rate will actually see almost all the time —
rather than from a single run or a mean a slow tail can hide behind.

| Path | n | mean | p50 | p90 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|
| Immediate read (`immediate_read`) | 200 | 0.5 µs | 0.5 µs | 0.7 µs | 0.7 µs | 0.9 µs |
| push()-to-wake (`push_to_wake`, R-3) | 200 | 15.0 µs | 13.8 µs | 24.8 µs | 53.0 µs | 60.2 µs |
| close()-to-wake (`close_to_wake`, R-5) | 200 | 16.4 µs | 14.1 µs | 29.3 µs | 53.5 µs | 75.2 µs |

(checked-in run; see "Explaining the outlier" below for why an earlier
revision of this table showed a 680.3µs `push_to_wake` max that doesn't
appear in this particular run — it's a rare event, not a fixed one, and
recurs at roughly the rate that section quantifies)

**Headroom against concrete control-loop budgets** (budget − p99, as a
percentage of budget still unused after this design's own wake-path
latency):

| Control-loop rate | Period (budget) | push-to-wake headroom (p99 = 53.0µs) | close-to-wake headroom (p99 = 53.5µs) |
|---|---:|---:|---:|
| 1 kHz | 1,000 µs | 947.0 µs (94.7%) | 946.5 µs (94.7%) |
| 100 Hz | 10,000 µs | 9,947.0 µs (99.5%) | 9,946.5 µs (99.5%) |
| 10 Hz | 100,000 µs | 99,947.0 µs (99.95%) | 99,946.5 µs (99.95%) |

Even the tightest case evaluated (a 1kHz loop, the rate at which many
robot control loops run) has this design's wake path consuming under 6%
of the available period at p99 — comfortable headroom, with 94%+ of the
budget left for the rest of the loop's own work (control computation,
actuator I/O, other topics).

**On sample size**: n=200 means p99 is governed by roughly the 2 worst
samples observed — a real 99th-order-statistic, not "the 2nd-worst of 6"
the way the single-sample table above effectively was, but still a rough
estimate of the *true* p99 rather than a tight one. A production
capacity decision for a 1kHz-or-tighter deployment should re-run this
with a larger n (1,000+) before trusting the exact p99 figure to the
microsecond; the order-of-magnitude conclusion (tens of microseconds,
not milliseconds) is not in question at n=200.

### Explaining the outlier

The first revision of the distribution tests showed a single 680.3µs
`push_to_wake` sample against a 54.3µs p99 — an order of magnitude
higher, one sample out of 200. Two explanations were worth
distinguishing before trusting either: (a) genuine OS scheduling jitter,
inherent to running on a non-real-time OS and not fixable in this
design, or (b) an artifact of the test's own per-trial overhead — each
trial created a *fresh* `std::thread`, a fresh shared-memory segment, and
a fresh Windows `Event`, any of which could itself have caused a rare
scheduling hiccup that had nothing to do with the ring's actual wake
path.

**Test redesign to isolate the two.** `PushToWakeLatencyDistribution` was
rewritten to create the publisher, the subscriber, and the signaling
thread *once* and reuse them across all 200 trials (`push()` doesn't
destroy anything, so this is safe) — a `std::condition_variable` signals
the persistent producer thread which trial to run instead of spawning a
new thread each time. This removes per-trial thread creation and
per-trial OS-object creation entirely from the measured path.
`CloseToWakeLatencyDistribution` still needs a fresh publisher/subscriber
per trial (`close()` destroys the segment, so a closed publisher can't
be reused) but reuses the signaling thread, isolating thread-creation
churn specifically. (Rewriting `CloseToWakeLatencyDistribution` this way
surfaced a real bug in the *test*, not in `ring.hpp`: letting the next
trial's `unique_ptr<RingPublisher>` destroy the previous trial's
publisher as soon as `pop_wait()` returned raced the closer thread's
still-in-flight `close()` call — `close()`'s `signal_closed()` wakes the
subscriber before the call itself finishes unmapping/destroying the
segment, so the object being closed could be freed out from under that
still-running call. Fixed with a second condition variable the main
thread waits on, confirming `close()` has *fully* returned before
touching `pub` again.)

**Result: the outlier still occurred after removing all per-trial OS
object creation** — a 599.5µs `push_to_wake` sample appeared in one of
six reruns of the fully-reused-thread-and-object version (`p99` was
48.4µs that run). This rules out explanation (b): it is not a test
artifact. **It is genuine OS scheduler jitter** — an occasional delay in
getting the woken thread actually scheduled back onto a core, a normal
characteristic of any general-purpose (non-real-time) OS under any load
from other processes, background services, or power-management
transitions, and not something `platform.hpp`'s wake mechanism causes or
can prevent. Observed rate across both the original and redesigned
tests: roughly 1 such outlier (an order of magnitude above p99) per
1,000-1,200 trials — consistent with a rare-but-real tail, not noise
from the test harness.

**What this means for testing, and for the design**: this can't be
*eliminated*, only accounted for correctly.
- **Use p99, not max, as the acceptance basis** (already this report's
  practice, and now `docs/design_plan_guideline.md`'s stated guidance) —
  max from a 200-trial run is one data point that may or may not recur,
  and gating a pass/fail bound on it would make the test fail roughly
  once every 5-6 runs for a reason that has nothing to do with a
  regression.
- **Report what the outlier would cost if it mattered**, rather than
  silently reporting only the favorable p99 number: at a 1kHz loop
  period, a recurrence of the ~600µs-class outlier would consume ~60% of
  the budget on that one cycle — still survivable (the loop doesn't miss
  its deadline), but worth a designer's awareness if the deployment is
  that tight.
- **This is a hard limit on what any user-space wake path on a
  general-purpose OS can promise**, not specific to this design: a *hard*
  (not statistical) real-time guarantee would require a real-time-patched
  kernel (e.g. Linux `PREEMPT_RT`) or a dedicated RTOS, which is out of
  scope for shmbridge and worth stating explicitly rather than implying
  this design achieves hard real-time when it demonstrably achieves
  strong soft-real-time (p99-class) performance instead.

Before the R-18 fix (`platform.hpp`'s Windows notify mechanism), the R-3
and R-5 cases did not wake at all — both timed out at the full 5000ms on
every run, since `WaitOnAddress`/`WakeByAddressAll` never propagated a
wake across the publisher's and subscriber's separate mappings of the
same segment (see `design_ring_zero_copy.md` R-18 and revision 0.21 for
the root-cause repro).

### A real Windows finding: a process's first short wait can return early

Shrinking `PopWaitTimesOutWhenNoDataArrives`'s timeout from 100ms to 10ms
surfaced a genuine, 100%-reproducible Windows characteristic, confirmed
with a minimal repro that has nothing to do with shmbridge: **a freshly
started process's very first short `WaitForSingleObject` call can return
well before its requested timeout**, independent of any event ever being
signaled.

```
handle=00000000000000a8 err=0
iter  0: WaitForSingleObject(10ms) returned 258 after 753 us   <- fresh process, no signal, no event activity
```

Run as 6 separate fresh processes, the bare `CreateEventA` +
`WaitForSingleObject(10ms)` repro (no shmbridge code at all) returned
early on 2 of 6 — as low as 753µs against the 10ms requested — while
every *subsequent* wait in the same process, and every process where this
was not the first wait, was accurate to normal timer-rounding. The same
shape of failure (`elapsed` as low as ~2ms against a 10ms request)
reproduced in `test_ring`'s own `PopWaitTimesOutWhenNoDataArrives` before
a fix was added, confirming it's a process cold-start effect, not
anything specific to this design's event-binding mechanism. This is an
OS/environment characteristic `platform.hpp` has no way to correct —
there is no portable way to ask Windows to "warm up" its wait subsystem
in advance other than making a wait call.

**Fix**: added a `::testing::Environment` to `test_ring.cpp` that
performs one throwaway `pop_wait()` before any test runs, absorbing this
one-time cost so the real assertions measure steady-state behavior. 10/10
subsequent fresh-process runs of the full suite passed with this in
place (see the table above). This is also the practical takeaway for a
robotics deployment: **warm up any timing-critical wait path once at
process startup, before relying on a tight timeout in the hot loop** —
not specific to this design, but worth stating plainly rather than
leaving an operator to discover it the first time a startup-adjacent
timeout looks inexplicably short.

## Tightened immediate-read verification

`RingBlockingWait.PopWaitReturnsImmediatelyWhenDataAlreadyPresent`
originally asserted only `elapsed < 500ms` against a 5000ms timeout —
loose enough to pass even if `pop_wait()` had gone through one Windows
`notify_wait` slice (15ms, §8's documented margin) instead of genuinely
taking the `pop()` fast path with no wait call at all. That bound could
only catch a gross regression (blocking for most of the timeout), not a
smaller one.

Tightened to `elapsed < 50ms` against a 10-second timeout, and added a
second case, `PopWaitReturnsImmediatelyForEveryAlreadyPublishedItem`,
calling `pop_wait()` 5 times back-to-back against 5 already-published
items — asserting both the cumulative elapsed time across all 5 calls and
that the items come back in publish order, so a single lucky fast call
can't mask a real regression the way one call alone could.

Rebuilt and reran through the same `cmake --build`/`ctest` pipeline, 6
repeated runs:

```
[ RUN      ] RingBlockingWait.PopWaitReturnsImmediatelyWhenDataAlreadyPresent
[       OK ] RingBlockingWait.PopWaitReturnsImmediatelyWhenDataAlreadyPresent (0 ms)
[ RUN      ] RingBlockingWait.PopWaitReturnsImmediatelyForEveryAlreadyPublishedItem
[       OK ] RingBlockingWait.PopWaitReturnsImmediatelyForEveryAlreadyPublishedItem (0 ms)
```

Both pass at 0ms (GTest's own millisecond-granularity timer) on every
run; the full suite (11 cases: 2 `RingBasic` + 6 `RingBlockingWait` + 3
`RingLatencyDistribution`) remains green.

## Sanitizer verification (WSL Ubuntu)

Every result above is from native Windows (MinGW), which has no
sanitizer runtime installed. `CMakeLists.txt`'s new `SHMBRIDGE_SANITIZE`
option (`address,undefined` or `thread`) builds `test_ring` with
sanitizer instrumentation — verified on WSL Ubuntu (GCC 13.3.0), the
toolchain phase 9 already designates for this.

```
# Plain (closes the "WSL not yet exercised" open item carried since 0.21)
$ ./build_plain/test_ring
[==========] 11 tests from 3 test suites ran. (915 ms total)
[  PASSED  ] 11 tests.

# AddressSanitizer + UndefinedBehaviorSanitizer
$ ASAN_OPTIONS=detect_leaks=1 ./build_asan/test_ring
[==========] 11 tests from 3 test suites ran. (925 ms total)
[  PASSED  ] 11 tests.
# -- zero ASan/UBSan findings (no heap/stack error, no leak, no UB) --

# ThreadSanitizer
$ setarch $(uname -m) -R ./build_tsan/test_ring
[==========] 11 tests from 3 test suites ran. (934 ms total)
[  PASSED  ] 11 tests.
# -- zero TSan findings (no data race) --
```

**TSan requires ASLR disabled to run at all** against this design's
shared-memory mappings: without `setarch $(uname -m) -R`, the binary
exits immediately with `FATAL: ThreadSanitizer: unexpected memory
mapping 0x...` — a known TSan/shared-memory interaction, not a bug in
this design, but worth stating plainly since the fatal exit otherwise
looks indistinguishable from "TSan is incompatible with this code."

**Validating the sanitizer pipeline's own sensitivity**, rather than
trusting a clean run at face value: a minimal standalone program
reproducing the exact bug shape found and fixed in
`CloseToWakeLatencyDistribution` (a thread signaling "done" *before*
finishing a delayed write to an object the main thread is free to
destroy once signaled — the same ordering `close()`'s
`signal_closed()`-before-full-teardown creates) was compiled with
`-fsanitize=address`. The buggy version was caught immediately and
precisely:

```
==397==ERROR: AddressSanitizer: heap-use-after-free on address 0x502000000014
WRITE of size 4 at 0x502000000014 thread T1
    #0 in Obj::slow_close() ...
freed by thread T0 here:
    #0 in operator delete(void*, unsigned long) ...
    #2 in std::unique_ptr<Obj>::~unique_ptr() ...
previously allocated by thread T0 here:
    #0 in operator new(unsigned long) ...
```

The fixed version (joining the thread before the object can destruct)
produced no error. This confirms the sanitizer setup would have caught
the real bug 0.27 found and fixed in the test harness, had it still been
present — the clean runs above are meaningful, not just "the tool didn't
trip."

## Native-Windows AddressSanitizer (MSVC)

WSL's sanitizer coverage above is still GCC-family tooling. MSVC is a
genuinely different compiler, and worth trying specifically for that
reason, not as a redundant check.

**VS2019's ASan runtime doesn't work on this host.** `cl.exe
/fsanitize=address` compiles cleanly (VS2019 Build Tools 16.11, MSVC
14.29), but the resulting binary fails before `main()` runs at all:

```
==79580==AddressSanitizer CHECK failed: ...asan_rtl.cpp:397
"((!asan_init_is_running && "ASan init calls itself!")) != (0)"
```

Confirmed with a trivial "print and exit" program — no user code, no
bug — ruling out anything specific to this project. Windows Defender
real-time protection was already off, and no recognizable EDR process
was found; the most likely explanation is a genuine incompatibility
between VS2019's several-years-old ASan runtime and this host's very
recent Windows build. A newer Visual Studio installed on the same
machine (MSVC ~19.50) was used instead, and its ASan runtime initializes
and detects a deliberately-introduced heap-buffer-overflow correctly,
confirmed before trusting it against the real code.

**That installation's legitimacy was checked, not assumed**, given how
new it is: `vswhere` reports it as "Visual Studio Community 2026"
(internal version 18.3.11520.95) on the `VisualStudio.18.Release`
channel — `isPrerelease: False`, `isComplete: True`,
`productMilestone: RTW` — a finished general-availability release, not a
Preview/Insider build. `cl.exe` and `clang_rt.asan_dynamic-x86_64.dll`
both carry valid Authenticode signatures from Microsoft Corporation, and
AddressSanitizer is installed via Microsoft's own formal component ID
(`Microsoft.VisualStudio.Component.VC.ASAN`), confirmed present on this
instance via `vswhere -requires` — not an accidental side effect of some
other component. Combined with the functional check above (correctly
flags a deliberate bug), this toolchain is a genuine, licensed,
officially-supported basis for the R-19 finding below, not a fluke of an
unstable or incomplete install.

**Getting it to actually link required two MSVC-specific workarounds**,
both Microsoft-documented, neither invented here, both now encoded in
`CMakeLists.txt`'s MSVC branch of `SHMBRIDGE_SANITIZE`:

1. **Static-CRT GTest.** `/fsanitize=address` forces static-CRT codegen
   regardless of `/MD` requested elsewhere; linking against a shared-CRT
   GTest build failed with `LNK2038: mismatch detected for
   'RuntimeLibrary'`. Fix: build GTest without
   `-Dgtest_force_shared_crt=ON` (its default is static CRT).
2. **Disabled container annotations.** Even with matching CRT, linking
   still failed: `LNK2038: mismatch detected for 'annotate_string'/
   'annotate_vector'` — ASan's `std::string`/`std::vector`
   container-overflow instrumentation changes those types' ABI, so an
   ASan-compiled translation unit can't link against a dependency (GTest)
   that wasn't also compiled with `/fsanitize=address`. Fix:
   `-D_DISABLE_VECTOR_ANNOTATION -D_DISABLE_STRING_ANNOTATION`, Microsoft's
   documented workaround for exactly this situation.

**Result: a real, previously undetected portability bug (R-19).** With
both workarounds in place, the actual `ring.hpp` failed to even compile:

```
ring.hpp(237): error C2065: '__ATOMIC_ACQUIRE': undeclared identifier
ring.hpp(237): error C3861: '__atomic_load_n': identifier not found
```

`RingHeader::capacity` was a plain (non-atomic) `uint32_t`, accessed via
GCC/Clang's `__atomic_load_n`/`__atomic_store_n` builtins — inconsistent
with `write_idx`/`read_idx`/`closed` in the same struct, which already
use `std::atomic<T>`. Every toolchain this design had ever been built
with (MinGW GCC, WSL GCC) accepts these GNU builtins, so the gap was
invisible until a genuinely different compiler family was tried. Fixed
by making `capacity` a proper `std::atomic<uint32_t>` and switching the
three call sites to `.load(std::memory_order_acquire)`/
`.store(N, std::memory_order_release)` — same memory ordering, now
expressed portably. With the fix, the full 11-test suite builds and
passes clean under MSVC ASan.

**Confirmed end to end through the real build system**, not just the
ad-hoc `cl.exe`/batch-script invocations used above while diagnosing the
CRT and container-annotation link errors: a fresh configure (VS2026,
`cmake -S shmbridge -B build -G Ninja -DSHMBRIDGE_SANITIZE=address
-DGTEST_ROOT=<static-CRT GTest>`), `cmake --build build --target
test_ring`, and `ctest --output-on-failure` from that build directory —

```
Test project .../build
    Start 1: test_ring
1/1 Test #1: test_ring ........................   Passed    5.83 sec
100% tests passed, 0 tests failed out of 1
```

— with `test_ring_sanitize_results.json` generated in the build
directory exactly as `CMakeLists.txt` routes sanitizer-build output
(kept separate from the unsanitized `tests/test_ring_results.json` this
report's timing numbers come from, so a sanitizer run never clobbers
them).

## Findings

1. **Clean build, no new warnings.** The only warning present is
   pre-existing and unrelated to this phase's changes.
2. **All 11 tests pass, repeated runs (10 fresh-process runs of the
   8-case core suite, plus the 200-trial distribution runs covered
   above), both through the compiled binary directly and through
   `ctest`.**
3. **Realistic robotics-facing timings, measured and recorded precisely,
   with tail (p99) behavior evaluated, not just a mean or a single
   sample**: immediate reads of already-available data complete in
   p99 = 0.7µs (n=200); `push()`-to-wake and `close()`-to-wake latency
   have p99 = 53.0µs / 53.5µs respectively (n=200 each, checked-in run)
   — comfortable headroom (94%+) against even a tight 1kHz control-loop
   budget, see the p99/headroom section above for the full table and the
   "Explaining the outlier" section for what a rare (~1-in-1,000-trial)
   OS-jitter outlier would cost if it recurred; a 10ms timeout request is
   honored (returns in 13-18ms, consistent with Windows' ~15.6ms default
   timer-tick rounding, not a correctness problem).
4. **Sanitizer-clean on WSL Ubuntu**: AddressSanitizer+UndefinedBehavior
   Sanitizer and, separately, ThreadSanitizer both report zero findings
   across all 11 tests — validated against a known-buggy repro first to
   confirm the sanitizers are actually catching what they're supposed to,
   not just passing silently. This also closed the long-open "WSL not yet
   exercised" item: the Linux futex notify path, untouched by R-18's
   Windows-specific fix, is now confirmed working, not just assumed.
5. **A real, OS-level finding, not a shmbridge bug**: a Windows process's
   first short kernel wait of its life can return well before its
   requested timeout (confirmed with a bare `CreateEventA`/
   `WaitForSingleObject` repro having nothing to do with this codebase).
   Fixed in `test_ring.cpp` with a one-time warm-up wait before any
   timing-sensitive assertion runs; worth the same treatment in any
   Windows deployment of this design (warm up timing-critical paths at
   startup, don't trust the very first tight-timeout wait of a process's
   life).
6. **A real, previously undetected portability bug (R-19), found only by
   trying a genuinely different compiler**: `ring.hpp` used GCC/Clang-only
   atomic builtins that are a hard compile error under MSVC, invisible
   under every GCC-family toolchain (MinGW, WSL GCC) this design had been
   built with. Fixed (`RingHeader::capacity` is now a proper
   `std::atomic<uint32_t>`); the full suite passes clean under MSVC ASan
   with the fix applied. VS2019's own ASan runtime doesn't work at all on
   this host (confirmed with a no-op program, ruling out a code cause) —
   a newer installed Visual Studio was used instead, after confirming its
   ASan runtime actually detects a known bug first.
7. **Phase 1's exit criterion (§15) is met** on native Windows (MinGW
   build; MSVC ASan build) and WSL Ubuntu, with memory-safety and
   data-race verification (not just the test suite's own assertions) on
   both GCC-family and MSVC toolchains.
