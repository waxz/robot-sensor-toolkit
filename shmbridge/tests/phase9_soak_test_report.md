# Ring transport — Phase 9 (soak test) report

Generated while completing `docs/design_ring_zero_copy.md` §15 phase 9's
exit criterion: the §11.2 soak test, run on both target platforms. This
is the final phase of the ring transport's implementation plan — with
this phase done, all 9 phases are `[DONE]`.

**Headline result**: native Windows/MSVC passed with zero violations
across 1.50 billion messages. WSL/GCC found 11 violations across 289
million messages — a direct, empirical measurement of **R-2** (§7), an
already-accepted risk, actually occurring under realistic conditions.
Not a regression; a confirmation of something the design already
expected but had never actually measured at this scale before.

## What the soak test does

`tests/soak_test.cpp` (orchestrator) and `tests/soak_consumer_helper.cpp`
(one independent consumer process), gated behind a new
`SHMBRIDGE_BUILD_SOAK_TEST` CMake option (default `OFF` — a 30-minute run
has no place in a routine `ctest` pass):

- One `RingPublisher<SoakMsg>` opens a topic with `RingConfig::capacity =
  1024` — a realistic value, not the deliberately adversarial `capacity=2`
  `RingTornReadRetry` (phase 8) uses to force eviction races on purpose.
- `SoakMsg { uint64_t seq; uint64_t pattern; }`, `pattern = seq *
  0x9E3779B97F4A7C15ULL` — a redundant checksum a consumer can verify
  independently of `seq` itself, to catch corruption that happens to
  preserve a plausible-looking sequence number.
- 4 independent consumer **processes** (not threads — spawned via
  `platform::run_and_capture()`, the same cross-platform subprocess
  mechanism `test_topic.cpp`'s multi-process test already established,
  chosen specifically because "multiple independent consumer processes"
  is §11.2's literal wording) at poll intervals 1/5/20/100ms.
- The producer pushes flat-out (no per-push sleep — see "A pacing bug
  found and fixed" below) for 1800 seconds (30 minutes, §11.2's stated
  minimum).
- Each consumer checks every message it receives for: `pattern` matching
  the checksum (corrupted payload), `seq` strictly increasing within its
  own stream (duplicate/out-of-order delivery), and `write_ns` strictly
  increasing (non-monotonic timestamp) — exactly the three violation
  classes §11.2 names.

## Result

| Platform | Messages pushed | Messages received (sum, 4 consumers) | Violations (sum) | Violation rate |
|---|---|---|---|---|
| Native Windows/MSVC 19.50 (VS2026) | 375,742,481 | 1,502,638,992 | **0** | 0 |
| WSL Ubuntu/GCC 13.3.0 | 72,414,116 | 288,858,267 | **11** | ≈1 per 26.3 million |

Per-consumer WSL detail (every violation was a clean, previously-pushed
value delivered out of order or a timestamp that moved backward — never
a corrupted/garbled payload in either platform's run):

| Consumer (poll interval) | Received | Violations | Rate |
|---|---|---|---|
| 1ms | 72,207,953 | 5 | ≈1 per 14.4M |
| 5ms | 72,218,526 | 2 | ≈1 per 36.1M |
| 20ms | 72,211,032 | 2 | ≈1 per 36.1M |
| 100ms | 72,220,756 | 2 | ≈1 per 36.1M |

Full raw logs from both runs: `tests/phase9_soak_logs/windows_soak30.log`,
`tests/phase9_soak_logs/wsl_soak30.log` (kept alongside this report rather
than only in this session's now-deleted scratch directories).

## Why WSL found violations and Windows didn't: this is R-2, not a new bug

§7's R-2 risk-table entry already stated, before this test ever ran: the
bulk `memcpy` in `pop_ex()`/`borrow()` is undefined behavior in the
strict C++ memory-model sense whenever a reader's copy overlaps the
producer's next write to the same physical slot (which happens on every
wrap, by design) — "certain (by design, on every wrap)... empirically
low... relies entirely on the `start_idx` before/after check to discard
any torn result." This test is the first time that claim was checked
against a long, realistic-capacity run rather than either short unit
tests or phase 8's deliberately adversarial `capacity=2` stress test.

The result is consistent with — not contradictory to — what R-2 already
said: a UB-but-rare race, occurring at a rate that depends on the host's
specific scheduler, memory subsystem, and timing characteristics (WSL's
Linux futex-based notify path vs. Windows' named-Event path are genuinely
different code paths with different scheduling interactions, so a rate
difference between them isn't itself surprising). What's new here is
having an actual number: roughly 1 in 26 million messages on this WSL
host, zero observed in almost 1.5 billion on this Windows host. Neither
number proves the other platform is "safe" in an absolute sense — only
that the rate is low enough that billions of messages are needed to
observe it at all.

This result directly contradicted §11.2's original "zero correctness
violations" acceptance criterion, which was never reconciled against
R-2's already-accepted non-zero rate when it was written. Resolved by
revising §11.2 (not by disputing the soak test's result) — see the design
document's own revision history (0.39) for the exact wording change.

**Prompted a direct comparison** of `ring.hpp`'s detection mechanism
against `topic.hpp`'s `SeqlockSlot`, which R-2 cites as precedent:
`topic.hpp` brackets each write with a per-slot `seq`/`seq2` pair checked
by the reader before *and* after the copy — a direct, local, tight
invariant. `ring.hpp` instead checks `start_idx`, a ring-wide eviction
cursor updated in a separate step *after* the slot write completes — a
reader whose acquire-load of `start_idx` races ahead of the writer's
corresponding release-store can accept data from an already-overwritten
slot with literally no synchronization established on those bytes at
all. This is a real structural gap relative to the seqlock precedent
`ring.hpp`'s own risk-table entry invokes, not an equivalent mechanism —
confirmed here, not merely argued. A per-slot seqlock bracket (distinct
from, and much cheaper than, the word-wise-atomic alternative
§6 already evaluated and rejected) would close it at a fixed, small
per-slot cost. Recorded as a new §16 open item, not implemented — R-2
remains Approver-owned, and the measured rate (≈1 per 26M even on the
platform where it was observed) doesn't currently justify an unreviewed
change to a Tier 1 design's hot path mid-session.

## A pacing bug found and fixed before the real run

The first version of `soak_test.cpp` paced the producer with
`platform::sleep_ns(50'000)` targeting ~20kHz. A 20-second smoke test
caught a platform discrepancy before committing to a 30-minute run:
~5.9kHz achieved on WSL/Linux vs. ~90Hz on native Windows for the
*identical* requested interval — Windows' `Sleep()` rounds any
sub-~15ms request up to its ~15.6ms timer-tick granularity (the same
characteristic already documented elsewhere in this design, R-18/NFR-6),
so the same code produced throughput almost two orders of magnitude
apart on the two platforms, for no benefit (less stress exposure, not
more realism — a real control loop at 10-1kHz is already far below
either accidental rate). Fixed by pushing flat-out instead (same
convention `bench_migration.cpp`'s `bench_ring_spsc_throughput` already
uses), which removed the platform-dependent sleep granularity from the
equation and maximized eviction/overwrite cycles over the run — directly
responsible for finding R-2 at all, since a throttled run would have
generated far fewer wrap-around races to observe.

## Getting a trustworthy 30-minute run: the WSL VM idle-teardown problem

The first two attempts at the WSL run were silently killed partway
through (around minute 17 and minute 25-30 respectively) — not a crash
in the soak test itself, but the WSL2 VM's own lifecycle tearing down
the whole instance. Confirmed via `uptime` resetting to "0 min" inside
WSL immediately after each failure, while the Windows host's own
`LastBootUpTime`/`Get-Uptime` stayed continuous throughout — ruling out
a host sleep or restart. Checking back more frequently (every 5 minutes
instead of 15) did not prevent the second failure either, which ruled
out "no `wsl.exe` connection for too long" as the direct trigger.

What worked: restructuring the launch so the long-running command was
the **foreground** process of a `wsl.exe` invocation that stayed
continuously connected for the full 1800 seconds (not something detached
via `setsid`+`disown` *inside* WSL and polled from outside), with that
`wsl.exe` invocation itself launched through Windows Task Scheduler so it
would survive independent of the orchestrating session's own background-
task time limits. The third attempt ran the full 30 minutes without
interruption on both platforms. See the design document's §17 lessons
learned for the generalized version of this finding.

## Reproduce

```powershell
# Windows / Visual Studio 2026
cmake -S shmbridge -B build -G Ninja -DCMAKE_BUILD_TYPE=Release `
    -DSHMBRIDGE_BUILD_SOAK_TEST=ON -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded
cmake --build build --config Release
.\build\soak_test.exe 1800 1024 4
```

```bash
# WSL / Linux — launch as a continuously-connected foreground process
# (see "Getting a trustworthy 30-minute run" above for why this matters)
cmake -S shmbridge -B build -DCMAKE_BUILD_TYPE=Release -DSHMBRIDGE_BUILD_SOAK_TEST=ON
cmake --build build -j"$(nproc)"
./build/soak_test 1800 1024 4
```

Run against the live working tree, not a fresh clone — this phase's
changes are not yet committed.
