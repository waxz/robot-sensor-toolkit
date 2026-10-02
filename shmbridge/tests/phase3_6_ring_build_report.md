# Ring transport — Phases 3–6 compile, correctness, and performance report

Generated while completing `docs/design_ring_zero_copy.md` §15 phases 3–6:
zero-copy write (F-4), zero-copy read (F-5), resilient attach/reattach
(F-9), and the fixed-size point-cloud message family (F-12). Every number
below comes from a `ctest` run executed in this session against the
actual, current working tree — see "Reproduce" below.

## What this covers

Phases 3, 4, and 5's code (`reserve()`/`commit()`, `borrow()`/
`end_borrow()`, `try_attach()`/`detach()`/the reattach-after-close path)
was already written and tested as part of phase 2's single-pass
`ring.hpp` rewrite — `tests/phase2_ring_build_report.md` is the primary
evidence for those three phases' exit criteria and is not re-measured
here; this report's new content is phase 6 only.

Phase 6 added one new header addition and two new tests:

| File | Change |
|---|---|
| `include/shmbridge/messages.hpp` | + `PointCloudFixed<MaxPoints>` template and six named variants (`PointCloud64` through `PointCloud65536`), per §5.11 |
| `tests/test_ring.cpp` | + `RingPointCloud` suite: `RoundTripPreservesNPointsAndData`, `DifferentMaxPointsVariantThrowsOnAttach` |

No changes to `ring.hpp`/`platform.hpp` were needed for phase 6 — F-12's
type-mismatch rejection reuses F-7's existing `type_hash` mechanism
unchanged, exactly as §5.11 anticipated ("no new validation logic
needed").

## Correctness — was it built to the specification?

| Test | Verifies |
|---|---|
| `RingPointCloud.RoundTripPreservesNPointsAndData` | A `PointCloudFixed<64>` round trip through `push()`/`pop_ex()` preserves `n_points` and every filled point's (x, y, z, intensity) exactly |
| `RingPointCloud.DifferentMaxPointsVariantThrowsOnAttach` | A `RingSubscriber<PointCloudFixed<256>>` attaching to a `PointCloudFixed<64>` topic is rejected with `std::invalid_argument`, confirming each `MaxPoints` instantiation gets a distinct `type_hash` |

**Result: 29/29 pass (27 carried from phase 2 + 2 new) on every toolchain
exercised**, run fresh in this session against the current working tree:

| Toolchain | Build type | Result |
|---|---|---|
| Native Windows, MSVC 19.50.35725.0 (Visual Studio 2026, `cl.exe`/Ninja) | Release, plain | 29/29 pass, 6.36s |
| WSL Ubuntu 24.04, GCC 13.3.0 | Release, plain | 29/29 pass, 0.95s |
| WSL Ubuntu 24.04, GCC 13.3.0 | AddressSanitizer+UndefinedBehaviorSanitizer | 29/29 pass, 0.99s |
| WSL Ubuntu 24.04, GCC 13.3.0 | ThreadSanitizer (`setarch $(uname -m) -R`) | 29/29 pass, 0.99s |

No regressions against phase 2's 27 cases; no flaky reruns needed.

## Performance

Phase 6 adds no new code to the ring's hot path (push/pop/wake) — it is a
new message type, not a transport change — so no new NFR-6 measurement
applies here. `RingLatencyDistribution`'s three 200-trial cases (carried
over from phase 2, exercised against `int`, not `PointCloudFixed`) still
pass as part of the 29-case suite above; see
`tests/phase2_ring_build_report.md` for those numbers, which remain
representative since the wake path itself is unchanged.

## Reproduce

Same procedure as `tests/phase2_ring_build_report.md`'s Reproduce
section — `test_ring` is the one target that changed — run against
the current working tree (this phase's code is not yet committed, so a
plain `git clone`/checkout of HEAD will not reproduce it until it is).
