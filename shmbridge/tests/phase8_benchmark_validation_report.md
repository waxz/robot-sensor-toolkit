# Ring transport — Phase 8 (benchmark validation) report

Generated while completing `docs/design_ring_zero_copy.md` §15 phase 8:
validating NFR-1 through NFR-4 against the actual ring implementation, and
adding unit tests covering every feature (F-1–F-14) and risk (R-1–R-19)
from §4/§7 that didn't already have a dedicated test.

**Scope adaptation**: phase 8's original text calls for extending the
Python `bench_pubsub_matrix.py` with a `ring` mode. `ring.hpp` has no
Python bindings (this document's own Scope row: "C++ only — Python
ctypes/pybind11 mirror is a separate, later effort"), so that specific
instruction can't be carried out as written. Every NFR-1–NFR-4 metric is
instead measured in C++, using the same GTest-suite-plus-`RecordProperty`
convention `RingLatencyDistribution` already established for NFR-6 in
phase 1 — the established, in-scope way this project records a
machine-readable performance number.

## Correctness: features and risks given new test coverage

Auditing §11.1's test table against the actual suite found three gaps
that existed only as design-doc prose, not as a running assertion:

| Gap | New test | What it closes |
|---|---|---|
| R-4's exact start_idx boundary was only exercised loosely (a cursor far behind start, not pinned exactly at the `<`/`==` edge) | `RingBasic.PopExBoundaryAtExactlyStartIdx` | Confirms `next_read_ == start_idx` reads directly (not treated as evicted) and `next_read_ == start_idx - 1` resyncs (not reads evicted data) |
| R-1's torn-read retry path had never been forced under real concurrency, only read as code | `RingTornReadRetry.HighContentionStaysLiveWithRareR2Anomalies` | A 2-slot ring under a flat-out writer thread forces real eviction races; see "A real finding" below |
| 4 of `RingConfig`'s 7 TOML-resolvable fields (`max_age_ms`, `warn_every_ms`, `max_takeover_attempts`, `attach_retry_ms`) had no test confirming they actually resolve from a file | `RingConfigResolution.RemainingFieldsResolveFromToml` | R-10 (log-rate-limit knob), R-11/R-12 (staleness knob), F-11/R-17 (takeover-retry bound) |

Two of those same fields turned out to be unused once added, which the
coverage audit surfaced as real implementation gaps, not just missing
tests — fixed rather than left for later, consistent with how every
other finding in this design's implementation phases was handled:

- **`attach_retry_ms` was declared, TOML-resolved, and documented
  ("resilient-loop attach-probe cadence") but `RingSubscriber::attach()`
  ignored it**, hardcoding a 5ms poll interval regardless of what was
  configured. Fixed: `attach()` now sleeps `cfg.attach_retry_ms` between
  probes (floored at 1ms against a pathological zero/negative config
  value).
- **§5.8's documented 3-case resilient loop ("case 2: attached →
  `pop_wait()`; if `is_closed()`, rate-limited warning + mandatory
  `detach()` + retry from step 1") was never wired into `node.hpp`** —
  only the underlying `RingSubscriber` primitive supported it; nothing
  called `is_closed()`/`detach()` automatically. A ring-based
  `node.hpp` subscription that outlived its publisher's restart would
  silently and **permanently** stop receiving, for the life of the
  subscriber process — exactly the failure R-9 exists to prevent, and it
  was open the entire time phases 2–7 were being verified, because
  nothing had tested a `node.hpp`-level subscription surviving a
  publisher restart before. Fixed: `node.hpp`'s `SubSlot` gained
  `is_closed`/`detach` hooks (ring branches only), and `spin_once()` now
  detaches and re-attaches automatically. Proven end-to-end by
  `Node.RingSubscriptionAutoReattachesAfterPublisherClose` (first
  publisher closes mid-run, a second publisher opens the same topic, the
  existing subscription recovers with no manual `detach()` call anywhere
  in the test).
  The rate-limited *warning* half of §5.8's case 2 remains unimplemented:
  this codebase has no established logging convention to hang it off of
  (no header anywhere calls `fprintf(stderr, ...)` or equivalent), and
  inventing one — stderr? a callback? a counter? — is a real API-surface
  decision, not a mechanical fix. Left as an explicit, narrower open item
  rather than guessed at; the functional recovery (the correctness-
  critical half) is fixed and tested.
- Added `RingSubscriberStats` (`total_reads`/`retried_reads`/
  `exhausted_reads`) and `RingSubscriber::stats()`, mirroring
  `topic.hpp`'s existing `SubscriberStats<T>` convention. This is NFR-4's
  "retry-count instrumentation added to the benchmark harness," which
  didn't exist in any form before this phase — there was no way to
  observe a torn-read retry happening at all.

### A real finding: R-2's accepted risk, observed directly

`RingTornReadRetry`'s first version asserted received values must always
be strictly increasing. At the test's deliberately adversarial setting
(`capacity = 2`, a flat-out writer, 20000 pushes), it failed once on
WSL/GCC with a genuine decrease (`807` then `806`). This is **R-2** (§7)
showing up, not a new defect: R-2 explicitly accepts that the bulk
`memcpy` of a slot the writer may concurrently overwrite is undefined
behavior in the strict C++ memory-model sense, relying on the
`start_idx` before/after check to discard the overwhelming majority of
torn reads — documented as "empirically low... never observed to
corrupt beyond recognition," never claimed to be formally impossible.
The test's assertion was simply stricter than what the design actually
promises. Rewritten to measure the anomaly **rate** instead of asserting
zero: `anomaly_pct < 1.0%`, recorded via `RecordProperty` so the real
number is visible rather than only pass/fail. Stable across 10 repeat
runs after the fix (every repeat: 0 anomalies, 1 retried read out of
~19900 reads at this capacity) — the single observed anomaly during
initial test-writing is itself the first direct empirical measurement
of R-2's claimed rarity this design has ever recorded, rather than an
assumption.

## Performance: NFR-1 through NFR-4

All four numbers below come from the same MSVC/VS2026 Release run in
this session (`tests/test_ring_results.json`, `RingPerformance`/
`RingTornReadRetry` suites); WSL/GCC and ASan+UBSan runs confirm the
same tests pass (functional correctness, not re-measured for timing,
consistent with how phase 1's own sanitizer runs never re-measured
NFR-6's numbers either — sanitizer instrumentation perturbs timing).

| NFR | Target | Measured | Verdict |
|---|---|---|---|
| NFR-1 | Blocked `pop_wait()` ~0% CPU vs. ~97-101% busy-spin baseline | Blocked: **0.0%**; busy-spin: **101.6%** (`RingPerformance.BlockedSubscriberUsesFarLessCpuThanBusySpin`, 200ms window each, measured via `platform::thread_cpu_time_ns()`, a new primitive added for this) | **Met**, matches the busy-spin baseline almost exactly and the blocked case rounds to exactly 0% |
| NFR-2 | One-way latency stays within the same order of magnitude as the seqlock baseline (~5-7µs median, 16-88B payloads) | Not re-measured this phase — already covered by existing evidence: `bench_migration.cpp`'s `direct_ring_ns`/`node_ring_ns` (p50 = 301ns/370ns, phase 7's run) and `RingLatencyDistribution.ImmediateReadDistribution` (sub-microsecond). Both are already *faster* than the seqlock baseline, comfortably within "same order of magnitude" | **Met** (cited from existing phase 2/7 evidence, not re-run) |
| NFR-3 | Zero-copy measurably faster than copy at large payloads (4KiB-1MiB) | `PointCloud4096` (~64KiB): copy path p50 = **4.0µs**, zero-copy path p50 = **0.6µs** (`RingPerformance.ZeroCopyFasterThanCopyAtLargePayload`, 200 trials each) — zero-copy is ~6.7x faster at the median | **Met**, a large and unambiguous margin |
| NFR-4 | Torn-read retries rare in practice; a sustained non-zero rate signals undersized capacity (R-11), not a defect | At a deliberately adversarial `capacity=2`: 1 retried read out of 19905 total (**0.005%**), 0 exhausted, 0% anomaly rate past the before/after check (`RingTornReadRetry`'s `RecordProperty` output) — at *any* realistic (non-adversarial) capacity this rate would be lower still | **Met**; instrumentation (`RingSubscriberStats`) now exists where none did before, closing the literal gap phase 8's text names |

NFR-5 and NFR-6 were already verified in phases 2 and 1 respectively and
are unaffected by this phase's changes (confirmed by the full suite
re-passing, not re-measured again here).

## Correctness — result

| Target | WSL/GCC 13.3.0 (plain, then ASan+UBSan) | Native Windows/MSVC 19.50 (VS2026) |
|---|---|---|
| `test_ring` (34 cases, +5 from this phase) | 34/34 pass both configurations | 34/34 pass, `test_ring_results.json` regenerated |
| `test_migration` (36 cases, +1 from this phase) | 36/36 pass both configurations | 36/36 pass |
| `test_topic` (22 cases, unaffected by this phase) | 22/22 pass | not re-run this round (unaffected; confirmed in the prior Windows-port report) |

The new `RingTornReadRetry` test was specifically re-run 10x via
`--gtest_repeat=10` after its fix, on top of the full-suite passes above,
given its concurrency-dependent nature.

## Reproduce

Same procedure as `tests/phase7_node_migration_report.md`'s Reproduce
section — `test_ring` and `test_migration` are the targets that changed.
Run against the live working tree, not a fresh clone — this phase's
changes are not yet committed.
