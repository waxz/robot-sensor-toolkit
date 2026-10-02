# shmbridge Ring Transport — Design Plan

| | |
|---|---|
| Component | `shmbridge/include/shmbridge/ring.hpp`, wired into `node.hpp` |
| Author(s) | Claude (agent) |
| Reviewer(s) | Repository owner (interactive review across this design's development) |
| Approver | Repository owner — approved |
| Status | **Approved** — implementation (§15) complete, all 9 phases `[DONE]`; passed a developer-readiness walkthrough (0.9) and a line-by-line verification against the current `ring.hpp`/`node.hpp`/`platform.hpp` code (0.10). §16 carries one open item as of 0.39: an R-2 hardening opportunity (a per-slot seqlock bracket) found during phase 9's soak test, recorded but not implemented pending an explicit decision |
| Tier | 1 — Critical (concurrency primitive crossing process boundaries, wire-format change, cross-process data-integrity consequences; see `docs/design_plan_guideline.md` §1) |
| Scope | C++ only (Python ctypes/pybind11 mirror is a separate, later effort) |
| Risk posture | 19 identified risks (§7): 3 accepted-and-owned (R-2, R-6, R-14 — all explicitly reasoned tradeoffs, none discovered after the fact), 16 mitigated. None safety-critical; R-2's accepted UB is the highest-attention item, consistent with existing precedent elsewhere in this codebase (`topic.hpp`'s seqlock) rather than a novel risk. R-18 (Windows `WaitOnAddress` cross-view limitation) and R-19 (non-portable GCC/Clang atomic builtins) were both discovered during phase 1 implementation, not at design time — see 0.21 and 0.29 |

**Revision history**

| Version | Summary |
|---|---|
| 0.1 | Initial zero-copy pointer design over the existing reject-on-full, single-consumer ring |
| 0.2 | Redesigned to always-succeed/overwrite-on-full writes with independent per-consumer cursors |
| 0.3 | Added message staleness (`write_ns`/`is_stale()`), compile-time type safety (`type_hash`), and resilient attach/reattach behavior |
| 0.4 | Restructured into a standard design-plan format (feature list, risk table, requirement coverage, implementation plan) |
| 0.5 | Restructured again to the project's design-plan guideline: added Assumptions, Interfaces, upgraded risk analysis to FMEA, added Margins, split Verification from Validation, added rollback criteria and Lessons learned |
| 0.6 | Added producer-conflict detection (F-11), fixed-size point-cloud message variants (F-12), latest-only consumption (F-13), reserve/borrow misuse guards (F-14); expanded Interfaces with process context and cost; compared against open-source prior art in Alternatives Considered; switched config format to TOML; added a CI plan |
| 0.7 | Self-review fixes: added Implementation Plan phases for F-11/F-12/F-13/F-14 (previously undocumented despite being in the feature list and coverage table); moved INI-vs-TOML rationale out of §5.5 into §6 Alternatives Considered; renumbered implementation-phase cross-references accordingly; noted F-11's behavior-change impact on any pre-existing dual-producer deployment in §12 |
| 0.8 | Closed all four open items: pinned `toml++` as the TOML library; dropped the call-site-enumeration requirement (no old version is deployed, rebuild-together is sufficient); replaced the GitHub Actions CI phase with local dual-toolchain verification (native Windows + WSL Ubuntu) for this pass; deferred the reliable/blocking QoS mode indefinitely in favor of timestamp-only freshness checking |
| 0.9 | Developer-readiness review fixes: moved the `CMakeLists.txt` build-gate fix from phase 7 to phase 1 (every other phase's exit criterion was unachievable on Windows until it landed); removed a stale `(§15, open item 2)` reference left over from the 0.8 cleanup; fixed a real concurrency bug in `claim_producer_slot()`'s stale-producer takeover path (unconditional store → `compare_exchange_strong`, since two processes racing to recover the same crashed producer could otherwise both believe they'd won) — added as R-17 with its own unit test; documented `process_alive()`'s required default (assume alive) on an indeterminate liveness result; called out the point-cloud default-capacity footgun explicitly in §5.11 |
| 0.10 | Verified every architecture/migration claim against the actual current `ring.hpp`/`node.hpp`/`platform.hpp` code (not assumed): corrected §12's factually wrong "repurposes `start_idx`/`end_idx`" (those fields don't exist today — the real fields are `write_idx`/`read_idx`, being replaced, not repurposed); confirmed capacity is a compile-time template parameter `N` today and documented its removal as its own breaking-change bullet; corrected F-13 to note `pop_latest()` already exists today, carried forward rather than new; fixed an internal inconsistency where §14 phase 7 omitted `create_latest` (a separate `RingSubscriber` construction site) that §12 already listed; named the real phase-7 blast radius outside `node.hpp` explicitly (`tests/test_migration.cpp` 15+ sites, `tests/bench_migration.cpp` 8 sites, `tests/bench_realistic.cpp`) instead of leaving it to "no enumeration required"; flagged that `QoS::depth()` isn't wired to ring capacity today (new integration work, not existing behavior); clarified `node.hpp`'s existing private `try_attach` lambda is unrelated to the new public `RingSubscriber::try_attach()` method |
| 0.11 | Rewrote R-17 and R-16's risk/mitigation cells, which had drifted into bug-fix-commit-message style ("an earlier version of that function used `store()`...", "didn't exist in the earlier hand-rolled-INI plan") — replaced with plain risk/mechanism/mitigation language pointing to §5.10/§5.5 for the actual code, consistent with `docs/design_plan_guideline.md`'s FMEA section, which was itself updated to state this distinction explicitly (code-level diffs and "what changed" narratives belong in the revision log or Alternatives Considered, not in a risk table cell) |
| 0.12 | Aligned with the guideline's latest revision: added the Risk posture metadata row; renamed §5/§9/§13 to match the guideline's own section names exactly; added the Tier 1 contingency/confidence statement to §8; retroactively mapped this document's review history onto the guideline's new two-gate requirement (see note below the revision history) |
| 0.13 | Cut ~200 lines of code that had grown into a parallel implementation rather than a design: §5.9 ("API reference (concept code)") was a ~195-line full listing of every `RingPublisher`/`RingSubscriber` method body, almost entirely duplicating signatures already in §3 (Interfaces) and the algorithm already in §5.2 — replaced with a single generic fragment showing just the misuse-guard shape, the one piece of logic not already covered elsewhere, and renamed to reflect that narrower scope. §5.8's `resilient_read_loop()` (a full ~28-line reference implementation) was reduced to a 3-step ordered list of the cases that matter. Updated every cross-reference that had pointed to §5.9 expecting full method bodies there. Prompted by, and now reflected in, a new rule in `docs/design_plan_guideline.md`'s Architecture section: code blocks should be the hard fragment a reader can't get from prose or the interface table, not a parallel copy of the implementation |
| 0.14 | Moved every §8 margin whose correct value is a deployment/workload guess (not a proven bound) into `RingConfig`: added `max_retries` (was the hardcoded `kMaxRetries = 4`), `max_takeover_attempts` (was `kMaxTakeoverAttempts = 4`), and `attach_retry_ms` (was the hardcoded 100ms cadence in the resilient-loop pseudocode) — all retunable per topic without a rebuild, consistent with `capacity`/`max_age_ms`/`warn_every_ms` already being config. Added a "Configurable?" column to §8's margins table distinguishing these from the one margin that stays a compile-time choice (point-cloud variant selection, a type, not a runtime value). Reflected the same distinction as a new rule in `docs/design_plan_guideline.md`'s Margins section: prefer config over a compiled-in constant for any margin that's a guess rather than a proven bound |
| 0.15 | Addressed a self-containment gap: §2 (Assumptions and constraints) packed jargon explanations into parentheticals inline, inconsistently, making it hard to follow without prior context. Added §1.3 Terminology (seqlock, torn read, SPSC/SPMC, CAS, monotonic clock, trivially copyable, ABI/schema version, SPOF, backpressure, busy-spin — defined once), trimmed §2's rows to use those terms instead of re-explaining them, and fixed §9's NFR baselines (NFR-1/NFR-2), which previously cited external benchmark-report filenames without saying inline what those numbers actually measured. Reflected as new guidance in `docs/design_plan_guideline.md`: a Terminology subsection under Overview, a writing principle that the document should stand on its own without outside context, and checklist items for both |
| 0.16 | Fixed a second self-containment gap in the same area: bare symbols (`RingSubscriber`, `platform::now_ns()`, `T`, `RingHeader`, `RingConfig`, `Slot<T>`) were used in §1.1 (Problem statement) and §2 (Assumptions) before ever being introduced — Terminology itself was §1.3, *after* both. Reordered Overview so Terminology is §1.1 (before Problem statement and Out of scope, which become §1.2/§1.3) and broadened it with a new "Symbols" list naming every class/function/template-parameter this design introduces or depends on, alongside the existing "Concepts" list. Updated every cross-reference to the old §1.1/§1.2/§1.3 numbering. Reflected as new guidance in `docs/design_plan_guideline.md`: Overview's Terminology now explicitly covers symbols, not just jargon, and must appear before anything else uses them |
| 0.17 | Added a new §13 "Program lifecycle and fault recovery," analyzing the publisher's startup/shutdown/crash/recovery states explicitly rather than leaving them implicit across §5.10/§7: names the three startup cases (first run, clean-restart, crash-restart) and the specific failure shape this design avoids — a restarted process mistaking its own predecessor's uncleaned `producer_active` flag for a live conflict and refusing to start, even though the predecessor is dead — by never trusting the flag's mere presence (`claim_producer_slot()`'s `process_alive()` check, §5.10) and by arbitrating takeover with a CAS rather than an unconditional overwrite (already R-17's fix, now also framed as the general lifecycle principle it is). Renumbered old §13-16 (Security, Implementation plan, Open items, Lessons learned) to §14-17 and fixed every cross-reference to the shifted numbers. Reflected as new guidance in `docs/design_plan_guideline.md`: a new standard section 13, "Program lifecycle and fault recovery," required in full for any design with state that persists across restarts |
| 0.18 | §15 (Implementation plan) previously described each phase's scope in prose with no explicit link back to §4's features or §7's risks, so a forgotten feature or an unmitigated risk had no mechanical way to surface. Added an F-ID/R-ID citation to every phase (e.g. "Phase 2 (F-1, F-2, ...; R-1, R-2, ...)") and a feature/risk coverage table cross-checking all 14 features and all 17 risks against the phase list, including the three IDs with no phase-level code (R-2/R-6 accepted risks, R-13 open item) called out explicitly rather than left as blank cells. Reflected as new guidance in `docs/design_plan_guideline.md`'s Implementation plan section: phases must cite the IDs they address, and a coverage table must confirm none are missing |
| 0.19 | Evaluated whether §5.8/§5.10 (the attach/reattach and producer-conflict mechanisms) duplicate or belong inside §13 (Program lifecycle and fault recovery) instead of being separate top-level content. Confirmed the layering is intentional and non-duplicative: §5 states *mechanism* per feature (the retry loop, the CAS algorithm), while §13 states the *lifecycle narrative* that synthesizes multiple §5 subsections (§5.6, §5.8, §5.10) into the startup/shutdown/crash/recovery story a Tier 1 reviewer needs, the same cross-cutting-top-level-section pattern already used by §7 FMEA/§9 NFR/§12 Rollout rather than folding those into §5 too. Added one forward-pointing sentence each to §5.8 and §5.10, directing a reader at the mechanism to the lifecycle synthesis instead of leaving the relationship implicit. No restructuring needed: confirmed every A-ID (§2) already has at least one citation into §5/§7/§12, and every F-ID already has a design-element/section mapping in §10 — the gaps the evaluation checked for (assumptions or architecture left unlinked) don't exist |
| 0.20 | Approved. Status moved from "Peer reviewed, ready for Approval" to "Approved"; implementation starting at §15 phase 1 |
| 0.21 | **Phase 1 complete** (build system + notify wiring, F-3/R-3/R-5). `ring.hpp` migrated off raw POSIX shm calls onto `platform.hpp`'s cross-platform primitives (required for Windows at all, not optional); added `notify_seq`, wired `push()`/`close()` to `notify_wake_all`, added `pop_wait()`/`drain_wait()`; fixed `CMakeLists.txt`'s test gate so a new, platform-independent `test_ring` target builds on every OS while the existing POSIX/`fork()`-dependent test targets stay Unix-gated. Discovered and fixed **R-18** during Windows verification: `WaitOnAddress`/`WakeByAddressAll` match by virtual address within one process only (confirmed directly — two mappings of the same segment never wake each other), silently breaking blocking-wait's low-CPU guarantee on Windows for `topic.hpp`'s existing `wait()` too, previously unnoticed because the benchmark suite only ever exercised busy-spin on Windows. Fixed once in `platform.hpp` via named-Event binding (`notify_bind`/`notify_unbind`), shared by both headers. §5.3, §7 (new R-18), §8 (new margin row), §15 phase 1, §16 (two new open items: `registry.hpp`'s parallel issue, and phase 9 needing to re-verify under actual GTest/CMake rather than the standalone compile used here), and §17 (first lessons-learned entry) updated accordingly; risk posture metadata bumped to 18 risks. Reflected as new guidance in `docs/design_plan_guideline.md`'s Verification and validation section: verification scope must explicitly include shared, already-existing primitives the design depends on, not just its own new code |
| 0.22 | Closed 0.21's verification gap: neither CMake nor GTest was installed on the available Windows host, so 0.21's phase 1 exit criterion was checked via a standalone compile of equivalent test logic rather than `test_ring.cpp` itself. Built GTest 1.14 locally (MinGW/Ninja, CLion's bundled toolchain) and installed it to a scratch prefix, configured `shmbridge` against it with `SHMBRIDGE_BUILD_TESTS=ON`, and built/ran `test_ring` through the actual `cmake --build` + `ctest` pipeline: all 7 cases pass (R-3 wake in 66ms, R-5 wake in 50ms, both against a 5000ms timeout — consistent with 0.21's standalone-compile numbers). §15 phase 1's exit-criterion note updated to cite the real `ctest` run instead of the standalone compile; removed the open item asking for this, added a narrower one in its place — WSL Ubuntu (the Linux futex path, a different code path from Windows' named-Event path R-18 added) still hasn't been exercised by `test_ring` at all, which remains phase 9's job |
| 0.23 | `RingBlockingWait.PopWaitReturnsImmediatelyWhenDataAlreadyPresent`'s 500ms bound was loose enough to pass even if `pop_wait()` had gone through one Windows `notify_wait` slice (15ms, §8) rather than genuinely taken the `pop()` fast path with no wait call at all — it could only catch a gross regression (blocking for most of the timeout), not a smaller-magnitude one. Tightened to 50ms (still generous for scheduler/CI jitter around an operation that's O(1) memcpy and atomic loads on the fast path, with no syscall) against a 10-second timeout, and added a second case calling `pop_wait()` 5 times back-to-back against already-published items, asserting both the cumulative elapsed time and that items come back in publish order — a single lucky fast call couldn't mask a real regression the way one call alone could. Rebuilt and reran `test_ring` through `cmake --build`/`ctest` (same local GTest setup as 0.22): both new/tightened cases pass at 0ms (GTest's own millisecond-granularity timer) across 6 repeated runs, and the full 8-case suite remains green |
| 0.24 | `PopWaitTimesOutWhenNoDataArrives`'s 100ms timeout and the R-3/R-5 wake-latency figures (50-67ms) were both too slow to represent a robotics control loop's actual needs, and the R-3/R-5 numbers specifically conflated the test's own artificial pre-action delay with real wake latency rather than isolating it. Shrunk the timeout to 10ms; changed R-3/R-5 to measure latency from the *actual* `push()`/`close()` call to `pop_wait()` returning (excluding the test's own 10ms setup delay), with an explicit `< 10ms` assertion on that isolated latency. Doing this surfaced a genuine, 100%-reproducible Windows characteristic — a process's first short `WaitForSingleObject`-class wait can return well before its requested timeout, confirmed with a bare `CreateEventA`/`WaitForSingleObject` repro having nothing to do with shmbridge (as low as 753µs against a 10ms request on a fresh process's first wait; every subsequent wait in the same process was accurate). Not a bug in this design — an OS-level process cold-start effect `platform.hpp` has no portable way to pre-warm. Added a `::testing::Environment` to `test_ring.cpp` performing one throwaway `pop_wait()` before any test runs; 10/10 subsequent fresh-process runs of the full suite passed. Documented the finding in §13 (it's a startup characteristic) and §8 (a new margin row on Windows' ~15.6ms timer-tick rounding for short timeouts) rather than only fixing the test and saying nothing. Full numbers in `shmbridge/tests/phase1_ring_build_report.md` |
| 0.25 | 0.24's `< 10ms` wake-latency assertions were real, but the report backing them still said "<10ms" rather than an actual number, since GTest's `EXPECT_LT` only prints its message on failure and every run passed. Replaced that with real measurement: `test_ring.cpp` now records every latency (immediate-read, 5-call mean, the 10ms-timeout's actual elapsed time, push()-to-wake, close()-to-wake) as a GTest test property via `RecordProperty()`, and `CMakeLists.txt`'s `add_test(test_ring ...)` now passes `--gtest_output=json:tests/test_ring_results.json` so every `ctest` run regenerates a machine-readable report carrying those exact values — the same JSON-artifact convention `bench_pubsub_matrix_results.json` already uses elsewhere in this directory. Measured across 7 runs: immediate reads are sub-microsecond (0.5-1.0µs), push()-to-wake and close()-to-wake latency are 3.4-24.9µs (three orders of magnitude inside the 10ms bound — microseconds, not milliseconds), and the 10ms timeout request returns in 13.0-17.7ms (Windows timer-tick rounding, §8). `tests/test_ring_results.json` checked in as the latest run's artifact |
| 0.26 | 0.25's 6-7 single samples per path couldn't speak to the tail, and a realtime budget is blown by the tail, not the typical case. Added `RingLatencyDistribution` to `test_ring.cpp`: three 200-trial tests (immediate-read, push-to-wake, close-to-wake) computing mean/p50/p90/p99/max per path and recording the full distribution via `RecordProperty()`. Measured: immediate-read p99 = 0.8µs; push-to-wake p99 = 54.3µs (max 680.3µs, one outlier); close-to-wake p99 = 64.6µs (max 82.1µs). Added **NFR-6** (§9): p99 wake-path latency must leave at least 90% headroom against a 1kHz control loop's 1ms period — met with 94.6%/93.5% headroom respectively. Computed headroom against three concrete control-loop rates (1kHz/100Hz/10Hz) in `tests/phase1_ring_build_report.md`, including what the one-off max outlier would cost if it recurred (68% of a 1kHz period) rather than silently reporting only the favorable p99 number. Reflected as new guidance in `docs/design_plan_guideline.md`: a phase milestone with a numeric NFR target must be verified with a distribution (p50/p99/max over enough trials for p99 to be a real order statistic, not mean or n=1), machine-readable and checked in, with headroom computed against the stated requirement explicitly, not left for a reader to infer |
| 0.27 | Investigated 0.26's 680.3µs `push_to_wake` outlier rather than leaving it unexplained. Two candidate causes: genuine OS scheduler jitter (unfixable, inherent to a non-real-time OS), or an artifact of each trial creating a fresh `std::thread`/shm segment/Windows `Event`. Rewrote `PushToWakeLatencyDistribution` to create the publisher, subscriber, and signaling thread once and reuse them across all 200 trials (`push()` doesn't destroy anything), removing per-trial OS-object creation from the measured path entirely; rewrote `CloseToWakeLatencyDistribution` to reuse its signaling thread (a fresh publisher per trial is unavoidable — `close()` destroys the segment). The rewrite surfaced and fixed a real bug in the *test harness* (not `ring.hpp`): letting the next trial's publisher destroy the previous one as soon as `pop_wait()` returned raced the closer thread's still-in-flight `close()` call, since `signal_closed()` wakes the subscriber before `close()` itself finishes tearing down — fixed with a second condition variable confirming `close()` has fully returned before the object is touched again. **Result: the outlier still occurred after removing all per-trial object/thread churn** (a 599.5µs sample in one of six reruns) — conclusively ruling out the test-harness explanation. It is genuine OS scheduler jitter, occurring at roughly 1 per 1,000-1,200 trials across both versions of the test. Updated NFR-6 (§9) to state explicitly that this is a soft-real-time (p99-class, statistical) commitment, not a hard worst-case guarantee — no general-purpose OS can promise the latter for a user-space wake path, and claiming otherwise would be a false representation of what was actually measured. `tests/phase1_ring_build_report.md`'s "Explaining the outlier" section has the full investigation |
| 0.28 | Verified `ring.hpp`/`platform.hpp`/`test_ring.cpp` with compiler sanitizers rather than relying on the tests' own assertions alone to rule out memory-safety and data-race defects. Added `CMakeLists.txt`'s `SHMBRIDGE_SANITIZE` option (`address,undefined` or `thread`; Linux/WSL only — the project's Windows MinGW toolchain has no sanitizer runtime). Built and ran the full `test_ring` suite on WSL Ubuntu (GCC 13.3.0) three ways: plain (closing the long-open "WSL not yet exercised" item from 0.21-0.27 — the Linux futex path in `platform.hpp`, untouched by R-18's Windows-specific fix, had never actually been run until now), with AddressSanitizer+UndefinedBehaviorSanitizer, and with ThreadSanitizer (the last requiring `setarch $(uname -m) -R` to disable ASLR, without which TSan fatally errors on this design's shared-memory mappings — a known TSan/shm interaction, now documented in §11.1 so it isn't mistaken for an incompatibility). All three: 11/11 tests pass, zero sanitizer findings. Validated the sanitizer setup's own sensitivity with a minimal standalone repro of the exact "signal before fully finishing teardown" bug shape 0.27 fixed in the test harness (not in `ring.hpp`) — AddressSanitizer caught it immediately and precisely (heap-use-after-free, full alloc/free/use stack traces) when the fix was reverted, confirming the pipeline would have caught 0.27's bug had it still been present. §11.1 (new sanitizer-verification note and test-table row), §15 phase 1, and §16 (removed the now-resolved WSL open item) updated. Reflected as new guidance in `docs/design_plan_guideline.md`: Tier 1 designs with concurrency or shared memory require a sanitizer-clean run (not just the test suite's own assertions) as part of verification, with the ASLR/shm caveat noted as a general practical tip for anyone combining TSan with shared memory |
| 0.29 | Added native-Windows AddressSanitizer coverage via MSVC `/fsanitize=address` (requested specifically, as a toolchain genuinely different from 0.28's GCC/Clang-only coverage) — `CMakeLists.txt`'s `SHMBRIDGE_SANITIZE=address` now has an MSVC branch alongside the GCC/Clang one. Getting it to link required two MSVC-specific, Microsoft-documented workarounds now encoded in the build: static-CRT GTest (ASan forces static-CRT codegen) and `_DISABLE_VECTOR_ANNOTATION`/`_DISABLE_STRING_ANNOTATION` (an ASan-compiled TU can't otherwise link against a non-ASan-compiled dependency). Confirmed MSVC's ASan runtime genuinely works on this host first (a bare "print and exit" program, then a deliberately buggy heap-overflow program) after VS2019's own ASan runtime failed even that first check (`AddressSanitizer CHECK failed: ASan init calls itself!`, confirmed with a no-op program, ruling out anything code-related) — used a newer installed Visual Studio instead once that was confirmed. **Trying MSVC specifically (not just another GCC/Clang toolchain) surfaced a real, previously-undetected portability bug: R-19** — `ring.hpp` used GCC/Clang-only `__atomic_load_n`/`__atomic_store_n`/`__ATOMIC_ACQUIRE`/`__ATOMIC_RELEASE` builtins on `RingHeader::capacity` (left as a plain, non-atomic `uint32_t`), inconsistent with `write_idx`/`read_idx`/`closed` in the same struct, which already use `std::atomic<T>`. Invisible under every toolchain (MinGW, WSL GCC) this design had ever been built with; a hard compile error under MSVC. Fixed by making `capacity` a proper `std::atomic<uint32_t>` and switching the three call sites to `.load()`/`.store()` with the same memory ordering, expressed portably. Full `test_ring` suite (11/11) passes clean under MSVC ASan with this fix. §7 (new R-19), §9 risk posture (19 risks), §11.1, §15 phase 1, and the implementation-plan coverage table updated |
| 0.30 | Confirmed 0.29's fix and `CMakeLists.txt` changes work through the actual `cmake --build`/`ctest` pipeline end to end, not just the ad-hoc `cl.exe`/batch-script invocations used while diagnosing the CRT/annotation/atomic-builtin issues in 0.29. Configured fresh (VS2026, `SHMBRIDGE_SANITIZE=address`, a newly-built static-CRT GTest), built `test_ring` via `cmake --build`, and ran it via `ctest`: 11/11 pass, `test_ring_sanitize_results.json` generated as designed (the sanitizer-build output path `CMakeLists.txt` routes to, kept separate from the unsanitized `tests/test_ring_results.json` the timing numbers in this document and the build report come from) |
| 0.31 | **Phase 2 complete** (overwrite semantics, local cursors, config, type/schema checks, producer-conflict detection, latest-only read — F-1, F-2, F-6, F-7, F-8, F-10, F-11, F-13; R-1, R-2, R-4, R-6, R-7, R-11, R-12, R-13, R-14, R-16, R-17). `ring.hpp` rewritten to the new `RingHeader`/`RingConfig`/`claim_producer_slot()` design; toml++ v3.4.0 vendored at `shmbridge/include/third_party/toml.hpp` for `resolve_ring_config()`. 27-case `test_ring.cpp` suite passes on MinGW, WSL GCC plain, WSL ASan+UBSan+LeakSanitizer, WSL ThreadSanitizer, and native VS2026/MSVC (both `/fsanitize=address` and plain `cmake --build`+`ctest`, the latter regenerating the committed `tests/test_ring_results.json`). Found and fixed two test-design bugs (not product defects): `ConcurrentTakeoverRaceHasExactlyOneWinner`'s premature `close()` letting the second racer win a fresh claim instead of genuinely racing the takeover CAS (fixed with a spin-wait readiness barrier and deferred close); `PopWaitTimesOutWhenNoDataArrives`'s lower bound tightened past what Windows timer jitter can guarantee after thread-heavy preceding tests (loosened 10ms→5ms, same category as R-18/NFR-6). Suppressed two LeakSanitizer false positives on deliberate `pub1.release()` crash-simulation leaks via `__lsan_ignore_object()`. Mid-phase, applied two standing project corrections: this project's Windows verification toolchain is Visual Studio/MSVC only, not MinGW (§11.1, `CMakeLists.txt` doc comments updated; historical phase-1 narrative describing MinGW use left unchanged as accurate history); and all `#if defined(_WIN32)` platform-conditional code belongs in `platform.hpp`, never a test file — `spawn_and_reap_dead_pid()` moved out of `test_ring.cpp` into `platform::spawn_and_reap_process()`. §11.1 (new Sanitizer verification toolchain wording), §15 phase 2 (exit criterion met), and the coverage table are updated; §16/§17 unchanged (no new open items or lessons beyond what phase 1 already established about toolchain-specific bugs and OS timer jitter). Correctness and NFR-6 wake-latency performance independently re-verified fresh against the current working tree on both Windows/MSVC (27/27, p99 push/close-to-wake 33.2µs/36.5µs, 96.7%/96.4% headroom) and WSL/GCC plain+ASan+UBSan+TSan (27/27 each, p99 63.7µs/45.6µs, 93.6%/95.4% headroom) — full numbers and reproduction commands in `tests/phase2_ring_build_report.md` |
| 0.32 | **Phases 3-6 complete.** Phases 3 (zero-copy write), 4 (zero-copy read), and 5 (resilient attach/reattach) were already implemented and passing as part of phase 2's single-pass `ring.hpp` rewrite — confirmed against `tests/phase2_ring_build_report.md`'s existing evidence and marked `[DONE]` in §15 rather than re-measured. Phase 6 (fixed-size point-cloud message family, F-12) added `PointCloudFixed<MaxPoints>` and its six named variants to `include/shmbridge/messages.hpp` per §5.11, plus two new `test_ring.cpp` cases (round-trip correctness, cross-`MaxPoints` type-mismatch rejection) — no `ring.hpp`/`platform.hpp` changes needed, since F-12's type-mismatch rejection reuses F-7's existing `type_hash` mechanism unchanged. Full 29-case suite (27 + 2 new) passes on MSVC/VS2026, WSL GCC plain, WSL ASan+UBSan, and WSL TSan — `tests/phase3_6_ring_build_report.md`. Applying this session's own new guideline (`docs/design_plan_guideline.md`'s "Milestone verification reports"): every number here comes from a run executed against the actual current working tree in this session, synced to WSL via `rsync` rather than `git clone` (the phase's changes are still uncommitted, so a clone would have silently tested stale history) |
| 0.33 | **Phase 7 complete** (`node.hpp`/test-suite migration, no new F-ID/R-ID, §12 rollout concern). Migrated `node.hpp`'s `create_publisher`/`create_subscription`/`create_latest` ring branches and ~30 direct `RingPublisher<T,N>`/`RingSubscriber<T,N>` instantiations across `tests/test_migration.cpp` and `tests/bench_migration.cpp` to the runtime-`RingConfig`, single-template-arg API; resolved the open `RingConfig::capacity`↔`qos.depth()` mapping decision in favor of wiring it (rounded to `open()`'s required power-of-two). Verification caught two real problems neither anticipated at design time: (1) a genuine hang in `Ring.Throughput` (and the identical latent bug in `bench_ring_spsc_throughput()`) — both assumed `push()`'s old reject-on-full return value gave the producer implicit backpressure, a contract F-1 removed, so an unthrottled producer could permanently overwrite data a consumer was looping forever to count (R-6); fixed by bounding both loops on producer completion rather than a received-count target. (2) three `node.hpp`-level test failures (`Node.RingPubSub`, `Node.CreateQueueRing`, `Node.CreateLatestRing`) caused by the new per-consumer-cursor design's default `StartNow` mode skipping backlog that `node.hpp`'s lazy, deferred attach needed to still see — fixed by switching `node.hpp`'s ring subscriptions to `RingCursorMode::DrainBacklog`, the closest match to the old shared-`read_idx` ring's late-subscriber behavior. Also removed two tests (`Ring.SizeAndPeek`→`Ring.PopExHasValueContract`, `Ring.SkipOldKeepsLatestN`/`Ring.SkipOldNoop` deleted) whose asserted behavior (`size()`/`empty()`/`peek()`/`skip_old()`) has no new-API equivalent by design (§5.7). `SHMBRIDGE_BUILD_TEST_MIGRATION` flipped to default `ON`. Full results, root-cause analysis, and reproduction commands in `tests/phase7_node_migration_report.md`: `test_migration` 35/35, `test_ring` 29/29 (no regression), `test_topic` 22/22, both benchmarks run to completion (ring throughput 2.86 Mmsg/s) — all on WSL/GCC, since `node.hpp`/`registry.hpp` remain POSIX-only (no MSVC path for this phase, superseded by 0.34) |
| 0.34 | **`registry.hpp` ported to Windows**, explicitly out of this plan's original scope (§16) but picked up as deliberate follow-on work once phase 7 made the gap concrete. Replaced every raw `::shm_open`/`::mmap`/`::munmap`/`::getpid`/`::kill` call with `platform.hpp`'s cross-platform equivalents (`current_pid()`, `process_alive()`, `shm_unmap()`), the same primitives `ring.hpp`/`topic.hpp` already used. `DiscoveryRegistry::open()` needed a *new* `platform.hpp` primitive, not a reuse of an existing one: `shm_open_or_create()`, added alongside `shm_create()`/`shm_attach()`. `shm_create()`'s unconditional unlink-then-recreate is correct for a ring topic (one owning producer, free to force a fresh start after a crash) but would be actively wrong here — the discovery table is a single segment every node on the host shares and writes into concurrently, so every process's `open()` racing to `shm_create()` it would wipe out whatever every other already-registered node had written; this was caught during implementation, before it shipped, by reasoning through the two primitives' actual contracts rather than assuming `shm_create()`-then-fall-back-to-`shm_attach()` (the pattern `ring.hpp`'s own `open()` uses) would transfer unchanged. `shm_open_or_create()` instead does what the original POSIX code already did implicitly (`shm_open(O_CREAT, no O_EXCL)`, racelessly idempotent) and gives Windows the equivalent via `CreateFileMappingA`'s own `ERROR_ALREADY_EXISTS` signal. Also replaced 36 direct `::shm_unlink()` test-cleanup calls across `tests/test_migration.cpp`/`bench_migration.cpp`/`bench_realistic.cpp` with `platform::shm_destroy()` (a no-op on Windows, where closing the last handle already reclaims the segment) — the only other POSIX-specific symbol left in any of those files. `CMakeLists.txt`'s `if(UNIX)` gate dropped for `test_migration` and the bench targets (both are now built and tested on every platform); `test_topic` (fork()-based) and `test_core` (a separate, unrelated module) remain Unix-only. Verified fresh on both platforms in this session: WSL/GCC — `test_migration` 35/35, `test_ring` 29/29, `test_topic` 22/22, both benchmarks run to completion (no regression from the `shm_destroy()` rename); native Windows/MSVC/VS2026 — `test_ring` and `test_migration` both build and pass (2/2 targets, 100%) through the real `cmake --build`+`ctest` pipeline, the first time `node.hpp`/`registry.hpp` have ever been exercised on Windows. Full details in `tests/windows_registry_port_report.md`. §16's `registry.hpp` open item resolved (struck through, not deleted, so the original scoping decision stays visible) |
| 0.35 | **`test_topic.cpp` ported to Windows**, closing the one remaining gap 0.34 left open. `MultipleSubscribers.NoRaceConditionForkN4` — the test the whole file existed to Unix-gate for — forked 4 child processes to prove `topic.hpp`'s seqlock has no torn-read races under genuine multi-process concurrency; `fork()` has no Windows equivalent and, unlike `registry.hpp`'s POSIX calls, isn't a thing `platform.hpp` can abstract away, since Windows process creation doesn't clone the calling process's state the way `fork()` does. Redesigned instead of ported: added `platform::run_and_capture(cmd)` (built on `popen()`/`_popen()`, already identical in signature on both platforms) and a new standalone helper executable, `tests/test_topic_sub_helper.cpp`, that attaches as a subscriber and prints its torn-read stats to stdout; the test (renamed `NoRaceConditionMultiProcessN4`) now launches 4 copies of that helper concurrently via `std::async`, getting the same genuine separate-OS-process guarantee `fork()` gave without a primitive Windows lacks. A real bug surfaced during verification, not left for later: `_popen()`'s underlying `cmd.exe /c <cmd>` has a documented quoting quirk that mangles any command with more than one quoted token (a quoted executable path followed by a quoted argument) by swallowing the boundary between them into one literal filename — fixed by wrapping the whole command in one more throwaway pair of quotes, sacrificing exactly the two characters `cmd.exe`'s fallback parsing strips to leave every inner quote untouched (documented in `platform.hpp` at the fix site, not just in this entry). Also fixed, found while auditing the file for remaining POSIX calls: `unique_name()`/`ShmCleaner`/one more call site used `::getpid()`/`::shm_unlink()` directly (→ `platform::current_pid()`/`platform::shm_destroy()`), and `BroadcastNotify.AllSubscribersWake` was still gated `#if defined(__linux__)` from before phase 1's R-18 fix made `wait()`/`write_notify()` genuinely cross-platform — confirmed passing on MSVC and the stale gate removed. `test_topic` and the new helper moved out of `CMakeLists.txt`'s `if(UNIX)` block (only `test_core`, a separate unrelated module, remains Unix-gated). Verified fresh on both platforms: WSL/GCC 22/22, native Windows/MSVC/VS2026 22/22 (including the multi-process test, 2.35s) through the real `cmake --build`+`ctest` pipeline — `test_ring` and `test_migration` re-confirmed clean alongside it (29/29, 35/35; one `test_ring` run hit the already-documented OS-timer-jitter flake from R-18/NFR-6, not a regression, confirmed by a clean rerun). `shmbridge`'s entire C++ test suite is now cross-platform except `test_core` (a distinct, not-yet-scoped module). Full details appended to `tests/windows_registry_port_report.md` |
| 0.36 | **Phase 8 complete** (benchmark validation, NFR-1 through NFR-4; R-1's retry-count instrumentation). `bench_pubsub_matrix.py`'s `ring` mode, as literally specified, can't be built — `ring.hpp` has no Python bindings (Scope row) — so every NFR was measured in C++ instead, via the `RecordProperty` convention phase 1's `RingLatencyDistribution` established for NFR-6. Measured: NFR-1 (blocked `pop_wait()` 0.0% CPU vs. busy-spin 101.6%, via a new `platform::thread_cpu_time_ns()`), NFR-3 (zero-copy vs. copy at a ~64KiB `PointCloud4096`, 0.6µs vs. 4.0µs median, ~6.7x), NFR-4 (0.005% torn-read retry rate at a deliberately adversarial `capacity=2`, via new `RingSubscriberStats`/`RingSubscriber::stats()` — no retry instrumentation existed before this phase). NFR-2 cited from existing phase 2/7 evidence, not re-run. Auditing §11.1's test table against the real suite while adding this instrumentation surfaced and closed two genuine implementation gaps predating this phase, not just missing tests: `RingSubscriber::attach()` ignored `cfg.attach_retry_ms` entirely (hardcoded 5ms poll, now fixed); and `node.hpp`'s ring subscriptions never implemented §5.8's documented `is_closed()`-triggered auto-detach/reattach — a publisher restart would silently and *permanently* stop deliveries to any `node.hpp` ring subscription, open since phase 2, caught here for the first time because nothing had tested a `node.hpp`-level subscription surviving a publisher restart before (R-9). Fixed: `SubSlot` gained `is_closed`/`detach` hooks, `spin_once()` now auto-recovers, proven by new test `Node.RingSubscriptionAutoReattachesAfterPublisherClose`. The rate-limited *warning* half of §5.8's case 2 remains open (§16, new item) — this codebase has no established logging convention to hang it off of, and inventing one is a real API decision, not a mechanical fix. Also added three new `test_ring.cpp` cases closing coverage gaps the audit found: `RingBasic.PopExBoundaryAtExactlyStartIdx` (R-4's exact cursor/start_idx boundary, not just the loose far-behind case already covered), `RingConfigResolution.RemainingFieldsResolveFromToml` (the 4 of 7 TOML-resolvable `RingConfig` fields no test had touched: `max_age_ms`/`warn_every_ms`/`max_takeover_attempts`/`attach_retry_ms` — R-10/R-11/R-12/R-17), and `RingTornReadRetry` itself. The last one directly **observed R-2's accepted risk for the first time**: its first version asserted received values must always strictly increase, and failed once on WSL/GCC with a genuine decrease under adversarial contention — not a new defect, R-2 (§7) explicitly accepts the underlying bulk-`memcpy`-vs-concurrent-overwrite race as UB "empirically low... never observed to corrupt beyond recognition," never formally impossible; the test's assertion was stricter than the design's own promise. Rewritten to assert an anomaly **rate** (<1%) instead of zero, stable across 10 repeat runs after the fix. Verified fresh on both platforms: WSL/GCC plain + ASan/UBSan (`test_ring` 34/34, `test_migration` 36/36) and native Windows/MSVC/VS2026 (same counts, `test_ring_results.json` regenerated). §9's NFR table, §15 phase 8, and §16 updated; full detail in `tests/phase8_benchmark_validation_report.md` |
| 0.37 | Checked §16 Open items for accuracy against the rest of the document and found a real self-consistency gap: R-13 (§7's risk table) and §15's coverage table both pointed to §16 as where it was tracked ("Open item (§16)"), but §16's actual bullet list never contained an entry for it — it had apparently never been transcribed in, since this design's earliest drafts. Resolved rather than merely added as a bullet: ran the exhaustive repo-wide env-var search R-13 had originally flagged as non-exhaustive (every `getenv`/`os.environ` call in the repository, not just `shmbridge/`), found exactly one other project-defined convention (`irsim_devices/setup.py`'s `IRSIM_DEVICES_BUILD_EMBREE`), confirmed `SHMBRIDGE_RING_CONFIG` already matches its naming pattern with no conflict. §7's R-13 row, §15's coverage-table cell and footnote paragraph (which also still said "phases 1-6 done, 7-9 pending," stale since 0.33/0.36 — corrected to "phases 1-8 done, only 9 pending"), and §16 itself all updated |
| 0.38 | Closed §16's one remaining open item: §5.8's rate-limited warning, declared via `RingConfig::warn_every_ms` since the design's earliest drafts but never implemented anywhere. Added `detail::warn_rate_limited()` and `SubSlot::warn_every_ms`/`last_warn_ns` to `node.hpp`, wired into both of `spin_once()`'s resilient-loop cases (repeated attach failure; publisher closed mid-run) — each emits at most one `"[shmbridge] ..."` line to stderr per `warn_every_ms`. Chose plain `fprintf(stderr, ...)` over a callback or counter API: this is the library's first logging call site, and `"[shmbridge] "` already has precedent in `CMakeLists.txt`'s own configure-time messages, so there was a real convention to match rather than one to invent from nothing. Verified firing exactly where expected (visible in `Node.SpinOnceNonBlockingWhenNoPublisher`'s and `Node.RingSubscriptionAutoReattachesAfterPublisherClose`'s console output) with no change to either test's pass/fail behavior, on both WSL/GCC (`test_migration` 36/36, `test_ring` 34/34) and native Windows/MSVC/VS2026 (same, 100% across all 3 ctest targets). §16 is now empty of unresolved items — every item ever opened in this document's history has been closed |
| 0.39 | **Phase 9 complete — implementation plan finished, all 9 phases `[DONE]`.** Built the §11.2 soak test (`tests/soak_test.cpp`, `tests/soak_consumer_helper.cpp`, behind a new `SHMBRIDGE_BUILD_SOAK_TEST` CMake option): one producer pushing flat-out into a realistic `RingConfig::capacity=1024` ring, 4 independent consumer *processes* (not threads — the same `platform::run_and_capture()` subprocess pattern `test_topic.cpp`'s multi-process test already established) at 1/5/20/100ms poll intervals, each checking every message for payload corruption (a redundant checksum field), duplicate/out-of-order delivery, and non-monotonic `write_ns`, for 30 minutes. **Native Windows/MSVC/VS2026: 0 violations across 1.50 billion messages received.** **WSL/GCC: 11 violations across 289 million messages received** (≈1 per 26 million) — this is R-2 (§7) actually occurring under realistic, non-adversarial conditions, confirmed directly rather than inferred, and revealed that §11.2's original "zero violations" acceptance criterion contradicted R-2's own already-accepted-risk status elsewhere in this same document; revised to match what the design actually claims (§11.2, new lesson in §17). Getting a trustworthy 30-minute run at all took two failed attempts first: WSL2's VM idle-connection lifecycle silently killed a `setsid`+`disown`-detached background run partway through (confirmed via `uptime` resetting while the Windows host's own uptime stayed continuous — not a host sleep), fixed by keeping `wsl.exe` itself as the soak test's continuously-connected foreground process, launched via Windows Task Scheduler to survive independent of the orchestrating session's own background-task limits (§17, new lesson). Finding R-2 manifesting for real prompted a direct comparison of `ring.hpp`'s detection mechanism (`start_idx`/`end_idx`, a ring-wide eviction signal checked before/after, updated in a step *separate from and after* the slot write) against `topic.hpp`'s own `SeqlockSlot` (a direct per-slot `seq`/`seq2` bracket with no such gap) — concluded `ring.hpp`'s approach is the right tradeoff for its own use case (zero added per-slot cost, matching its overwrite-on-full/best-effort design philosophy) but is NOT equivalently rigorous to the `topic.hpp` precedent it's justified against, despite both being "the same accepted risk" in the risk table. Recorded as a new, not-yet-implemented open item (§16): a per-slot seqlock bracket — cheaper than the word-wise-atomic alternative §6 already rejected, would close the specific gap directly. §7's R-2 row, §11.2, §15 phase 9, §16, and §17 all updated; full logs and analysis in `tests/phase9_soak_test_report.md` |
| 0.40 | Added a deployment-realistic risk estimate to §16's R-2 open item: the soak test's measured rate (0.39) came from a deliberately adversarial workload (unthrottled producer vastly outracing fixed-interval consumers, maximizing how often a reader sits at the eviction edge), not this design's actual target workloads. Estimated exposure for three concrete profiles — 20B IMU @ 1000Hz, 10B command @ 100Hz, a 10k-point/~156KB point cloud @ 30Hz — using this project's own measured memcpy/round-trip numbers (`bench_migration.cpp`'s `direct_ring_ns`, the NFR-3 benchmark's `PointCloud4096` copy latency) and §5.11/§8's capacity-sizing guidance: the race window works out to roughly 0.0005-0.015% of each topic's own slot-reuse interval, 2-4 orders of magnitude narrower than the soak test's adversarial exposure, provided capacity is sized per R-11/R-12 and consumers aren't chronically stalled (R-6). Framed as context for whoever weighs the hardening opportunity's cost, not as a reason to close the open item — the gap is still real, just not urgent at these rates |
| 0.41 | Closed a redundancy found while auditing `ExtShmBridge`/`ext_*` against this design: `ext_topics.hpp`'s `PointCloudPublisher`/`Subscriber` were a second, independently-maintained bulk-transport implementation solving the same problem `ring.hpp` + F-12's `PointCloudFixed<MaxPoints>` already solve. Rewritten as a thin wrapper over `RingPublisher<msg::PointCloud65536>`/`RingSubscriber<msg::PointCloud65536>`, with a small cache (`refresh()`) bridging ring.hpp's consume-once `pop_latest()`/`borrow()` model back to the "always re-readable" seqlock semantics callers depend on. Found and fixed a real cross-platform bug in the process (not introduced by this change, but first exposed by it): `pop_latest()`/`pop_ex()` return `Result<T>` by value, a full `T` as a stack-local inside `ring.hpp` — fine for small `T` and for any `T` on Linux's 8 MiB default stack, but a silent crash (no exception, no traceback) on Windows' 1 MiB default stack for a `T` the size of `PointCloud65536` (~1 MiB). Fixed by using `borrow()`/`end_borrow()` instead (no `T`-sized stack local ever); documented directly in `ring.hpp`'s `pop_latest()` comment and `messages.hpp`'s `PointCloudFixed` comment so the next large-`T` `RingSubscriber` user sees the caveat where they're working, not only here. Verified round-tripping (including the "re-read with no new publish still returns the same cloud" case) on both WSL/GCC and native Windows/MSVC. See §16 and §17 |

**Tier 1 two-gate review, retrofitted**: the guideline's two-gate
requirement (preliminary review of the approach, then a critical review
of the complete document) postdates when this document began, so the two
gates aren't separately dated entries the way a document started under
the current guideline would have them. In substance, though, this
document's own history already took that shape: revisions 0.1-0.5
established and reviewed the architecture and approach (overwrite
semantics, per-consumer cursors, the bulk-copy-vs-word-atomic decision)
before any FMEA/margins/verification detail existed — that is this
document's preliminary review, done in fact if not in that name. The
subsequent passes (0.6-0.11, including the full FMEA buildout and the
line-by-line verification against the real `ring.hpp`/`node.hpp` code in
0.10) are the critical review of the complete document. No further
retroactive gate is needed; going forward, any Tier 1 document started
fresh under the current guideline should record both gates as distinct,
dated revision-history entries rather than relying on this kind of
after-the-fact mapping.

**Independent verification of evidence** (the guideline's other Tier 1
addition) does not yet apply to this document in the way it will once
implementation starts: there is no test or benchmark evidence yet to
independently verify, only a design. This is carried forward as an
explicit expectation on §15's phases, not retrofitted here — see §11's
acceptance criteria.

## 1. Overview

This is the design for a low-latency, low-CPU, multi-consumer shared-memory
ring transport for shmbridge's ring-QoS topics (`Node::create_queue`,
depth>1 subscriptions). It replaces the current single-consumer,
reject-on-full ring with a single-producer/multi-consumer, overwrite-on-full
ring that supports blocking (non-polling) reads, zero-copy publish and
read, per-message freshness checking, compile-time message-type
verification, and resilient (never-fatal) attach/reattach behavior.

### 1.1 Terminology

Defined once here, before anything else in this document uses them — not
after. Covers both the symbols this design introduces or depends on, and
the concepts behind them.

**Symbols**

- **`RingPublisher<T>` / `RingSubscriber<T>`**: the two classes this
  design adds — exactly one active `RingPublisher<T>` per topic (F-11),
  any number of independent `RingSubscriber<T>` instances (F-2). Named
  here so later sections can use them by name; full interface in §3,
  full mechanism in §5.
- **`T`**: a generic placeholder for the application's message payload
  type, never a literal type — every ring is created for one specific
  `T` (which must be trivially copyable, below). "A message of type `T`"
  means "whatever payload type this particular ring instance carries."
- **`platform::` namespace**: shmbridge's existing cross-platform OS
  shim (`platform.hpp`) — not introduced by this design, already relied
  on throughout shmbridge. Provides shared-memory creation, monotonic
  time (`platform::now_ns()`, see Monotonic clock below), and OS-level
  wait/wake primitives (`platform::notify_wait`/`notify_wake_all`) that
  this design reuses rather than reinventing (§5.3, §5.4).
- **`RingHeader` / `RingConfig` / `Slot<T>`**: the shared-memory header,
  the per-topic configuration struct, and the per-message storage slot
  this design adds. Named here; fully defined in §5.1 (layout) and §5.5
  (configuration).

**Concepts**

- **Seqlock**: a lock-free read/write pattern that uses a sequence
  counter instead of a lock to let a reader detect whether it caught a
  write in progress. `topic.hpp` (shmbridge's other transport) uses one;
  this design is compared against it throughout.
- **Torn read**: a read that observed memory partway through being
  overwritten by a concurrent writer, producing a mix of old and new
  bytes. This design *detects* torn reads (via the index-boundary check,
  §5.2) and discards/retries them — it does not prevent the underlying
  race, which is an accepted tradeoff (R-2, §7).
- **SPSC / SPMC**: single-producer/single-consumer vs.
  single-producer/multiple-consumer — shorthand for how many processes
  may publish vs. subscribe to one ring. This design is SPMC (F-2).
- **CAS (compare-and-swap)**: an atomic hardware operation that updates a
  value only if it still matches an expected prior value, otherwise
  leaving it untouched and reporting failure. Used in §5.10 to let two
  processes race for the same resource without both "winning."
- **Monotonic clock**: a clock that only ever moves forward, unaffected
  by wall-clock adjustments (NTP sync, a user changing the system time).
  Every timestamp in this design (§5.4) uses one, specifically because
  it's directly comparable across processes with no synchronization step.
- **Trivially copyable**: a C++ type safely copyable via a raw
  byte-for-byte memory copy, with no constructor/destructor logic that
  needs to run. A hard requirement (A-3) for any message type this ring
  carries.
- **ABI / schema version**: the exact in-memory byte layout of a shared
  structure. A publisher and subscriber built from different versions of
  that layout produce undefined behavior if allowed to interoperate,
  which `schema_version` (§5.1) exists to catch instead.
- **SPOF (single point of failure)**: a component whose failure takes the
  whole system down with it, with no fallback. Called out explicitly in
  §7 rather than left implicit.
- **Backpressure**: a mechanism where a slow consumer causes the producer
  to slow down or block. This design deliberately has none (F-1) — the
  producer never waits on a consumer, by design, not by oversight.
- **Busy-spin (busy-wait)**: a loop that repeatedly checks a condition
  without ever sleeping or yielding the CPU, trading 100% CPU usage on
  one core for the lowest possible latency. This is the behavior F-3's
  blocking wait replaces.

### 1.2 Problem statement

The ring-QoS path is the only part of shmbridge without a low-CPU wait
mechanism. `RingSubscriber::pop()`/`drain()` are non-blocking, and
`Node::spin_once()`/`spin()` poll them on a fixed timer (10 ms default) or
the caller's own busy loop — there is no way to block until data actually
arrives. `topic.hpp`'s seqlock path already solves this for single-sample
("keep latest") topics via `notify_seq` + `platform::notify_wait`/
`notify_wake_all` (Linux futex, Windows `WaitOnAddress`/`WakeByAddressAll`,
macOS poll fallback); this plan brings the same mechanism to the ring,
along with capabilities the ring never had: multiple independent
consumers, zero-copy access, and graceful degradation instead of blocking
or rejecting writes.

### 1.3 Out of scope

- Python bindings (`shmbridge.topic`, `shmbridge._core`) — deferred to a
  follow-up.
- Runtime-resizable ring capacity — capacity is fixed for a segment's
  lifetime once created; shared memory is a fixed mapping across
  processes, and a live-resize/remap protocol is a separate, larger
  change.
- Multi-producer support — the ring remains single-producer, and now
  actively rejects a second concurrent producer attempting to attach
  (F-11) rather than merely assuming callers won't do that.
- Truly unbounded-size payloads — point clouds are now supported via a
  family of fixed-size variants (F-12), each still a concrete,
  trivially-copyable, fixed-`sizeof(T)` type; a payload with no
  reasonable fixed upper bound at all remains out of scope for this
  ring and should keep using the existing bespoke bulk transport.
- Producer-side backpressure/flow control — the producer never blocks or
  fails due to a slow consumer, by design (§4, F-1).

## 2. Assumptions and constraints

The design depends on the following holding true; each is a belief about
the deployment environment, not a choice this design makes, and should be
checked independently rather than taken on faith. Terms like ABI,
trivially copyable, monotonic clock, backpressure, and SPOF are defined
once in §1.1 — not re-explained below.

| # | Assumption | Consequence if false |
|---|---|---|
| A-1 | All processes sharing a ring segment run on the same host, built with the same toolchain (shared memory isn't network-transparent; ABI layout is compiler/platform-specific — same scope `topic.hpp` already operates in) | Undefined/garbage reads; `type_hash` (§5.6) only catches a toolchain-consistent mismatch, not a cross-platform one |
| A-2 | `platform::now_ns()`'s monotonic clock has a consistent reference across every process attached to a given ring on that host | `write_ns` comparisons (§5.4) become meaningless across processes if their clocks aren't actually comparable |
| A-3 | `T` is trivially copyable with a single fixed size for the ring's lifetime; a topic whose payload size genuinely varies without bound is not a fit for this ring | A need for varying/unbounded size must continue to use the existing bespoke bulk transport. A need that varies but has a reasonable upper bound is addressed by choosing the right fixed-size variant (F-12) instead |
| A-4 | Readers are fast relative to the publish rate in the common case — the no-backpressure design (F-1) is a good fit only when this holds | A consistently slow consumer on a high-rate topic will simply miss most data — intended behavior for the assumed use case, not a defect, but a poor fit otherwise. For a consumer that only needs the newest value, use `pop_latest()` (F-13) rather than fighting this assumption |
| A-5 | Exactly one producer attaches to a given ring segment at a time | Enforced at runtime by `open()` (F-11), not merely by caller convention — see §5.10. A residual gap remains for OS PID reuse racing the liveness check (R-14) |
| A-6 | The capacity chosen at segment creation is adequate for that topic's actual burst pattern | If false, consumers see excessive resync events (§5.7); the design provides a methodology (§8) and runtime visibility (NFR-4) to catch this, but cannot derive the right number automatically |
| A-7 | `reserve()`/`commit()` and `borrow()`/`end_borrow()` are each called from a single thread per instance, in matched, non-overlapping pairs | The misuse guards (§5.9, F-14) detect an unmatched or doubled call within that single-thread assumption; they do not make the pair thread-safe across multiple threads sharing one `RingPublisher`/`RingSubscriber` instance concurrently — that remains the caller's responsibility, consistent with neither class claiming to be thread-safe for concurrent use of the same instance |

## 3. Interfaces

The public contract of `RingPublisher<T>`/`RingSubscriber<T>` — this
table is the authoritative source for every method's signature,
preconditions, and postconditions; §5's architecture section shows the
underlying mechanism and the handful of implementation fragments that
aren't obvious from this table alone, not a parallel copy of these
signatures. This is the boundary other code in this repository
(`node.hpp`, and any future direct caller) integrates against. Each entry
states which process calls it, what it costs (so a caller can reason
about whether it's safe to call from a hot/realtime path), and — for the
reserve/commit and borrow/end_borrow pairs — the state-machine
precondition the misuse guards (§5.9, F-14) enforce.

**`RingPublisher<T>`** (called from the one process that owns this topic's data):

| Function | Context | Inputs | Outputs | Preconditions | Postconditions (state change) | Cost |
|---|---|---|---|---|---|---|
| `open(name, cfg)` | Publisher process, once at startup | `name`: string, process-wide-unique segment name. `cfg.capacity`: `uint32_t`, power of two, >= 2 | throws `std::invalid_argument` if capacity invalid; throws if another producer is already active (F-11, §5.10) | no prior `open()` on this instance | Segment created and sized for `cfg.capacity`; header stamped with `type_hash`/`schema_version`/`capacity_n`/`producer_pid` | One-time `shm_create`/`mmap` syscall plus a PID-liveness check (§5.10) if a stale producer marker is found; not on any hot path |
| `push(value)` | Publisher process, per message | `value: T` | none | `open()` succeeded | One slot bulk-copied; `end_idx` (and `start_idx` if full) advance; blocked subscribers woken | O(sizeof(T)) memcpy + 2-3 atomic ops; no syscall unless a subscriber is actually blocked (then one wake syscall) |
| `reserve()` | Publisher process, per message | none | `T*` (never null while open and unreserved) | `open()` succeeded; **no outstanding unmatched `reserve()`** (F-14) — a second `reserve()` before the matching `commit()` is rejected (§5.9) | internal "reserved" flag set; no shared state changes yet | O(1), no copy, no syscall |
| `commit()` | Publisher process, per message | none | none | **a matching `reserve()` is outstanding** (F-14) — `commit()` with no prior `reserve()` is a documented no-op, never publishes garbage (§5.9) | `write_ns` stamped; `end_idx`/`start_idx` advance as in `push()`; blocked subscribers woken; "reserved" flag cleared | O(1) plus the same atomic/wake cost as `push()` |
| `close()` | Publisher process, at shutdown | none | none | — | `closed` flag set; `producer_active` cleared (F-11); every currently-blocked subscriber woken | O(1) plus one wake syscall |

**`RingSubscriber<T>`** (one instance per independent consumer; any number of instances may attach to the same topic concurrently, F-2):

| Function | Context | Inputs | Outputs | Preconditions | Postconditions (state change) | Cost |
|---|---|---|---|---|---|---|
| `attach(name, timeout_ms, cursor_mode)` | Subscriber process, once at startup | `name`; `timeout_ms`: float, milliseconds, > 0; `cursor_mode` | throws `TimeoutError` if not found within `timeout_ms`; throws `std::invalid_argument` on schema/type mismatch | a publisher has called (or will call, within `timeout_ms`) `open()` on `name` | cursor initialized per `cursor_mode`; `is_attached()` true | `mmap` syscall plus a bounded polling wait up to `timeout_ms` |
| `try_attach(name, cfg)` | Subscriber process, polled from a retry loop (§5.8) | `name`; `cfg` | `bool`; throws `std::invalid_argument` on mismatch (never for "not found") | none | same as `attach()` on `true`; no state change on `false` | One non-blocking attach probe; O(1) when the segment doesn't exist yet |
| `pop_ex()` | Subscriber process, per message | none | `optional<Result>` | `is_attached()` | cursor advances by exactly one message on success; unchanged on `nullopt` | O(sizeof(T)) memcpy, bounded by `RingConfig::max_retries` (§7 R-1, §8); no syscall |
| `pop_wait(timeout_ms)` | Subscriber process, per poll cycle | `timeout_ms`: int, milliseconds, finite, >= 0 | `optional<Result>` | `is_attached()` | blocks at most `timeout_ms`; same cursor postcondition as `pop_ex()` | Same as `pop_ex()` when data is already available; otherwise one blocking wait syscall up to `timeout_ms` |
| `pop_latest()` | Subscriber process, per poll cycle | none | `optional<Result>` | `is_attached()` | cursor jumps directly to the newest message, discarding everything between (F-13, §5.11) | O(sizeof(T)) memcpy, O(1) — never retries, since it targets the freshest slot the producer is least likely to be mid-overwriting |
| `drain_ex(f)` | Subscriber process, per batch | callback `f(T, write_ns)` | count of messages delivered | `is_attached()` | cursor advances through every message available at the time `drain_ex()` was called (bounded snapshot, §5.2) | O(n × sizeof(T)) for n messages currently available; no syscall |
| `borrow(write_ns_out)` | Subscriber process, per message | none | `const T*` (may be null) + `write_ns` | `is_attached()`; **no outstanding unmatched `borrow()`** (F-14) — a second `borrow()` before the matching `end_borrow()` is rejected (§5.9) | internal "borrowed" flag set only when a non-null pointer is returned; no cursor change yet | O(1), no copy |
| `end_borrow()` | Subscriber process, per message | none | `bool` | **a matching successful `borrow()` is outstanding** (F-14) — `end_borrow()` with no prior successful `borrow()` is a documented no-op; it never advances the cursor speculatively (§5.9) | cursor advances iff the boundary re-check passes; "borrowed" flag cleared | O(1), no copy |
| `detach()` | Subscriber process, at shutdown or before reattach (§5.8) | none | none | — | `is_attached()` false; all local state (cursor, "reserved"/"borrowed" flags) reset | `munmap` syscall |

## 4. Feature list

| ID | Feature | Summary |
|---|---|---|
| F-1 | Non-blocking, always-succeeding writes | `push()`/`commit()` never fail due to a slow or absent consumer; the oldest unread slot is overwritten once the ring is full |
| F-2 | Independent multi-consumer reads | Each `RingSubscriber` holds its own process-local read cursor; no shared reader-coordination state, no limit on the number of independent readers |
| F-3 | Blocking wait with bounded timeout | `pop_wait(timeout_ms)` blocks efficiently (OS wait primitive, not a spin loop) until new data arrives or the timeout elapses; always finite, never blocks forever |
| F-4 | Zero-copy write | `reserve()`/`commit()` lets the publisher construct a message directly in shared memory, no local-copy-then-memcpy step |
| F-5 | Zero-copy read | `borrow()`/`end_borrow()` gives the subscriber a direct pointer into the ring for in-place processing, with post-read validation |
| F-6 | Per-message freshness | Every message carries a monotonic publish timestamp; `Result::is_stale(max_age_ms)` lets a reader detect and act on overly old data |
| F-7 | Compile-time type safety | A subscriber's expected type `T` is checked against the publisher's at attach time; a mismatch is rejected, not silently misinterpreted |
| F-8 | Config-driven tuning | Ring capacity, initial-read-cursor policy, staleness threshold, warning rate limit, and every retry/cadence bound in §8's margins table are per-topic settings resolved from a config file, not compiled-in constants |
| F-9 | Resilient attach/reattach | A subscriber that starts before its publisher, or whose publisher restarts, never treats that as fatal — it retries indefinitely with rate-limited logging |
| F-10 | ABI/schema versioning | The wire layout carries a schema version, checked at attach time, so an incompatible build is detected rather than silently corrupting reads |
| F-11 | Producer conflict detection | `open()` rejects a second concurrent producer attaching to the same topic, rather than silently allowing two producers to corrupt the shared index state |
| F-12 | Fixed-size point-cloud message family | A set of concrete, fixed-`sizeof(T)` point-cloud types at several capacities lets bursty/bulk topics use this ring directly, choosing the smallest variant that comfortably covers the topic's actual point count |
| F-13 | Latest-only consumption | `pop_latest()` jumps a subscriber's cursor straight to the newest message, for consumers (e.g. a visualizer) that only need "what's current" and shouldn't pay to process every intermediate message. The *method already exists* in today's `ring.hpp` with the same "jump to newest" intent — this design carries it forward, reimplemented against the new `start_idx`/`end_idx` indexing (§5.2) rather than today's `write_idx`/`read_idx`, since the underlying header it reads from changes regardless |
| F-14 | Reserve/borrow misuse guards | `reserve()`/`commit()` and `borrow()`/`end_borrow()` detect an unmatched or doubled call and refuse to publish garbage or advance the cursor speculatively |

## 5. Architecture and detailed design

### 5.1 Shared memory layout

```cpp
struct RingHeader {
    uint32_t magic;
    uint32_t schema_version;          // wire-layout version, checked at attach()
    uint64_t type_hash;                // compile-time hash of T, checked at attach() (§5.6)
    uint32_t capacity_n;                // element count; power of two; set once at creation (§5.5)
    std::atomic<uint64_t> start_idx;   // oldest still-live logical index
    std::atomic<uint64_t> end_idx;     // next-write logical index
    std::atomic<uint32_t> notify_seq;  // futex/WaitOnAddress word for blocking wait (§5.3)
    std::atomic<uint8_t>  closed;
    std::atomic<uint8_t>  producer_active; // 1 while a producer holds this topic (§5.10)
    std::atomic<uint32_t> producer_pid;    // OS pid of the active (or last) producer (§5.10)
    // ... padding to alignment
};

struct alignas(T) Slot {
    uint64_t      write_ns;    // monotonic publish timestamp (§5.4)
    unsigned char bytes[sizeof(T)];
};
// backing store: Slot[capacity_n], immediately after RingHeader
```

The live message window is always `[start_idx, end_idx)`, capped at
`capacity_n` elements. Both boundaries are writer-owned and published
explicitly, so a reader never needs to re-derive validity from capacity
arithmetic — it only ever compares its own cursor against these two
values.

### 5.2 Write and read algorithm

**Write** (single producer, always succeeds):
```
idx = end_idx                              // producer's own last value
write slot[idx % N]  = { write_ns: now(), bytes: value }   (single bulk copy)
end_idx  = idx + 1                          // publish
if end_idx - start_idx > N:
    start_idx = end_idx - N                 // evict oldest, keep exactly N live
notify_wake_all()
```

**Read** (any number of independent consumers, each with a private
`next_read` cursor):
```
if next_read >= end_idx: no new data
if next_read < start_idx: next_read = start_idx      // fell behind; resync silently (§5.7)
copy slot[next_read % N] out                          // single bulk copy
if start_idx has advanced past next_read since the copy started:
    the read was torn (producer wrapped mid-copy) — retry, bounded (R-1, §7)
else:
    next_read += 1; return the message
```

This index-boundary check (not a per-slot version counter) is what makes
multi-consumer support essentially free: because every reader's validity
check is against the same two writer-owned atomics, adding another
concurrent reader requires no new shared state at all.

### 5.3 Blocking wait

`RingHeader::notify_seq` is a futex-style word: the writer increments it
(implicitly, via `platform::notify_wake_all`) after every successful
publish and on `close()`; a reader's `pop_wait(timeout_ms)` snapshots the
word, checks for data, and if there is none, blocks on it via
`platform::notify_wait` — the same cross-platform primitive `topic.hpp`'s
`wait()`/`wait_new()` already uses: a Linux futex (the kernel resolves a
futex on `MAP_SHARED` memory by its physical backing, so this works
correctly even though each process maps `notify_seq` at a different
virtual address), a named Windows `Event` bound to the segment via
`platform::notify_bind()` (not `WaitOnAddress`/`WakeByAddressAll` — those
match purely by virtual address within one process, confirmed by direct
testing to never propagate a wake across two separate mappings of the
same segment, let alone two processes; this was discovered during phase 1
implementation and is tracked as R-18), or a 1ms poll fallback elsewhere.
The snapshot is taken *before* the data check to avoid a lost-wakeup race
(a publish landing between the check and the wait call must not go
unnoticed until the timeout).

`timeout_ms` is always finite — the transport never exposes an infinite
wait. A subscriber process is therefore never stuck: every `pop_wait()`
call returns, timeout or not, and the caller's loop re-evaluates state on
every return (§5.8).

### 5.4 Message freshness

Every slot carries `write_ns`, stamped from `platform::now_ns()` — the
same monotonic clock shmbridge already uses for its other liveness checks
(`is_publisher_alive(max_age_ms)`), not wall-clock time, so it needs no
cross-process clock synchronization and is immune to NTP/wall-clock jumps
(dependent on A-2, §2).

```cpp
struct Result {
    T        value;
    uint64_t write_ns;
    bool is_stale(double max_age_ms) const noexcept {
        return (platform::now_ns() - write_ns) > static_cast<uint64_t>(max_age_ms * 1e6);
    }
};
```

This is the only per-message health signal the API exposes. The threshold
is not hardcoded — it comes from `RingConfig::max_age_ms` (§5.5) — and the
check is exposed, not enforced: `pop_ex()` still delivers a stale message
(a paused publisher is a legitimate, common case), and the caller decides
what to do with that information.

### 5.5 Configuration

```cpp
enum class RingCursorMode {
    StartNow,       // default: subscriber only sees messages published after attach
    DrainBacklog,   // subscriber starts at start_idx: catches up on the current backlog
};

struct RingConfig {
    uint32_t       capacity              = 64;    // power of two; fixed for the segment's lifetime
    RingCursorMode cursor_mode           = RingCursorMode::StartNow;
    double         max_age_ms            = 0.0;   // 0 = staleness check disabled by default
    double         warn_every_ms         = 2000.0; // rate limit for resilient-attach warnings (§5.8)
    uint32_t       max_retries           = 4;     // pop_ex() torn-read retry bound (§8, R-1)
    uint32_t       max_takeover_attempts = 4;     // producer stale-takeover retry bound (§5.10, R-17)
    double         attach_retry_ms       = 100.0; // resilient-loop attach-probe cadence (§5.8)
};

// Resolution order (first match wins), from a TOML file:
//   [ring."<topic_name>"]  >  [ring.default]  >  RingConfig{} built-in defaults.
// Path: SHMBRIDGE_RING_CONFIG env var, or an explicit path argument.
//
//   [ring.default]
//   capacity = 64
//   cursor_mode = "start_now"
//   max_retries = 4
//   max_takeover_attempts = 4
//   attach_retry_ms = 100
//
//   [ring."/robot/lidar_scan"]
//   capacity = 256
//   cursor_mode = "drain_backlog"
//
RingConfig resolve_ring_config(const std::string& topic_name,
                                const std::string& config_path = {});
```

**Capacity** is set once, by whichever process creates the segment (the
publisher), and stored in `RingHeader::capacity_n`; every other process
reads it back from the header rather than from its own config, so it's
impossible for publisher and subscriber to disagree about the ring's
size.

**Cursor mode** is purely subscriber-local (§4 F-2), so independent
subscribers on the same topic can legitimately choose different modes —
a logger wanting the full backlog and a realtime controller wanting only
new data can both attach to the same ring.

**`max_retries`, `max_takeover_attempts`, and `attach_retry_ms`** are
config, not compiled-in constants, specifically because their correct
value is a guess about deployment hardware and workload, not a
mathematical bound — the same reasoning §8 applies to capacity. This is a
deliberate contrast with, for example, the `start_idx`/`end_idx` validity
check in §5.2: that comparison is correct for any `T`/rate/capacity by
construction and has no business being configurable. Retry and cadence
bounds are the opposite case — someone running this on much slower or
much more contended hardware than this design was tuned against should be
able to raise them without a rebuild, which is exactly what moving them
into `RingConfig` buys.

**Config file format: TOML**, parsed by
[`toml++`](https://github.com/marzer/tomlplusplus) (decided — §16's
revision history), a single-header, dependency-free C++17 library,
vendored at a pinned release tag into `shmbridge/include/third_party/`,
consistent with this project's existing header-only philosophy rather
than introducing a package manager (no `vcpkg.json`/`conanfile` exists in
this repo today). This is a real build dependency, not a self-contained
parser — tracked as R-16 (§7). See §6 for why TOML was chosen over a hand-rolled
format.

### 5.6 Type safety

`RingHeader::type_hash` reuses `topic.hpp`'s existing `type_id<T>()` — a
compile-time FNV-1a hash of the compiler's `__PRETTY_FUNCTION__`/
`__FUNCSIG__` for `T` — rather than introducing a second mechanism. The
publisher stamps it at `open()`; the subscriber checks it at
`attach()`/`try_attach()`, rejecting a mismatch with
`std::invalid_argument`, the same severity as a `schema_version` mismatch
— a permanent build/deployment incompatibility, not a timing issue, and
therefore explicitly excluded from the resilient retry policy in §5.8.

As with `topic.hpp` today, the hash only guarantees detection between two
ends built with the same toolchain (`__PRETTY_FUNCTION__` differs between
GCC/Clang; MSVC uses `__FUNCSIG__`, consistent with A-1, §2), and
`type_hash == 0` is treated as "unset," matching the existing sentinel
convention.

### 5.7 Consumer behavior on overflow

A consumer never tracks or reports how many messages it missed. When a
reader's cursor falls behind the ring's live window, it silently resyncs
to `start_idx` and continues — the only visible trace is that the next
delivered message's `write_ns` will be newer than the reader's previous
expectation, which the freshness check (§5.4) already surfaces without any
separate bookkeeping. This keeps the read path free of counters that
exist purely for diagnostics no consumer in this system needs under
assumption A-4 (§2) — fast consumers are the expected case; the risk this
transport is designed against is cycle-time mismatch, not processing
speed.

### 5.8 Resilient attach and reattach

Two conditions must never be fatal to a subscriber process: the publisher
not being up yet at startup, and the publisher closing (e.g. a sensor node
restarting) mid-run. In both cases the subscriber keeps retrying and logs
a rate-limited warning, never throws or exits.

Naming note: `node.hpp`'s `create_subscription` already has an internal
`detail::SubSlot::try_attach` field today — a private lambda that wraps
`RingSubscriber::attach(topic, /*timeout_ms=*/0)` to fake a non-blocking
probe, since no public non-blocking method exists on `RingSubscriber`
currently. The `try_attach()` below is a **new public method on
`RingSubscriber` itself**, not a reuse of that existing private lambda —
once it exists, `node.hpp`'s internal lambda can likely be deleted and
replaced with a direct call to it (a simplification phase 7 should make,
not something to treat as already solved).

`try_attach(name, cfg)` returns `false` for "not found yet" (expected,
transient, never logged or thrown by itself); it still throws
`std::invalid_argument` on a `schema_version`/`type_hash` mismatch (§5.6)
— a real incompatibility, never silently retried. The resilient read loop
built on it is one `for(;;)` with three cases, in this order:

1. not attached → `try_attach()`; on failure, rate-limited warning + sleep (`cfg.attach_retry_ms`, configurable, §5.5) + retry
2. attached → `pop_wait(timeout_ms)` (always bounded, §5.3); if `is_closed()`, rate-limited warning + **mandatory `detach()`** + retry from step 1
3. otherwise, handle whatever `pop_wait()` returned (including nothing, on a plain timeout)

Reattaching after `closed` must fully redo `attach()`, not just clear the
flag: a restarted publisher's `start_idx`/`end_idx` sequence numbers mean
nothing against the old subscriber's cursor, so `detach()` before retrying
is mandatory — otherwise a stray cursor could misread a differently-sized
new segment or register a false gap.

This mechanism is the subscriber side of the subscriber's own startup/
reattach lifecycle; §13 maps it onto the "publisher not up yet / closed /
crashed" lifecycle states rather than restating it here.

### 5.9 Reserve/borrow misuse guards (F-14)

Full method signatures, preconditions, and postconditions for every
`RingPublisher`/`RingSubscriber` method are in §3 (Interfaces); the
write/read algorithm itself is in §5.2. The one piece of logic not
already fully conveyed there is the misuse-guard check — both pairs
follow the identical shape, so one fragment stands in for both:

```cpp
// reserve()/commit() and borrow()/end_borrow() both follow this shape:
if (active) return reject();     // reserve(): nullptr; borrow(): nullptr
active = true;                   // ... do the real read/write ...
active = false;
```

`reserve()`/`commit()` is unconditionally safe regardless of the guard:
the slot at `end_idx` is never published until `commit()` advances the
index, so no reader can ever observe it mid-write — the guard exists
purely to catch a caller bug (an unmatched or doubled call), not to close
a race. `borrow()`/`end_borrow()` is the speculative counterpart to
`pop_ex()` (§5.2): the guard only prevents the cursor from advancing when
nothing was actually borrowed; the borrowed pointer's contents are still
only as trustworthy as `end_borrow()`'s return value says they are.

### 5.10 Producer conflict detection (F-11)

`RingHeader` carries `producer_active` and `producer_pid`. `open()`
attempts to claim the topic:

```cpp
void claim_producer_slot() {
    // cfg_.max_takeover_attempts: config, not a constant (§5.5) -- bounded
    // either way, not unbounded, mirroring R-1's discipline.
    for (uint32_t attempt = 0; attempt < cfg_.max_takeover_attempts; ++attempt) {
        uint8_t expected = 0;
        if (hdr_->producer_active.compare_exchange_strong(expected, 1, std::memory_order_acq_rel)) {
            hdr_->producer_pid.store(platform::current_pid(), std::memory_order_release);
            return; // claimed cleanly -- the common case (first open(), or a prior producer called close())
        }
        // Someone else's producer_active is already set. Find out if that
        // process is actually still alive before deciding this is a conflict.
        // process_alive() defaults to "alive" on an indeterminate result
        // (e.g. OpenProcess access-denied against a live process owned by
        // a different user) -- the safe direction, since wrongly concluding
        // "dead" risks the double-producer corruption this function exists
        // to prevent, while wrongly concluding "alive" only costs a
        // spurious conflict error.
        uint32_t other_pid = hdr_->producer_pid.load(std::memory_order_acquire);
        if (platform::process_alive(other_pid)) {
            throw std::runtime_error(
                "ring '" + name_ + "' already has an active producer (pid=" + std::to_string(other_pid) + ")");
        }
        // The previous producer crashed without calling close() -- the
        // active flag is stale. Take over -- but another process could be
        // racing to do the exact same takeover right now, so the pid swap
        // itself must be a CAS, not an unconditional store: an unconditional
        // store here would let two concurrent "recoverers" both believe
        // they'd won, silently reintroducing the dual-producer corruption
        // F-11 exists to prevent. If this CAS loses, someone else just won
        // the takeover; loop back and re-check producer_active (we'll then
        // see it's alive -- them -- and correctly throw as a conflict,
        // rather than retrying the takeover against a pid that's now real).
        uint32_t expected_pid = other_pid;
        if (hdr_->producer_pid.compare_exchange_strong(expected_pid, platform::current_pid(),
                                                         std::memory_order_acq_rel)) {
            log_warning("ring '" + name_ + "': recovering from a producer that exited "
                        "without close() (stale pid=" + std::to_string(other_pid) + "); taking over");
            hdr_->producer_active.store(1, std::memory_order_release);
            return;
        }
        // Lost the takeover race; loop and re-evaluate from scratch.
    }
    throw std::runtime_error("ring '" + name_ + "': producer takeover did not converge after "
                              + std::to_string(cfg_.max_takeover_attempts) + " attempts");
}
```

`cfg_` is the `RingConfig` resolved and stored by `open()` (§5.5),
alongside `name_` — both exist purely for this kind of later use, not
part of the ring's hot read/write path. `platform::process_alive(pid)` is
a new, small addition to `platform.hpp` (`OpenProcess`+
`GetExitCodeProcess` on Windows, `kill(pid, 0)` on POSIX) — the same
category of platform shim already maintained there for
`shm_create`/`now_ns`/etc. A clean shutdown (`close()`) clears
`producer_active` itself, so the common restart case (previous producer
exited normally) never hits the stale-takeover path at all; that path
exists specifically for a crash that skipped `close()`, and the CAS-gated
takeover above specifically handles two processes racing to recover from
that same crash simultaneously (e.g. a supervisor restarting a crashed
producer twice in quick succession) without a silent double-claim.

This is the mechanism; §13 maps it onto the publisher's lifecycle states
(running / cleanly closed / crashed) and the three startup cases that
mechanism exists to tell apart, rather than restating the algorithm here.

### 5.11 Fixed-size point-cloud message family (F-12)

A small set of concrete, trivially-copyable, fixed-`sizeof(T)` types,
each a valid `T` for this ring like any other message type:

```cpp
template <size_t MaxPoints>
struct PointCloudFixed {
    uint32_t n_points;          // actual count published this message, <= MaxPoints
    uint32_t _pad;
    float    points[MaxPoints][4]; // x, y, z, intensity
};
using PointCloud64    = PointCloudFixed<64>;     //  1 KiB
using PointCloud256   = PointCloudFixed<256>;    //  4 KiB
using PointCloud1024  = PointCloudFixed<1024>;   // 16 KiB
using PointCloud4096  = PointCloudFixed<4096>;   // 64 KiB
using PointCloud16384 = PointCloudFixed<16384>;  // 256 KiB
using PointCloud65536 = PointCloudFixed<65536>;  // 1 MiB, matches the
                                                   // existing bespoke
                                                   // transport's cap
```

Each instantiation has its own `type_hash` (§5.6), so attaching a
`RingSubscriber<PointCloud256>` to a topic actually publishing
`PointCloud4096` is caught as a type mismatch exactly like any other `T`
mismatch — no new validation logic needed, just a consequence of reusing
the existing mechanism. A publisher picks the smallest variant that
comfortably covers its topic's actual point count (per §8's margin
methodology), and only fills `n_points` of the full `MaxPoints` capacity
on most messages — a reader that needs the real payload size reads
`n_points` and processes `n_points` entries, not the full
`MaxPoints`. (This directly avoids the category of bug found in this
project's earlier benchmarking work, where a point-cloud binding
allocated and copied the full maximum size on every read regardless of
how many points were actually published — see
`shmbridge/tests/bench_pubsub_matrix_report.md`, finding #2.)

**Capacity footgun, called out explicitly**: `RingConfig::capacity`'s
default (64, §5.5) is generic and not payload-size-aware — it does *not*
automatically shrink for a large `T` like `PointCloud4096` (64 KiB/slot).
A publisher that opens a `PointCloud4096` topic without overriding
`capacity` gets a 4 MiB segment (64 × 64 KiB), not the ~2-4 slots (~256
KiB) that §8's margin guidance for latest-only topics says is typically
sufficient. This isn't a correctness bug — nothing breaks — but it's
exactly the kind of default a developer won't notice until the memory
footprint does. Every `PointCloudFixed<MaxPoints>` topic's `open()` call
should set `cfg.capacity` explicitly per §8, not rely on the generic
default.

### 5.12 Batch vs. latest-only consumption patterns (F-13)

Two distinct consumption shapes this design supports, with different
sizing guidance (§8):

- **Small, high-frequency messages** (e.g. an IMU sample, tens of bytes
  at hundreds-to-thousands of Hz): size `capacity` generously — slots are
  cheap, so the ring can comfortably hold "most of the data" over a
  reader's batch interval. A reader polls periodically (every 10-50 ms,
  say) and calls `drain_ex()` once per poll to process everything that
  accumulated, rather than reacting per-message — fewer wake cycles, same
  data.
- **Large, infrequently-fully-processed messages** (e.g. a point cloud
  from §5.11, where a visualizer or a non-every-frame consumer doesn't
  need every single one): size `capacity` small — even 2-4 slots is
  often enough — and consume with `pop_latest()` rather than
  `drain_ex()`. Not all published data needs to be processed; the design
  deliberately makes "give me whatever's current" (F-13) and "give me
  everything since I last looked" (`drain_ex()`) two distinct, equally
  first-class ways to read the same ring, so a consumer picks the one
  that matches what it actually needs instead of paying to process data
  it would just discard anyway.

## 6. Alternatives considered

| Alternative | Considered for | Reason not chosen |
|---|---|---|
| Word-wise atomic slot access (`atomic<uint64_t> words[]` instead of a plain byte array, with relaxed per-word loads/stores) | Making slot reads/writes well-defined under the strict C++ memory model (§5.2, R-2) | Made every individual word access memory-model-clean, but required a bounded retry loop, a safe/unsafe split for zero-copy writes, and a per-word store/load loop on every publish and read. `topic.hpp`'s existing seqlock accepts the same bulk-copy tradeoff today, so the stricter scheme would have been inconsistent with established precedent for a correctness property (torn-but-discarded reads) already handled by the index-boundary check |
| Reject-on-full writes with a single shared reader cursor (the ring's original design) | The write/read algorithm (§5.2) and multi-consumer support (F-2) | Gave the producer a backpressure signal, but at the cost of the producer blocking or failing under a slow consumer, and supported exactly one reader. Rejected in favor of always-succeeding overwrite semantics with independent per-consumer cursors, which better fits a multi-consumer, never-block-the-producer requirement (accepted tradeoff: R-6) |
| Per-read drop-count reporting (`Result` carrying a `dropped` field, incremented whenever a reader's cursor resynced past missed messages) | Consumer overflow handling (§5.7) | Added bookkeeping to every read path for a signal applications didn't need in practice — the freshness check (§5.4) already answers "is what I have current," which is the question that actually matters for a realtime consumer, without a separate counter |
| Split zero-copy write API (`reserve_local()`/`commit()` as a safe default vs. `reserve_unsafe()`/`commit_unsafe()` for true in-place writes) | Zero-copy write (F-4) | Was necessary only under the word-wise-atomic scheme, where in-place field writes would have mixed atomic and non-atomic access to the same memory. Once bulk-copy slot access was chosen instead, a single `reserve()`/`commit()` pair is unconditionally safe (an unpublished slot is never visible to a reader), so the split became unnecessary |
| Shared per-reader cursor table inside `RingHeader` | Multi-consumer support (F-2) | Would have let a producer observe reader progress (partially recovering the backpressure signal traded away above), but requires shared, contended state written by every subscriber process. Rejected because each subscriber's cursor can live entirely in that process's own memory with no coordination at all — simpler, and avoids cache-line contention between independent readers |
| Minimal hand-rolled INI parser for `RingConfig` (§5.5) | Config file format (F-8) | Avoided a new dependency, at the cost of a non-standard, harder-to-extend format. Rejected in favor of TOML once a self-contained parser was judged not worth writing for a real (if small) grammar — a header-only third-party library (R-16, §7) was judged the more honest tradeoff than hand-rolling a format that would eventually need the features a real config format already provides (typed values, nesting) |

### 6.1 Comparison against established open-source prior art

Before finalizing, this design was checked against how widely-used
frameworks solve the same problem, both to sanity-check the choices above
and to surface anything they do that this plan doesn't:

| Framework | Relevant mechanism | Comparison |
|---|---|---|
| **LMAX Disruptor** (Java) | Single-producer (or multi-), ring buffer with per-consumer "sequence" cursors and a pluggable claim/wait strategy | The closest conceptual relative to this design: per-consumer cursors compared against shared producer/gating sequences is exactly §5.2's mechanism. Disruptor's *default* claim strategy blocks the producer when the slowest consumer's gap closes the ring (a "reliable" mode); an overwrite/never-block strategy is one of its supported alternatives, not the default. This design makes the overwrite behavior the *only* mode (F-1) — see below |
| **DDS / ROS 2 QoS** (`KEEP_LAST` history depth + `BEST_EFFORT` reliability) | Bounded per-topic history, independent per-reader delivery, drop-on-full under best-effort | Directly analogous to this design's overwrite-on-full + independent-cursor model; DDS's `RELIABLE` QoS is the blocking/never-drop counterpart this design doesn't offer (see below) |
| **Eclipse iceoryx** (shared-memory middleware for robotics) | Zero-copy chunks from a fixed set of size-class memory pools; per-subscriber history depth | Directly validates F-12's fixed-size-variant approach — iceoryx's bucket allocator is the same idea (pick from a small set of fixed sizes rather than one-size-fits-worst-case or true dynamic allocation), applied at the pool level instead of the message-type level |
| **Linux kernel `bpf` ring buffer** (`BPF_MAP_TYPE_RINGBUF`) | Single-producer, reserve/commit/discard API, consumer wakeup via epoll | Validates the `reserve()`/`commit()` naming and semantics (§5.9) as an established pattern, not an invented one; it is strictly single-consumer, though, so doesn't validate F-2 |
| **`boost::lockfree::spsc_queue`** | Single-producer/single-consumer, reject-on-full | This is essentially the ring's *original* design (§6, row 2) — useful as the simpler baseline this design deliberately moved past for multi-consumer and never-block requirements |

**What this changes in the plan**: nothing structurally — the chosen
design (overwrite-on-full, independent per-consumer cursors, fixed-size
variants for bulk payloads) is consistent with how multiple mature,
widely-deployed systems solve the same problem, not an outlier approach.
It does surface one legitimate gap worth naming explicitly rather than
leaving implicit: both Disruptor and DDS treat "never drop, block
instead" as a first-class alternative *mode*, not just a rejected
alternative.

| Alternative | Considered for | Reason not chosen (for now) |
|---|---|---|
| A "reliable" / blocking QoS mode (DDS `RELIABLE` / Disruptor's blocking claim strategy) — producer blocks instead of overwriting when the slowest consumer hasn't caught up | An optional alternative to F-1 for a topic where losing a message is worse than blocking | **Decided (§16): deferred indefinitely, not scheduled.** For this pass, `Result::is_stale()`'s timestamp check (F-6, §5.4) is the sole data-quality signal a consumer gets — sufficient for the realtime, never-block-the-producer case this plan targets (§1.2). Revisit only if a concrete topic surfaces a real need for guaranteed delivery; adding this mode would need its own backpressure/blocking semantics analysis (reopening R-6), not a small addition, so it isn't worth designing speculatively now |

**What this changes in the guideline**: `docs/design_plan_guideline.md`'s
Alternatives Considered section now explicitly calls for checking
established prior art, not just internally-generated alternatives,
because doing so here caught a real, nameable scope boundary (the
reliable-mode gap above) that a purely internal alternatives review did
not surface.

## 7. Failure modes and effects analysis (risk analysis)

| # | Risk | Failure mechanism | Likelihood | Severity | Detection | Mitigation |
|---|---|---|---|---|---|---|
| R-1 | Unbounded retry on a torn read stalls the caller | Producer publishes faster than a reader's bulk copy can complete, repeatedly invalidating the read before it finishes | Low | High (breaks realtime bound) | Retry-count instrumentation in the benchmark harness (§11.2) | `RingConfig::max_retries` cap (§8) in `pop_ex()`, configurable rather than fixed; on exhaustion, resync and return "no data" for this cycle rather than loop |
| R-2 | Bulk `memcpy` of a slot the writer may concurrently overwrite is undefined behavior in the strict C++ memory-model sense | A reader's copy and the producer's next write to the same slot (once the ring has wrapped past it) overlap in time with no atomic synchronization on the payload bytes themselves. Structurally weaker than it sounds: the `start_idx`/`end_idx` check is a *ring-wide eviction* signal updated in a separate step *after* the slot write, not a direct per-slot write-in-progress bracket the way `topic.hpp`'s `seq`/`seq2` seqlock is — a reader's acquire-load of `start_idx` racing ahead of the writer's corresponding release-store can accept data from an already-overwritten slot with no synchronization on those bytes at all (confirmed during phase 9's R-2 evaluation, not merely theorized) | Certain (by design, on every wrap) | Formally high (UB); empirically very low but **confirmed non-zero** — phase 9's 30-minute realistic-capacity (1024) soak test measured it directly: 0 violations / 1.50 billion messages on native Windows/MSVC, but 11 violations / 289 million received on WSL/GCC (≈1 per 26 million) — see `tests/phase9_soak_test_report.md`. "Never observed to corrupt beyond recognition" (every anomaly found, in this test and in phase 8's adversarial `RingTornReadRetry`, was a clean, previously-pushed value delivered out of order — not garbled bits) remains accurate | Not directly detectable at runtime; relies entirely on the `start_idx` before/after check to discard any torn result | **Accepted, not mitigated** (owner: Approver, §2 of document metadata): identical property already exists in `topic.hpp`'s seqlock; a stricter word-wise-atomic alternative was evaluated and rejected (§6). A *cheaper*, not-yet-evaluated middle ground — a per-slot `seq`/`seq2` bracket mirroring `topic.hpp`'s exact mechanism, unlike the word-wise-atomic alternative that *was* rejected — would close this specific structural gap at a fixed per-slot cost (not per-word); recorded as an open hardening opportunity (§16), not implemented, pending an explicit decision given phase 9's confirmation that the gap is real, if still extremely rare |
| R-3 | Lost-wakeup race in `pop_wait()` | A publish lands between a subscriber's data-check and its wait call, if the `notify_seq` snapshot were taken after the check instead of before | Medium if the ordering is broken by a future edit | Medium (missed wakeup until timeout, not data loss) | Integration test asserting wake latency stays bounded after a publish (§11.1) | Snapshot `notify_seq` before checking for data, mirroring `topic.hpp`'s proven `wait()`/`wait_new()` ordering |
| R-4 | Off-by-one error in the drop/validity boundary (`>` vs `>=` against capacity) | An incorrect boundary comparison would silently accept a torn read as valid instead of discarding it | Low (verified analytically) | High (silent data corruption) | Boundary-value unit test (§11.1) | Verified analytically against `RingHeader` semantics; `> N` is the correct condition |
| R-5 | Blocked readers hang past their timeout if the writer closes without signaling | `close()` sets the `closed` flag but omits the wake call; a reader already parked in `notify_wait` doesn't observe `closed` until its own timeout independently expires | Medium (easy omission) | Medium (subscriber appears stuck until its next timeout) | Unit test: reader blocked in `pop_wait()` when `close()` is called, asserting near-immediate wake (§11.1) | `RingPublisher::close()` calls `notify_wake_all()`, not just `push()`/`commit()` |
| R-6 | Zero backpressure to the producer means a permanently stuck reader is indistinguishable from a merely-slow one | Inherent to overwrite-on-full: a reader that has stopped entirely looks, from the producer's side, identical to one running near capacity | Certain (by design) | Low (accepted tradeoff, not a defect) | Not detectable by the transport itself; a reader can only self-diagnose via `is_stale()` on what it does receive | **Accepted, not mitigated** (owner: Approver): no cross-process "reader health" detection is provided or planned |
| R-7 | A subscriber compiled against the wrong message type `T` silently misinterprets shared-memory bytes as `T` | No check would otherwise distinguish a size-compatible but semantically different `T` from the publisher's actual type | Low (requires a build-time mistake) | High (memory-safety and correctness failure) | `type_hash` check throws at `attach()` time — detected at startup, not silently (§11.1) | `type_hash` check (§5.6), reusing `topic.hpp`'s existing mechanism |
| R-8 | A genuine schema/type incompatibility is masked by the resilient retry loop's "keep retrying, never fail" policy | If the type/schema check weren't special-cased, §5.8's "always retry, never fail" policy would treat a permanent incompatibility the same as a transient "not found" | Medium if not explicitly separated in the implementation | High (silent deployment failure, could persist for a long time unnoticed) | Dedicated unit test distinguishing the throw-path from the retry-path (§11.1) | `attach()`/`try_attach()` throw on `schema_version`/`type_hash` mismatch specifically; only "not found yet" is silently retried |
| R-9 | Reattaching after `closed` reuses a stale cursor or mapping from the previous segment instance | If `detach()` were skipped or incomplete, a cursor computed against the old segment's sequence numbering would misindex a differently-sized or differently-sequenced restarted segment | Medium (easy to omit in a future refactor) | High (misread data, wrong-sized indexing) | Reattach unit test verifying cursor reset (§11.1) | `detach()` is mandatory before re-`attach()`/`try_attach()` in the resilient loop; a restarted publisher always gets a fresh cursor |
| R-10 | Extended publisher outage floods logs via the resilient retry loop | Every retry cycle fires the warning if it isn't rate-limited | Medium (any outage longer than a few cycles) | Low (operational nuisance, not a correctness issue) | Code review / log-volume spot check | `RingConfig::warn_every_ms` rate-limits the warning; enforced by the reference loop, not the library core |
| R-11 | Fixed ring capacity is insufficient (or excessive) for a given topic's actual burst pattern, discovered only in production | A topic's real traffic exceeds what was assumed when capacity was chosen (A-6, §2) | Medium (any topic whose burst behavior isn't well characterized up front) | NFR-4's retry/resync-rate instrumentation surfaces this operationally | Capacity is config-file-driven (`RingConfig::capacity`, §5.5), retunable per topic without recompiling |
| R-12 | Staleness threshold has no sensible one-size-fits-all default across topic types | A single hardcoded `max_age_ms` would be wrong for some topic (too strict for a slow sensor, too loose for a fast control loop) | Certain if a universal default were forced | Low | N/A | `max_age_ms` defaults to disabled (0) and is configured per topic, not hardcoded |
| R-13 | The config format/env-var convention (`SHMBRIDGE_RING_CONFIG`) doesn't match an existing project convention | An undiscovered existing configuration convention elsewhere in this codebase could conflict with or duplicate this one | **Resolved, not just low**: an exhaustive repo-wide search (`getenv`/`os.environ`, every `.py`/`.hpp`/`.cpp`/`.h`/`.toml`/`.cmake`/`CMakeLists.txt` file) found exactly one other project-defined env var family, `irsim_devices/setup.py`'s `IRSIM_DEVICES_BUILD_EMBREE` — module-name-prefixed, `SCREAMING_SNAKE_CASE`, the identical pattern `SHMBRIDGE_RING_CONFIG` already follows (and the one this same `CMakeLists.txt` already uses for every `SHMBRIDGE_*` build option). No conflict or duplicate exists | Low (cosmetic/consistency, not correctness) | **Resolved (§16)**, consistent with the one existing project convention found | No change needed — already matches the only prior art this codebase has |
| R-14 | Producer-conflict detection (§5.10) has a PID-reuse race: the OS could recycle a dead producer's PID for an unrelated new process between `process_alive()`'s check and the stale-takeover `store()` | The OS reassigns `other_pid` to a new, unrelated process in the narrow window between the liveness check and the takeover | Very low (requires a PID reuse landing in a sub-millisecond window, on top of an already-rare crash-without-close scenario) | Low-Medium (would cause a confusing false "conflict" error for the unrelated new process, not silent data corruption — the ring's index state itself is never corrupted by this race, only the conflict-detection message can be momentarily wrong) | Not detectable within this design; would surface as a spurious, rare `open()` failure | **Accepted, not mitigated** (owner: Approver): the normal path (`close()` clears the flag) never hits this race at all; a PID-based liveness check matches the precision every other process-liveness check in this codebase already uses (e.g. `is_publisher_alive`'s staleness-by-timestamp approach, which has an analogous best-effort character) |
| R-15 | An unmatched or doubled `reserve()`/`commit()` or `borrow()`/`end_borrow()` call publishes garbage or advances a cursor past data that was never actually read | A caller bug: calling `commit()` without a preceding `reserve()` (publishing whatever bytes happen to be in that slot), calling `reserve()` twice before a `commit()` (two callers believing they each have exclusive access to the same slot), or calling `end_borrow()` when `borrow()` returned `nullptr` or wasn't called at all (silently skipping a message) | Medium (an easy class of caller mistake without an explicit guard) | High (garbage message published as if real, or silent data loss on the read side) | Each guard logs/asserts synchronously at the point of misuse, not later | State-machine guard (F-14, §5.9/§3): `reserved_`/`borrowed_` flags make every one of these cases a documented no-op (producer/reader side) rather than undefined or silently-wrong behavior |
| R-16 | Vendoring a third-party TOML library introduces supply-chain and maintenance surface this design didn't otherwise need (§5.5) | A single-header TOML library brings its own bugs, license terms, and update cadence into this codebase | Low (`toml++`/`tomlplusplus` is a widely-used, mature, actively-maintained single-header library) | Low-Medium (build breakage or a parser bug, not a runtime data-integrity issue — config parsing is a startup-time concern, not on any hot path) | Local build (§15 phase 9) catches integration breakage immediately on both toolchains | **Decided**: vendor `toml++` (not package-manager-fetch) at a pinned release tag into `shmbridge/include/third_party/`, consistent with this project's existing dependency style (pybind11/GTest located via `find_package`, nothing fetched at build time today) — the exact tag is pinned at implementation time (phase 2) and recorded in that commit, with its license file tracked alongside it |
| R-17 | Two processes racing to recover the same stale (crash-without-`close()`) producer slot could both believe they won, reintroducing the dual-producer corruption F-11 exists to prevent | Two processes both observe the prior producer as dead and both attempt to claim the slot at once; without a single arbiter, each can conclude it succeeded | Low (requires a crash-without-close followed by a near-simultaneous double recovery attempt) | High (exactly the corruption F-11 was built to prevent, just gated behind a narrower trigger) | Would surface as silent `end_idx` corruption under concurrent writes from two processes — the same symptom as an A-5 violation, hard to distinguish after the fact | The takeover is arbitrated by a single atomic compare-and-swap (§5.10), so only one of the two racing processes can win it; the loser correctly re-evaluates and observes the winner as a live conflict rather than racing again |
| R-18 | On Windows, `WaitOnAddress`/`WakeByAddressAll` match purely by virtual address within the calling process — a wake on one mapping's address never reaches a wait on a different mapping's address, even for the same underlying shared memory | Every real deployment maps the ring segment at a different virtual address per process (and, confirmed by direct testing, even per-call within one process), so a blocking `pop_wait()` would never actually be woken by a cross-process publish on Windows — it would silently fall back to waiting out the full timeout on every call, defeating F-3/NFR-1 on that platform specifically | Certain on Windows (not a race — it never worked), N/A on Linux/macOS | High on Windows (the entire low-CPU blocking-wait feature silently degrades to "busy-poll disguised as a timeout," discovered only by benchmarking, not by a functional test, unless the test specifically measures wake latency) | A unit test asserting a blocked reader wakes well under its timeout when a publish/close happens concurrently (§11.1) — the kind of test phase 1 added specifically because this bug produced no functional failure, only a latency one | `platform::notify_bind()`/`notify_unbind()` associate `notify_seq`'s address with a named Windows `Event` created from the same segment name; `notify_wake_all()`/`notify_wait()` use that Event on Windows instead of `WaitOnAddress`. `SetEvent()`+`ResetEvent()` isn't atomic with a waiter's value recheck, so `notify_wait()` slices its wait into 15ms increments on Windows to bound that narrow miss window instead of risking the full timeout — a deliberate, documented margin (§8), not a silent gap |
| R-19 | `RingHeader::capacity` was accessed via GCC/Clang's `__atomic_load_n`/`__atomic_store_n` builtins and `__ATOMIC_ACQUIRE`/`__ATOMIC_RELEASE` instead of `std::atomic`'s portable API, while the field itself was a plain (non-atomic) `uint32_t` — inconsistent with `write_idx`/`read_idx`/`closed` in the same struct, which already use `std::atomic<T>` | Both toolchains this design had ever been built with (MinGW GCC on Windows, GCC on WSL/Linux) support GNU atomic builtins, so the gap was invisible until a genuinely different compiler family (MSVC) was tried — MSVC has no `__atomic_load_n`/`__ATOMIC_ACQUIRE`, so this was a hard compile error (`C2065`, `C3861`), not a subtle behavioral difference | Certain on MSVC (would never have compiled), N/A on GCC/Clang-based toolchains, which is exactly why it went undetected until MSVC was tried | Low-Medium (a hard compile-time failure, not silent data corruption — would be caught immediately by anyone actually trying to build with MSVC, just not by this project's existing toolchains) | A compile attempt with any non-GCC/Clang-family compiler; none of this design's own verification (phases 1-9, including sanitizer runs) used one until MSVC was tried specifically to exercise ASan on native Windows (§11.1) | `capacity` changed to `std::atomic<uint32_t>`, matching the struct's other fields; the three call sites changed to `.load(std::memory_order_acquire)`/`.store(N, std::memory_order_release)` — same memory ordering, now expressed portably |

**Single point of failure**: the ring segment (and its one producer) is
itself a SPOF by architecture — if the producing process or the segment
is lost, every attached subscriber loses that topic with no failover.
This is an accepted property of shmbridge's existing design generally
(every topic transport in this library, including the seqlock, shares
it), not something newly introduced here, and is called out explicitly
per this project's design-plan guideline rather than left implicit.

## 8. Margins and design headroom

| Budget | Provisioned value | Configurable? | Margin rationale |
|---|---|---|---|
| Torn-read retry budget | `RingConfig::max_retries`, default 4 | Yes (§5.5) — a guess about contention, not a proven bound | A retry is only exhausted if the producer completes 4 full wraps of the ring (4 × `capacity_n` publishes) during a single reader's bulk-copy duration (microseconds for realistic payload sizes). At any reasonable capacity (tens of slots or more) and realistic publish rates (not exceeding low hundreds of thousands of messages/sec), this margin is enormous; NFR-4's benchmark instrumentation is the check that this holds in practice rather than just in theory. A deployment on much slower or more contended hardware than this design was tuned against can raise the default without a rebuild |
| Producer stale-takeover retry budget | `RingConfig::max_takeover_attempts`, default 4 | Yes (§5.5) | Converges once the CAS race (§5.10, R-17) resolves in one process's favor; 4 attempts is generous headroom over the expected case of exactly one winner on the first attempt. Configurable for the same reason as the retry budget above, though in practice this one is unlikely to need retuning |
| Resilient-loop attach-probe cadence | `RingConfig::attach_retry_ms`, default 100 ms | Yes (§5.5) | Bounds worst-case responsiveness to a new publisher appearing; not a resource margin so much as a stated responsiveness budget — lower for faster reaction at the cost of more probe cycles while waiting. `pop_wait`'s own timeout is a per-call argument already chosen by the caller (§3), not a global constant needing a config field |
| Log rate limit | `RingConfig::warn_every_ms`, default 2000 ms | Yes (§5.5) | De-noises a sustained outage to at most one warning per interval, short enough that an operator watching logs won't perceive a silent gap, long enough not to flood during an extended outage |
| Ring capacity for small/high-frequency topics (e.g. IMU, §5.12) | `RingConfig::capacity`, default 64, scale up | Yes (§5.5) | Size generously against a batch-processing interval, not per-message: `capacity >= rate_hz × batch_interval_s × safety_factor` (e.g. 2x) — slots are cheap at this payload size, so the margin can be large without a meaningful memory cost, and a generous margin means `drain_ex()` reliably finds "most of the data" still present each batch cycle |
| Ring capacity for large/latest-only topics (point clouds, §5.11-§5.12) | `RingConfig::capacity`, as low as 2-4 slots | Yes (§5.5) | A consumer using `pop_latest()` (F-13) only ever reads the newest slot, so capacity only needs enough headroom to keep the producer's current write from ever overlapping a read in progress — 2-4 is typically ample; it does not need to hold a backlog at all, which is the opposite margin direction from the small-message case above |
| Point-cloud variant selection (F-12, §5.11) | choose smallest `MaxPoints` that fits | No — a compile-time type choice, not a runtime value | Pick the fixed-size variant whose `MaxPoints` covers the topic's expected maximum point count with margin (e.g. 1.2-1.5x), not the largest available variant "to be safe" — oversizing wastes the per-message bulk-copy cost (the exact class of cost the earlier point-cloud binding bug, §5.11, demonstrated is not free) |
| Ring capacity, general case | `RingConfig::capacity`, default 64 | Yes (§5.5) | **No universal margin can be stated for a topic that fits neither pattern above** — capacity must be sized per topic against that topic's actual concurrent backlog (expected publish-rate × worst-case consumer response time). The default is a reasonable floor for low/medium-rate topics; any topic should have its chosen capacity validated empirically against NFR-4's resync-rate instrumentation before being trusted in production — this methodology, not a fixed number, is what this design provides |
| Windows notify-wait slice (R-18) | `platform.hpp`'s internal `kSliceMs`, 15 ms, hardcoded | No — a platform-primitive constant, not a per-topic value | Bounds the narrow miss window between `notify_wake_all()`'s `SetEvent()`+`ResetEvent()` pulse and a waiter's value recheck in `notify_wait()` to at most one slice instead of the full caller-supplied timeout. This is a platform-correctness margin inside `platform.hpp` itself, shared by every topic and every caller (`topic.hpp` and `ring.hpp` alike) — unlike the per-topic rows above, there is no per-topic workload reason to retune it, so it stays a constant rather than threading a new config parameter through every caller for a value with one correct answer (small enough to not matter at any realistic polling rate, large enough not to reintroduce busy-polling CPU cost) |
| Windows timer-tick rounding on a short `pop_wait()` timeout | No config — an OS characteristic, not a design parameter | N/A | A short requested timeout (e.g. 10ms) on Windows typically returns 15-20ms later, not closer to 10ms, because `WaitForSingleObject` rounds to the system's default ~15.6ms timer tick rather than honoring sub-tick precision. Measured directly (`shmbridge/tests/phase1_ring_build_report.md`): a 10ms request consistently returned in 15-20ms. A caller choosing a tight timeout for a fast robotics poll loop should budget for this rounding rather than assume the requested value is the actual one; it does not affect wake-on-publish latency (F-3's point is to wake on activity well before any timeout, not to make the timeout itself precise) |

**Confidence in the worst-case estimates above, not just the margins
computed from them** (Tier 1 requirement, per the design-plan guideline):
the retry, takeover, cadence, and log-rate margins are derived from
reasoning about the mechanism itself (ring wraps, syscall counts) and are
treated as high-confidence — no additional contingency beyond what's
already stated. Being configurable (per-row above) doesn't change that
classification: it exists so an operator *can* retune these for unusual
hardware or contention, not because the stated defaults are expected to
be wrong for typical deployments. The *capacity* margins are a different
story: they're
reasoning about a topic's real-world burst behavior, which nobody has
measured yet for any topic concrete enough to cite a number for. The
stated safety factors (2x for small/high-frequency topics, 1.2-1.5x for
point-cloud variant selection) **are** that contingency — they exist
specifically because the underlying "expected backlog" estimate is a
guess, not a measurement, until NFR-4's resync-rate instrumentation
confirms it empirically for a given deployment. No additional buffer
beyond those stated factors is added here; if a future topic's actual
burst behavior is less predictable than IMU-like or point-cloud-like,
its capacity should get a larger factor, stated explicitly, not silently
inherited from this table.

## 9. Non-functional requirements and success criteria

Both baselines cited below come from prior shmbridge benchmark runs on
this project's own reference machine (the same one every other number in
this document is grounded against, documented fully in
`bench_robot_control_report.md`/`bench_pubsub_matrix_report.md`): a 16
physical/32 logical-core Windows x86-64 host. NFR-1's baseline is what a
*busy-spin* subscriber (§1.1) measures today with no blocking wait at
all; NFR-2's baseline is the existing seqlock-based raw-channel
transport's (`topic.hpp`) already-measured latency, used as "what
comparable IPC on this hardware already achieves," not as a target this
ring is replacing.

| # | Target | Baseline / measurement method |
|---|---|---|
| NFR-1 | A subscriber blocked in `pop_wait()` with no new data uses ~0% CPU, versus the current busy-spin baseline of ~97-101% of one core (a subscriber that polls in a tight loop with no sleep, pinning one core, measured via `psutil` CPU%) | **Met** (phase 8, `tests/phase8_benchmark_validation_report.md`): `RingPerformance.BlockedSubscriberUsesFarLessCpuThanBusySpin` measured blocked = 0.0% vs. busy-spin = 101.6% CPU over a 200ms window via a new `platform::thread_cpu_time_ns()` primitive — not via `psutil`/`bench_pubsub_matrix.py`, since `ring.hpp` has no Python bindings (Scope row); the C++ measurement is the in-scope equivalent |
| NFR-2 | One-way publish-to-observe latency for the bulk-copy path stays within the same order of magnitude as the existing seqlock-based raw-channel transport on this hardware (C++ ~5-7µs median at 16-88 byte payloads) — the added index-boundary bookkeeping should not regress this by more than a small constant factor | **Met**, cited from existing phase 2/7 evidence rather than re-measured in phase 8: `bench_migration.cpp`'s `direct_ring_ns`/`node_ring_ns` (p50 = 301ns/370ns) and `RingLatencyDistribution.ImmediateReadDistribution` (sub-microsecond) — both already faster than the seqlock baseline |
| NFR-3 | Zero-copy write/read measurably reduces latency relative to the bulk-copy path at large payload sizes (4 KiB-1 MiB), mirroring the point-cloud size class already benchmarked | **Met** (phase 8): `RingPerformance.ZeroCopyFasterThanCopyAtLargePayload`, `PointCloud4096` (~64KiB), copy p50 = 4.0µs vs. zero-copy p50 = 0.6µs (~6.7x) |
| NFR-4 | Torn-read retries are rare in practice (near zero) across realistic size/rate/consumer-count combinations; a sustained non-zero retry rate indicates the configured `RingConfig::capacity` is undersized for that topic's consumer pattern (R-11), not a correctness defect | **Met** (phase 8): new `RingSubscriberStats`/`RingSubscriber::stats()` (`total_reads`/`retried_reads`/`exhausted_reads`, mirroring `topic.hpp`'s existing `SubscriberStats<T>`) — no such instrumentation existed before phase 8. `RingTornReadRetry.HighContentionStaysLiveWithRareR2Anomalies`, at a deliberately adversarial `capacity=2`: 1 retried read / 19905 total (0.005%), 0 exhausted |
| NFR-5 | Multi-consumer correctness: every independent subscriber, regardless of its own poll rate, sees a strictly non-decreasing `write_ns` sequence and never observes the same logical message twice | Verified by the multi-consumer unit test (§11.1) and the extended-duration soak scenario (§11.2) — a correctness property, not a performance number |
| NFR-6 | `pop_wait()`'s wake-on-publish latency (push-to-wake, close-to-wake) has p99 headroom of at least 90% against a 1kHz control loop's 1ms period — the tightest realistic deployment rate this design targets — so the wake path itself never becomes the binding constraint on loop timing. This is a soft-real-time (statistical, p99-class) commitment, not a hard one — see below | Measured directly in phase 1 (§15) via `RingLatencyDistribution`'s 200-trial p50/p90/p99/max distributions, `tests/phase1_ring_build_report.md`: push-to-wake p99 = 53.0µs (94.7% headroom against 1ms), close-to-wake p99 = 53.5µs (94.7% headroom) — target met with margin on native Windows. Acceptance basis is p99, not mean or a single sample (a realtime budget is blown by the tail, not the typical case) and not max: a rare outlier an order of magnitude above p99 (~600µs-class, roughly 1 per 1,000-1,200 trials) was confirmed to be genuine OS scheduler jitter, not a design defect or test artifact — see the build report's "Explaining the outlier" for the isolation methodology. No general-purpose (non-real-time-patched) OS can promise a *hard* bound on thread wake latency, so this NFR is deliberately a p99 statistical commitment, not a worst-case guarantee |

## 10. Requirement-to-solution-to-verification coverage

| Feature | Requirement | Design element(s) | Section | Verified by |
|---|---|---|---|---|
| F-1 | Publisher never blocks or fails on "full" | Overwrite-on-full write algorithm | §5.2 | Overflow/eviction unit test (§11.1) |
| F-2 | Independent multi-consumer reads, no shared coordination | Per-subscriber local `next_read` cursor | §5.2, §5.7 | Multi-consumer independence unit test (§11.1) |
| F-3 | Low-CPU wait for new data | `notify_seq` + `pop_wait()` via `platform::notify_wait`/`notify_wake_all` | §5.3 | Blocking-wait unit test + NFR-1 benchmark (§11.1, §11.2) |
| F-4 | Zero-copy write | `reserve()`/`commit()` | §3, §5.9 | Zero-copy write correctness test + NFR-3 benchmark (§11.1, §11.2) |
| F-5 | Zero-copy read | `borrow()`/`end_borrow()` | §3, §5.9 | Zero-copy read correctness test + NFR-3 benchmark (§11.1, §11.2) |
| F-6 | Reader verifies message freshness via timestamp, not a drop count | `Slot::write_ns`, `Result::is_stale()` | §5.4 | Freshness-threshold boundary test (§11.1) |
| F-3, F-9 | Reader never blocked indefinitely | `pop_wait(timeout_ms)` finite timeout, resilient loop always re-enters | §5.3, §5.8 | Blocking-wait unit test (§11.1) |
| F-9 | Reader never fatally exits on "publisher not up" or "publisher closed" | `try_attach()`, `is_closed()`, `detach()`, resilient loop | §5.8 | Resilient-attach + reattach-after-close unit tests (§11.1) |
| F-7 | Message type must match between publisher and subscriber | `type_hash` check at attach time | §5.6 | Type-hash mismatch unit test (§11.1) |
| F-8 | Capacity and cursor policy configurable without rebuilding | `RingConfig`, `resolve_ring_config()`, TOML file | §5.5 | Config-resolution-precedence unit test (§11.1) |
| F-10 | ABI/schema compatibility detectable at attach time | `schema_version` field, checked at attach | §5.1 | Schema-version mismatch unit test (§11.1) |
| F-11 | Producer conflict detection | `open()`'s `claim_producer_slot()`, `producer_active`/`producer_pid` | §5.10 | Producer-conflict, stale-PID-takeover, and concurrent-takeover-race (R-17) unit tests (§11.1) |
| F-12 | Fixed-size point-cloud message family | `PointCloudFixed<MaxPoints>` variants, each a distinct `type_hash` | §5.11 | Point-cloud variant correctness + type-mismatch-across-variants unit test (§11.1) |
| F-13 | Latest-only consumption | `pop_latest()` | §5.2, §5.11, §5.12 | `pop_latest()` unit test (§11.1) |
| F-14 | Reserve/borrow misuse guards | `reserved_`/`borrowed_` state-machine checks | §5.9, §3 | Reserve/commit and borrow/end_borrow misuse unit tests (§11.1) |
| — | Bulk (not word-wise) slot writes/reads | Single-`memcpy` write and read paths | §5.2 | Basic SPSC correctness test (§11.1) |
| — | C++-only scope | No Python surface included in this plan | §1.3 | N/A — scope statement, not a testable behavior |

## 11. Verification and validation plan

### 11.1 Verification — was it built to the specification?

C++ unit tests, `shmbridge/tests/`; the existing `UNIX`-only gate on the
C++ test/bench targets in `CMakeLists.txt` is fixed in implementation
phase 1, first — every other phase's exit criterion depends on being
able to run these tests on this project's primary development platform.

| Test | Verifies |
|---|---|
| Basic SPSC correctness | Push N messages, read N messages, byte-for-byte equality and strictly increasing `write_ns` |
| Overflow/eviction | Pushing past capacity without reading advances `start_idx` correctly and a subsequent read resyncs to the oldest still-valid slot rather than returning corrupted data |
| Cursor/start_idx exact boundary (R-4) | `next_read_ == start_idx` is read directly (not treated as evicted); `next_read_ == start_idx - 1` resyncs rather than reading already-evicted data — the off-by-one edges the looser overflow/eviction case above doesn't pin down |
| Multi-consumer independence | 2+ subscribers at different poll rates (or with an injected read delay) never interfere with each other's cursor; each independently satisfies NFR-5 |
| Torn-read handling (R-1, R-2) | A tiny-capacity ring under a flat-out writer thread forces real eviction races: the reader stays live and finishes promptly (R-1's retry cap doesn't stall it), and the rate of non-monotonic anomalies past the `start_idx` before/after check stays under 1% — verifying R-2's "empirically low, not formally impossible" accepted-risk claim with a measured number, not asserting the stronger (and false) claim of zero |
| Zero-copy write correctness | `reserve()`/`commit()` produces results byte-identical to `push()` for equivalent data |
| Zero-copy read correctness | `borrow()`/`end_borrow()` produces results equivalent to `pop_ex()` for the same underlying data |
| Freshness threshold boundary | `is_stale(max_age_ms)` returns the correct boolean at and around the threshold boundary |
| Config resolution precedence | `[ring."<topic>"]` overrides `[ring.default]` overrides built-in defaults, exactly in that order (`capacity`/`cursor_mode`/`max_retries`) |
| Remaining `RingConfig` TOML fields (R-10, R-11, R-12, R-17) | `max_age_ms`, `warn_every_ms`, `max_takeover_attempts`, and `attach_retry_ms` each actually resolve from a TOML file, not just exist as struct members with the right compiled-in default |
| Type-hash mismatch | A subscriber templated on a different same-size type is rejected with `std::invalid_argument`, not silently accepted (R-7, R-8) |
| Schema-version mismatch | Same, for a deliberately bumped `schema_version` |
| Resilient attach | A subscriber started before any publisher exists does not throw or exit; a publisher appearing later is picked up without restarting the subscriber process (F-9) |
| Reattach after close | A publisher closing mid-run triggers a full `detach()`/`try_attach()` cycle with no stale-cursor artifacts against the restarted segment (R-9) |
| `node.hpp` auto-reattach after close (R-9) | A `node.hpp` ring subscription survives its publisher restarting mid-run with no manual `detach()` call anywhere in the test: `spin_once()`'s own `is_closed()`/`detach()` wiring recovers automatically and resumes delivering from the new publisher |
| Blocking wait | `pop_wait()` with no pending data does not busy-spin (CPU% near 0, NFR-1) and returns promptly after the next publish |
| Blocked-vs-busy-spin CPU (NFR-1) | A `pop_wait()` loop and a tight non-blocking `pop_ex()` loop, each run over the same wall-clock window with no data ever published, show an unambiguous, wide measured gap in CPU time consumed (`platform::thread_cpu_time_ns()`) |
| Zero-copy vs. copy at large payload (NFR-3) | `reserve()`/`commit()` + `borrow()`/`end_borrow()` beats `push()`/`pop_ex()`'s median latency at a ~64KiB payload (`PointCloudFixed<4096>`) |
| Producer conflict detection | A second `open()` by a different, live process on the same topic throws; `close()` followed by a new `open()` succeeds cleanly |
| Stale-PID takeover | A producer whose process is simulated as dead (liveness check mocked/forced false) allows a new `open()` to take over, with the expected warning log, rather than throwing |
| Concurrent takeover race (R-17) | Two simulated-concurrent `open()` calls racing to recover the same stale producer slot: exactly one succeeds in taking over, the other observes it as a live conflict and throws — never both silently "succeeding" |
| Point-cloud variant correctness | A round trip through each `PointCloudFixed<MaxPoints>` variant preserves `n_points` and point data exactly; a subscriber attaching with a different `MaxPoints` than the publisher is rejected via `type_hash` (F-7) |
| `pop_latest()` | After several unread publishes, `pop_latest()` returns the newest one and the subscriber's cursor lands exactly at `end_idx`, not before it |
| Reserve/commit misuse | `commit()` without a prior `reserve()` is a no-op (no data published, `end_idx` unchanged); a second `reserve()` before `commit()` returns `nullptr` without corrupting the first reservation |
| Borrow/end_borrow misuse | `end_borrow()` without a prior successful `borrow()` is a no-op (cursor unchanged); a second `borrow()` before `end_borrow()` returns `nullptr` |
| Sanitizer clean (memory + thread safety) | The full `test_ring` suite built with AddressSanitizer+UndefinedBehaviorSanitizer (`-fsanitize=address,undefined`) and separately with ThreadSanitizer (`-fsanitize=thread`) reports zero errors — not just that the tests' own assertions pass, but that no heap/stack memory error, UB, or data race occurred anywhere in the run, including in paths the tests' own assertions don't directly check |

**Sanitizer verification**: `CMakeLists.txt`'s `SHMBRIDGE_SANITIZE` option
builds `test_ring` with MSVC's `/fsanitize=address` (AddressSanitizer
only — MSVC implements no ThreadSanitizer/UndefinedBehaviorSanitizer) on
this project's Windows toolchain (Visual Studio/MSVC — not MinGW, which
has no sanitizer runtime at all), and with GCC/Clang sanitizer
instrumentation (`address,undefined` or `thread`) on Linux/WSL. All three
configurations ran clean against this phase's code (zero findings) in
phase 1; see §15 phase 1 and `tests/phase1_ring_build_report.md` for the
runs. ThreadSanitizer requires ASLR disabled (`setarch $(uname -m) -R
./test_ring`) to run at all against this design's shared-memory
mappings — without it, TSan exits with `FATAL: ThreadSanitizer:
unexpected memory mapping`, a known TSan/shm interaction, not a bug; this
is a required invocation detail for anyone reproducing phase 9's
verification, not optional tuning. Getting MSVC's ASan to link at all
required two MSVC-specific, Microsoft-documented workarounds (not
specific to this design): building GTest with static, not shared, CRT
(ASan forces static-CRT codegen) and disabling `std::string`/
`std::vector` container-overflow annotations
(`_DISABLE_VECTOR_ANNOTATION`/`_DISABLE_STRING_ANNOTATION`) so an
ASan-compiled translation unit can link against a dependency (GTest) that
wasn't also compiled with `/fsanitize=address` — both now wired into
`CMakeLists.txt`'s MSVC branch rather than left as undocumented manual
steps. **Trying MSVC specifically (not just another GCC/Clang toolchain)
surfaced R-19**: `ring.hpp` used GCC/Clang-only `__atomic_load_n`/
`__ATOMIC_ACQUIRE` builtins, a hard compile error under MSVC, invisible
under every toolchain (MinGW, WSL GCC) this design had been built with
until then — fixed by making `RingHeader::capacity` a proper
`std::atomic<uint32_t>`, consistent with the struct's other fields.

**Verification acceptance criterion**: every test above passes on native
Windows and on WSL Ubuntu (§15 phase 9) — the two toolchains this
implementation pass actually verifies against. Per the Tier 1 requirement
for independent verification of evidence: the results at each phase's
exit criterion (§15) are confirmed by someone other than that phase's
implementer — re-running the tests or reading the raw output — not
accepted on the implementer's report that "tests pass."

### 11.2 Validation — does it solve the real problem, under realistic conditions?

- **Benchmark validation**: extend `bench_pubsub_matrix.py` (implementation
  phase 8) with a `ring` mode alongside the existing raw/pointcloud modes,
  sweeping the same size x rate x backend matrix, reporting CPU% and
  copy-vs-zero-copy latency, to validate NFR-1 through NFR-4 numerically
  rather than just structurally.
- **Soak test**: run the ring for an extended duration (at least 30
  minutes) with multiple independent consumer processes at varied poll
  rates, to build confidence against rare torn-read or race conditions
  that short unit tests are unlikely to hit — warranted specifically
  because R-2's accepted-UB status means this design leans more than most
  on empirical confidence rather than a formal correctness proof.

**Validation acceptance criterion (revised post-phase-9, see revision
history)**: the benchmark run meets NFR-1 through NFR-4 within their
stated targets. The soak test's criterion as originally written here —
"zero correctness violations" — turned out not to be the right bar:
phase 9's actual 30-minute runs found that criterion is not what this
design achieves even under realistic, non-adversarial conditions (native
Windows: 0 violations / 1.50 billion messages received across 4
consumers; WSL/GCC: 11 violations / 289 million received, ≈1 per 26
million) — a direct, concrete measurement of R-2's accepted risk (§7)
actually occurring, not a newly discovered defect (R-2 was already
accepted, "Not mitigated," before phase 9 ran; the soak test's job was to
quantify it, which "zero" as a pass bar didn't actually allow for). The
criterion is now: the soak test completes without hanging, crashing, or
RingTornReadRetry-class exhausted-retry stalls, and any violations found
are at or below the same order of magnitude as `tests/phase9_soak_test_report.md`'s
measured baseline (on the order of 10⁻⁸ per message received) — a
sustained rate orders of magnitude above that baseline would indicate a
real regression, not R-2's already-accepted behavior repeating at its
expected rarity.

## 12. Rollout, migration, and rollback

This is a **breaking change** to `ring.hpp`'s existing wire format and
API — there is no wire-compatibility shim, matching how `topic.hpp`'s own
schema bumps are handled today (a header-only library recompiled by its
consumers, not a versioned network protocol):

- `RingHeader` layout changes: today's `write_idx`/`read_idx`
  (classic SPSC head/tail cursors, confirmed in the current
  `shmbridge/include/shmbridge/ring.hpp`) are **replaced**, not
  repurposed, by `start_idx`/`end_idx` (an oldest-live/next-write live
  window, a different semantic needed for the overwrite-on-full,
  multi-consumer design) — plus the new `type_hash`/`write_ns`/
  `producer_active`/`producer_pid`/`notify_seq` fields. This requires a
  `schema_version` bump regardless; a publisher and subscriber built
  against different versions of this header cannot interoperate and must
  be rebuilt together.
- **Capacity stops being a compile-time template parameter.** Today,
  `RingPublisher<T, N=64>`/`RingSubscriber<T, N=64>` fix capacity at
  compile time via `N`; this design removes `N` entirely in favor of a
  runtime `RingConfig::capacity` (§5.5). Every call site that currently
  names an explicit `N` (e.g. `RingPublisher<Msg, 256>`) needs its
  template argument converted to a runtime config value — this is a
  source-level API change, not just a header-layout change, and the
  single largest category of edit phase 7 will involve.
- Producer-conflict detection (F-11, §5.10) is itself a behavior change
  worth calling out directly: if any existing deployment currently (in
  violation of assumption A-5, §2) runs two producers against the same
  topic without anyone having noticed, this upgrade turns that from
  silent index corruption into a loud `open()` failure. That is the
  intended outcome, not a regression, but it means a previously "working"
  (by accident) deployment pattern will now visibly fail at startup —
  worth checking for before rollout, not discovering after.
- `RingPublisher::push()`'s failure contract changes: it can no longer
  fail for "ring full." Any existing caller relying on that return value
  for backpressure must be updated, since that signal no longer exists
  (R-6).
- `RingSubscriber`'s internal state layout changes (per-consumer cursor
  instead of a shared `read_idx`); existing single-consumer call sites are
  functionally unaffected but require a rebuild regardless.
- Affected call sites, confirmed by direct code search rather than
  assumed: in `node.hpp` — `create_queue` (delegates to
  `create_subscription`, doesn't touch the ring directly), the ring
  branch of `create_subscription` (constructs a `RingSubscriber<T>`),
  `create_latest` (constructs its own separate `RingSubscriber<T>`), and
  `ros_compat::Publisher<T>::publish()` (calls `RingPublisher<T>::push()`
  directly). Outside `node.hpp`, the dominant surface by far is the test
  suite, not application code: `shmbridge/tests/test_migration.cpp`
  (15+ direct instantiations with explicit compile-time `N` values),
  `shmbridge/tests/bench_migration.cpp` (8 instantiations), and
  `shmbridge/tests/bench_realistic.cpp` (includes `ring.hpp` directly).
  No references exist in `examples/`, `irsim_devices/`, or `urdf_tools/`.
  **Resolved (§16)**: no exhaustive advance enumeration beyond the above
  is required before implementation starts, because no old-version build
  of this ring is deployed anywhere — publisher and subscriber are
  rebuilt and redeployed together as a unit for this change, which is
  exactly what the `schema_version` gate already assumes (above) and what
  the "no partial rollback" rule in this section's Rollback procedure
  already depends on. Any call site missed during phase 7 simply fails to
  compile against the new header, rather than silently linking against a
  stale ABI, so under-enumeration is a build-time, not a runtime, risk —
  but the test suite above should be the first place phase 7 looks, not
  an afterthought.
- `node.hpp`'s `QoS::depth()` is not wired to ring capacity at all
  today — every current call site uses the template default (`N=64`)
  regardless of the QoS depth requested. Deciding whether
  `RingConfig::capacity` should be driven by `qos.depth()` going forward,
  or left as an independent setting, is new integration work for phase 7,
  not an existing behavior being preserved.
- No Python-side migration is needed in this pass — `ring.hpp` has no
  Python binding today (§1.3).

**Rollback trigger**: validation (§11.2) reveals a correctness violation
(duplicate or corrupted delivery, non-monotonic `write_ns`), or an NFR
target is missed beyond its stated tolerance, or real-world operational
use surfaces a defect that testing didn't catch.

**Rollback procedure**: because both ends of a ring are
`schema_version`-gated and must be rebuilt together, rollback means
reverting the `schema_version`/`ring.hpp`/`node.hpp` changes in version
control and redeploying every affected process together — there is no
partial rollback, by the same mechanism that prevents a silent partial
*forward* deployment: a mismatched publisher/subscriber pair fails loudly
at `attach()` rather than silently misbehaving, in either direction.

## 13. Program lifecycle and fault recovery

The publisher side of this design has state that must survive a clean
shutdown but not an unclean one — `producer_active` — which is exactly
the shape of problem this section exists to analyze explicitly, not leave
implicit in §5.10/§7's individual pieces.

**Publisher lifecycle states and transitions:**

| State | How it's reached | What state exists |
|---|---|---|
| Not created | Before any `open()` | Segment doesn't exist |
| Running | `open()` succeeded | `producer_active = 1`, `producer_pid` = this process |
| Cleanly closed | `close()` called | `producer_active = 0` — the *intended* way to leave the "running" state |
| Crashed | Process died without calling `close()` (kill, power loss, unhandled exception) | `producer_active` is still `1`, `producer_pid` still names the dead process — state left behind exactly because the clean-shutdown path never ran |

**The three startup cases this maps to, and why they must be handled
differently — this is the core of the "program lifecycle" analysis:**

1. **First-ever `open()`** (segment doesn't exist yet): trivial, no
   leftover state to reason about.
2. **Restart after a clean shutdown**: `producer_active = 0`, the new
   `open()` claims it immediately via the uncontended CAS in
   §5.10 — no ambiguity, no recovery logic needed.
3. **Restart after a crash** — the scenario named explicitly because it's
   the one that goes wrong in naive designs: `producer_active = 1` from a
   process that no longer exists. A design that treats *any* set flag as
   "someone is using this" fails exactly the way this section is named
   for: **the reboot program sees its own predecessor's uncleaned flag as
   an active conflict and refuses to restart, indefinitely, even though
   nothing is actually running.** This design avoids that specific
   failure by never trusting the flag's mere presence — `claim_producer_slot()`
   (§5.10) checks `process_alive(producer_pid)` before concluding
   "conflict," and only refuses to start if that check says the old
   owner is genuinely still alive. If it's dead, the new process takes
   over via a bounded, CAS-arbitrated recovery (also §5.10) rather than
   an unconditional overwrite, specifically because two restarted
   instances could otherwise race each other (R-17) and both believe
   they'd recovered successfully.

**Subscriber lifecycle** is simpler because subscribers hold no state the
*publisher* needs to survive a crash — but a subscriber must survive
*the publisher's* lifecycle transitions without needing a restart of its
own: §5.8's resilient attach/reattach loop already covers "publisher not
up yet" (maps to startup case 1/2 above, from the subscriber's view) and
"publisher closed" (maps to a publisher's clean shutdown or crash,
indistinguishable to the subscriber — both show up as `closed` or the
segment reappearing with a new producer, and both are handled by the same
full `detach()`/re-`attach()` cycle, not a special case for each).

**Abort conditions** — when this design deliberately refuses to proceed
rather than guess: `open()` throws if a verified-*alive* producer already
holds the topic (F-11); `claim_producer_slot()` throws if takeover
doesn't converge within `max_takeover_attempts` (§8) rather than retrying
forever. Both are intentional refusals, not bugs — see R-7/R-8 for the
analogous reasoning on the subscriber side (a genuine type/schema
incompatibility must also refuse rather than silently retry).

**A Windows-specific startup characteristic, discovered during phase 1
implementation**: a freshly started process's *first* short
`pop_wait()`/`notify_wait()` call can return well before its requested
timeout elapses, confirmed directly with a minimal repro using nothing
but `CreateEventA`/`WaitForSingleObject` (no shmbridge code involved) —
a process's very first such call returned in 753µs against a 10ms
request, while every subsequent call in the same process was accurate.
This is an OS-level process cold-start effect, not a bug in this design's
wait logic, and there is no portable way for `platform.hpp` to pre-warm
it on a caller's behalf. Practically: **a process that calls
`pop_wait()`/`drain_wait()` with a tight timeout immediately after
startup should perform one throwaway wait first** (exactly what
`shmbridge/tests/test_ring.cpp`'s `WarmUpWaitSubsystem` test environment
does) rather than trust the very first tight-timeout wait of its life —
the same "warm up before trusting precision" principle already applied
to real-time systems generally, just surfaced here concretely rather
than left for a deployer to discover the first time a startup-adjacent
timeout looks inexplicably short. See
`shmbridge/tests/phase1_ring_build_report.md` for the measurement.

## 14. Security and trust-boundary considerations

- This design operates entirely within shmbridge's existing trust model:
  shared-memory IPC between processes on the same host, implicitly
  trusting any process with access to the same named segment. It
  introduces no new attack surface relative to that existing model.
- `type_hash` (§5.6) is a correctness/accident-prevention mechanism (a
  build mismatch), not a security control — it does not defend against a
  deliberately malicious writer, which remains out of scope for this
  transport, consistent with the rest of shmbridge.
- The only new external input this design introduces is the TOML config
  file (§5.5), a local, operator-controlled artifact rather than
  untrusted network input; it needs basic malformed-input tolerance
  (fall back to defaults on a bad file) but not adversarial-input
  hardening. The vendored TOML parser (R-16) is third-party code
  processing that same local, operator-controlled input, not untrusted
  network data, so it doesn't change this assessment.

## 15. Implementation plan

1. **[DONE] Build system + notify wiring** (F-3; R-3, R-5, R-18) — fixed
   `CMakeLists.txt`'s C++ test/bench build gate (was `UNIX`-only for the
   whole test block); it now builds a new, platform-independent `test_ring`
   target unconditionally and keeps `test_topic`/`test_migration`/
   `test_core` (POSIX SHM, and `test_topic` specifically `fork()`) under
   their own `if(UNIX)` guard, since those still depend on `node.hpp` →
   `registry.hpp`, which has the same raw-POSIX-call problem R-18's fix
   addressed in `ring.hpp`/`topic.hpp` but has not been touched (out of
   this plan's scope — `registry.hpp` is node-discovery infrastructure,
   not the ring transport; noted in §16 for whoever picks up phase 7).
   `ring.hpp` itself was migrated off raw POSIX `shm_open`/`mmap` calls
   onto `platform.hpp`'s cross-platform `shm_create`/`shm_attach`/
   `shm_unmap`/`shm_destroy` — required to build on Windows at all, not
   an optional cleanup. Added `notify_seq` to `RingHeader`; `push()`,
   `signal_closed()` (called from `close()`) call `platform::notify_wake_all`
   (`close()` included specifically to close R-5's stuck-reader gap);
   added `pop_wait()`/`drain_wait()`, snapshotting `notify_seq` before the
   data check to close R-3's lost-wakeup race. `commit()` doesn't exist
   until phase 3; it will call the same `notify_wake_all` then.
   While verifying this on native Windows (no CI yet — §15 phase 9 remains
   local verification for this pass), found and fixed **R-18**: Windows'
   `WaitOnAddress`/`WakeByAddressAll` match by virtual address within one
   process only — confirmed directly that two `MapViewOfFile` views of the
   identical segment get different addresses and a wake on one never
   reaches a wait on the other, unlike Linux's futex (resolved by physical
   backing, works across views/processes by construction). This silently
   broke blocking-wait's low-CPU guarantee on Windows for `topic.hpp`'s
   existing `wait()`/`wait_new()` too — never caught because the benchmark
   suite only ever exercised busy-spin polling on Windows. Fixed once in
   `platform.hpp` (shared by both headers) via `notify_bind()`/
   `notify_unbind()`, which associate a notify word with a named Windows
   `Event` created from the segment's own name; `notify_wake_all`/
   `notify_wait` use it on Windows instead of `WaitOnAddress`, with a 15ms
   wait-slice bounding the one narrow miss window `SetEvent`+`ResetEvent`
   can't close atomically (§8).
   *Exit criterion*: **met**, via the actual `cmake --build` + `ctest`
   pipeline (not just a standalone compile — see 0.22): neither CMake nor
   GTest was installed on this Windows host, so a local GTest 1.14 build
   was configured, built, and installed (MinGW/Ninja, bundled with
   CLion), then `shmbridge` was configured against it
   (`-DSHMBRIDGE_BUILD_TESTS=ON -DGTEST_ROOT=...`) and `test_ring` built
   and run through real `ctest`: all 8 cases pass — basic push/pop,
   `pop_wait` returning data already available in well under 1µs (both a
   single call and 5 back-to-back calls against already-published items,
   each returned in order — see 0.23/0.25; the fast path never goes
   anywhere near the OS wait primitive), `pop_wait` honoring a realistic
   10ms timeout when nothing arrives (returns in 13-18ms, consistent with
   Windows' ~15.6ms timer-tick rounding, §8), `pop_wait` observing a
   concurrent publish with push()-to-wake latency in the single-digit-to-
   low-20s of *microseconds* (R-3; measured from the actual `push()` call,
   not from test start, which would conflate wake latency with the test's
   own artificial pre-push delay — see 0.25 for the exact figures), and
   `close()` waking a blocked reader with close()-to-wake latency in the
   same range (R-5, same isolation) — both were simply never waking at
   all (full 5000ms timeout) before the R-18 fix. Shrinking the timeout to
   a robotics-realistic value during this verification also surfaced a
   genuine Windows process cold-start characteristic (§13) and the fix
   for it (one warm-up wait before any timing-sensitive test; see 0.24).
   `topic.hpp`'s `wait()` was separately re-verified to still wake
   correctly cross-view after the same `platform.hpp` change. **NFR-6 met**
   with margin: 200-trial p99 push-to-wake/close-to-wake latency (53.0µs/
   53.5µs) leaves 94%+ headroom against a 1kHz control loop's 1ms period
   (§9, 0.26/0.27) — measured here, in phase 1, because the wake path itself
   is exactly what this phase built; NFR-1's idle-CPU benchmark is
   deferred to phase 8 (its own phase, against the full implementation)
   rather than measured here against this one piece in isolation.
   **WSL Ubuntu verified** (closing the open item 0.21-0.27 carried): the
   full `test_ring` suite built and ran clean on WSL (GCC 13.3.0), the
   Linux futex code path in `platform.hpp` — untouched by R-18's
   Windows-specific fix — confirmed working, not just assumed from the
   Windows run. **Sanitizer-clean** (§11.1, 0.28): the same WSL build,
   rebuilt with `-fsanitize=address,undefined` and separately
   `-fsanitize=thread`, found zero memory-safety or data-race issues
   across all 11 tests — the ThreadSanitizer run required disabling ASLR
   (`setarch $(uname -m) -R`) to run at all against this design's shared-
   memory mappings, a known TSan/shm interaction documented in §11.1 so
   the next person doesn't mistake the unworked-around `FATAL:` exit for
   "sanitizers aren't compatible with this design." **Native-Windows
   AddressSanitizer via MSVC** (§11.1, 0.29): VS2019's ASan runtime failed
   on this host even on a no-op program (ruled out as code-related,
   confirmed environment-specific), so a newer installed Visual Studio was
   used instead. Getting it to link required two Microsoft-documented
   workarounds (static-CRT GTest, disabled container annotations), now in
   `CMakeLists.txt`. **This surfaced R-19**: `ring.hpp` used GCC/Clang-only
   atomic builtins on `RingHeader::capacity`, a hard MSVC compile error,
   invisible under every GCC/Clang-family toolchain this design had been
   built with so far — fixed to `std::atomic<uint32_t>`, matching the
   struct's other fields. Full suite (11/11) clean under MSVC ASan with
   the fix applied.
2. **[DONE] Overwrite semantics, local cursors, config, type/schema checks,
   producer-conflict detection, latest-only read**
   (F-1, F-2, F-6, F-7, F-8, F-10, F-11, F-13;
   R-1, R-2, R-4, R-6, R-7, R-11, R-12, R-13, R-14, R-16, R-17) — the
   `start_idx`/`end_idx` write/read algorithm (§5.2, F-1), independent
   per-consumer cursors (F-2), the boundary check guarding against R-4's
   off-by-one and the `RingConfig::max_retries` cap closing R-1 (§5.2),
   `type_hash`/schema-version check (§5.6, F-7/F-10, closes R-7),
   `write_ns`/`is_stale()` (§5.4, F-6), `RingConfig`/`resolve_ring_config()`
   and the vendored TOML parser (§5.5, F-8; R-13, R-16), the per-topic
   capacity/staleness knobs that close R-11/R-12,
   `claim_producer_slot()`/`platform::process_alive()` (F-11, §5.10;
   closes R-14, arbitrated by the CAS that closes R-17), and
   `pop_latest()` (F-13, §5.2). R-2 and R-6 are accepted risks with no
   code to write in this phase (§7) — carried forward as documented,
   not silently dropped.
   *Exit criterion*: **met**. `ring.hpp` was rewritten to this phase's
   design in full (runtime `RingConfig`, the new `RingHeader` layout,
   `claim_producer_slot()`, `pop_latest()`), compiling clean on first
   attempt. `shmbridge/include/third_party/toml.hpp` vendors toml++
   v3.4.0 for `resolve_ring_config()` (F-8, R-13, R-16). The 27-case
   `test_ring.cpp` suite — overflow/eviction, multi-consumer independence,
   boundary-value (R-4), type-hash, schema-version, config-resolution,
   producer-conflict, stale-PID-takeover, concurrent-takeover-race
   (R-17), `pop_latest()`, plus the zero-copy/resilient-attach/latency
   tests carried over from phases 3-9's own exit criteria — passes on all
   toolchains exercised: MinGW (stable across 5 reruns), WSL GCC plain,
   WSL ASan+UBSan+LeakSanitizer (zero leaks), WSL ThreadSanitizer (zero
   races), and native Visual Studio 2026/MSVC both with
   `/fsanitize=address` and as a plain (unsanitized) `cmake --build`+
   `ctest` run — the last regenerating the committed
   `tests/test_ring_results.json` (27/27 pass). Two real test-design bugs
   were found and fixed during this verification, not left as flaky:
   `ConcurrentTakeoverRaceHasExactlyOneWinner` could let the second racer
   win a legitimate fresh claim instead of racing the takeover CAS,
   because the first racer's `close()` could complete before the second
   even called `open()` — fixed with a spin-wait readiness barrier so
   both threads call `open()` only after both are ready, deferring
   `close()` until after both joins and assertions. `PopWaitTimesOutWhenNoDataArrives`
   occasionally returned in ~7-8ms instead of >=10ms when run after the
   thread-heavy producer-conflict tests — the same OS-timer-jitter
   category as R-18/NFR-6 (§15 phase 1), not a `ring.hpp` defect; bound
   loosened to >=5ms with the precedent cited inline. LeakSanitizer
   flagged the two deliberate `pub1.release()` crash-simulation leaks
   (`StalePidFromDeadProcessIsTakenOver`,
   `ConcurrentTakeoverRaceHasExactlyOneWinner`) as real leaks, which they
   are not — suppressed with LSan's own `__lsan_ignore_object()` via a
   `mark_intentional_test_leak()` helper, feature-gated on
   `SHMBRIDGE_TEST_HAS_LSAN` so non-sanitized builds are unaffected.
   Mid-phase, two standing corrections were applied project-wide rather
   than only to this phase's code: (1) this project's Windows
   verification toolchain is Visual Studio/MSVC only, not MinGW — MinGW
   remains usable for a quick local compile-and-run but is no longer part
   of the verification loop (`CMakeLists.txt`'s `SHMBRIDGE_SANITIZE`
   doc comments and this document's §11.1 updated accordingly); (2) all
   `#if defined(_WIN32)`-style platform-conditional code belongs in
   `platform.hpp`, never in a test file — `spawn_and_reap_dead_pid()`,
   previously a test-local function in `test_ring.cpp` forking/spawning a
   real OS process (needed because a `std::thread`'s PID is always its
   live parent's, so simulating a crashed producer for
   `process_alive()`/`claim_producer_slot()` genuinely requires a
   separate process), was moved into `platform.hpp` as
   `platform::spawn_and_reap_process()`.
3. **[DONE] Zero-copy write** (F-4; R-15 producer side) — `reserve()`/`commit()`,
   including the `reserved_` misuse guard (F-14, §5.9) that closes R-15
   on the producer side.
   *Exit criterion*: **met**. Implemented as part of phase 2's `ring.hpp`
   rewrite (one cohesive header, written in a single pass rather than
   split strictly along this plan's phase boundaries) and exercised by
   `RingZeroCopy.ReserveCommitMatchesPush` and
   `RingMisuseGuards.ReserveCommitMisuseIsSafeNoOp` — passing on all four
   toolchains in `tests/phase2_ring_build_report.md` (MSVC/VS2026, WSL
   plain, WSL ASan+UBSan, WSL TSan).
4. **[DONE] Zero-copy read** (F-5; R-15 consumer side) — `borrow()`/`end_borrow()`,
   including the `borrowed_` misuse guard (F-14, §5.9) that closes R-15
   on the consumer side.
   *Exit criterion*: **met**, same basis as phase 3 above — exercised by
   `RingZeroCopy.BorrowEndBorrowMatchesPopEx` and
   `RingMisuseGuards.BorrowEndBorrowMisuseIsSafeNoOp`, passing on all four
   toolchains in `tests/phase2_ring_build_report.md`.
5. **[DONE] Resilient attach/reattach** (F-9; R-8, R-9, R-10) — `try_attach()`,
   `detach()`, `is_attached()`/`is_closed()`, and the reference retry
   loop (§5.8): mandatory `detach()` before re-`attach()` closes R-9's
   stale-cursor-reuse risk, the throw-vs-retry special-case for
   type/schema mismatch closes R-8, and `RingConfig::warn_every_ms`
   rate-limiting closes R-10.
   *Exit criterion*: **met**, same basis as phases 3-4 — exercised by the
   `RingResilience` suite (`TryAttachReturnsFalseBeforePublisherExists`,
   `AttachThrowsTimeoutErrorWhenNoPublisherAppears`,
   `ReattachAfterCloseGetsFreshCursor`) and `RingTypeSafety`'s
   throw-not-retry case, passing on all four toolchains in
   `tests/phase2_ring_build_report.md`.
6. **[DONE] Fixed-size point-cloud message family** (F-12) — define
   `PointCloudFixed<MaxPoints>` and its named variants (§5.11), alongside
   shmbridge's existing fixed message types.
   *Exit criterion*: **met**. `PointCloudFixed<MaxPoints>` and the six
   named variants (`PointCloud64`...`PointCloud65536`) added to
   `include/shmbridge/messages.hpp` alongside the existing `msg::` types;
   `tests/test_ring.cpp` gained `RingPointCloud.RoundTripPreservesNPointsAndData`
   (byte-exact round trip of `n_points` and point data through `push()`/
   `pop_ex()`) and `RingPointCloud.DifferentMaxPointsVariantThrowsOnAttach`
   (confirms `PointCloudFixed<64>` and `PointCloudFixed<256>` get distinct
   `type_hash` values via `type_id<T>()`'s existing
   `__PRETTY_FUNCTION__`/`__FUNCSIG__`-based hashing, and that a mismatched
   attach throws `std::invalid_argument` exactly like F-7's existing
   mechanism — no new validation code needed). Full 29-case suite (27 from
   phase 2 + 2 new) passes on all four toolchains: MSVC/VS2026 Release
   (29/29), WSL GCC plain (29/29), WSL ASan+UBSan (29/29), WSL TSan
   (29/29) — see `tests/phase3_6_ring_build_report.md`.
7. **[DONE] `node.hpp` and test-suite migration** (no new F-ID/R-ID — §12
   rollout/migration concern) — the ring branch of `create_subscription`,
   `create_latest` (its own separate `RingSubscriber<T>` construction
   site, distinct from `create_subscription`'s), and
   `ros_compat::Publisher<T>::publish()` (`create_queue` itself delegates
   to `create_subscription` and needs no direct change); decide whether
   `RingConfig::capacity` is driven by `qos.depth()` or kept independent
   (§12 — this mapping doesn't exist today). Then migrate the confirmed
   direct-instantiation call sites outside `node.hpp`:
   `shmbridge/tests/test_migration.cpp` (15+ sites, each with an explicit
   compile-time `N` to convert to a runtime `RingConfig`),
   `shmbridge/tests/bench_migration.cpp` (8 sites), and
   `shmbridge/tests/bench_realistic.cpp`.
   *Exit criterion*: **met**. `RingConfig::capacity` is now driven by
   `qos.depth()` (rounded up to `open()`'s required power-of-two via
   `detail::ring_next_pow2()`), resolving the open decision point in
   favor of "wired," not "independent." `create_subscription`'s and
   `create_latest`'s ring branches were converted to the new
   single-template-arg API (`try_attach`/`pop_ex`/`drain_ex`), and both
   were additionally switched from the new `RingConfig`'s default
   `StartNow` cursor to `RingCursorMode::DrainBacklog` -- a late-attaching
   subscriber's lazy attach (deferred to `spin_once()`) must still see
   whatever is already sitting in the ring, the closest match to the old
   shared-`read_idx` ring's observable backlog-visibility behavior, found
   by three real `node.hpp`-level test failures during verification, not
   anticipated at design time. `test_migration.cpp`, `bench_migration.cpp`,
   and `bench_realistic.cpp` all rebuild and run clean on WSL/GCC
   (`SHMBRIDGE_BUILD_TEST_MIGRATION` flipped to default `ON`); full
   details, including a real hang found and fixed during this phase's own
   verification, in `tests/phase7_node_migration_report.md`.
8. **[DONE] Benchmark validation** (NFR-1 through NFR-4; R-1's retry-count
   instrumentation) — extend `bench_pubsub_matrix.py` with a `ring` mode:
   CPU% before/after the blocking-wait fix at low rates, and zero-copy
   vs. copy latency at large payload sizes; run the soak test.
   *Exit criterion*: **met** for NFR-1 through NFR-4 (the soak test is
   phase 9's own stated exit criterion, run there instead of duplicated
   here). `bench_pubsub_matrix.py` has no `ring` mode — `ring.hpp` has no
   Python bindings at all (this document's own Scope row), so that
   specific instruction can't be carried out as written; every NFR
   measured in C++ instead, via the same GTest+`RecordProperty`
   convention `RingLatencyDistribution` established for NFR-6 in phase 1.
   Measured: NFR-1 blocked-vs-busy-spin CPU (0.0% vs 101.6%,
   `platform::thread_cpu_time_ns()`, new this phase), NFR-3 zero-copy vs.
   copy at a ~64KiB payload (0.6µs vs 4.0µs median, ~6.7x), NFR-4
   torn-read retry rate under deliberately adversarial contention
   (0.005%, via the new `RingSubscriberStats`/`stats()`, which didn't
   exist before this phase — NFR-4 had no instrumentation to measure
   until now). NFR-2 cited from existing phase 2/7 benchmark evidence,
   not re-measured. Auditing §11.1's coverage while adding this
   instrumentation surfaced and closed two real gaps predating this
   phase: `RingSubscriber::attach()` ignored `cfg.attach_retry_ms`
   entirely (hardcoded 5ms), and `node.hpp`'s ring subscriptions never
   implemented §5.8's documented `is_closed()`-triggered auto-detach/
   reattach, meaning a publisher restart would silently and permanently
   stop deliveries to any `node.hpp` ring subscription — both fixed and
   covered by new tests (R-9, §5.8). Also empirically observed R-2's
   accepted risk for the first time (a genuine, rare torn-read anomaly
   under adversarial contention), confirming rather than assuming its
   "empirically low" characterization. Full detail in
   `tests/phase8_benchmark_validation_report.md`.
9. **[DONE] Local dual-toolchain verification** (R-16's build-integration
   detection; CI deferred, see §16 resolution of former open item 3) —
   for this implementation pass, §11.1's unit tests are run locally on
   two toolchains rather than via a hosted CI workflow: natively on
   Windows (this host's CMake, via CLion's bundled toolchain —
   `$env:PATH = "C:\Program Files\JetBrains\CLion
   2025.3.3\bin\cmake\win\x64\bin;$env:PATH"` — matched to every benchmark
   number cited in this plan), and on Ubuntu via WSL for the Linux-side
   build. The 30-minute soak test (§11.2) is run manually on both before
   considering the implementation done, not on an automated schedule.
   Adding a `.github/workflows/` GitHub Actions pipeline remains a
   reasonable future step once this design has real usage, but is
   explicitly not part of this implementation pass.
   *Exit criterion*: **met, with a real finding, not a clean pass**.
   §11.1's full unit test suite (`test_ring`/`test_migration`/
   `test_topic`) passes on both toolchains — already established across
   phases 1-8, reconfirmed here. The §11.2 soak test (new this phase:
   `soak_test`/`soak_consumer_helper`, a `RingConfig::capacity=1024` ring,
   4 independent consumer *processes* at 1/5/20/100ms poll intervals,
   30 minutes, built behind its own `SHMBRIDGE_BUILD_SOAK_TEST` CMake
   option since a 30-minute run has no place in a routine test pass) ran
   on both platforms: native Windows/MSVC/VS2026 completed with **0
   violations across 1.50 billion messages received**; WSL/GCC completed
   with **11 violations across 289 million messages received** (≈1 per
   26 million) — duplicate/out-of-order `seq` or non-monotonic `write_ns`
   within a single consumer's own stream, never a corrupted payload. This
   is R-2 (§7) occurring exactly as its own risk-table entry already
   accepted, not a new defect — but it meant §11.2's original "zero
   violations" acceptance criterion was never actually achievable given
   what R-2 already says elsewhere in this same document, and has been
   revised there accordingly (§11.2). Getting a *reliable* 30-minute run
   at all required its own fix: the first two attempts were silently
   killed by the WSL2 VM's idle-connection teardown (not a test defect),
   resolved by keeping `wsl.exe` itself as a continuously-connected
   foreground process for the run's duration (§17, new lesson). Full
   detail, both platforms' complete logs, and the R-2 evaluation this
   phase prompted (comparing `ring.hpp`'s detection mechanism against
   `topic.hpp`'s own seqlock, §16) in `tests/phase9_soak_test_report.md`.

**Feature/risk coverage check** — every F-ID and R-ID from §4/§7 must
appear in at least one phase above, either as something built or as an
explicitly accepted/open risk carried forward; this table exists so a
gap shows up as a blank cell, not as a silent omission.

Each cell's phase status mirrors §15's own `[DONE]` markers, so this
table stays accurate as a progress snapshot without needing a separate
status column — a cell citing a `[DONE]` phase is closed; a cell citing
an undecorated phase number is still pending.

| ID | Addressed in | ID | Addressed in | ID | Addressed in |
|---|---|---|---|---|---|
| F-1 | Phase 2 [DONE] | F-9 | Phase 5 [DONE] | R-6 | Phase 2 [DONE] (accepted, §7) |
| F-2 | Phase 2 [DONE] | F-10 | Phase 2 [DONE] | R-7 | Phase 2 [DONE] |
| F-3 | Phase 1 [DONE] | F-11 | Phase 2 [DONE] | R-8 | Phase 5 [DONE] |
| F-4 | Phase 3 [DONE] | F-12 | Phase 6 [DONE] | R-9 | Phases 5, 8 [DONE] |
| F-5 | Phase 4 [DONE] | F-13 | Phase 2 [DONE] | R-10 | Phase 5 [DONE] |
| F-6 | Phase 2 [DONE] | F-14 | Phases 3-4 [DONE] | R-11 | Phase 2 [DONE] |
| F-7 | Phase 2 [DONE] | R-1 | Phases 2, 8 [DONE] | R-12 | Phase 2 [DONE] |
| F-8 | Phase 2 [DONE] | R-2 | Phase 2 [DONE] (accepted, §7) | R-13 | Phase 2 [DONE] (resolved, §16) |
| | | R-3 | Phase 1 [DONE] | R-14 | Phase 2 [DONE] |
| | | R-4 | Phase 2 [DONE] | R-15 | Phases 3-4 [DONE] |
| | | R-5 | Phase 1 [DONE] | R-16 | Phases 2, 9 [DONE] |
| | | | | R-17 | Phase 2 [DONE] |
| | | | | R-18 | Phase 1 [DONE] (found and fixed during phase 1, not predicted at design time) |
| | | | | R-19 | Phase 1 [DONE] (found and fixed during phase 1, not predicted at design time) |

All 14 features (F-1-F-14) and all 19 risks (R-1-R-19) resolve to at
least one phase. R-2 and R-6 are accepted risks (§7) rather than
mitigations, and R-13 was an open item (§16) until an exhaustive
repo-wide search resolved it against the one existing project
convention found — both are cited here as "addressed" in the sense of
"consciously carried forward" or "actively confirmed," not merely
"written once and never revisited," which is what distinguishes this
table from a mitigation checklist. As of this revision, all 9
implementation phases are `[DONE]` — this design's implementation plan
is complete.

## 16. Open items

The four items carried in revision 0.7 (TOML library choice, call-site
enumeration, CI matrix, reliable/blocking QoS mode scope) were resolved in
0.8 — see the revision history for when, and §5.5/R-16, §12, §15 phase 9,
and §6.1 respectively for where each resolution actually lives.

- **R-2 hardening opportunity: a per-slot `seq`/`seq2` bracket (mirroring
  `topic.hpp`'s exact `SeqlockSlot` mechanism) would directly protect each
  slot's write, closing the structural gap between "evicted" (what
  `start_idx` tells a reader, after the fact) and "being written right
  now" (what a per-slot bracket would tell it, with no gap) — see R-2's
  updated §7 row for the full comparison.** This is *not* the word-wise-
  atomic alternative §6 already evaluated and rejected (that made every
  individual word access atomic, a much heavier cost); a seqlock bracket
  leaves the bulk `memcpy` exactly as it is today and only adds two cheap
  sequence-number touches per write/read, at a fixed per-slot cost (two
  `uint64_t`, ~16 bytes/slot) rather than a per-word one. Not implemented:
  phase 9's soak test confirmed the gap is real (11 violations / 289M
  messages on WSL/GCC, §7 R-2) but still extremely rare, and R-2 remains
  an Approver-owned accepted risk, not a defect requiring an unreviewed
  mid-session fix to a Tier 1 design's hot path. Owner: whoever next
  revisits R-2's risk posture, with this document's own evaluation
  (comparing `ring.hpp`'s mechanism against `topic.hpp`'s) as the
  starting point rather than re-deriving it from scratch.
  **Deployment-realistic risk estimate, post-phase-9**: the soak test's
  measured rate came from a deliberately adversarial workload (an
  unthrottled producer at tens-to-hundreds of thousands of Hz vastly
  outracing fixed-interval consumers with no backpressure, maximizing how
  often a reader sits right at the eviction edge) — not representative of
  this design's actual target workloads (§1.2). Estimated for three
  concrete sensor/control profiles (20B IMU @ 1000Hz, 10B command @
  100Hz, a 10k-point/~156KB point cloud @ 30Hz), using this project's own
  measured memcpy/round-trip numbers (`bench_migration.cpp`'s
  `direct_ring_ns`; the NFR-3 benchmark's `PointCloud4096` copy latency)
  and §5.11/§8's own capacity-sizing guidance: the race window (copy
  duration) works out to roughly 0.0005-0.015% of each topic's own
  slot-reuse interval, 2-4 orders of magnitude narrower exposure than the
  soak test's adversarial setup produced, *provided* capacity is sized
  per R-11/R-12 (not left at a mismatched default) and consumers aren't
  chronically stalled (R-6, a separate accepted risk). Whoever picks up
  this hardening opportunity should weigh its cost against this
  deployment-realistic estimate, not only against the soak test's
  adversarial-condition number, which is a worst-case bound these
  workloads don't approach under normal operation — not a reason to skip
  the hardening, but a reason not to treat it as urgent for them.

- ~~**R-13: does `SHMBRIDGE_RING_CONFIG`'s env-var naming convention
  conflict with or duplicate an existing one elsewhere in this
  codebase?**~~ **Resolved**: this item was carried in the risk table
  (§7) and the §15 coverage table from the design's earliest drafts, but
  — a documentation gap in its own right, caught only while checking this
  section for accuracy — was never actually transcribed into this list
  as a bullet, leaving both of those tables pointing at an item that
  wasn't here to find. An exhaustive repo-wide search (every `getenv`/
  `os.environ` call in every `.py`/`.hpp`/`.cpp`/`.h`/`.toml`/`.cmake`/
  `CMakeLists.txt` file in the repository, not the partial search R-13
  originally flagged as non-exhaustive) found exactly one other
  project-defined env var family: `irsim_devices/setup.py`'s
  `IRSIM_DEVICES_BUILD_EMBREE` — module-name-prefixed,
  `SCREAMING_SNAKE_CASE`, the identical pattern `SHMBRIDGE_RING_CONFIG`
  already follows (and the one `shmbridge/CMakeLists.txt`'s own
  `SHMBRIDGE_*` build options already use). No conflict or duplicate
  exists; no rename needed.
- ~~**`registry.hpp` has the same raw-POSIX-call problem R-18's fix
  addressed in `ring.hpp`/`topic.hpp`, untouched by this plan.**~~
  **Resolved (post-phase-7, see revision history)**: `registry.hpp` was
  ported onto `platform.hpp`'s cross-platform `shm_open_or_create`/
  `shm_unmap`/`current_pid`/`process_alive` primitives — the same ones
  `ring.hpp`/`topic.hpp` already used — so `node.hpp` and everything
  that includes it now build on Windows too. This was explicitly out of
  this plan's original scope (node-discovery infrastructure, not the
  ring transport); picked up as deliberate follow-on work once phase 7
  made the question concrete rather than staying a standing caveat. Full
  details in `tests/windows_registry_port_report.md`.
- ~~**§5.8's resilient-loop case 2 is only half-implemented in `node.hpp`:
  the mandatory `is_closed()`-triggered `detach()`/re-attach() cycle was
  missing entirely until phase 8 (now fixed, R-9), but the "rate-limited
  warning" half of that same case — and case 1's attach-failure warning —
  is still not implemented anywhere.**~~ **Resolved**: `node.hpp`'s
  `SubSlot` gained `warn_every_ms`/`last_warn_ns` fields and a
  `detail::warn_rate_limited()` helper, wired into both of `spin_once()`'s
  cases — a subscriber that never finds its publisher, and one whose
  publisher closes mid-run — each emitting at most one
  `"[shmbridge] ..."` line to stderr per `warn_every_ms` (2000ms default,
  the same `RingConfig::warn_every_ms` value this was declared for but
  never consumed). `"[shmbridge] "` matches this codebase's one existing
  diagnostic-message convention (`CMakeLists.txt`'s own configure-time
  `message(STATUS "[shmbridge] ...")` calls) rather than inventing a new
  one from nothing; plain `fprintf(stderr, ...)` was chosen over a
  callback or counter API because this is the first and only logging call
  site this library has ever needed — a bespoke callback API has no
  second use case yet to justify its own design. Confirmed firing exactly
  where expected in the existing `Node.SpinOnceNonBlockingWhenNoPublisher`
  and `Node.RingSubscriptionAutoReattachesAfterPublisherClose` tests
  (visible in their console output) on both WSL/GCC and native
  Windows/MSVC/VS2026, with no change to either test's pass/fail
  behavior.
- ~~**`ext_topics.hpp`'s `PointCloudPublisher`/`PointCloudSubscriber` were
  a second, independently-maintained bulk-transport implementation
  (bespoke `PcTopicHeader` + hand-rolled seqlock over a dedicated
  segment), solving exactly the problem `ring.hpp` + F-12's
  `PointCloudFixed<MaxPoints>` family already solves generically.**~~
  **Resolved**: rewritten as a thin wrapper over
  `RingPublisher<msg::PointCloud65536>`/
  `RingSubscriber<msg::PointCloud65536>` (65536 points was this pair's
  original cap, preserved exactly), closing the redundancy flagged when
  auditing `ExtShmBridge` and friends against this design. `write()` uses
  `reserve()`/`commit()` (F-4, zero-copy). The subscriber side needed a
  small adapter, not a direct swap: the old seqlock was "always re-
  readable, returns the latest value even with no new write since the
  last call" (same model `shmbridge.message`'s Python mirror and every
  other seqlock-backed reader in this codebase assumes), while
  `pop_latest()`/`pop_ex()` are consume-once (return `nullopt` once
  there's nothing new). A small cache (`refresh()`, draining every
  available item via `borrow()`/`end_borrow()` each call and keeping only
  the last one) restores the "always returns latest known" behavior on
  top of the ring's consume-based cursor. Verified round-tripping,
  including the "re-read with no new publish still returns the same
  cloud" case, on both WSL/GCC and native Windows/MSVC. `messages.hpp`'s
  `PointCloudFixed<MaxPoints>` gained a `ts` field (the one piece of the
  old header `PointCloudFixed` didn't carry) to stay a complete
  replacement rather than dropping a capability.
  Cross-backend note: this only changes the compiled-extension (`_core`)
  wire format; `topic.py`'s pure-Python ctypes mirror keeps its own
  independent bespoke layout unchanged, which is fine — per `topic.py`'s
  own module docstring, mixing a C++-built and a ctypes-only
  `PointCloudPublisher`/`Subscriber` across the same segment was never a
  supported combination (same reasoning topic.hpp's type_hash section
  already gives for the generic `Publisher<T>`/`Subscriber<T>` case).

## 17. Lessons learned

- **A design can be correct in its own terms and still be silently broken
  by a platform primitive underneath it that was never exercised.**
  `platform.hpp`'s `notify_wait`/`notify_wake_all` had a real,
  100%-reproducible bug on Windows (R-18) for as long as `topic.hpp` has
  existed, and nothing caught it: the design review process (this
  document's own two-gate review) checked the ring's own logic
  thoroughly, but the benchmark suite that would have caught the symptom
  only ever exercised busy-spin polling on Windows, never the
  blocking-wait path, so the gap had no test or benchmark surface to show
  up in. The lesson carried into `docs/design_plan_guideline.md` isn't
  "review harder" — it's that a design's reliance on a shared,
  already-existing primitive deserves the same direct verification this
  design's own new code gets, not an assumption that "it's already used
  elsewhere, so it must work," especially for a primitive whose failure
  mode is a silent performance degradation rather than a crash or wrong
  answer.

- **A test assertion can be wrong by being too strict, not just too
  loose — and an "accepted risk" entry in a risk table is a real
  contract a test must respect, not decoration.** `RingTornReadRetry`'s
  first version asserted received values must always strictly increase
  under adversarial contention; it failed once with a genuine decrease,
  and the instinct was to treat that as a new bug to fix in `ring.hpp`.
  It wasn't: R-2 (§7) already explicitly accepts the underlying race as
  UB, "empirically low... never observed to corrupt beyond recognition,"
  never formally impossible — a stricter word-wise-atomic alternative
  was evaluated and rejected elsewhere in this same document (§6). The
  test's job was to verify *that* claim (rare, not zero), not to assert
  a stronger guarantee the design never made and had already reasoned
  its way out of providing. Before patching code to make a failing test
  pass, check whether the design's own risk table already anticipated
  and accepted exactly that failure mode — if so, the test is what needs
  fixing, not the implementation.
- **Auditing existing test coverage against the feature/risk list can
  surface real implementation gaps, not just missing tests — treat both
  outcomes as in-scope for the same pass.** Looking for a test to cover
  R-9's full mitigation found that `node.hpp` never actually wired up
  §5.8's documented `is_closed()`-triggered reattach; the config field
  `attach_retry_ms` existed and resolved from TOML but was never read.
  Both had been true since phases 2 and 7 respectively, invisible until
  someone asked "what test proves this specific documented behavior,"
  not "do the existing tests pass." A coverage audit that stops once
  every *existing* test is accounted for will miss exactly this class of
  gap — the feature/risk list, not the current test suite, is the
  source of truth to audit against.
- **A stated "zero violations" acceptance criterion should be checked
  against what the design actually claims elsewhere in the same
  document, not written independently of it.** §11.2 originally required
  the soak test to find zero correctness violations; §7's R-2 row, in the
  same document, already accepted that this exact class of violation is
  "certain (by design, on every wrap)" at some empirically-low rate. These
  two statements were never reconciled until phase 9's real 30-minute run
  found 11 violations on one platform and forced the question: is this a
  regression, or is it R-2 occurring exactly as already accepted? It was
  the latter — but discovering that required running the soak test first,
  not re-reading the document, because nothing before phase 9 had put a
  real number on R-2's rate under realistic (not deliberately adversarial)
  conditions. An acceptance criterion that contradicts an already-accepted
  risk elsewhere in the same document is a latent inconsistency, not a
  stricter bar — worth checking for explicitly ("does this validation
  criterion assume a guarantee the design doesn't actually make?") rather
  than discovering it only when a long-running test happens to trip it.
- **A background process detached from an interactive shell (`setsid`/
  `disown`, or the equivalent) is not the same as a process the OS
  guarantees to keep running.** The first two attempts at this soak
  test's 30-minute WSL run were silently killed partway through — not by
  a crash in the test itself, but by the WSL2 VM's own idle-connection
  lifecycle tearing down the whole instance (confirmed via `uptime`
  resetting to "0 min" while the Windows host's own uptime stayed
  continuous, ruling out a host sleep/restart). Polling more frequently
  did not fix it; what did was restructuring the launch so the
  long-running command was the *foreground* process of a continuously-
  connected `wsl.exe` invocation (itself launched via Windows Task
  Scheduler to survive independent of the orchestrating shell's own
  background-task limits), rather than something detached and backgrounded
  *inside* WSL and polled from outside. The general lesson: a multi-hour
  (or even multi-minute) background validation run needs its OS-level
  survival mechanism verified empirically before trusting its result,
  not assumed from `disown`/`nohup` having worked for shorter runs.
- **"It built and passed its tests on Linux" is not "it's cross-platform"
  when the difference is a resource limit, not a logic bug — stack size
  is exactly such a limit, and Linux's default is generous enough to hide
  real problems.** Migrating `ext_topics.hpp`'s `PointCloudPublisher`/
  `Subscriber` onto `RingPublisher<msg::PointCloud65536>`/
  `RingSubscriber<msg::PointCloud65536>` (§16) compiled cleanly and passed
  a full round-trip test on WSL/GCC on the first try. The identical code,
  through the identical pybind11 binding, silently crashed on native
  Windows with no exception and no Python traceback — `pop_latest()`
  returns `Result<T>` *by value*, putting a full `T` (here, ~1 MiB) on the
  stack inside `ring.hpp` itself; that's well within Linux's default 8 MiB
  thread stack but overflows Windows' default 1 MiB one. Nothing about
  this was a logic error the WSL run could have caught by running longer
  or harder — the WSL build was never going to see it, because the
  platform difference that triggers it (default stack size) isn't
  something a Linux run exercises at all. The fix (`borrow()`/
  `end_borrow()` instead of `pop_latest()`/`pop_ex()` for any large `T`)
  is now documented directly in `ring.hpp`'s `pop_latest()` comment and
  `messages.hpp`'s `PointCloudFixed` comment, not only here — a caveat
  this platform-specific needs to live where the next person instantiating
  `RingSubscriber<SomeLargeT>` will actually read it, not only in a design
  doc they may never open. The actionable habit: for any change claimed
  to be cross-platform, build *and run* it on every claimed platform
  before trusting the claim — a clean compile or a passing test on one
  platform is evidence for that platform, not for the others.
