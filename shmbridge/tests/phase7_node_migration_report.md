# Ring transport — Phase 7 (`node.hpp`/test-suite migration) report

Generated while completing `docs/design_ring_zero_copy.md` §15 phase 7:
migrating `node.hpp`'s ring branches and the direct-instantiation call
sites in `tests/test_migration.cpp`, `tests/bench_migration.cpp`, and
`tests/bench_realistic.cpp` off the pre-phase-2 `RingPublisher<T,N>`/
`RingSubscriber<T,N>` API onto the runtime-`RingConfig`, single-template
-arg API. `node.hpp`/`registry.hpp` are POSIX-only (raw `shm_open`/
`fork()`), so this phase's code and all verification below is Unix-only
(WSL), consistent with `CMakeLists.txt`'s existing `if(UNIX)` gate —
no Windows/MSVC verification applies to this phase.

## What changed

| File | Change |
|---|---|
| `include/shmbridge/node.hpp` | `create_publisher`'s ring branch now builds a `RingConfig` with `capacity = ring_next_pow2(max(qos.depth(), 2))` instead of relying on the old template default; `create_subscription`'s and `create_latest`'s ring branches converted to `try_attach()`/`pop_ex()`/`drain_ex()`/`pop_latest()->value`, both using `RingCursorMode::DrainBacklog` (see "A real design gap found during verification" below); `skip_old()` call removed (no replacement needed, §5.7) |
| `tests/test_migration.cpp` | ~20 `RingPublisher<T,N>`/`RingSubscriber<T,N>` instantiations converted; `Ring.FullDrops` renamed/rewritten to `Ring.PushPastCapacityNeverFails` (the old reject-on-full contract it tested no longer exists, F-1); `Ring.SizeAndPeek` rewritten to `Ring.PopExHasValueContract` (`size()`/`empty()`/`peek()` have no new-API equivalent, §5.7); `Ring.SkipOldKeepsLatestN`/`Ring.SkipOldNoop` removed (`skip_old()` removed, §5.7); `Ring.Throughput` rewritten (see "A real hang found during verification" below) |
| `tests/bench_migration.cpp` | ~10 instantiations converted; `bench_ring_spsc_throughput()` rewritten for the same reason as `Ring.Throughput` |
| `tests/bench_realistic.cpp` | No direct edits needed — every ring use goes through `node.hpp`'s high-level API, already fixed |
| `CMakeLists.txt` | `SHMBRIDGE_BUILD_TEST_MIGRATION` flipped from default `OFF` to default `ON`; comment updated from "blocked on phase 7" to "migration done" |

## A real hang found during verification

The first full-suite run hung indefinitely in `Ring.Throughput` (confirmed via `ps aux` showing the test binary pegged at 100% CPU for 12+ minutes with no progress). Root cause: the old test's writer/reader loop assumed `push()`'s old reject-on-full return value gave the producer implicit backpressure (`if (pub.push(b)) ++i;` — `i` didn't advance while the ring was full, so the writer effectively paused until the reader caught up). That contract is gone by design (F-1): `push()` always succeeds now, so with `N=10000` against a 64-slot ring and no artificial delay, the writer thread can — and did — finish pushing all 10000 items before the reader thread was ever scheduled, overwriting all but the last 64. The reader's `while (received < N)` loop then waited forever for a count the new contract can no longer guarantee (R-6, accepted risk, §7, explicitly documents this exact tradeoff).

Fixed by bounding the reader's loop by the writer's completion (plus one final catch-up drain) instead of by a received count, and strengthening the check to assert monotonicity of whatever *is* received rather than an exact count. The identical latent bug existed in `bench_migration.cpp`'s `bench_ring_spsc_throughput()` (not caught by `ctest` since benchmark binaries aren't run as tests, but would have hung the same way) — fixed the same way, with throughput now reported against the actually-received count rather than against `N`.

## A real design gap found during verification (not a hang — silent failures)

After fixing the hang, three `node.hpp`-level tests failed outright (no crash, just the expected message never arriving): `Node.RingPubSub`, `Node.CreateQueueRing`, `Node.CreateLatestRing`. All three publish messages *before* creating the subscription. Root cause: `RingSubscriber::try_attach()`'s new default cursor mode is `StartNow` (per-consumer cursors are new in this design, F-2) — a subscriber starts reading from whatever `end_idx` is *at attach time*, deliberately skipping backlog. Combined with `node.hpp`'s lazy attach (deferred to the subscriber's first `spin_once()`, which can run well after `create_subscription()`/`create_latest()` returns and after more messages have been published), every one of these three tests' already-published messages were invisible to the newly-attached subscriber. The old ring's shared `read_idx` had no such gap — a late subscriber implicitly saw anything still sitting in the buffer, since there was only one cursor.

Fixed by switching both of `node.hpp`'s ring-subscription code paths to `RingCursorMode::DrainBacklog`, which starts the cursor at `start_idx` (the oldest still-live entry) instead of `end_idx` — the closest match to the old observable behavior. This was not anticipated in the design doc's phase 7 text (which only called out the `capacity`↔`qos.depth()` mapping as an open decision) and is recorded here as a finding, not a pre-planned change.

## Correctness — result

| Target | Result |
|---|---|
| `test_migration` (35 cases: Ring, Registry, QoS, Node, MessageQueue, LatestSlot) | 35/35 pass, WSL/GCC 13.3.0, 319ms |
| `test_ring` (29 cases, unchanged by this phase) | 29/29 pass — confirms no regression from `node.hpp`'s changes |
| `test_topic` (22 cases, unrelated to ring, unaffected) | 22/22 pass |
| `bench_migration` | Runs to completion, `ring_throughput_mmsg_per_s: 2.86` (sane, non-zero — confirms the throughput-measurement fix works) |
| `bench_realistic` | Runs to completion, writes `bench_realistic.json` |

All five run against the actual current working tree in this session (`rsync`'d to WSL, not `git clone`, per this session's own "Milestone verification reports" guideline addition — the phase's changes are still uncommitted).

## Reproduce

```bash
# WSL / Linux (GCC) -- node.hpp/registry.hpp are POSIX-only, no MSVC path for this phase
cmake -S shmbridge -B build -DCMAKE_BUILD_TYPE=Release \
    -DSHMBRIDGE_BUILD_TESTS=ON -DSHMBRIDGE_BUILD_BENCH=ON \
    -DGTEST_ROOT=gtest_install -DCMAKE_PREFIX_PATH=gtest_install
cmake --build build -j"$(nproc)"
./build/test_migration
./build/test_ring
./build/test_topic
timeout 60 ./build/bench_migration > bench_migration.json
timeout 90 ./build/bench_realistic
```

Run against the live working tree, not a fresh clone — this phase's
changes are not yet committed.
