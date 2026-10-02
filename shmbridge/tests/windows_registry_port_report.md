# `registry.hpp`/`test_topic.cpp` Windows port — report

Generated while resolving `docs/design_ring_zero_copy.md` §16's open item:
`registry.hpp` (node-discovery infrastructure, included by `node.hpp`) had
the same raw-POSIX-call problem R-18's fix addressed in `ring.hpp`/
`topic.hpp` back in phase 1, but was explicitly out of that design plan's
scope (it's not part of the ring transport). This was picked up as
deliberate follow-on work once phase 7's `node.hpp` migration made the
gap concrete again, not as part of the original phase plan.

## What changed

| File | Change |
|---|---|
| `include/shmbridge/platform.hpp` | + `shm_open_or_create(name, size)`: a new primitive (not a reuse of `shm_create`/`shm_attach`), needed because neither existing one has the right contract for a table multiple processes share — see "A real design decision" below |
| `include/shmbridge/registry.hpp` | Every raw POSIX call replaced with a `platform.hpp` equivalent: `::shm_open`/`::ftruncate`/`::mmap` → `platform::shm_open_or_create()`; `::munmap` → `platform::shm_unmap()`; `::getpid()` → `platform::current_pid()`; `detail::pid_alive()` (a local `::kill(pid,0)` wrapper) → `platform::process_alive()` directly, the local wrapper deleted; the local `reg_now_ns()` (a `clock_gettime` wrapper) → `platform::now_ns()` directly, the local wrapper deleted. The `#if defined(__APPLE__) \|\| defined(__linux__)` POSIX-header include block is gone — no OS-conditional code remains in this file at all now, all of it lives in `platform.hpp` |
| `tests/test_migration.cpp` | 17 direct `::shm_unlink("...")` test-cleanup calls → `platform::shm_destroy("...")` — the only POSIX-specific symbol left in the file |
| `tests/bench_migration.cpp` | 12 `::shm_unlink()` calls → `platform::shm_destroy()` |
| `tests/bench_realistic.cpp` | 7 `::shm_unlink()` calls (including one dynamic-name call site) → `platform::shm_destroy()` |
| `CMakeLists.txt` | `test_migration`'s `add_executable`/`add_test` moved out of the `if(UNIX)` block (built on every platform now); `bench_migration`/`bench_realistic`'s `if(UNIX AND SHMBRIDGE_BUILD_BENCH)` guard relaxed to `if(SHMBRIDGE_BUILD_BENCH)`. `test_topic` (fork()-based) and `test_core` (a separate, unrelated module with its own direct `::getpid()`/POSIX dependency) stay Unix-gated |

## A real design decision caught before it shipped

The obvious first approach — mirror `ring.hpp::open()`'s "try `shm_attach()`
first, fall back to `shm_create()` on failure" pattern — is wrong for this
file, and the reasoning is worth recording since it isn't obvious from the
primitives' names alone.

`platform::shm_create()` *unconditionally* unlinks and re-zeros any
existing segment under the given name before creating a fresh one. That's
exactly right for a `ring.hpp` topic: each topic has exactly one owning
producer, and `claim_producer_slot()`'s whole job is deciding whether an
existing segment's `producer_active`/`producer_pid` means "still alive,
refuse" or "crashed, safe to recover" — the recovery path genuinely does
want a clean slate. The discovery table is a different shape of resource
entirely: it's a single, shared, persistent segment that *every node
process on the host* reads and writes into concurrently (one slot each).
If `DiscoveryRegistry::open()` used `shm_attach()`-then-`shm_create()`,
every process still sees the same race: the first process to find no
existing segment calls `shm_create()`, but so might a second process
starting around the same time, and whichever one's `shm_create()` runs
second would unlink and re-zero the first one's freshly-created (and by
then possibly already-registered-into) table, silently discarding
another node's registration.

The original (pre-port) POSIX code never had this bug, because
`shm_open(name, O_CREAT | O_RDWR, 0666)` **without** `O_EXCL` and with no
preceding `unlink` is itself atomically idempotent: the kernel either
creates a new object or hands back the existing one, with no window for
two racing callers to produce different outcomes. Porting this file
needed a `platform.hpp` primitive with that exact contract, not reuse of
an existing one — `shm_open_or_create()`, added for this purpose, doing
the equivalent raceless idempotent open on Windows via
`CreateFileMappingA`'s own `ERROR_ALREADY_EXISTS` signal (the call
succeeds and returns a handle to the existing object either way; the
error code only tells the caller which case it was, used solely to decide
whether to skip zeroing — a freshly created segment's pages are already
OS-zero-filled on both platforms, so there is never an explicit `memset`
needed for the "created fresh" case; there just must never be one for the
"already existed" case).

## Correctness — result

Verified fresh in this session against the current working tree on both
platforms:

| Target | WSL/GCC 13.3.0 | Native Windows/MSVC 19.50 (VS2026) |
|---|---|---|
| `test_migration` (35 cases — Ring, Registry, QoS, Node, MessageQueue, LatestSlot) | 35/35 pass, 321ms | 35/35 pass (reported as 1 passing ctest target), 0.48s |
| `test_ring` (29 cases, unrelated to this change) | 29/29 pass | 29/29 pass, 5.55s |
| `test_topic` (22 cases, Unix-only, unaffected) | 22/22 pass | N/A (stays Unix-only) |
| `bench_migration` | Runs to completion | Not built on Windows in this pass (not requested; `registry.hpp`'s correctness is what this port is about, not benchmark portability) |
| `bench_realistic` | Runs to completion, writes `bench_realistic.json` | Not built on Windows in this pass |

This is the first time `node.hpp`/`registry.hpp` have ever been exercised
on Windows — not a regression check against a prior Windows baseline,
since none existed.

## Reproduce

```powershell
# Windows / Visual Studio 2026 (this project's canonical Windows toolchain)
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
# WSL / Linux (GCC)
cmake -S shmbridge -B build -DCMAKE_BUILD_TYPE=Release \
    -DSHMBRIDGE_BUILD_TESTS=ON -DSHMBRIDGE_BUILD_BENCH=ON \
    -DGTEST_ROOT=gtest_install -DCMAKE_PREFIX_PATH=gtest_install
cmake --build build -j"$(nproc)"
./build/test_migration && ./build/test_ring && ./build/test_topic
```

Run against the live working tree, not a fresh clone — this change is not
yet committed.

---

# Part 2: `test_topic.cpp` Windows port

`registry.hpp`'s POSIX calls (`shm_open`/`mmap`/`kill`/`getpid`) each had a
direct `platform.hpp`-abstractable Windows counterpart, so porting them was
mechanical substitution. `test_topic.cpp`'s one remaining Unix-only
dependency — `fork()`, in `MultipleSubscribers.NoRaceConditionForkN4` — is
a different shape of problem: Windows has no process-duplication
primitive at all, so there is nothing for `platform.hpp` to wrap. This
part of the report covers redesigning that test instead of porting it.

## What changed

| File | Change |
|---|---|
| `include/shmbridge/platform.hpp` | + `run_and_capture(cmd)`: launches `cmd` via `popen()`/`_popen()` (identical signature on both platforms already — no OS-conditional process/pipe plumbing needed beyond exit-status translation), captures its stdout, returns `{exit_code, stdout}` |
| `tests/test_topic_sub_helper.cpp` | New standalone executable: attaches as `Subscriber<Pose2d>`, reads a fixed count, prints `torn=<N> total=<N> last_x=<N>` to stdout |
| `tests/test_topic.cpp` | `MultipleSubscribers.NoRaceConditionForkN4` → `NoRaceConditionMultiProcessN4`, rewritten to launch 4 copies of the helper via `std::async`+`platform::run_and_capture()` instead of `fork()`+pipes; `unique_name()`/`ShmCleaner`/one more call site's `::getpid()`/`::shm_unlink()` → `platform::current_pid()`/`platform::shm_destroy()`; `BroadcastNotify.AllSubscribersWake`'s stale `#if defined(__linux__)` gate removed (see below) |
| `CMakeLists.txt` | `test_topic` and the new `test_topic_sub_helper` moved out of the `if(UNIX)` block; `test_topic_sub_helper`'s built path is injected into `test_topic` via `target_compile_definitions(... SHMBRIDGE_SUB_HELPER_PATH="$<TARGET_FILE:test_topic_sub_helper>")` so the test never has to guess a generator's output-directory layout |

## A real bug found during verification: `cmd.exe`'s quoting quirk

The first MSVC run of the rewritten test failed with a bizarre error:
`cmd.exe` reported that a filename like
`test_topic_sub_helper.exe" "multi_sub_72624` — the helper's closing
quote and the topic-name argument's opening quote glued together — "is
not recognized as an internal or external command." `_popen()` on
Windows runs its argument via `cmd.exe /c <cmd>`, and `cmd /?` documents
the relevant quirk: unless the entire string consists of *exactly* two
quote characters wrapping a single executable name, `cmd.exe` falls back
to stripping only the very first and very last quote character of the
whole string and passing everything between them through as one
undivided token — which is exactly wrong for a quoted executable path
followed by a quoted argument (two separate quoted tokens, four quote
characters total). Fixed by wrapping the whole command in one more
throwaway pair of quotes before calling `_popen()`: `cmd.exe`'s fallback
parsing strips exactly those two newly-added outer characters, leaving
every inner quote — the ones that actually matter — untouched. Not
needed on POSIX, where `popen()` uses `/bin/sh -c` and ordinary shell
quoting applies; the fix is Windows-only, inside `run_and_capture()`
itself, so no caller needs to know about it.

## A stale platform gate found while auditing for POSIX calls

`BroadcastNotify.AllSubscribersWake` was gated `#if defined(__linux__)`,
dating from when `topic.hpp`'s `wait()`/`write_notify()` used a raw Linux
futex directly. Both now go through `platform.hpp`'s
`notify_bind`/`notify_wake_all`/`notify_wait` (fixed for Windows during
the ring design plan's phase 1, R-18) — the gate had simply never been
revisited since. Removed, and confirmed passing on native Windows/MSVC in
this session (not assumed from the code path being "probably fine now").

## Correctness — result

| Target | WSL/GCC 13.3.0 | Native Windows/MSVC 19.50 (VS2026) |
|---|---|---|
| `test_topic` (22 cases, including the rewritten multi-process test and the un-gated `BroadcastNotify`) | 22/22 pass | 22/22 pass, 2.52s (`NoRaceConditionMultiProcessN4` alone: 2.35s) |
| `test_ring` (29 cases, unrelated to this change, re-run alongside it) | 29/29 pass | 29/29 pass, 6.42s (one earlier run hit the pre-existing, already-documented OS-timer-jitter flake in `PopWaitTimesOutWhenNoDataArrives`, R-18/NFR-6 — not a regression, confirmed by this clean rerun) |
| `test_migration` (35 cases, unrelated to this change, re-run alongside it) | 35/35 pass | 35/35 pass, 0.48s |

With this, every C++ test target in `shmbridge/tests/` builds and passes
on both platforms except `test_core` (a distinct module with its own,
separately-scoped POSIX dependency — `::getpid()`, `rt`/`pthread` linking
— not touched by either part of this report).

## Reproduce

Same procedure as Part 1's Reproduce section — `test_topic` and
`test_topic_sub_helper` are the targets that changed. Run against the
live working tree, not a fresh clone — this change is not yet committed.

