# shmbridge

POSIX shared-memory publish/subscribe for Python ↔ C++ robot control —
sub-microsecond write/read, zero copies in the critical path.

---

## Contents

- [What's in the box](#whats-in-the-box)
- [Quick start — C++ Node API](#quick-start--c-node-api)
- [Quick start — Python](#quick-start--python)
- [C++ Node API (v3)](#c-node-api-v3)
- [Python / Robot Bridge API (v2)](#python--robot-bridge-api-v2)
- [Transport internals](#transport-internals)
- [Performance](#performance)
- [Build](#build)
- [Examples](#examples)

---

## What's in the box

| Layer | Header / Module | Description |
|---|---|---|
| **Node API** (v3, C++) | `shmbridge/node.hpp` | ROS 2-compatible publisher/subscriber node; seqlock + SPSC ring transports; `MessageQueue<T>` for deferred callbacks |
| **Robot Bridge** (v2, Python + C++) | `shmbridge/core.hpp` / Python package | single/multi-robot `RobotState`/`RobotCmd` exchange; `ShmPublisher`/`ShmSubscriber`; `LoopSleeper`; `TaskScheduler` |
| **Low-level SHM** | `shmbridge/topic.hpp` `ring.hpp` | Seqlock slots and SPSC ring — used directly or via the Node API |
| **Discovery** | `shmbridge/registry.hpp` | Live node/topic registry in shared memory |

---

## Quick start — C++ Node API

Two processes, no boilerplate. The library is header-only; no `make` step for the C++ side.

### Publisher (simulator / sensor)

```cpp
// talker.cpp
#include <shmbridge/node.hpp>
#include <shmbridge/messages.hpp>
#include <chrono>
#include <thread>

namespace sb  = shmbridge::ros_compat;
namespace msg = shmbridge::msg;

int main() {
    sb::init();
    auto node = sb::make_node("talker");

    // SensorDataQoS (depth=1) → seqlock, keep-latest
    auto pub = node->create_publisher<msg::Pose2d>("robot/pose",
                                                    sb::SensorDataQoS());
    msg::Pose2d pose;
    unsigned step = 0;
    while (true) {
        pose.x        = step++ * 0.01;
        pose.stamp_ns = shmbridge::detail::now_ns();
        pub->publish(pose);                     // zero-overhead handle path
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    sb::shutdown();
}
```

### Subscriber (controller) — callback style

```cpp
// listener_cb.cpp
#include <shmbridge/node.hpp>
#include <shmbridge/messages.hpp>
#include <cstdio>

namespace sb  = shmbridge::ros_compat;
namespace msg = shmbridge::msg;

int main() {
    sb::init();
    auto node = sb::make_node("listener");

    auto sub = node->create_subscription<msg::Pose2d>(
        "robot/pose", sb::SensorDataQoS(),
        [](const msg::Pose2d& p) {
            uint64_t age_us = (shmbridge::detail::now_ns() - p.stamp_ns) / 1000;
            std::printf("x=%.2f m  age=%llu µs\n", p.x, (unsigned long long)age_us);
        });

    sb::spin(node);   // spin_once() + sleep loop until SIGINT
    sb::shutdown();
}
```

### Subscriber — queue / deferred style

Decouples message arrival from processing; no callback timing pressure.

```cpp
// listener_queue.cpp
#include <shmbridge/node.hpp>
#include <shmbridge/messages.hpp>
#include <cstdio>
#include <thread>
#include <chrono>

namespace sb  = shmbridge::ros_compat;
namespace msg = shmbridge::msg;

int main() {
    sb::init();
    auto node = sb::make_node("listener");

    // create_queue returns {subscription_handle, shared_ptr<MessageQueue<T>>}
    auto [sub, queue] = node->create_queue<msg::Pose2d>("robot/pose",
                                                         sb::SensorDataQoS());

    while (true) {
        node->spin_once();               // attaches lazily; never blocks

        // pop_latest(): discard stale data, keep only the newest
        if (auto pose = queue->pop_latest()) {
            uint64_t age_us = (shmbridge::detail::now_ns() - pose->stamp_ns) / 1000;
            std::printf("x=%.2f m  age=%llu µs\n",
                        pose->x, (unsigned long long)age_us);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sb::shutdown();
}
```

### Build

```cmake
# CMakeLists.txt
find_package(shmbridge CONFIG REQUIRED)          # or add_subdirectory

add_executable(talker talker.cpp)
target_link_libraries(talker PRIVATE shmbridge::shmbridge)
```

```bash
cmake -B build && cmake --build build
```

---

## Quick start — Python

### Python publisher (simulator) → Python subscriber (controller)

```python
# sim.py — publisher
from shmbridge import ShmPublisher, RobotState

with ShmPublisher("/shmbridge", n_robots=1, n_consumers=1,
                  heartbeat_every=1) as pub:
    state = RobotState()
    state.x, state.y, state.heading = 1.0, 0.0, 0.0
    state.step = 42
    pub.write_state(0, state)

    cmd = pub.read_best_cmd(0)
    if cmd:
        print(f"cmd: linear={cmd.linear:.2f}  angular={cmd.angular:.2f}")
```

```python
# controller.py — subscriber
import time
from shmbridge import ShmSubscriber

sub = ShmSubscriber("/shmbridge")
with sub:                              # attach() / detach() via context manager
    state = sub.read_state_spin(0)     # blocks until a clean seqlock read
    if state:
        age_us = (time.monotonic_ns() - state.write_ns) / 1e3
        print(f"x={state.x:.3f}  age={age_us:.1f} µs")
        sub.write_cmd(0, 0, linear=0.5, angular=0.1)
```

### Python publisher → C++ subscriber

```python
# sim.py (same as above)
from shmbridge import ShmPublisher, RobotState
with ShmPublisher("/shmbridge") as pub:
    pub.write_state(0, state)
```

```cpp
// controller.cpp — C++ subscriber reading from Python sim
#include <shmbridge/core.hpp>
#include <iostream>

int main() {
    shmbridge::ShmSubscriber sub("/shmbridge");
    sub.attach();

    while (true) {
        auto s = sub.read_state_spin(0);
        if (!s) continue;

        uint64_t age_ns = shmbridge::detail::now_ns() - s->write_ns;
        std::cout << "x=" << s->x
                  << "  age=" << age_ns / 1000 << " µs\n";
        sub.write_cmd(0, 0, /*linear=*/0.5f, /*angular=*/0.0f);
    }
}
```

### Precise timing with `LoopSleeper`

```python
from shmbridge._core import LoopSleeper

sleeper = LoopSleeper(200.0)   # 200 Hz
while True:
    sleeper.start()
    # ... read state, compute cmd, write cmd ...
    sleeper.sleep()             # hybrid nanosleep + spin; releases GIL
```

### Multi-task control with `TaskScheduler`

```python
from shmbridge._core import TaskScheduler

sched = TaskScheduler(loop_hz=200.0)
sched.add_task("state_read", lambda: True, period_ms=5.0)    # 200 Hz
sched.add_task("cmd_write",  lambda: True, period_ms=10.0)   # 100 Hz
sched.run_threaded()
# ... later:
sched.stop()
print(sched.report())   # markdown table of timing stats
```

---

## C++ Node API (v3)

### Creating a node

```cpp
#include <shmbridge/node.hpp>
namespace sb = shmbridge::ros_compat;

sb::init();                                   // no-op; mirrors rclcpp::init
auto node = sb::make_node("my_node");         // registers in discovery registry
// ...
sb::shutdown();                               // no-op; mirrors rclcpp::shutdown
```

### QoS profiles

| QoS | Transport | Semantics |
|---|---|---|
| `SensorDataQoS()` | Seqlock MRSW | Keep-latest; depth=1; best-effort |
| `SystemDefaultsQoS()` | SPSC ring, depth=10 | Queue; reliable |
| `QoS(N)` | SPSC ring (N>1) or seqlock (N≤1) | Custom depth |

### Publishing

```cpp
// Create once at startup
auto pub = node->create_publisher<msg::Pose2d>("robot/pose", sb::SensorDataQoS());

// Publish — zero map lookup, no cast
msg::Pose2d p; p.x = 1.0; p.stamp_ns = shmbridge::detail::now_ns();
pub->publish(p);
```

### Subscribing — immediate callback

Callback fires from `spin_once()` in the same thread. No executor pool.

```cpp
auto sub = node->create_subscription<msg::Pose2d>(
    "robot/pose", sb::SensorDataQoS(),
    [](const msg::Pose2d& p) {
        /* called by spin_once() when a new message is available */
    });

node->spin_once();   // poll once
sb::spin(node);      // poll until SIGINT
```

### Subscribing — `MessageQueue<T>` (deferred)

Best when the processing rate differs from the message rate, or when
you want to batch-process a burst of messages.

```cpp
auto [sub, queue] = node->create_queue<msg::Twist>("cmd_vel", sb::QoS(32));

while (running) {
    node->spin_once();

    // Option A: process all accumulated messages in arrival order
    queue->drain([](const msg::Twist& t) { apply_twist(t); });

    // Option B: discard all but the newest
    if (auto t = queue->pop_latest()) apply_twist(*t);

    // Option C: pop one at a time (FIFO)
    while (auto t = queue->pop()) apply_twist(*t);
}
```

`MessageQueue<T>` is bounded (default 32 slots); oldest messages are
silently dropped when the queue is full. Not thread-safe — call
`spin_once()` and `pop*()`/`drain()` from the same thread.

### `spin_once()` behaviour

```
spin_once()
├── heartbeat(registry)         — at most once every 50 ms
└── for each subscriber:
    ├── if not attached:
    │   └── try_attach()        — non-blocking; retried at most every 100 ms
    └── if attached:
        └── poll_fn()           — read_if_new() or ring drain, fires callbacks
```

`spin_once()` never sleeps. With no active publishers the total cost is
**~45 ns** (single `clock_gettime` call).

### Built-in message types (`shmbridge/messages.hpp`)

| Type | Size | Fields |
|---|---|---|
| `msg::Pose2d` | 32 B | `x`, `y`, `heading` (double); `stamp_ns` |
| `msg::Pose3d` | 64 B | `x`,`y`,`z` + quaternion (double); `stamp_ns` |
| `msg::Twist` | 32 B | `vx`,`vy`,`vz`,`wx`,`wy`,`wz` (float); `stamp_ns` |
| `msg::Imu` | 56 B | accel + gyro + magnetometer (float); temp; `stamp_ns` |
| `msg::Odometry` | 48 B | pose (double) + velocity (float); `stamp_ns` |
| `msg::BatteryState` | 24 B | voltage, current, charge_pct, status; `stamp_ns` |
| `msg::LaserScan2d` | 40 B | scan header; ranges stored via `BulkPublisher` |

All types are trivially copyable, `static_assert`-checked for size, and
≤ 104 bytes so the seqlock slot fits in two 64-byte cache lines.

---

## Python / Robot Bridge API (v2)

### `ShmPublisher`

```python
ShmPublisher(name="/shmbridge_v2", n_robots=1, n_consumers=1,
             heartbeat_every=1)
```

| Method | Description |
|---|---|
| `open(mlock=False)` | Create and zero-init the shm segment. |
| `close()` | Unmap and unlink the segment. |
| `write_state(robot_idx, state)` | Seqlock-write robot state (GIL released). |
| `read_cmd(robot_idx, consumer_idx)` | Non-blocking read of one consumer slot. |
| `read_cmd_blocking(timeout_ms, poll_sleep_ns, robot_idx, consumer_idx)` | Block until a cmd arrives (GIL released). |
| `read_best_cmd(robot_idx)` | Valid cmd with the highest `seq` across all consumers. |
| `is_controller_alive(max_age_ms, robot_idx, consumer_idx)` | Check cmd heartbeat. |

### `ShmSubscriber`

```python
ShmSubscriber(name="/shmbridge_v2", n_robots=1)
```

| Method | Description |
|---|---|
| `attach(timeout_ms=30000)` | Attach to an existing segment. |
| `detach()` | Unmap the segment. |
| `read_state(robot_idx)` | Single seqlock read; returns `None` on torn read. |
| `read_state_spin(robot_idx, max_retries=64)` | Retry until clean. |
| `write_cmd(robot_idx, consumer_idx, linear, angular)` | Seqlock-write velocity command. |
| `is_publisher_alive(max_age_ms, robot_idx)` | Check state heartbeat. |

### `RobotState`

| Field | Type | Description |
|---|---|---|
| `x`, `y`, `heading` | float64 | World-frame pose (m, rad) |
| `vx`, `vy`, `omega` | float32 | World-frame velocity (m/s, rad/s) |
| `goal_x`, `goal_y`, `goal_dist` | float32 | Current goal position and distance |
| `step` | uint64 | Simulation step counter |
| `sim_time` | float64 | Simulated time (s) |
| `reached` | bool | Goal reached flag |
| `collision` | bool | In-collision flag |
| `write_ns` | uint64 | `CLOCK_MONOTONIC` ns at write time |

### `RobotCmd`

| Field | Type | Description |
|---|---|---|
| `linear` | float32 | Forward velocity (m/s) |
| `angular` | float32 | Angular velocity (rad/s, CCW+) |
| `seq` | uint32 | Monotone command counter |

---

## Transport internals

### Seqlock (keep-latest, `SensorDataQoS`)

Used when `QoS::depth <= 1`. Multiple readers, single writer (MRSW).

```
Writer                              Reader
──────                              ──────
seq  = ++counter   (now odd)        s1  = seq
_SB_FENCE_W()                       _SB_FENCE_R()
  ... write payload ...               ... read payload ...
  stamp_ns = now_ns()                 out.write_ns = stamp_ns
_SB_FENCE_W()                       _SB_FENCE_R()
seq  = ++counter   (now even)       s2  = seq
                                    ok  = (s1 == s2) && !(s1 & 1)
```

A torn read is detected when `s1 ≠ s2` or `s1` is odd (writer in
progress). `read_state_spin` retries; `read_if_new` skips if no new data.

Memory fences on x86/TSO compile to compiler barriers only. On AArch64
they emit `dmb ishst` / `dmb ish`.

### SPSC ring (`QoS(N)` with N > 1)

Single-producer single-consumer lock-free ring of depth N.
Index encoding uses `(write_idx - read_idx) % (2·N)` so the full/empty
states are unambiguous without a separate flag.

```
push(msg):   memcpy(slots[w % N], msg);  w.store(w+1, release)
pop(msg):    if (w - r == 0) return false;
             memcpy(msg, slots[r % N]);  r.store(r+1, release)
```

Optional futex notification (`SB_ENABLE_NOTIFY=1`) wakes blocked
subscribers without spinning.

### Shared-memory layout (seqlock)

```
offset 0          TopicHeader  (128 B — magic, version, ready flag, notify futex)
offset 128        SeqlockSlot  (128 B — seq×2, payload, stamp_ns)
```

Each object fits in exactly two 64-byte cache lines, so reads and writes
never straddle a cache-line boundary.

---

## Performance

Benchmarked on Linux x86-64 with `-O3 -march=native`.

### spin_once overhead — before and after bottleneck fixes

| Scenario | Before | After | Δ |
|---|---|---|---|
| No publisher (idle) | **10 246 486 ns** | **45 ns** | **227 699×** faster |
| Attached, no new msg | — | 41 ns | — |
| Attached, with new msg | — | 41 ns | — |

The 10 ms idle cost was caused by `attach(timeout_ms=10)` being called on
every `spin_once()` for each unconnected subscriber. It is now a single
non-blocking `shm_open` attempt retried at most once every 100 ms.

### Core transport (50 000 samples)

| Operation | p50 | p95 | p99 | p999 |
|---|---|---|---|---|
| Ring `push` | 21 ns | 22 ns | 23 ns | 34 ns |
| Ring `pop` | 21 ns | 22 ns | 25 ns | 102 ns |
| Ring `push + pop` (round-trip) | 27 ns | 31 ns | 36 ns | 77 ns |
| Seqlock `write` | 49 ns | 50 ns | 98 ns | 141 ns |
| Seqlock `read` | 41 ns | 42 ns | 43 ns | 166 ns |
| Seqlock `write + read` | 75 ns | 76 ns | 91 ns | 197 ns |
| Ring throughput | **36.6 M msg/s** | — | — | — |

### MessageQueue (50 000 samples)

| Operation | p50 | p99 |
|---|---|---|
| `push` | 23 ns | 24 ns |
| `pop` | 21 ns | 39 ns |
| `drain` (amortized per message) | **1 ns** | 2 ns |

### Realistic workload — robot control scenarios

| Scenario | p50 | p95 |
|---|---|---|
| Seqlock 1 kHz control loop (pub → sub detect) | 527 µs | 1 012 µs |
| Seqlock fan-out: 1 subscriber | 396 µs | 1 068 µs |
| Seqlock fan-out: 4 subscribers | 570 µs | 1 054 µs |
| Ring sensor end-to-end (futex wake) | **100 ns** | 100 ns |
| Handle `pub->publish()` | 422 ns | 487 ns |
| String-keyed `node->publish("topic", msg)` | 430 ns | 546 ns |

Seqlock control-loop latency reflects the polling model: expected average
detection delay = ½ × poll period (500 µs at 1 kHz). For latency-critical
paths use the ring transport with `SB_ENABLE_NOTIFY=1` (futex wake → 100 ns
end-to-end).

### Budget in a 100 Hz (10 ms) control cycle

| Item | Cost |
|---|---|
| `pub->publish()` seqlock | ~422 ns |
| `spin_once()` (idle) | ~45 ns |
| `spin_once()` (message available) | ~41 ns |
| `MessageQueue::pop_latest()` | ~21 ns |
| **Total IPC overhead** | **< 1 µs** |
| Remaining for control algorithm | **> 9 999 µs** |

### Shared-memory footprint

| Object | Size |
|---|---|
| `TopicHeader` | 128 B |
| `SeqlockSlot<Pose2d>` | 128 B |
| `RingHeader` | 64 B |
| Full seqlock segment (Pose2d topic) | 256 B |
| Ring segment N=64 (Pose2d) | 2 112 B |
| Ring segment N=1024 (Pose2d) | 32 832 B |
| Discovery segment (64 nodes) | 30 208 B |
| `MessageQueue<Pose2d>` (empty) | 88 B |

---

## Build

### Python package (with C++ extension)

```bash
pip install -e .
# or:
uv pip install -e .
```

Requires: `scikit-build-core >= 0.8`, `pybind11 >= 2.12`, a C++17 compiler.
The pure-Python ctypes fallback is used automatically if the C++ build fails.

### C++ header-only (copy-paste)

```bash
cp -r include/shmbridge  path/to/myproject/include/
```

```cmake
target_include_directories(my_node PRIVATE path/to/myproject/include)
```

No library to link. The headers pull in only C++17 standard library and
POSIX (`shm_open`, `mmap`, `futex`).

### C++ standalone (CMake FetchContent)

```cmake
include(FetchContent)
FetchContent_Declare(shmbridge
    GIT_REPOSITORY https://github.com/<your-org>/robot-sensor-toolkit
    GIT_TAG        main
    SOURCE_SUBDIR  shmbridge
)
FetchContent_MakeAvailable(shmbridge)

target_link_libraries(my_node PRIVATE shmbridge::shmbridge)
```

### C++ tests and benchmarks

```bash
cmake -B build \
    -DSHMBRIDGE_BUILD_TESTS=ON \
    -DSHMBRIDGE_BUILD_BENCH=ON
cmake --build build -j4

# Tests
./build/test_topic
./build/test_migration

# Benchmarks (write JSON to stdout / bench_realistic.json)
./build/bench_migration | python -m json.tool
./build/bench_realistic
```

---

## Examples

| Path | Language | Description |
|---|---|---|
| `examples/cpp_node/talker.cpp` | C++ | Publishes `Pose2d` at 100 Hz using Node API |
| `examples/cpp_node/listener.cpp` | C++ | Receives via `MessageQueue`; prints age |
| `examples/python_node_demo.py` | Python | In-process publisher + proportional controller |
| `examples/python_writer.py` | Python | Publishes fake robot state at 100 Hz (v2 API) |
| `examples/cpp_controller/controller.cpp` | C++ | Heading controller reading from Python sim (v2 API) |
| `tests/bench_migration.cpp` | C++ | Core transport microbenchmarks (ring, seqlock, registry) |
| `tests/bench_realistic.cpp` | C++ | Realistic robot workload benchmarks (control loop, fan-out, sensor) |

---

## License

MIT
