/*
 * py_bindings.cpp  -  pybind11 Python bindings for shmbridge::ShmPublisher
 *                     and shmbridge::ShmSubscriber.
 *
 * Compiled by scikit-build-core into _core.cpython-*.so and installed
 * alongside the pure-Python shmbridge package.  When present it is loaded
 * by __init__.py and takes precedence over the ctypes fallback.
 */

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <shmbridge/core.hpp>
#include <shmbridge/ext_core.hpp>
#include <shmbridge/ext_topics.hpp>
#include <shmbridge/spin_sleep.hpp>
#include <shmbridge/scheduler.hpp>

namespace py = pybind11;
using namespace shmbridge;

PYBIND11_MODULE(_core, m) {
    m.doc() = "shmbridge C++ core: ShmPublisher / ShmSubscriber";

    /* ── RobotState ────────────────────────────────────────────────────── */
    py::class_<RobotState>(m, "RobotState")
        .def(py::init<>())
        .def_readwrite("x",         &RobotState::x)
        .def_readwrite("y",         &RobotState::y)
        .def_readwrite("heading",   &RobotState::heading)
        .def_readwrite("vx",        &RobotState::vx)
        .def_readwrite("vy",        &RobotState::vy)
        .def_readwrite("omega",     &RobotState::omega)
        .def_readwrite("goal_x",    &RobotState::goal_x)
        .def_readwrite("goal_y",    &RobotState::goal_y)
        .def_readwrite("goal_dist", &RobotState::goal_dist)
        .def_readwrite("step",      &RobotState::step)
        .def_readwrite("sim_time",  &RobotState::sim_time)
        .def_readwrite("reached",   &RobotState::reached)
        .def_readwrite("collision", &RobotState::collision)
        .def_readwrite("write_ns",  &RobotState::write_ns)
        .def("__repr__", [](const RobotState& s) {
            return "<RobotState x=" + std::to_string(s.x) +
                   " y=" + std::to_string(s.y) +
                   " step=" + std::to_string(s.step) + ">";
        });

    /* ── RobotCmd ──────────────────────────────────────────────────────── */
    py::class_<RobotCmd>(m, "RobotCmd")
        .def(py::init<>())
        .def(py::init([](float lin, float ang, uint32_t seq) {
            return RobotCmd{lin, ang, seq};
        }), py::arg("linear") = 0.f, py::arg("angular") = 0.f,
            py::arg("seq") = 0u)
        .def_readwrite("linear",  &RobotCmd::linear)
        .def_readwrite("angular", &RobotCmd::angular)
        .def_readwrite("seq",     &RobotCmd::seq)
        .def("__repr__", [](const RobotCmd& c) {
            return "<RobotCmd linear=" + std::to_string(c.linear) +
                   " angular=" + std::to_string(c.angular) +
                   " seq=" + std::to_string(c.seq) + ">";
        });

    /* ── ShmPublisher ──────────────────────────────────────────────────── */
    py::class_<ShmPublisher>(m, "ShmPublisher",
        R"doc(
        Creates and owns the shm segment; writes robot state, reads cmds.

        Parameters
        ----------
        name : str
            POSIX shm name (default "/shmbridge_v2").
        n_robots : int
            Number of robot slots (default 1).
        n_consumers : int
            Independent cmd writers per robot (default 1).
        heartbeat_every : int
            Update writer_ts_ns every N writes (default 1).
        )doc")
        .def(py::init<std::string, unsigned, unsigned, unsigned>(),
             py::arg("name")            = SHMBRIDGE_SHM_NAME,
             py::arg("n_robots")        = 1u,
             py::arg("n_consumers")     = 1u,
             py::arg("heartbeat_every") = 1u)
        .def("open",    &ShmPublisher::open, py::arg("mlock") = false,
             "Create and zero-init the shm segment.")
        .def("close",   &ShmPublisher::close,
             "Unmap and unlink the segment.")
        .def("is_open", &ShmPublisher::is_open)
        .def("__enter__", [](ShmPublisher& self) -> ShmPublisher& {
            self.open(); return self;
        })
        .def("__exit__", [](ShmPublisher& self, py::object, py::object,
                             py::object) { self.close(); })
        .def("write_state", &ShmPublisher::write_state,
             py::arg("robot_idx"), py::arg("state"),
             "Seqlock-write robot state (fast; GIL released).")
        .def("read_cmd",
             [](const ShmPublisher& self, unsigned r, unsigned c) {
                 return self.read_cmd(r, c);
             },
             py::arg("robot_idx") = 0u, py::arg("consumer_idx") = 0u,
             "Non-blocking seqlock read from one consumer slot. Returns None "
             "if mid-write or no valid cmd.")
        .def("read_cmd_blocking",
             [](const ShmPublisher& self, double timeout_ms,
                int64_t poll_sleep_ns, unsigned r, unsigned c) {
                 return self.read_cmd_blocking(timeout_ms, poll_sleep_ns, r, c);
             },
             py::call_guard<py::gil_scoped_release>(),
             py::arg("timeout_ms")    = 10.0,
             py::arg("poll_sleep_ns") = 500'000LL,
             py::arg("robot_idx")     = 0u,
             py::arg("consumer_idx")  = 0u,
             R"doc(
             Block until a valid cmd arrives or timeout_ms elapses (GIL released).

             Between poll attempts nanosleep(poll_sleep_ns) is issued — a direct
             OS-level yield that drops idle CPU from 100 % to ~5 % without going
             through Python or the spin_sleep Welford estimator.

             poll_sleep_ns = 0       -> pure busy-poll (lowest latency, 100 % CPU)
             poll_sleep_ns = 500_000 -> 500 µs OS sleep between polls (default)

             Returns None on timeout.
             )doc")
        .def("read_best_cmd",
             [](const ShmPublisher& self, unsigned r) {
                 return self.read_best_cmd(r);
             },
             py::arg("robot_idx") = 0u,
             "Return the valid cmd with the highest seq across all consumer "
             "slots, or None.")
        .def("is_controller_alive", &ShmPublisher::is_controller_alive,
             py::arg("max_age_ms")   = 100.0,
             py::arg("robot_idx")    = 0u,
             py::arg("consumer_idx") = 0u)
        .def_property_readonly("n_robots",    &ShmPublisher::n_robots)
        .def_property_readonly("n_consumers", &ShmPublisher::n_consumers);

    /* ── ShmSubscriber ─────────────────────────────────────────────────── */
    py::class_<ShmSubscriber>(m, "ShmSubscriber",
        R"doc(
        Attaches to an existing shm segment; reads state, writes cmds.

        Multiple ShmSubscribers may attach to the same segment simultaneously.
        Each uses a distinct consumer_idx so the publisher can arbitrate via
        read_best_cmd().

        Parameters
        ----------
        name : str
            POSIX shm name (must match the publisher's).
        n_robots : int
            Hint; overridden by the segment header on attach().
        )doc")
        .def(py::init<std::string, unsigned>(),
             py::arg("name")     = SHMBRIDGE_SHM_NAME,
             py::arg("n_robots") = 1u)
        .def("attach", &ShmSubscriber::attach, py::arg("timeout_ms") = 30000.0,
             "Attach to an existing segment; retries if publisher not started yet.")
        .def("detach",      &ShmSubscriber::detach)
        .def("is_attached", &ShmSubscriber::is_attached)
        .def("__enter__", [](ShmSubscriber& self) -> ShmSubscriber& {
            self.attach(); return self;
        })
        .def("__exit__", [](ShmSubscriber& self, py::object, py::object,
                             py::object) { self.detach(); })
        .def("read_state",
             [](const ShmSubscriber& self, unsigned r) {
                 return self.read_state(r);
             },
             py::arg("robot_idx") = 0u,
             "Seqlock read; returns RobotState or None on torn read.")
        .def("read_state_spin",
             [](const ShmSubscriber& self, unsigned r, unsigned retries) {
                 return self.read_state_spin(r, retries);
             },
             py::arg("robot_idx") = 0u, py::arg("max_retries") = 64u,
             "Spin until a clean read; returns None only on persistent torn reads.")
        .def("write_cmd",
             [](ShmSubscriber& self, unsigned r, unsigned c,
                float lin, float ang) {
                 self.write_cmd(r, c, lin, ang);
             },
             py::arg("robot_idx"), py::arg("consumer_idx"),
             py::arg("linear"), py::arg("angular"),
             "Seqlock-write a velocity command to the specified consumer slot.")
        .def("is_publisher_alive", &ShmSubscriber::is_publisher_alive,
             py::arg("max_age_ms") = 100.0, py::arg("robot_idx") = 0u)
        .def_property_readonly("n_robots",    &ShmSubscriber::n_robots)
        .def_property_readonly("n_consumers", &ShmSubscriber::n_consumers);

    /* ── ImuSample / EncoderSample / PcHeaderSample / TopicInfo ───────────── */
    py::class_<ImuSample>(m, "ImuSample")
        .def(py::init<>())
        .def_readwrite("ax", &ImuSample::ax)
        .def_readwrite("ay", &ImuSample::ay)
        .def_readwrite("az", &ImuSample::az)
        .def_readwrite("gx", &ImuSample::gx)
        .def_readwrite("gy", &ImuSample::gy)
        .def_readwrite("gz", &ImuSample::gz)
        .def_readwrite("mx", &ImuSample::mx)
        .def_readwrite("my", &ImuSample::my)
        .def_readwrite("mz", &ImuSample::mz)
        .def_readwrite("ts", &ImuSample::ts);

    py::class_<EncoderSample>(m, "EncoderSample")
        .def(py::init<>())
        .def_readwrite("ticks", &EncoderSample::ticks)
        .def_readwrite("speed", &EncoderSample::speed)
        .def_readwrite("ts",    &EncoderSample::ts);

    py::class_<PcHeaderSample>(m, "PcHeaderSample")
        .def(py::init<>())
        .def_readwrite("n_points", &PcHeaderSample::n_points)
        .def_readwrite("max_pts",  &PcHeaderSample::max_pts)
        .def_readwrite("ts",       &PcHeaderSample::ts);

    py::class_<TopicInfo>(m, "TopicInfo")
        .def_readonly("name",  &TopicInfo::name)
        .def_readonly("alive", &TopicInfo::alive)
        .def_property_readonly("age_ms", [](const TopicInfo& t) -> py::object {
            if (!t.has_age) return py::none();
            return py::cast(t.age_ms);
        })
        .def("__repr__", [](const TopicInfo& t) {
            return "<TopicInfo name='" + t.name + "' alive=" +
                   (t.alive ? "True" : "False") + ">";
        });

    /* ── ExtShmBridge ──────────────────────────────────────────────────── */
    py::class_<ExtShmBridge>(m, "ExtShmBridge",
        R"doc(
        State + cmd (as ShmPublisher/ShmSubscriber) plus IMU, encoder, and
        point-cloud channels in one shm segment.

        Single robot/consumer, single class for both roles: whichever side
        calls open() creates the segment, whichever calls attach() maps an
        existing one. Either side may read or write any channel.

        Parameters
        ----------
        name : str
            POSIX shm name (default "/shmbridge_ext_v2").
        )doc")
        .def(py::init<std::string>(), py::arg("name") = EXT_SHM_NAME_DEFAULT)
        .def("open",  &ExtShmBridge::open,  "Create and zero-init the shm segment.")
        .def("close", &ExtShmBridge::close, "Unmap and unlink the segment.")
        .def("is_open", &ExtShmBridge::is_open)
        .def("attach", &ExtShmBridge::attach, py::arg("timeout_ms") = 30000.0,
             "Attach to an existing segment; retries if publisher not started "
             "yet. Raises ValueError on a schema/magic mismatch.")
        .def("detach", &ExtShmBridge::detach)
        .def("is_attached", &ExtShmBridge::is_attached)
        .def("__enter__", [](ExtShmBridge& self) -> ExtShmBridge& {
            self.open(); return self;
        })
        .def("__exit__", [](ExtShmBridge& self, py::object, py::object,
                             py::object) { self.close(); })
        .def("write_state", &ExtShmBridge::write_state, py::arg("state"),
             "Seqlock-write robot state from a RobotState object.")
        .def("write_state_fields",
             [](ExtShmBridge& self, double x, double y, double heading,
                float vx, float vy, float omega, float goal_x, float goal_y,
                float goal_dist, uint64_t step, double sim_time,
                bool reached, bool collision) {
                 RobotState s;
                 s.x = x; s.y = y; s.heading = heading;
                 s.vx = vx; s.vy = vy; s.omega = omega;
                 s.goal_x = goal_x; s.goal_y = goal_y; s.goal_dist = goal_dist;
                 s.step = step; s.sim_time = sim_time;
                 s.reached = reached; s.collision = collision;
                 self.write_state(s);
             },
             py::arg("x"), py::arg("y"), py::arg("heading"),
             py::arg("vx"), py::arg("vy"), py::arg("omega"),
             py::arg("goal_x"), py::arg("goal_y"), py::arg("goal_dist"),
             py::arg("step"), py::arg("sim_time"),
             py::arg("reached") = false, py::arg("collision") = false,
             "Seqlock-write robot state from flat args — builds the "
             "RobotState on the C++ side of the boundary instead of the "
             "caller constructing one field-by-field first (that "
             "construction, not the write itself, was the dominant cost: "
             "~2.3us for 11 pybind11 attribute setters vs ~0.2us for the "
             "actual seqlock write).")
        .def("read_state", &ExtShmBridge::read_state,
             "Seqlock read; returns RobotState or None on torn read / no data.")
        .def("write_cmd", &ExtShmBridge::write_cmd,
             py::arg("linear"), py::arg("angular"),
             "Seqlock-write a velocity command.")
        .def("read_cmd", &ExtShmBridge::read_cmd,
             "Non-blocking seqlock read. Returns None if mid-write or no "
             "valid cmd.")
        .def("write_imu", &ExtShmBridge::write_imu,
             py::arg("ax"), py::arg("ay"), py::arg("az"),
             py::arg("gx"), py::arg("gy"), py::arg("gz"),
             py::arg("mx") = 0.f, py::arg("my") = 0.f, py::arg("mz") = 0.f,
             py::arg("ts") = 0.f)
        .def("read_imu", &ExtShmBridge::read_imu)
        .def("write_encoder",
             [](ExtShmBridge& self, std::array<int32_t, 4> ticks,
                std::array<float, 4> speeds, float ts) {
                 self.write_encoder(ticks, speeds, ts);
             },
             py::arg("ticks"), py::arg("speeds"), py::arg("ts") = 0.f)
        .def("read_encoder", &ExtShmBridge::read_encoder)
        .def("write_pointcloud",
             [](ExtShmBridge& self, py::buffer points, double ts) {
                 py::buffer_info info = points.request();
                 if (info.itemsize != sizeof(float)) {
                     throw std::invalid_argument(
                         "write_pointcloud expects a float32 buffer");
                 }
                 size_t total = 1;
                 for (auto d : info.shape) total *= static_cast<size_t>(d);
                 if (info.shape.empty()) total = static_cast<size_t>(info.size);
                 self.write_pointcloud(static_cast<const float*>(info.ptr),
                                       total / 4, ts);
             },
             py::arg("points"), py::arg("ts") = 0.0,
             "Write N points (x, y, z, intensity float32 each) from any "
             "buffer-protocol object (numpy array, memoryview, ...).")
        .def("read_pointcloud_bytes",
             [](const ExtShmBridge& self) -> py::object {
                 std::vector<float> buf(EXT_PC_MAX_POINTS * 4);
                 size_t n = self.read_pointcloud(buf.data());
                 if (n == 0) return py::none();
                 return py::bytes(reinterpret_cast<const char*>(buf.data()),
                                   n * EXT_PC_POINT_BYTES);
             },
             "Raw (N*16)-byte float32 payload [x,y,z,intensity]*N, or None. "
             "The Python ExtShmBridge wrapper turns this into an (N,4) "
             "ndarray — kept as raw bytes here so this extension has no "
             "numpy build dependency.")
        .def("read_pointcloud_header", &ExtShmBridge::read_pointcloud_header)
        .def("write_channel",
             [](ExtShmBridge& self, const std::string& name, py::buffer data) {
                 py::buffer_info info = data.request();
                 self.write_channel(name, info.ptr,
                                     static_cast<size_t>(info.size) * info.itemsize);
             },
             py::arg("name"), py::arg("data"),
             "Seqlock-write raw bytes to a generic named user channel — see "
             "shmbridge.message.Channel for a struct.pack/unpack layer on "
             "top of this. Claims a free slot (of a fixed "
             "EXT_N_USER_CHANNELS pool) the first time `name` is used.")
        .def("read_channel",
             [](ExtShmBridge& self, const std::string& name) -> py::object {
                 std::array<uint8_t, EXT_USER_CHANNEL_PAYLOAD_BYTES> buf{};
                 if (!self.read_channel(name, buf.data())) return py::none();
                 return py::bytes(reinterpret_cast<const char*>(buf.data()),
                                   buf.size());
             },
             py::arg("name"),
             "Seqlock read of a generic named user channel's raw "
             "EXT_USER_CHANNEL_PAYLOAD_BYTES-byte payload, or None if the "
             "channel doesn't exist yet or the read was torn.")
        .def("is_imu_alive", &ExtShmBridge::is_imu_alive,
             py::arg("max_age_ms") = 100.0)
        .def("is_pointcloud_alive", &ExtShmBridge::is_pointcloud_alive,
             py::arg("max_age_ms") = 100.0)
        .def("list_topics", &ExtShmBridge::list_topics,
             py::arg("max_age_ms") = 300.0,
             "Enumerate every channel (state/cmd/imu/encoder/pointcloud, "
             "plus any claimed user channels) with liveness info.");

    /* ── Independent per-topic primitives (ext_topics.hpp) ────────────────
     *
     * Each of these is its own shm segment: a completely separate node/
     * process can publish (or subscribe to) just this one topic, without
     * ExtShmBridge's combined segment existing at all — see
     * ext_topics.hpp's own header comment for why this exists alongside
     * ExtShmBridge rather than replacing it outright. */

    auto raise_topic_error = [](TopicError err) {
        switch (err) {
            case TopicError::Ok:
                return;
            case TopicError::Timeout:
            case TopicError::NotReady:
                throw std::runtime_error(
                    "attach timeout: topic not found or publisher not ready");
            case TopicError::TypeMismatch:
            case TopicError::SizeMismatch:
                throw std::invalid_argument(
                    "topic type/size mismatch — publisher and subscriber "
                    "disagree on the message type");
            case TopicError::ShmFailed:
                throw std::runtime_error("shared-memory operation failed");
        }
    };

    // NOTE: no ImuPublisher/ImuSubscriber/EncoderPublisher/EncoderSubscriber
    // classes here (a previous revision had them as thin aliases over
    // Publisher<ShmImu>/Subscriber<ShmImu> etc.) — that's exactly the
    // per-message-type duplication the generic RawChannelPublisher/
    // RawChannelSubscriber below (plus a message *definition*, not a new
    // class, on the Python side — see shmbridge.message.Publisher/
    // Subscriber and urdf_tools.pubsub) exists to avoid, ROS-style. A C++
    // node that wants a real typed message can instantiate
    // shmbridge::Publisher<ShmImu>/Subscriber<ShmImu> (from topic.hpp)
    // directly; see ext_topics.hpp's header comment.

    py::class_<RawChannelPublisher>(m, "RawChannelPublisher",
        "Independent custom-message topic (own shm segment, unlimited "
        "distinct names) — Publisher<RawMsg88>. Backs "
        "shmbridge.message.Channel's struct.pack/unpack layer when a "
        "channel is run as its own node rather than through ExtShmBridge's "
        "fixed 8-slot pool.")
        .def(py::init<uint32_t>(), py::arg("heartbeat_every") = 1u)
        .def("open", [raise_topic_error](RawChannelPublisher& self, const std::string& name) {
            raise_topic_error(self.open(name));
        }, py::arg("name"))
        .def("close", &RawChannelPublisher::close)
        .def("is_open", &RawChannelPublisher::is_open)
        .def("write", [](RawChannelPublisher& self, py::bytes data) {
            std::string s = data;
            RawMsg88 msg{};
            std::memcpy(msg.data, s.data(), std::min(s.size(), sizeof(msg.data)));
            self.write(msg);
        }, py::arg("data"));

    py::class_<RawChannelSubscriber>(m, "RawChannelSubscriber",
        "Independent custom-message topic subscriber — Subscriber<RawMsg88>.")
        .def(py::init<>())
        .def("attach", [raise_topic_error](RawChannelSubscriber& self,
                                            const std::string& name, int timeout_ms) {
            raise_topic_error(self.attach(name, timeout_ms));
        }, py::arg("name"), py::arg("timeout_ms") = 5000)
        .def("detach", &RawChannelSubscriber::detach)
        .def("is_attached", &RawChannelSubscriber::is_attached)
        .def("read", [](RawChannelSubscriber& self) -> py::object {
            auto r = self.spin();
            if (!r || r->write_ns == 0) return py::none();  // see ImuSubscriber::read
            return py::bytes(reinterpret_cast<const char*>(r->value.data),
                              sizeof(r->value.data));
        })
        .def("is_publisher_alive", &RawChannelSubscriber::is_publisher_alive,
             py::arg("max_age_ms") = 500.0);

    py::class_<PointCloudPublisher>(m, "PointCloudPublisher",
        "Independent point-cloud topic (own shm segment) — bespoke bulk "
        "pair, since topic.hpp's SeqlockSlot<T> caps at 104 B and a cloud "
        "can be up to 1 MiB.")
        .def(py::init<>())
        .def("open", &PointCloudPublisher::open, py::arg("name"))
        .def("close", &PointCloudPublisher::close)
        .def("is_open", &PointCloudPublisher::is_open)
        .def("write", [](PointCloudPublisher& self, py::buffer points, double ts) {
            py::buffer_info info = points.request();
            if (info.itemsize != sizeof(float)) {
                throw std::invalid_argument("write expects a float32 buffer");
            }
            size_t total = info.shape.empty() ? static_cast<size_t>(info.size) : 1;
            for (auto d : info.shape) total *= static_cast<size_t>(d);
            self.write(static_cast<const float*>(info.ptr), total / 4, ts);
        }, py::arg("points"), py::arg("ts") = 0.0);

    py::class_<PointCloudSubscriber>(m, "PointCloudSubscriber",
        "Independent point-cloud topic subscriber.")
        .def(py::init<>())
        .def("attach", &PointCloudSubscriber::attach,
             py::arg("name"), py::arg("timeout_ms") = 30000.0)
        .def("detach", &PointCloudSubscriber::detach)
        .def("is_attached", &PointCloudSubscriber::is_attached)
        .def("read_bytes", [](const PointCloudSubscriber& self) -> py::object {
            // Scratch buffer sized for the worst case (PC_TOPIC_MAX_POINTS),
            // but allocated+zero-filled only once per thread rather than on
            // every call -- a fresh std::vector here cost ~200-300us/call
            // regardless of the actual point count (measured via
            // tests/bench_pubsub_matrix.py), dwarfing the real memcpy cost
            // for any cloud smaller than the 1 MiB max.
            static thread_local std::vector<float> buf(PC_TOPIC_MAX_POINTS * 4);
            size_t n = self.read(buf.data());
            if (n == 0) return py::none();
            return py::bytes(reinterpret_cast<const char*>(buf.data()), n * PC_TOPIC_POINT_BYTES);
        }, "Raw (N*16)-byte float32 payload [x,y,z,intensity]*N, or None — "
           "kept as raw bytes here so this extension has no numpy build "
           "dependency (mirrors ExtShmBridge.read_pointcloud_bytes).")
        .def("read_header", &PointCloudSubscriber::read_header)
        .def("is_publisher_alive", &PointCloudSubscriber::is_publisher_alive,
             py::arg("max_age_ms") = 100.0);

    /* ── spin_sleep utilities ──────────────────────────────────────────── */
    m.def("spin_sleep_us", &spin_sleep_us, py::arg("us"),
          "Precise hybrid sleep for `us` microseconds (nanosleep + spin tail).\n"
          "Releases the GIL so other Python threads can run during the coarse phase.");

    m.def("spin_sleep_ms", &spin_sleep_ms, py::arg("ms"),
          "Precise hybrid sleep for `ms` milliseconds.");

    m.def("now_ns_mono", &now_ns_mono,
          "Monotonic raw clock in nanoseconds (CLOCK_MONOTONIC_RAW).");

    py::class_<LoopSleeper>(m, "LoopSleeper",
        R"doc(
        Fixed-rate loop timer using precise hybrid sleep.

        Call start() at the top of each iteration and sleep() at the bottom.
        The remaining time in each period is consumed by nanosleep (coarse)
        followed by a _SB_PAUSE() spin loop (fine), so actual jitter is
        typically < 1 µs.

        Parameters
        ----------
        hz : float
            Target loop rate in Hz (default 200.0).
        )doc")
        .def(py::init<double>(), py::arg("hz") = 200.0)
        .def("set_hz",      &LoopSleeper::set_hz,      py::arg("hz"))
        .def("start",       &LoopSleeper::start)
        .def("sleep",       &LoopSleeper::sleep,
             "Sleep for the remainder of this period (releases GIL during wait).")
        .def("elapsed_ns",  &LoopSleeper::elapsed_ns,
             "Nanoseconds since the last start() call.")
        .def_readonly("target_ns", &LoopSleeper::target_ns,
                      "Target period in nanoseconds.");

    /* ── TaskScheduler ────────────────────────────────────────────────── */
    py::class_<TaskScheduler>(m, "TaskScheduler",
        R"doc(
        Lightweight in-process task scheduler with priority-based tick dispatch.

        Runs tasks at specified periods within a single thread.  Use run() in
        your own loop or run_threaded() to let it manage its own background
        thread.

        Priority rules
        --------------
        prio 0        -> runs every tick
        prio N        -> runs every (N+1) ticks; tasks at the same prio are
                         spread across N+1 slots so they don't all fire at once
        prio max_prio -> "lazy" time-based: fires when elapsed >= period

        Parameters
        ----------
        loop_hz  : float  Target base tick rate (default 200.0 Hz).
        max_prio : int    Max priority level; prio==max_prio tasks are lazy.
        )doc")
        .def(py::init<double, int>(),
             py::arg("loop_hz")  = 200.0,
             py::arg("max_prio") = 10)
        .def("add_task",
             [](TaskScheduler& sched, const std::string& name,
                py::object func, double period_ms) {
                 /* Wrap the Python callable; acquire GIL before calling it
                  * (important when run_threaded() is used). */
                 auto pyfunc = std::make_shared<py::object>(func);
                 sched.add_task(name.c_str(),
                     [pyfunc]() -> bool {
                         py::gil_scoped_acquire gil;
                         py::object ret = (*pyfunc)();
                         return ret.cast<bool>();
                     },
                     period_ms);
             },
             py::arg("name"), py::arg("func"), py::arg("period_ms"),
             R"doc(
             Register a periodic task.

             Parameters
             ----------
             name      : str    Label shown in report().
             func      : callable() -> bool
                         Returns True to keep running, False for one-shot removal.
             period_ms : float  Desired call period in milliseconds.
             )doc")
        .def("run", &TaskScheduler::run, py::call_guard<py::gil_scoped_release>(),
             "Execute one scheduler tick (releases GIL during C++ sleep phase).")
        .def("run_threaded", &TaskScheduler::run_threaded,
             "Start the scheduler in a background C++ thread; returns immediately.")
        .def("stop", &TaskScheduler::stop,
             "Stop the background thread and join it.")
        .def("is_running", &TaskScheduler::is_running)
        .def("tick",       &TaskScheduler::tick,
             "Number of ticks executed so far.")
        .def("target_ns",  &TaskScheduler::target_ns,
             "Base tick period in nanoseconds.")
        .def("report",     &TaskScheduler::report,
             "Return a markdown table with per-task timing statistics.");
}
