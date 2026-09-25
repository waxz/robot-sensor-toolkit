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
