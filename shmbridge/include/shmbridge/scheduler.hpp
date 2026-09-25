#pragma once
/*
 * scheduler.hpp — Lightweight in-process task scheduler for shmbridge.
 *
 * Adapted from waxz/cmake_super_build include/common/task.h with the
 * following improvements:
 *   - Uses shmbridge::spin_sleep (CLOCK_MONOTONIC_RAW + Welford adaptive)
 *     instead of condition_variable + busy spin.
 *   - Nanosecond-resolution timestamps and periods.
 *   - run_threaded() / stop() lifecycle for background operation.
 *   - Correct one-shot removal (func returns false → remove).
 *   - report() returns a markdown table with per-task timing stats.
 *
 * Priority rules (same as reference TaskManager):
 *   prio 0        → runs every tick  (highest frequency)
 *   prio N        → runs every (N+1) ticks; N tasks at this prio are spread
 *                   across N+1 slots so they never all fire on the same tick
 *   prio max_prio → "lazy": time-based only, runs when elapsed ≥ delay_ns
 *
 * Typical usage:
 *   TaskScheduler sched(200.0);                // 200 Hz base tick
 *   sched.add_task("pub",  pub_fn,  5.0);     // 5 ms period
 *   sched.add_task("slow", slow_fn, 20.0);    // 20 ms period
 *   sched.run_threaded();
 *   // ... do other work ...
 *   sched.stop();
 */

#include <functional>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <sstream>

#include <shmbridge/spin_sleep.hpp>

namespace shmbridge {

struct TaskScheduler {

    /* ── Task record ───────────────────────────────────────────────── */
    struct Task {
        std::string           name;
        bool                  valid            = true;
        int                   prio             = 0;
        int                   slot             = 0;
        int64_t               delay_ns         = 0;
        int64_t               last_run_ns      = 0;
        int64_t               run_time_ns      = 0;
        int64_t               max_run_time_ns  = 0;
        int64_t               run_counter      = 0;
        std::function<bool()> func;

        Task(const char* name_, std::function<bool()> func_,
             int64_t delay_ns_, int prio_, int slot_)
            : name(name_), func(std::move(func_)),
              delay_ns(delay_ns_), prio(prio_), slot(slot_),
              last_run_ns(now_ns_mono()) {}
    };

    /* ── Scheduler state ───────────────────────────────────────────── */
    int              max_prio_;
    uint64_t         tick_       = 0;
    LoopSleeper      loop_;
    std::vector<Task> tasks_;
    std::vector<int>  order_;          /* sorted indices into tasks_ */
    std::vector<int>  prio_counters_;  /* counts tasks added per priority level */

    std::thread       thread_;
    std::atomic<bool> running_{false};

    /* ── Construction ──────────────────────────────────────────────── */

    /* loop_hz : base tick rate (all periods must be multiples of 1/loop_hz).
     * max_prio: prio == max_prio tasks are time-based ("lazy").  */
    explicit TaskScheduler(double loop_hz = 200.0, int max_prio = 10)
        : max_prio_(max_prio),
          loop_(loop_hz),
          prio_counters_(max_prio + 1, 0) {}

    ~TaskScheduler() { stop(); }

    /* ── Adding tasks ──────────────────────────────────────────────── */

    /* Add a periodic task.
     *
     * name      : human-readable label for report().
     * func      : callable returning bool; false → one-shot (removed after run).
     * period_ms : desired call period in milliseconds.
     *
     * The actual priority and slot are derived automatically so that tasks
     * with the same period are interleaved across ticks rather than piling
     * up on the same tick. */
    void add_task(const char* name, std::function<bool()> func, double period_ms) {
        double base_ms = static_cast<double>(loop_.target_ns) / 1'000'000.0;
        int    prio    = static_cast<int>(period_ms / base_ms);
        prio = std::max(0, std::min(prio, max_prio_));

        int slot = prio_counters_[prio] % (prio + 1);
        ++prio_counters_[prio];

        int64_t delay_ns = static_cast<int64_t>(period_ms * 1'000'000.0);
        tasks_.emplace_back(name, std::move(func), delay_ns, prio, slot);
        _rebuild_order();
    }

    /* ── Run loop ──────────────────────────────────────────────────── */

    /* Execute one scheduler tick.  Call this in your own loop, or use
     * run_threaded() to let the scheduler manage its own thread. */
    bool run() {
        loop_.start();
        int64_t now = now_ns_mono();
        bool    gc  = false;

        for (int idx : order_) {
            Task& t = tasks_[idx];

            bool should_run;
            if (t.prio == max_prio_) {
                /* Lazy task: fires when elapsed time ≥ delay. */
                should_run = (now - t.last_run_ns) >= t.delay_ns;
            } else {
                /* Tick-based: slot assigned at add_task distributes load. */
                int slot_now = static_cast<int>(tick_ % static_cast<uint64_t>(t.prio + 1));
                should_run   = (slot_now == t.slot);
            }

            if (should_run) {
                int64_t t0 = now_ns_mono();
                t.valid      = t.func();
                int64_t t1  = now_ns_mono();
                t.run_time_ns = t1 - t0;
                if (t.run_time_ns > t.max_run_time_ns)
                    t.max_run_time_ns = t.run_time_ns;
                t.last_run_ns = now;
                ++t.run_counter;
                if (!t.valid) gc = true;
            }
        }

        if (gc) _gc();

        ++tick_;
        loop_.sleep();
        return true;
    }

    /* Start running in a background thread; returns immediately.
     * Call stop() to join the thread. */
    void run_threaded() {
        running_.store(true, std::memory_order_release);
        thread_ = std::thread([this] {
            while (running_.load(std::memory_order_acquire))
                run();
        });
    }

    /* Signal the background thread to stop and join it. */
    void stop() {
        running_.store(false, std::memory_order_release);
        if (thread_.joinable()) thread_.join();
    }

    bool is_running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    /* ── Diagnostics ───────────────────────────────────────────────── */

    /* Returns a markdown table with per-task timing statistics. */
    std::string report() const {
        std::ostringstream s;
        s << "\n| name | prio | slot | run_time_µs | max_run_time_µs | runs |\n";
        s <<   "|------|------|------|-------------|-----------------|------|\n";
        for (const auto& t : tasks_) {
            s << "| " << t.name
              << " | "  << t.prio
              << " | "  << t.slot
              << " | "  << (t.run_time_ns / 1'000)
              << " | "  << (t.max_run_time_ns / 1'000)
              << " | "  << t.run_counter << " |\n";
        }
        return s.str();
    }

    uint64_t tick()       const noexcept { return tick_; }
    int64_t  target_ns()  const noexcept { return loop_.target_ns; }

private:
    /* Sort task indices by (prio ASC, delay_ns ASC) so high-priority short-
     * period tasks are checked first.  O(N log N) but N is typically < 20. */
    void _rebuild_order() {
        order_.resize(tasks_.size());
        for (int i = 0; i < static_cast<int>(tasks_.size()); ++i) order_[i] = i;
        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const Task& ta = tasks_[a];
            const Task& tb = tasks_[b];
            return ta.prio < tb.prio
                || (ta.prio == tb.prio && ta.delay_ns < tb.delay_ns);
        });
    }

    void _gc() {
        tasks_.erase(
            std::remove_if(tasks_.begin(), tasks_.end(),
                           [](const Task& t) { return !t.valid; }),
            tasks_.end());
        _rebuild_order();
    }
};

}  /* namespace shmbridge */
