#pragma once
/*
 * spin_sleep.hpp — Precise hybrid spin-sleep for sub-millisecond scheduling.
 *
 * Algorithm (inspired by common/suspend.h's preciseSleep):
 *   1. Coarse phase: nanosleep(1ms) until remaining < OS_estimate
 *   2. Fine  phase: busy-spin with _SB_PAUSE() until deadline
 *   3. Adapts the OS overhead estimate per thread via Welford online stats.
 *
 * Uses CLOCK_MONOTONIC_RAW (immune to NTP adjustments) instead of
 * high_resolution_clock for lower jitter.
 */

#include "platform.hpp"

#include <cmath>
#include <cstdint>

/* CPU spin-wait hint — reduce pipeline stalls and memory traffic.
 * Guard avoids redefinition if core.hpp is also included. */
#ifndef _SB_PAUSE
#  if defined(__x86_64__) || defined(__i386__)
#    define _SB_PAUSE() __builtin_ia32_pause()
#  elif defined(_M_X64) || defined(_M_IX86)
#    define _SB_PAUSE() _mm_pause()
#  elif defined(__aarch64__) || defined(__ARM_ARCH_8A__)
#    define _SB_PAUSE() __asm__ volatile("yield" ::: "memory")
#  else
#    define _SB_PAUSE() ((void)0)
#  endif
#endif

namespace shmbridge {

/* Returns monotonic nanoseconds.
 * Linux uses CLOCK_MONOTONIC_RAW (immune to NTP); other platforms use
 * platform::now_ns() which selects the best available clock. */
inline int64_t now_ns_mono() noexcept {
#if defined(__linux__)
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
#else
    return static_cast<int64_t>(platform::now_ns());
#endif
}

/* Precise hybrid sleep: nanosleep bulk + spin tail.
 *
 * Thread-local Welford estimator tracks the actual OS sleep overhead so the
 * transition from coarse to fine happens at the right point without wasting
 * spin cycles. */
inline void spin_sleep_ns(int64_t ns) noexcept {
    if (ns <= 0) return;

    /* Thread-local adaptive estimate of OS sleep overhead (ns).
     * Starting at 1 ms ensures targets < 1 ms spin immediately with no
     * nanosleep overshoot; longer targets self-adapt via Welford. */
    thread_local double estimate = 1'000'000.0;  /* start at 1 ms */
    thread_local double mean     = 1'000'000.0;
    thread_local double m2       = 0.0;
    thread_local int64_t count   = 1;

    int64_t remaining = ns;

    /* Coarse phase: 1 ms sleep while remaining > estimated overhead. */
    while (remaining > static_cast<int64_t>(estimate)) {
        int64_t t0 = now_ns_mono();
        platform::sleep_ns(1'000'000LL);
        int64_t actual = now_ns_mono() - t0;
        remaining -= actual;

        /* Welford online mean + stddev. */
        ++count;
        double delta = static_cast<double>(actual) - mean;
        mean += delta / count;
        m2   += delta * (static_cast<double>(actual) - mean);
        double stddev = (count > 1) ? std::sqrt(m2 / (count - 1)) : mean * 0.1;
        estimate = mean + stddev;
    }

    /* Fine phase: spin until deadline with a CPU hint. */
    int64_t deadline = now_ns_mono() + remaining;
    while (now_ns_mono() < deadline) {
        _SB_PAUSE();
    }
}

/* Convenience wrappers. */
inline void spin_sleep_us(double us) noexcept {
    spin_sleep_ns(static_cast<int64_t>(us * 1'000.0));
}
inline void spin_sleep_ms(double ms) noexcept {
    spin_sleep_ns(static_cast<int64_t>(ms * 1'000'000.0));
}

/* Fixed-rate loop helper: call start() at the top, sleep() at the bottom.
 *
 * Maintains a target loop period via spin_sleep_ns for the remainder time
 * after the body has executed.  Accurate to within one spin iteration
 * (~0.3 µs on modern x86). */
struct LoopSleeper {
    int64_t target_ns;  /* desired period per iteration */
    int64_t stamp_ns;   /* time of last start() */

    explicit LoopSleeper(double hz = 200.0)
        : target_ns(static_cast<int64_t>(1e9 / hz)), stamp_ns(0) {}

    void set_hz(double hz) noexcept {
        target_ns = static_cast<int64_t>(1e9 / hz);
    }

    /* Record the start of this iteration. */
    void start() noexcept { stamp_ns = now_ns_mono(); }

    /* Sleep for whatever time remains in this period. */
    void sleep() noexcept {
        int64_t elapsed = now_ns_mono() - stamp_ns;
        int64_t rem     = target_ns - elapsed;
        if (rem > 10'000LL)  /* skip if < 10µs remains (already overrun) */
            spin_sleep_ns(rem);
    }

    /* Elapsed time since last start(), in nanoseconds. */
    int64_t elapsed_ns() const noexcept { return now_ns_mono() - stamp_ns; }
};

}  /* namespace shmbridge */
