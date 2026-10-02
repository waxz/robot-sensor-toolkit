/*
 * tests/soak_test.cpp — extended-duration multi-process soak test
 * (design_ring_zero_copy.md §11.2/§15 phase 9).
 *
 * Runs one producer against several independent consumer PROCESSES (via
 * soak_consumer_helper, launched through platform::run_and_capture() --
 * the same cross-platform subprocess mechanism test_topic.cpp's
 * MultipleSubscribers test uses, not threads) at varied, realistic poll
 * rates, for at least 30 minutes by default. §11.2's validation
 * acceptance criterion: the soak test completes with zero correctness
 * violations (duplicate delivery, corrupted payload, or non-monotonic
 * write_ns within a single subscriber's own observed sequence).
 *
 * This is a long-running manual validation tool, not a unit test: it is
 * not registered with ctest (a 30-minute default run has no place in a
 * routine test pass) and is gated behind its own SHMBRIDGE_BUILD_SOAK_TEST
 * CMake option, same convention as SHMBRIDGE_BUILD_BENCH.
 *
 * Usage: soak_test [duration_s] [capacity] [n_consumers]
 *   duration_s  default 1800 (30 minutes, §11.2's stated minimum)
 *   capacity    default 1024 (realistic, not R-2's deliberately
 *               adversarial capacity=2 -- see test_ring.cpp's
 *               RingTornReadRetry for that stress case instead)
 *   n_consumers default 4, at poll intervals 1/5/20/100 ms (cycled if
 *               n_consumers > 4, matching "multiple independent consumer
 *               processes at varied poll rates")
 */

#include "shmbridge/platform.hpp"
#include "shmbridge/ring.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <string>
#include <vector>

#ifndef SHMBRIDGE_SOAK_CONSUMER_HELPER_PATH
#  error "SHMBRIDGE_SOAK_CONSUMER_HELPER_PATH must be defined by CMakeLists.txt"
#endif

struct SoakMsg {
    uint64_t seq;
    uint64_t pattern;
};

constexpr uint64_t kPatternMul = 0x9E3779B97F4A7C15ULL;

int main(int argc, char** argv) {
    const int duration_s  = argc > 1 ? std::atoi(argv[1]) : 1800;
    const uint32_t capacity = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 1024u;
    const int n_consumers = argc > 3 ? std::atoi(argv[3]) : 4;
    const std::string topic = "soak_test_main";

    std::printf("soak_test: duration=%ds capacity=%u n_consumers=%d\n",
                duration_s, capacity, n_consumers);

    shmbridge::platform::shm_destroy("/sbr_" + topic); /* clean slate from any prior run */

    shmbridge::RingConfig cfg;
    cfg.capacity = capacity;
    shmbridge::RingPublisher<SoakMsg> pub;
    if (!pub.open(topic.c_str(), cfg)) {
        std::fprintf(stderr, "soak_test: failed to open publisher\n");
        return 2;
    }

    /* Launch all consumer processes concurrently; each run_and_capture()
     * call blocks until its own helper exits, so each needs its own
     * thread to run alongside the producer loop below. */
    const int poll_intervals_ms[] = {1, 5, 20, 100};
    std::vector<std::future<std::pair<int, std::string>>> consumers;
    consumers.reserve(static_cast<std::size_t>(n_consumers));
    for (int i = 0; i < n_consumers; ++i) {
        const int interval = poll_intervals_ms[i % 4];
        std::string cmd = std::string("\"") + SHMBRIDGE_SOAK_CONSUMER_HELPER_PATH + "\" \""
                        + topic + "\" " + std::to_string(interval) + " " + std::to_string(duration_s);
        consumers.push_back(std::async(std::launch::async, [cmd] {
            return shmbridge::platform::run_and_capture(cmd);
        }));
    }

    /* Producer: push flat-out for duration_s, same convention
     * bench_migration.cpp's bench_ring_spsc_throughput already uses --
     * deliberately not paced with a per-push sleep: an earlier version
     * tried platform::sleep_ns(50'000) targeting ~20kHz, but Windows'
     * Sleep() rounds any sub-~15ms request up to its ~15.6ms timer-tick
     * granularity (the same R-18/NFR-6 characteristic documented
     * elsewhere in this design), so the *same* requested pacing produced
     * ~5.9kHz on WSL/Linux but only ~90Hz on native Windows -- almost two
     * orders of magnitude apart, for no benefit (less stress exposure,
     * not more realism; a real control loop publishing at 10-1kHz is
     * already far below either rate). Pushing flat-out removes the
     * platform-dependent sleep granularity from the equation entirely and
     * maximizes eviction/overwrite cycles over the run, which is the
     * actual goal (§11.2: "build confidence against rare torn-read or
     * race conditions"). */
    uint64_t seq = 0;
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::seconds(duration_s);
    auto next_report = start + std::chrono::minutes(1);
    while (std::chrono::steady_clock::now() < deadline) {
        SoakMsg m{seq, seq * kPatternMul};
        pub.push(m);
        ++seq;

        if (std::chrono::steady_clock::now() >= next_report) {
            auto elapsed_min = std::chrono::duration_cast<std::chrono::minutes>(
                std::chrono::steady_clock::now() - start).count();
            std::printf("soak_test: %lldmin elapsed, %llu messages pushed so far\n",
                        static_cast<long long>(elapsed_min),
                        static_cast<unsigned long long>(seq));
            std::fflush(stdout);
            next_report += std::chrono::minutes(1);
        }
    }

    std::printf("soak_test: producer done, %llu messages pushed, waiting for consumers...\n",
                static_cast<unsigned long long>(seq));

    bool all_ok = true;
    for (int i = 0; i < n_consumers; ++i) {
        auto [exit_code, out] = consumers[static_cast<std::size_t>(i)].get();
        const int interval = poll_intervals_ms[i % 4];
        std::printf("soak_test: consumer[%d] (poll=%dms) exit=%d output: %s",
                    i, interval, exit_code, out.c_str());
        if (exit_code != 0) all_ok = false;
    }

    std::printf(all_ok ? "SOAK PASS\n" : "SOAK FAIL\n");
    return all_ok ? 0 : 1;
}
