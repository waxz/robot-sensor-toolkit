/*
 * tests/soak_consumer_helper.cpp — one independent consumer process for
 * soak_test.cpp (design_ring_zero_copy.md §11.2/§15 phase 9).
 *
 * A genuinely separate OS process (not a thread), the same reasoning as
 * test_topic_sub_helper.cpp: polls a topic at a fixed interval for a fixed
 * duration, checking every message it receives against three violation
 * classes the soak test exists to rule out over a long, realistic run:
 *   - corrupted payload: `pattern` doesn't match the redundant checksum
 *     derived from `seq`
 *   - duplicate/out-of-order delivery: `seq` doesn't strictly increase
 *   - non-monotonic `write_ns`: the slot timestamp goes backward
 *
 * Usage: soak_consumer_helper <topic> <poll_interval_ms> <duration_s>
 * Prints exactly one line on exit: "received=<N> violations=<N> last_seq=<N>"
 * Exit code: 0 if violations == 0, 1 if violations > 0, 2 if attach failed.
 */

#include "shmbridge/ring.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>

struct SoakMsg {
    uint64_t seq;
    uint64_t pattern;
};

constexpr uint64_t kPatternMul = 0x9E3779B97F4A7C15ULL;

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: %s <topic> <poll_interval_ms> <duration_s>\n", argv[0]);
        return 2;
    }
    const std::string topic = argv[1];
    const int poll_interval_ms = std::atoi(argv[2]);
    const int duration_s = std::atoi(argv[3]);

    shmbridge::RingSubscriber<SoakMsg> sub;
    try {
        sub.attach(topic.c_str(), 10000.0); /* 10s: the producer opens first, so this should be immediate */
    } catch (const shmbridge::TimeoutError&) {
        std::fprintf(stderr, "soak_consumer_helper: publisher never appeared on '%s'\n", topic.c_str());
        return 2;
    }

    uint64_t received = 0, violations = 0;
    uint64_t last_seq = 0, last_write_ns = 0;
    bool have_last = false;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
    while (std::chrono::steady_clock::now() < deadline) {
        auto item = sub.pop_wait(poll_interval_ms);
        if (!item) continue;
        ++received;
        const SoakMsg& m = item->value;

        if (m.pattern != m.seq * kPatternMul) ++violations; /* corrupted payload */
        if (have_last) {
            if (m.seq <= last_seq) ++violations;             /* duplicate/out-of-order */
            if (item->write_ns <= last_write_ns) ++violations; /* non-monotonic write_ns */
        }
        last_seq = m.seq;
        last_write_ns = item->write_ns;
        have_last = true;
    }

    std::printf("received=%llu violations=%llu last_seq=%llu\n",
                static_cast<unsigned long long>(received),
                static_cast<unsigned long long>(violations),
                static_cast<unsigned long long>(last_seq));
    return violations == 0 ? 0 : 1;
}
