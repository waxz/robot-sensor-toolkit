/*
 * tests/test_topic_sub_helper.cpp — standalone subscriber process for
 * test_topic.cpp's MultipleSubscribers test.
 *
 * Attaches as a Subscriber<Pose2d> to a named topic, reads a fixed number
 * of times, and prints its torn-read stats to stdout. A genuinely separate
 * OS process is the point: test_topic.cpp used to fork() a copy of itself
 * for this, which has no Windows equivalent; launching N copies of this
 * tiny helper via platform::run_and_capture() gives the same real,
 * separate-process concurrency on every platform instead.
 *
 * Usage: test_topic_sub_helper <topic_name> <reads>
 * Prints exactly one line on success: "torn=<N> total=<N> last_x=<N>"
 * Exit code: 0 on success, 1 if attach() failed.
 */

#include "shmbridge/messages.hpp"
#include "shmbridge/topic.hpp"

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <topic_name> <reads>\n", argv[0]);
        return 2;
    }
    const std::string topic = argv[1];
    const int reads = std::atoi(argv[2]);

    shmbridge::Subscriber<shmbridge::msg::Pose2d> sub;
    if (sub.attach(topic, 3000) != shmbridge::TopicError::Ok) {
        return 1;
    }

    int last_x = -1;
    for (int r = 0; r < reads; ++r) {
        if (auto res = sub.spin(256)) {
            last_x = static_cast<int>(res->value.x);
        }
    }

    const auto& stats = sub.stats();
    std::printf("torn=%llu total=%llu last_x=%d\n",
                static_cast<unsigned long long>(stats.torn_reads),
                static_cast<unsigned long long>(stats.total_reads),
                last_x);
    return 0;
}
