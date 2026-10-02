/*
 * talker.cpp -- publishes Pose2d at 100 Hz using the ros_compat Node API
 * (shmbridge/node.hpp). This is the reference implementation the README's
 * own "Quick start — C++ Node API" section documents inline; kept here as
 * a buildable, runnable copy so the two never drift apart.
 *
 * SensorDataQoS() (depth=1) maps to a seqlock (keep-latest) topic, not the
 * SPSC ring -- see node.hpp's own doc comment for the depth<=1 vs depth>1
 * rule. For a ring-backed walkthrough of zero-copy write/read, pop_latest,
 * freshness checks, config resolution, and the rest of ring.hpp's feature
 * set the Node API doesn't expose directly, see
 * ../cpp_ring_features/ring_features_demo.cpp.
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Run (either order -- listener attaches lazily and reattaches
 * automatically if talker restarts, §5.8/R-9):
 *   ./build/talker
 */

#include <shmbridge/node.hpp>
#include <shmbridge/messages.hpp>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sb  = shmbridge::ros_compat;
namespace msg = shmbridge::msg;

static volatile bool g_running = true;

int main() {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    sb::init();
    auto node = sb::make_node("talker");

    /* SensorDataQoS (depth=1) -> seqlock, keep-latest. */
    auto pub = node->create_publisher<msg::Pose2d>("robot/pose", sb::SensorDataQoS());

    std::printf("talker: publishing 'robot/pose' at 100 Hz -- Ctrl-C to stop\n");

    msg::Pose2d pose{};
    unsigned step = 0;
    while (g_running) {
        const double t = step * 0.01;
        pose.x        = std::cos(2.0 * M_PI * t / 5.0);
        pose.y        = std::sin(2.0 * M_PI * t / 5.0);
        pose.heading  = std::fmod(2.0 * M_PI * t / 5.0, 2.0 * M_PI);
        pose.stamp_ns = shmbridge::detail::now_ns();
        pub->publish(pose); /* zero-overhead handle path -- no map lookup, no cast */

        if (step % 100 == 0)
            std::printf("talker: step=%4u  x=%+.2f y=%+.2f heading=%+.2f\n",
                        step, pose.x, pose.y, pose.heading);
        ++step;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sb::shutdown();
    std::printf("talker: stopped (%u steps published)\n", step);
    return 0;
}
