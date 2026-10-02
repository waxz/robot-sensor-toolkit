/*
 * listener.cpp -- receives 'robot/pose' via a MessageQueue<Pose2d>,
 * printing each message's age (publish-to-observe latency, microseconds).
 * This is the reference implementation the README's own "Quick start — C++
 * Node API" section documents inline (the queue/deferred-style variant);
 * kept here as a buildable, runnable copy so the two never drift apart.
 *
 * Demonstrates the deferred-callback pattern: spin_once() only accumulates
 * messages into the queue (attaching lazily, never blocking), decoupling
 * message arrival from processing -- no callback-timing pressure the way
 * create_subscription()'s inline-callback style has. pop_latest() discards
 * any backlog and returns only the newest value (F-13's semantics,
 * surfaced here at the MessageQueue level).
 *
 * Starting this before talker, or restarting talker while this keeps
 * running, both just work with no special-case code here: spin_once()
 * retries attaching automatically (never throws for "not found yet"), and
 * node.hpp's resilient-loop wiring (§5.8/R-9) detaches and re-attaches on
 * its own if the publisher it was already attached to closes.
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Run (either order):
 *   ./build/listener
 */

#include <shmbridge/node.hpp>
#include <shmbridge/messages.hpp>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

namespace sb  = shmbridge::ros_compat;
namespace msg = shmbridge::msg;

static volatile bool g_running = true;

int main() {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    sb::init();
    auto node = sb::make_node("listener");

    /* create_queue returns {subscription_handle, shared_ptr<MessageQueue<T>>}. */
    auto [sub, queue] = node->create_queue<msg::Pose2d>("robot/pose", sb::SensorDataQoS());
    (void)sub; /* the handle just needs to stay alive; nothing else to call on it here */

    std::printf("listener: waiting for 'robot/pose' -- Ctrl-C to stop\n");

    uint64_t frames = 0;
    while (g_running) {
        node->spin_once(); /* attaches lazily; never blocks */

        if (auto pose = queue->pop_latest()) {
            uint64_t age_us = (shmbridge::detail::now_ns() - pose->stamp_ns) / 1000;
            ++frames;
            if (frames % 100 == 0 || frames == 1)
                std::printf("listener: frame=%llu  x=%+.2f y=%+.2f heading=%+.2f  age=%lluus\n",
                            (unsigned long long)frames, pose->x, pose->y, pose->heading,
                            (unsigned long long)age_us);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sb::shutdown();
    std::printf("listener: stopped (%llu frames received)\n", (unsigned long long)frames);
    return 0;
}
