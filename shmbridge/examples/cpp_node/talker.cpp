/*
 * talker.cpp — publishes robot state at 100 Hz using ShmPublisher.
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Run (start before or after listener — either order works):
 *   ./build/talker
 *
 * The publisher writes a circular-motion trajectory and prints any velocity
 * commands it receives from listener.
 */

#include <shmbridge/core.hpp>

#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace shmbridge;

static volatile bool g_running = true;

int main() {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    const std::string name = "/sb_demo";
    ShmPublisher pub(name, /*n_robots=*/1, /*n_consumers=*/1, /*heartbeat_every=*/1);

    std::printf("talker: opening segment '%s' at 100 Hz\n", name.c_str());
    pub.open();
    std::printf("talker: publishing — Ctrl-C to stop\n");

    unsigned step = 0;
    while (g_running) {
        const double t = step * 0.01;
        RobotState s;
        s.x        = std::cos(2.0 * M_PI * t / 5.0);
        s.y        = std::sin(2.0 * M_PI * t / 5.0);
        s.heading  = std::fmod(2.0 * M_PI * t / 5.0, 2.0 * M_PI);
        s.step     = step;
        s.sim_time = t;
        pub.write_state(0, s);

        auto cmd = pub.read_best_cmd(0);
        if (cmd && step % 100 == 0)
            std::printf("talker: step=%4u  x=%+.2f y=%+.2f"
                        "  cmd(lin=%.2f ang=%.2f)\n",
                        step, s.x, s.y, cmd->linear, cmd->angular);

        ++step;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    pub.close();
    std::printf("talker: stopped (%u steps published)\n", step);
    return 0;
}
