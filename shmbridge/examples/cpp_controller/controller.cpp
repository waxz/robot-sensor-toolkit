/*
 * controller.cpp -- proportional heading + speed controller using ShmSubscriber.
 *
 * Connects to a shmbridge publisher (e.g. a running robot simulator or the
 * Python publisher_demo.py), reads robot state, and writes velocity commands.
 *
 * The controller survives publisher restarts: when the publisher exits and
 * re-opens, attach() silently releases the old mapping and re-attaches to the
 * new segment.
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Run (after starting the Python sim or publisher_demo.py):
 *   ./build/controller [shm_name]   (default: /shmbridge_v2)
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

static constexpr double KP_HEADING  = 1.5;
static constexpr double KP_SPEED    = 0.5;
static constexpr double MAX_SPEED   = 1.0;
static constexpr double GOAL_RADIUS = 0.5;

static volatile bool g_running = true;

int main(int argc, char** argv) {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    const std::string name = (argc > 1) ? argv[1] : SHMBRIDGE_SHM_NAME;
    std::printf("controller: connecting to '%s'\n", name.c_str());

    ShmSubscriber sub(name, /*n_robots=*/1);

    while (g_running) {
        std::printf("controller: waiting for publisher...\n");

        /* Retry attach in 1-second chunks so Ctrl-C is responded to promptly */
        bool attached = false;
        while (g_running && !attached) {
            try {
                sub.attach(1000.0);
                attached = true;
            } catch (const std::exception&) { /* timeout -- keep waiting */ }
        }
        if (!attached) break;

        std::printf("controller: attached\n");
        uint64_t last_step = UINT64_MAX;

        while (g_running) {
            auto state = sub.read_state_spin(0);
            if (!state) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }

            /* Skip duplicate steps */
            if (state->step == last_step) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            last_step = state->step;

            if (!sub.is_publisher_alive(200.0))
                std::printf("WARN: publisher stale (> 200 ms)\n");

            /* Goal reached -- send stop and break out */
            if (state->reached || state->goal_dist < GOAL_RADIUS) {
                sub.write_cmd(0, 0, 0.0f, 0.0f);
                std::printf("controller: goal reached at step %llu\n",
                            (unsigned long long)state->step);
                g_running = false;
                break;
            }

            /* Proportional heading + speed controller */
            double goal_angle = std::atan2(state->goal_y - state->y,
                                           state->goal_x - state->x);
            double err = goal_angle - state->heading;
            while (err >  M_PI) err -= 2.0 * M_PI;
            while (err < -M_PI) err += 2.0 * M_PI;

            float angular = static_cast<float>(KP_HEADING * err);
            float linear  = 0.0f;
            if (std::fabs(err) < 0.3)
                linear = static_cast<float>(
                    std::min(KP_SPEED * state->goal_dist, MAX_SPEED));

            sub.write_cmd(0, /*consumer=*/0, linear, angular);

            if (state->step % 100 == 0)
                std::printf("controller: step=%6llu  pos(%.2f,%.2f)"
                            "  dist=%.2f  cmd(%.2f,%.2f)\n",
                            (unsigned long long)state->step,
                            state->x, state->y, state->goal_dist,
                            linear, angular);

            if (!sub.is_publisher_alive(500.0)) {
                std::printf("controller: publisher went away -- waiting for restart\n");
                sub.detach();
                break;
            }
        }
    }

    std::printf("controller: stopped\n");
    return 0;
}
