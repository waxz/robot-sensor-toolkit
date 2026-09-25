/*
 * listener.cpp -- receives robot state and writes velocity commands.
 *
 * Demonstrates the reconnect loop: attach() auto-detaches any previous
 * mapping so it is safe to call repeatedly when the publisher restarts.
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Run (either order; listener re-attaches each time talker restarts):
 *   ./build/listener
 *
 * Reconnect pattern:
 *   while (running) {
 *       // retry attach in 1-second chunks so Ctrl-C is processed promptly
 *       while (running) { try { sub.attach(1000); break; } catch (...) {} }
 *       while (sub.is_publisher_alive(500)) { ... read + write_cmd ... }
 *       sub.detach();               // publisher gone -- loop back
 *   }
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

    const std::string name    = "/sb_demo";
    const unsigned consumer   = 0;
    uint64_t       frames     = 0;

    ShmSubscriber sub(name, /*n_robots=*/1);

    while (g_running) {
        std::printf("listener: waiting for publisher '%s'...\n", name.c_str());

        /* Retry attach in 1-second chunks so Ctrl-C is responded to promptly */
        bool attached = false;
        while (g_running && !attached) {
            try {
                sub.attach(1000.0);
                attached = true;
            } catch (const std::exception&) { /* timeout -- keep waiting */ }
        }
        if (!attached) break;

        std::printf("listener: attached -- reading state\n");

        while (g_running) {
            auto state = sub.read_state_spin(0);
            if (state) {
                /* Proportional heading controller aimed at origin */
                double err_x = -state->x;
                double err_y = -state->y;
                double dist  = std::hypot(err_x, err_y);
                double desired = std::atan2(err_y, err_x);
                double hd_err  = desired - state->heading;
                while (hd_err >  M_PI) hd_err -= 2.0 * M_PI;
                while (hd_err < -M_PI) hd_err += 2.0 * M_PI;

                float linear  = static_cast<float>(std::min(0.5 * dist, 1.0));
                float angular = static_cast<float>(1.5 * hd_err);
                sub.write_cmd(0, consumer, linear, angular);
                ++frames;

                if (frames % 100 == 0)
                    std::printf("listener: step=%llu  x=%+.3f y=%+.3f"
                                "  -> lin=%.2f ang=%.2f\n",
                                (unsigned long long)state->step,
                                state->x, state->y, linear, angular);
            }

            if (!sub.is_publisher_alive(500.0)) {
                std::printf("listener: publisher went away -- waiting for restart\n");
                sub.detach();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::printf("listener: stopped (%llu frames received)\n",
                (unsigned long long)frames);
    return 0;
}
