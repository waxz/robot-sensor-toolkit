/*
 * planner_node.cpp -- potential-field path planner.
 *
 * Subscriber: /sb_sensor -- robot state (x, y, heading, goal info)  [LATEST-ONLY]
 * Publisher:  /sb_plan   -- blended velocity (vx=linear, vy=angular)
 *
 * Tasks (100 Hz loop):
 *   1. Wait for sensor data (LATEST-ONLY read; skip if no new frame)
 *   2. Attractive potential: pull toward current goal
 *   3. Repulsive potential: push away from known obstacle
 *   4. Blend forces -> unicycle velocity command
 *   5. Publish plan on /sb_plan
 *
 * Run order: any -- sensor publishes immediately; planner attaches later.
 *
 *   ./sensor_node &
 *   ./planner_node &
 *   ./control_node
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

static constexpr double DT          = 0.01;
static constexpr double K_ATT       = 0.8;    /* attractive gain  */
static constexpr double K_REP       = 0.6;    /* repulsive gain   */
static constexpr double D_INFLUENCE = 1.2;    /* obstacle influence radius (m) */
static constexpr double MAX_LINEAR  = 1.0;    /* m/s  */
static constexpr double MAX_ANGULAR = 3.0;    /* rad/s */
static constexpr double OBS_X       = 1.5;
static constexpr double OBS_Y       = 1.0;

int main() {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    /* Subscriber: receive robot state from sensor_node */
    ShmSubscriber sub_sensor("/sb_sensor", /*n_robots=*/1);

    std::printf("[planner] waiting for /sb_sensor...\n");
    while (g_running) {
        try { sub_sensor.attach(1000.0); break; }
        catch (const std::exception&) {}
    }
    if (!g_running) return 0;
    std::printf("[planner] /sb_sensor attached\n");

    /* Publisher: broadcast planned velocity so control_node can read it */
    ShmPublisher pub("/sb_plan", /*n_robots=*/1, /*n_consumers=*/1,
                     /*heartbeat_every=*/1);
    pub.open();
    std::printf("[planner] /sb_plan open\n");

    uint64_t last_step = UINT64_MAX;
    uint64_t frames    = 0;

    while (g_running) {
        /* ── Task 1: Read latest sensor state (LATEST-ONLY) ─────────────────── */
        auto state = sub_sensor.read_state_spin(0);
        if (!state) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        if (state->step == last_step) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        last_step = state->step;

        double x       = state->x;
        double y       = state->y;
        double heading = state->heading;
        double gx      = state->goal_x;
        double gy      = state->goal_y;

        /* ── Task 2: Attractive force toward goal ────────────────────────────── */
        double att_x = K_ATT * (gx - x);
        double att_y = K_ATT * (gy - y);

        /* ── Task 3: Repulsive force away from obstacle ──────────────────────── */
        double dx_obs = x - OBS_X;
        double dy_obs = y - OBS_Y;
        double d_obs  = std::hypot(dx_obs, dy_obs);

        double rep_x = 0.0, rep_y = 0.0;
        if (d_obs < D_INFLUENCE && d_obs > 1e-6) {
            double mag = K_REP * (1.0 / d_obs - 1.0 / D_INFLUENCE)
                               * (1.0 / (d_obs * d_obs));
            rep_x = mag * (dx_obs / d_obs);
            rep_y = mag * (dy_obs / d_obs);
        }

        /* ── Task 4: Blend forces -> unicycle velocity ───────────────────────── */
        double fx = att_x + rep_x;
        double fy = att_y + rep_y;

        double desired_angle = std::atan2(fy, fx);
        double hd_err        = desired_angle - heading;
        while (hd_err >  M_PI) hd_err -= 2.0 * M_PI;
        while (hd_err < -M_PI) hd_err += 2.0 * M_PI;

        double force_mag = std::hypot(fx, fy);
        double linear  = std::min(force_mag, MAX_LINEAR);
        double angular = std::max(-MAX_ANGULAR, std::min(MAX_ANGULAR, 2.0 * hd_err));

        /* Slow down when heading error is large */
        if (std::fabs(hd_err) > 0.5)
            linear *= std::max(0.1, 1.0 - std::fabs(hd_err) / M_PI);

        /* ── Task 5: Publish plan ────────────────────────────────────────────── */
        RobotState plan{};
        plan.x        = state->x;
        plan.y        = state->y;
        plan.heading  = state->heading;
        plan.vx       = static_cast<float>(linear);
        plan.vy       = static_cast<float>(angular);
        plan.goal_x   = state->goal_x;
        plan.goal_y   = state->goal_y;
        plan.goal_dist= state->goal_dist;
        plan.step     = state->step;
        plan.sim_time = state->sim_time;
        pub.write_state(0, plan);
        ++frames;

        if (frames % 100 == 0)
            std::printf("[planner] step=%5llu  pos(%+.2f,%+.2f)  force(%.2f,%.2f)"
                        "  -> lin=%.2f ang=%+.2f\n",
                        (unsigned long long)state->step, x, y,
                        fx, fy, linear, angular);

        if (!sub_sensor.is_publisher_alive(1500.0)) {
            std::printf("[planner] /sb_sensor lost -- stopping\n");
            break;
        }
    }

    pub.close();
    std::printf("[planner] stopped (%llu frames)\n", (unsigned long long)frames);
    return 0;
}
