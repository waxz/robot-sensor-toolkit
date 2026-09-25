/*
 * control_node.cpp -- velocity limiter + rate limiter (ONE-BY-ONE read).
 *
 * Subscriber: /sb_plan    -- planned velocity (vx=linear, vy=angular) [ONE-BY-ONE]
 * Publisher:  /sb_control -- final velocity command for sensor_node
 *
 * Tasks (inner loop, driven by each new plan step):
 *   1. Read EVERY plan step (ONE-BY-ONE via last_step tracking)
 *   2. Velocity clamping: clip to MAX_LINEAR / MAX_ANGULAR
 *   3. Acceleration limiting: rate-limit delta per DT
 *   4. Publish final command on /sb_control
 *
 * Run order: any -- blocks until /sb_plan is available.
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

static constexpr float DT           = 0.01f;
static constexpr float MAX_LINEAR   = 0.8f;   /* m/s    */
static constexpr float MAX_ANGULAR  = 2.0f;   /* rad/s  */
static constexpr float MAX_ACC_LIN  = 2.0f;   /* m/s^2  */
static constexpr float MAX_ACC_ANG  = 4.0f;   /* rad/s^2 */

static inline float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

int main() {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    /* Subscriber: receive planned velocity from planner_node */
    ShmSubscriber sub_plan("/sb_plan", /*n_robots=*/1);

    std::printf("[control] waiting for /sb_plan...\n");
    while (g_running) {
        try { sub_plan.attach(1000.0); break; }
        catch (const std::exception&) {}
    }
    if (!g_running) return 0;
    std::printf("[control] /sb_plan attached\n");

    /* Publisher: broadcast final velocity so sensor_node can read it */
    ShmPublisher pub("/sb_control", /*n_robots=*/1, /*n_consumers=*/1,
                     /*heartbeat_every=*/1);
    pub.open();
    std::printf("[control] /sb_control open\n");

    float    prev_linear  = 0.0f;
    float    prev_angular = 0.0f;
    uint64_t last_step    = UINT64_MAX;
    uint64_t frames       = 0;

    while (g_running) {
        /* ── Task 1: ONE-BY-ONE read -- process every plan step ─────────────── */
        auto plan = sub_plan.read_state_spin(0);
        if (!plan) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        if (plan->step == last_step) {
            /* Already processed this step; spin at ~10 kHz to catch the next */
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        last_step = plan->step;

        float desired_linear  = plan->vx;
        float desired_angular = plan->vy;

        /* ── Task 2: Velocity clamping ───────────────────────────────────────── */
        desired_linear  = clamp(desired_linear,  -MAX_LINEAR,  MAX_LINEAR);
        desired_angular = clamp(desired_angular, -MAX_ANGULAR, MAX_ANGULAR);

        /* ── Task 3: Acceleration limiting ──────────────────────────────────── */
        float max_delta_lin = MAX_ACC_LIN * DT;
        float max_delta_ang = MAX_ACC_ANG * DT;

        float delta_lin = desired_linear  - prev_linear;
        float delta_ang = desired_angular - prev_angular;

        if (delta_lin >  max_delta_lin) delta_lin =  max_delta_lin;
        if (delta_lin < -max_delta_lin) delta_lin = -max_delta_lin;
        if (delta_ang >  max_delta_ang) delta_ang =  max_delta_ang;
        if (delta_ang < -max_delta_ang) delta_ang = -max_delta_ang;

        float final_linear  = prev_linear  + delta_lin;
        float final_angular = prev_angular + delta_ang;

        prev_linear  = final_linear;
        prev_angular = final_angular;

        /* ── Task 4: Publish final command ───────────────────────────────────── */
        RobotState cmd{};
        cmd.x        = plan->x;
        cmd.y        = plan->y;
        cmd.heading  = plan->heading;
        cmd.vx       = final_linear;
        cmd.vy       = final_angular;
        cmd.goal_x   = plan->goal_x;
        cmd.goal_y   = plan->goal_y;
        cmd.goal_dist= plan->goal_dist;
        cmd.step     = plan->step;
        cmd.sim_time = plan->sim_time;
        pub.write_state(0, cmd);
        ++frames;

        if (frames % 100 == 0)
            std::printf("[control] step=%5llu  planned(%.2f,%+.2f)"
                        "  -> final(%.2f,%+.2f)\n",
                        (unsigned long long)plan->step,
                        desired_linear, desired_angular,
                        final_linear, final_angular);

        if (!sub_plan.is_publisher_alive(1500.0)) {
            std::printf("[control] /sb_plan lost -- stopping\n");
            break;
        }
    }

    pub.close();
    std::printf("[control] stopped (%llu frames)\n", (unsigned long long)frames);
    return 0;
}
