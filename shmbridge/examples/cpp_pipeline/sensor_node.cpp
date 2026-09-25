/*
 * sensor_node.cpp -- simulated robot with unicycle kinematics.
 *
 * Publisher:  /sb_sensor  -- robot state (x, y, heading, vx, vy, goal info)
 * Subscriber: /sb_control -- final velocity cmd from control_node  [LATEST-ONLY]
 *
 * Tasks (100 Hz loop):
 *   1. Non-blocking connect to /sb_control; run on zero cmd until attached
 *   2. Read latest velocity command (vx=linear, vy=angular)
 *   3. Integrate unicycle kinematics; detect collisions with known obstacle
 *   4. Cycle through goal list when each goal is reached
 *   5. Publish updated robot state
 *
 * Run order: any -- sensor publishes immediately; controller attaches later.
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

static constexpr double DT          = 0.01;   /* 100 Hz */
static constexpr double GOAL_RADIUS = 0.20;   /* metres */
static constexpr double OBS_X       = 1.5;
static constexpr double OBS_Y       = 1.0;
static constexpr double OBS_RADIUS  = 0.35;

static const double GOALS[][2] = {{3.0, 2.0}, {-2.0, 1.5}, {1.0, -2.5}};
static constexpr int N_GOALS   = 3;

int main() {
    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    /* Publisher: broadcast robot state so planner can read it */
    ShmPublisher pub("/sb_sensor", /*n_robots=*/1, /*n_consumers=*/1,
                     /*heartbeat_every=*/1);
    pub.open();
    std::printf("[sensor] /sb_sensor open\n");

    /* Subscriber: receive final velocity commands from control_node */
    ShmSubscriber sub_ctrl("/sb_control", /*n_robots=*/1);
    bool ctrl_attached = false;
    unsigned ctrl_retry = 0;

    /* Robot state */
    double x = 0.0, y = 0.0, heading = 0.0;
    double lin_cmd = 0.0, ang_cmd = 0.0;
    int goal_idx = 0;
    uint64_t step = 0;

    while (g_running) {
        /* ── Task 1: Non-blocking connect to /sb_control every ~500 ms ──────── */
        if (!ctrl_attached && (++ctrl_retry % 50 == 0)) {
            try {
                sub_ctrl.attach(100.0);   /* single non-blocking attempt */
                ctrl_attached = true;
                std::printf("[sensor] /sb_control attached\n");
            } catch (const std::exception&) {}
        }

        /* ── Task 2: Read latest velocity command (LATEST-ONLY) ─────────────── */
        if (ctrl_attached) {
            auto ctrl = sub_ctrl.read_state_spin(0);
            if (ctrl) {
                lin_cmd = ctrl->vx;
                ang_cmd = ctrl->vy;
            }
            if (!sub_ctrl.is_publisher_alive(1500.0)) {
                std::printf("[sensor] /sb_control lost -- pausing robot\n");
                sub_ctrl.detach();
                ctrl_attached = false;
                lin_cmd = ang_cmd = 0.0;
                ctrl_retry = 0;
            }
        }

        /* ── Task 3: Integrate unicycle kinematics ───────────────────────────── */
        x       += lin_cmd * std::cos(heading) * DT;
        y       += lin_cmd * std::sin(heading) * DT;
        heading += ang_cmd * DT;
        while (heading >  M_PI) heading -= 2.0 * M_PI;
        while (heading < -M_PI) heading += 2.0 * M_PI;

        double obs_dist = std::hypot(OBS_X - x, OBS_Y - y);
        bool   collision = (obs_dist < OBS_RADIUS);

        /* ── Task 4: Cycle goal when reached ────────────────────────────────── */
        double gx = GOALS[goal_idx][0], gy = GOALS[goal_idx][1];
        double dist = std::hypot(gx - x, gy - y);
        bool reached = (dist < GOAL_RADIUS);
        if (reached) {
            goal_idx = (goal_idx + 1) % N_GOALS;
            gx = GOALS[goal_idx][0]; gy = GOALS[goal_idx][1];
            dist = std::hypot(gx - x, gy - y);
            std::printf("[sensor] goal reached! next: (%.1f, %.1f)\n", gx, gy);
        }

        /* ── Task 5: Publish state ───────────────────────────────────────────── */
        RobotState s;
        s.x         = x;
        s.y         = y;
        s.heading   = heading;
        s.vx        = static_cast<float>(lin_cmd);
        s.vy        = static_cast<float>(ang_cmd);
        s.goal_x    = static_cast<float>(gx);
        s.goal_y    = static_cast<float>(gy);
        s.goal_dist = static_cast<float>(dist);
        s.reached   = reached;
        s.collision = collision;
        s.step      = step;
        s.sim_time  = step * DT;
        pub.write_state(0, s);

        if (step % 100 == 0)
            std::printf("[sensor] step=%5llu  pos(%+.2f,%+.2f)  hdg=%+.2f"
                        "  dist=%.2f  cmd(%.2f,%.2f)%s\n",
                        (unsigned long long)step, x, y, heading, dist,
                        lin_cmd, ang_cmd,
                        collision ? "  [COLLISION]" : "");

        ++step;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    pub.close();
    std::printf("[sensor] stopped after %llu steps\n", (unsigned long long)step);
    return 0;
}
