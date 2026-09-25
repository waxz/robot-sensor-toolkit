/*
 * shmbridge/messages.hpp — predefined message types for shmbridge v3.
 *
 * Every struct is trivially copyable, 8-byte aligned, and carries a
 * stamp_ns field (CLOCK_MONOTONIC ns at write time) so usable-latency
 * measurement is always available without relying on write_ns in the slot.
 *
 * Payload sizes are deliberately kept ≤ 104 bytes so the full seqlock slot
 * (8 + payload + 8 + 8 = ≤ 128 bytes) fits in two cache lines.
 */

#pragma once
#include <cstdint>

namespace shmbridge::msg {

/* ── 2-D pose (world-frame) ──────────────────────────────────────────────── */
struct Pose2d {
    double   x       = 0;  /* m */
    double   y       = 0;  /* m */
    double   heading = 0;  /* rad */
    uint64_t stamp_ns = 0;
};
static_assert(sizeof(Pose2d) == 32);

/* ── Full SE(3) pose ─────────────────────────────────────────────────────── */
struct Pose3d {
    double   x = 0, y = 0, z = 0;            /* position (m) */
    double   qx = 0, qy = 0, qz = 0, qw = 1; /* unit quaternion */
    uint64_t stamp_ns = 0;
};
static_assert(sizeof(Pose3d) == 64);

/* ── 3-D velocity twist ──────────────────────────────────────────────────── */
struct Twist {
    float    vx = 0, vy = 0, vz = 0;  /* linear  (m/s)   */
    float    wx = 0, wy = 0, wz = 0;  /* angular (rad/s) */
    uint64_t stamp_ns = 0;
};
static_assert(sizeof(Twist) == 32);

/* ── IMU ─────────────────────────────────────────────────────────────────── */
struct Imu {
    float    ax = 0, ay = 0, az = 0;  /* linear accel (m/s²)    */
    float    gx = 0, gy = 0, gz = 0;  /* angular velocity (rad/s)*/
    float    mx = 0, my = 0, mz = 0;  /* magnetometer (µT)      */
    float    temp = 0;                  /* temperature (°C)        */
    uint32_t _pad = 0;
    uint64_t stamp_ns = 0;
};
static_assert(sizeof(Imu) == 56);

/* ── 2-D odometry ────────────────────────────────────────────────────────── */
struct Odometry {
    double   x = 0, y = 0, heading = 0;  /* pose (m, m, rad)   */
    float    vx = 0, vy = 0, omega = 0;  /* velocity           */
    uint32_t _pad = 0;
    uint64_t stamp_ns = 0;
};
static_assert(sizeof(Odometry) == 48);

/* ── Battery state ───────────────────────────────────────────────────────── */
struct BatteryState {
    float    voltage    = 0;   /* V  */
    float    current    = 0;   /* A  */
    float    charge_pct = 0;   /* 0–100 */
    uint32_t status     = 0;   /* bitmask: bit0=charging, bit1=fault, bit2=low */
    uint64_t stamp_ns   = 0;
};
static_assert(sizeof(BatteryState) == 24);

/*
 * Large-message headers (payload > one seqlock slot) live in topic.hpp as
 * BulkPublisher / BulkSubscriber specialisations.  Registered here as tags:
 *
 *   struct LaserScan2d   — see topic.hpp (BulkTopic<LaserScan2d, SCAN_BULK_BYTES>)
 *   struct PointCloud    — see topic.hpp (BulkTopic<PointCloud, PC_BULK_BYTES>)
 *   struct OccupancyMap  — see topic.hpp (BulkTopic<OccupancyMap, MAP_BULK_BYTES>)
 *
 * For now these are declared as header-only structs so generic code can form
 * Publisher<LaserScan2d> without the bulk channel overhead.
 */

struct LaserScan2d {
    float    angle_min = 0, angle_max = 0, angle_incr = 0;
    float    range_min = 0, range_max = 0;
    uint32_t n_beams   = 0;
    uint64_t stamp_ns  = 0;
    uint64_t seq       = 0;
    /* payload: float ranges[n_beams] stored separately via BulkPublisher */
};
static_assert(sizeof(LaserScan2d) == 40);

struct PointXYZI { float x, y, z, intensity; };  /* 16 B */

struct PointCloud {
    uint32_t n_points   = 0;
    uint32_t max_points = 0;
    uint64_t stamp_ns   = 0;
    uint64_t seq        = 0;
    /* payload: PointXYZI[n_points] stored separately via BulkPublisher */
};
static_assert(sizeof(PointCloud) == 24);

struct OccupancyMap {
    uint32_t width = 0, height = 0;
    float    origin_x = 0, origin_y = 0;
    float    resolution = 0;           /* m/cell */
    uint32_t _pad = 0;
    uint64_t stamp_ns = 0;
    uint64_t seq      = 0;
    /* payload: int8_t[width*height] stored separately via BulkPublisher */
};
static_assert(sizeof(OccupancyMap) == 40);

} /* namespace shmbridge::msg */
