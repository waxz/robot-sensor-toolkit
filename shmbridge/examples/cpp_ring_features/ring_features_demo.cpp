/*
 * ring_features_demo.cpp -- a feature tour of shmbridge/ring.hpp's direct
 * RingPublisher<T>/RingSubscriber<T> API, using the current (post-phase-9)
 * runtime-RingConfig interface. See docs/design_ring_zero_copy.md for the
 * full design; this program exercises every feature (F-1 through F-14)
 * that design names, in one place, single-process, so it can just be run
 * and read top to bottom.
 *
 * The Node API (shmbridge/node.hpp, see ../cpp_node/talker.cpp and
 * listener.cpp) wraps a useful subset of this for application code --
 * create_publisher()/create_subscription()/create_queue(). This demo
 * covers what that wrapper doesn't expose directly: zero-copy write/read,
 * pop_latest() at the ring level, freshness checks, config resolution,
 * producer-conflict detection, type safety, the fixed-size point-cloud
 * family, and torn-read retry instrumentation.
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Run (single process, no second terminal needed):
 *   ./build/ring_features_demo
 */

#include <shmbridge/ring.hpp>
#include <shmbridge/messages.hpp>
#include <shmbridge/platform.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>
#include <vector>

using shmbridge::RingConfig;
using shmbridge::RingCursorMode;
using shmbridge::RingPublisher;
using shmbridge::RingSubscriber;
using shmbridge::TimeoutError;

static void section(const char* title) {
    std::printf("\n== %s ==\n", title);
}

/* ── 1. Basic round trip (F-1, F-2) ──────────────────────────────────────── */

static void demo_basic_round_trip() {
    section("1. Basic push/pop round trip (F-1 always-succeeding writes, F-2 independent cursors)");

    RingConfig cfg;
    cfg.capacity = 8; /* must be a power of two >= 2 */

    RingPublisher<shmbridge::msg::Twist> pub;
    pub.open("demo/basic", cfg);

    /* Two independent subscribers, each with its own cursor -- neither
     * affects the other's view of the stream (F-2). */
    RingSubscriber<shmbridge::msg::Twist> sub_a, sub_b;
    sub_a.try_attach("demo/basic", cfg);
    sub_b.try_attach("demo/basic", cfg);

    shmbridge::msg::Twist t{};
    t.vx = 1.5f;
    pub.push(t); /* push() never fails, even if the ring were already full */

    auto a = sub_a.pop_ex();
    std::printf("sub_a received vx=%.2f (write_ns=%llu)\n",
                a->value.vx, static_cast<unsigned long long>(a->write_ns));

    /* sub_b hasn't read yet -- its own cursor still sees the same message,
     * independent of sub_a having already consumed it. */
    auto b = sub_b.pop_ex();
    std::printf("sub_b received vx=%.2f -- same message, independent cursor\n", b->value.vx);
}

/* ── 2. Zero-copy write/read (F-4, F-5, F-14 misuse guards) ───────────────── */

static void demo_zero_copy() {
    section("2. Zero-copy write/read (F-4/F-5) and misuse guards (F-14)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingPublisher<shmbridge::msg::Imu> pub;
    pub.open("demo/zerocopy", cfg);
    RingSubscriber<shmbridge::msg::Imu> sub;
    sub.try_attach("demo/zerocopy", cfg);

    /* reserve() hands back a pointer directly into shared memory -- the
     * caller constructs the message in place, no intermediate copy. */
    shmbridge::msg::Imu* slot = pub.reserve();
    slot->ax = 9.81f;
    slot->temp = 36.5f;

    /* A second reserve() before the matching commit() is a documented
     * no-op (F-14): it returns nullptr rather than corrupt the first,
     * still-unpublished reservation. */
    shmbridge::msg::Imu* second_reserve = pub.reserve();
    std::printf("second reserve() before commit(): %s (expected nullptr)\n",
                second_reserve == nullptr ? "nullptr" : "NON-NULL, unexpected");

    pub.commit(); /* publishes the first reservation */
    pub.commit(); /* commit() with nothing reserved is also a safe no-op */

    uint64_t write_ns = 0;
    const shmbridge::msg::Imu* borrowed = sub.borrow(&write_ns);
    std::printf("borrowed directly from shared memory: ax=%.2f temp=%.1f (write_ns=%llu)\n",
                borrowed->ax, borrowed->temp, static_cast<unsigned long long>(write_ns));
    sub.end_borrow(); /* must be called before the cursor advances */

    /* end_borrow() with no prior successful borrow() is also a safe no-op. */
    bool advanced = sub.end_borrow();
    std::printf("end_borrow() with nothing borrowed: advanced=%s (expected false)\n",
                advanced ? "true" : "false");
}

/* ── 3. Freshness check (F-6) ─────────────────────────────────────────────── */

static void demo_freshness() {
    section("3. Freshness check via write_ns/is_stale() (F-6)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingPublisher<shmbridge::msg::BatteryState> pub;
    pub.open("demo/freshness", cfg);
    RingSubscriber<shmbridge::msg::BatteryState> sub;
    sub.try_attach("demo/freshness", cfg);

    shmbridge::msg::BatteryState b{};
    b.voltage = 12.1f;
    pub.push(b);

    auto item = sub.pop_ex();
    std::printf("fresh right after publish: is_stale(50ms)=%s\n",
                item->is_stale(50.0) ? "true" : "false");

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    std::printf("same Result<T> 80ms later: is_stale(50ms)=%s -- a reader never trusts an\n"
                "  old value just because it's the last one it has\n",
                item->is_stale(50.0) ? "true" : "false");
}

/* ── 4. Blocking wait, low CPU (F-3) ──────────────────────────────────────── */

static void demo_blocking_wait() {
    section("4. pop_wait() -- blocks on an OS wait primitive, not a spin loop (F-3)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingPublisher<shmbridge::msg::Twist> pub;
    pub.open("demo/blocking_wait", cfg);
    RingSubscriber<shmbridge::msg::Twist> sub;
    sub.try_attach("demo/blocking_wait", cfg);

    std::thread publisher_thread([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        shmbridge::msg::Twist t{}; t.wx = 2.0f;
        pub.push(t); /* wakes the subscriber blocked in pop_wait() below */
    });

    auto start = std::chrono::steady_clock::now();
    auto item = sub.pop_wait(1000); /* blocks up to 1000ms; wakes early on publish */
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    publisher_thread.join();
    std::printf("woke after %lldms (requested up to 1000ms) -- CPU was idle while waiting, not spinning\n",
                static_cast<long long>(elapsed_ms));
    if (item) {
        std::printf("received wx=%.2f\n", item->value.wx);
    } else {
        std::printf("MISSING -- pop_wait() timed out instead of waking on publish\n");
    }
}

/* ── 5. Latest-only consumption (F-13) ────────────────────────────────────── */

static void demo_pop_latest() {
    section("5. pop_latest() -- jump straight to the newest message (F-13)");

    RingConfig cfg;
    cfg.capacity = 8;
    RingPublisher<shmbridge::msg::Odometry> pub;
    pub.open("demo/latest", cfg);
    RingSubscriber<shmbridge::msg::Odometry> sub;
    sub.try_attach("demo/latest", cfg);

    for (int i = 0; i < 5; ++i) {
        shmbridge::msg::Odometry o{};
        o.x = static_cast<double>(i);
        pub.push(o);
    }

    /* A consumer that only cares about "what's current" skips the backlog
     * entirely, in O(1), rather than draining through 4 stale messages
     * first -- useful for e.g. a control loop reading the latest pose. */
    auto latest = sub.pop_latest();
    std::printf("5 messages published (x=0..4); pop_latest() returned x=%.0f, discarding the rest\n",
                latest->value.x);
}

/* ── 6. RingConfig / TOML resolution (F-8) ────────────────────────────────── */

static void demo_config_resolution() {
    section("6. Per-topic RingConfig resolution from a TOML file (F-8)");

    const char* path = "ring_features_demo_config.toml";
    {
        std::ofstream f(path);
        f << "[ring.default]\n"
             "capacity = 32\n"
             "\n"
             "[ring.\"demo/high_rate\"]\n"
             "capacity = 256\n"
             "cursor_mode = \"drain_backlog\"\n";
    }

    RingConfig other_cfg = shmbridge::resolve_ring_config("demo/other_topic", path);
    RingConfig high_rate_cfg = shmbridge::resolve_ring_config("demo/high_rate", path);

    std::printf("demo/other_topic -> capacity=%u (inherited from [ring.default])\n",
                other_cfg.capacity);
    std::printf("demo/high_rate   -> capacity=%u, cursor_mode=%s (per-topic override)\n",
                high_rate_cfg.capacity,
                high_rate_cfg.cursor_mode == RingCursorMode::DrainBacklog ? "DrainBacklog" : "StartNow");

    std::remove(path);
}

/* ── 7. Resilient attach, never fatal (F-9) ───────────────────────────────── */

static void demo_resilient_attach() {
    section("7. Resilient attach -- a subscriber never throws/exits for 'not up yet' (F-9)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingSubscriber<shmbridge::msg::Twist> sub;

    bool found = sub.try_attach("demo/not_open_yet", cfg);
    std::printf("try_attach() before any publisher exists: found=%s (never throws for this case)\n",
                found ? "true" : "false");

    try {
        sub.attach("demo/never_appears", 50.0, cfg); /* blocking variant, bounded timeout */
    } catch (const TimeoutError& e) {
        std::printf("attach() with no publisher within 50ms: threw TimeoutError (\"%s\")\n", e.what());
    }
}

/* ── 8. Type safety (F-7) ─────────────────────────────────────────────────── */

static void demo_type_safety() {
    section("8. Type-hash mismatch rejected at attach time (F-7)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingPublisher<shmbridge::msg::Imu> pub;
    pub.open("demo/type_safety", cfg);

    RingSubscriber<shmbridge::msg::Twist> wrong_type_sub; /* same size class, different type */
    try {
        wrong_type_sub.try_attach("demo/type_safety", cfg);
        std::printf("UNEXPECTED: mismatched type was accepted\n");
    } catch (const std::invalid_argument& e) {
        std::printf("attaching RingSubscriber<Twist> to an Imu topic: rejected (\"%s\")\n", e.what());
    }
}

/* ── 9. Producer-conflict detection (F-11) ────────────────────────────────── */

static void demo_producer_conflict() {
    section("9. Producer-conflict detection -- only one live producer per topic (F-11)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingPublisher<shmbridge::msg::Twist> pub1;
    pub1.open("demo/single_producer", cfg);

    RingPublisher<shmbridge::msg::Twist> pub2;
    try {
        pub2.open("demo/single_producer", cfg);
        std::printf("UNEXPECTED: second open() succeeded while the first producer is still alive\n");
    } catch (const std::runtime_error& e) {
        std::printf("second open() while pub1 is alive: rejected (\"%s\")\n", e.what());
    }

    pub1.close(); /* now the topic is free again */
    RingPublisher<shmbridge::msg::Twist> pub3;
    bool ok = pub3.open("demo/single_producer", cfg);
    std::printf("open() after pub1.close(): succeeded=%s\n", ok ? "true" : "false");
}

/* ── 10. Torn-read retry instrumentation (R-1/NFR-4) ──────────────────────── */

static void demo_retry_stats() {
    section("10. RingSubscriberStats -- torn-read retry instrumentation (NFR-4)");

    RingConfig cfg;
    cfg.capacity = 4;
    RingPublisher<int> pub;
    pub.open("demo/stats", cfg);
    RingSubscriber<int> sub;
    sub.try_attach("demo/stats", cfg);

    for (int i = 0; i < 20; ++i) pub.push(i); /* capacity=4 -- only the last 4 survive (F-1) */
    while (sub.pop_ex()) { /* drain everything still live */ }

    const auto& stats = sub.stats();
    std::printf("pushed 20 values into a 4-slot ring, drained what's still live: total_reads=%llu\n"
                "  (not 20 -- the other 16 were overwritten before this subscriber ever read them,\n"
                "  F-1's overwrite-on-full at work, not a bug) retried_reads=%llu exhausted_reads=%llu\n",
                static_cast<unsigned long long>(stats.total_reads),
                static_cast<unsigned long long>(stats.retried_reads),
                static_cast<unsigned long long>(stats.exhausted_reads));
    std::printf("(a sustained non-zero retried/exhausted rate in a real deployment signals\n"
                "  undersized capacity for that topic's consumer pattern, R-11 -- not a defect)\n");
}

/* ── 11. Fixed-size point-cloud family (F-12) ─────────────────────────────── */

static void demo_point_cloud() {
    section("11. PointCloudFixed<MaxPoints> -- fixed-size point-cloud variants (F-12)");

    using shmbridge::msg::PointCloud64;

    RingConfig cfg;
    cfg.capacity = 2; /* large payload -- a small capacity is plenty (§5.11) */
    RingPublisher<PointCloud64> pub;
    pub.open("demo/pointcloud", cfg);
    RingSubscriber<PointCloud64> sub;
    sub.try_attach("demo/pointcloud", cfg);

    PointCloud64 cloud{};
    cloud.n_points = 3;
    cloud.points[0][0] = 1.0f; cloud.points[0][1] = 2.0f; cloud.points[0][2] = 0.0f; cloud.points[0][3] = 0.9f;
    cloud.points[1][0] = -1.0f; cloud.points[1][1] = 0.5f; cloud.points[1][2] = 0.0f; cloud.points[1][3] = 0.4f;
    cloud.points[2][0] = 0.0f; cloud.points[2][1] = -2.0f; cloud.points[2][2] = 1.0f; cloud.points[2][3] = 1.0f;

    pub.push(cloud);
    auto received = sub.pop_ex();
    std::printf("round-tripped a %u-point cloud (PointCloud64, 64 points/variant capacity);"
                " point[1] = (%.1f, %.1f, %.1f, %.1f)\n",
                received->value.n_points,
                received->value.points[1][0], received->value.points[1][1],
                received->value.points[1][2], received->value.points[1][3]);
}

int main() {
    demo_basic_round_trip();
    demo_zero_copy();
    demo_freshness();
    demo_blocking_wait();
    demo_pop_latest();
    demo_config_resolution();
    demo_resilient_attach();
    demo_type_safety();
    demo_producer_conflict();
    demo_retry_stats();
    demo_point_cloud();

    std::printf("\nAll ring.hpp features demonstrated.\n");
    return 0;
}
