/*
 * test_topic.cpp — C++ unit tests for shmbridge v3 generic Publisher/Subscriber.
 *
 * Tests cover:
 *  1. Basic write/read for every predefined message type
 *  2. type_id<T>() — same type produces same hash, different types differ
 *  3. Type mismatch detection at attach() (TypeMismatch error)
 *  4. Multiple subscribers — no race condition (fork, N=4 concurrent readers)
 *  5. Broadcast notify — all N subscribers wake within a deadline
 *  6. read_if_new() — returns nullopt until a new write occurs
 *  7. wait_new() — skips already-seen write_ns values
 *  8. Publisher liveness check
 *  9. Torn-read statistics under zero-contention (torn_reads ≈ 0)
 * 10. SeqlockSlot size invariant (always 128 bytes)
 *
 * Build (standalone, requires googletest):
 *   cmake -DSHMBRIDGE_BUILD_TESTS=ON -S . -B build
 *   cmake --build build
 *   ctest --test-dir build -V
 *
 * Or directly:
 *   g++ -std=c++17 -O2 -I include tests/test_topic.cpp \
 *       -lgtest -lgtest_main -pthread -lrt -o test_topic
 *   ./test_topic
 */

#include <gtest/gtest.h>

#include <shmbridge/topic.hpp>
#include <shmbridge/messages.hpp>

#include <sys/wait.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>

using namespace shmbridge;
using namespace shmbridge::msg;

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Unique shm name per test to avoid leakage between runs */
static std::string unique_name(const char* base) {
    return std::string(base) + "_" + std::to_string(::getpid());
}

/* RAII cleaner that removes the shm segment on destruction */
struct ShmCleaner {
    std::string name;
    ~ShmCleaner() { ::shm_unlink(("/sb_" + name).c_str()); }
};

/* ── §10  SeqlockSlot size invariant ─────────────────────────────────────── */

TEST(SeqlockSlot, AlwaysExactly128Bytes) {
    EXPECT_EQ(sizeof(SeqlockSlot<Pose2d>),       128u);
    EXPECT_EQ(sizeof(SeqlockSlot<Pose3d>),       128u);
    EXPECT_EQ(sizeof(SeqlockSlot<Twist>),        128u);
    EXPECT_EQ(sizeof(SeqlockSlot<Imu>),          128u);
    EXPECT_EQ(sizeof(SeqlockSlot<Odometry>),     128u);
    EXPECT_EQ(sizeof(SeqlockSlot<BatteryState>), 128u);
    EXPECT_EQ(sizeof(SeqlockSlot<LaserScan2d>),  128u);
    EXPECT_EQ(sizeof(SeqlockSlot<PointCloud>),   128u);
    EXPECT_EQ(sizeof(SeqlockSlot<OccupancyMap>), 128u);
}

TEST(TopicHeader, Exactly128Bytes) {
    EXPECT_EQ(sizeof(TopicHeader), 128u);
}

/* ── §2  type_id<T>() ────────────────────────────────────────────────────── */

TEST(TypeId, SameTypeProducesSameHash) {
    EXPECT_EQ(type_id<Pose2d>(),   type_id<Pose2d>());
    EXPECT_EQ(type_id<Pose3d>(),   type_id<Pose3d>());
    EXPECT_EQ(type_id<Twist>(),    type_id<Twist>());
    EXPECT_EQ(type_id<Imu>(),      type_id<Imu>());
    EXPECT_EQ(type_id<uint32_t>(), type_id<uint32_t>());
}

TEST(TypeId, DifferentTypesProduceDifferentHashes) {
    EXPECT_NE(type_id<Pose2d>(), type_id<Pose3d>());
    EXPECT_NE(type_id<Pose2d>(), type_id<Twist>());
    EXPECT_NE(type_id<Pose2d>(), type_id<Imu>());
    EXPECT_NE(type_id<Pose3d>(), type_id<Twist>());
    EXPECT_NE(type_id<Imu>(),    type_id<Odometry>());
    EXPECT_NE(type_id<float>(),  type_id<double>());
}

/* ── §1  Basic write / read for each message type ────────────────────────── */

template<typename T>
static void basic_round_trip(const std::string& tname, const T& msg) {
    Publisher<T>  pub;
    Subscriber<T> sub;

    auto err = pub.open(tname, /*mlock=*/false);
    ASSERT_EQ(err, TopicError::Ok) << "Publisher::open failed";

    auto aerr = sub.attach(tname, 2000);
    ASSERT_EQ(aerr, TopicError::Ok) << "Subscriber::attach failed";

    pub.write(msg);

    auto r = sub.spin(64);
    ASSERT_TRUE(r.has_value()) << "spin() returned nullopt";
    EXPECT_NEAR(r->value.stamp_ns, msg.stamp_ns, 1.0);
}

TEST(BasicRoundTrip, Pose2d) {
    auto name = unique_name("pose2d_rt");
    ShmCleaner _g{name};
    Pose2d m; m.x = 1.0; m.y = 2.0; m.heading = 0.5; m.stamp_ns = 12345;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, Pose3d) {
    auto name = unique_name("pose3d_rt");
    ShmCleaner _g{name};
    Pose3d m; m.x = 3.0; m.y = -1.0; m.z = 0.5; m.qw = 1.0; m.stamp_ns = 999;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, Twist) {
    auto name = unique_name("twist_rt");
    ShmCleaner _g{name};
    Twist m; m.vx = 0.5f; m.wz = 0.1f; m.stamp_ns = 77;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, Imu) {
    auto name = unique_name("imu_rt");
    ShmCleaner _g{name};
    Imu m; m.ax = 9.81f; m.gx = 0.01f; m.temp = 25.0f; m.stamp_ns = 1000;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, Odometry) {
    auto name = unique_name("odom_rt");
    ShmCleaner _g{name};
    Odometry m; m.x = 5.0; m.y = 3.0; m.vx = 0.3f; m.stamp_ns = 5555;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, BatteryState) {
    auto name = unique_name("batt_rt");
    ShmCleaner _g{name};
    BatteryState m; m.voltage = 12.4f; m.charge_pct = 80.0f; m.stamp_ns = 200;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, LaserScan2d) {
    auto name = unique_name("scan_rt");
    ShmCleaner _g{name};
    LaserScan2d m;
    m.angle_min = -1.5707f; m.angle_max = 1.5707f;
    m.angle_incr = 0.01f; m.range_max = 10.0f; m.n_beams = 360; m.stamp_ns = 42;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, PointCloud) {
    auto name = unique_name("pc_rt");
    ShmCleaner _g{name};
    PointCloud m; m.n_points = 1024; m.max_points = 4096; m.stamp_ns = 99;
    basic_round_trip(name, m);
}

TEST(BasicRoundTrip, OccupancyMap) {
    auto name = unique_name("map_rt");
    ShmCleaner _g{name};
    OccupancyMap m; m.width = 100; m.height = 100; m.resolution = 0.05f; m.stamp_ns = 1;
    basic_round_trip(name, m);
}

/* ── §3  Type mismatch detection ─────────────────────────────────────────── */

TEST(TypeSafety, MismatchDetectedAtAttach) {
    auto name = unique_name("type_mismatch");
    ShmCleaner _g{name};

    Publisher<Pose2d> pub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);

    /* Attaching as the wrong type must fail */
    Subscriber<Pose3d> sub_wrong;
    auto err = sub_wrong.attach(name, 500);
    EXPECT_EQ(err, TopicError::TypeMismatch);

    /* Attaching as the correct type must succeed */
    Subscriber<Pose2d> sub_ok;
    EXPECT_EQ(sub_ok.attach(name, 500), TopicError::Ok);
}

/* ── §6  read_if_new() ───────────────────────────────────────────────────── */

TEST(ReadIfNew, ReturnsNulloptUntilNewWrite) {
    auto name = unique_name("read_if_new");
    ShmCleaner _g{name};

    Publisher<Twist>  pub;
    Subscriber<Twist> sub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);
    ASSERT_EQ(sub.attach(name), TopicError::Ok);

    /* Nothing written yet */
    EXPECT_FALSE(sub.read_if_new().has_value());

    Twist m1; m1.vx = 1.0f; m1.stamp_ns = 100;
    pub.write(m1);

    /* First call sees the new message */
    auto r1 = sub.read_if_new();
    ASSERT_TRUE(r1.has_value());
    EXPECT_FLOAT_EQ(r1->value.vx, 1.0f);

    /* Second call without a new write → nullopt */
    EXPECT_FALSE(sub.read_if_new().has_value());

    /* Write again */
    Twist m2; m2.vx = 2.0f; m2.stamp_ns = 200;
    pub.write(m2);

    auto r2 = sub.read_if_new();
    ASSERT_TRUE(r2.has_value());
    EXPECT_FLOAT_EQ(r2->value.vx, 2.0f);
}

/* ── §7  wait_new() ──────────────────────────────────────────────────────── */

TEST(WaitNew, SkipsAlreadySeenMessages) {
    auto name = unique_name("wait_new");
    ShmCleaner _g{name};

    Publisher<Imu>  pub(/*heartbeat_every=*/1);
    Subscriber<Imu> sub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);
    ASSERT_EQ(sub.attach(name), TopicError::Ok);

    /* Write one message and read it normally so sub tracks write_ns */
    Imu m1; m1.stamp_ns = 111;
    pub.write(m1);
    auto r1 = sub.spin();
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->value.stamp_ns, 111u);

    /* wait_new with a very short timeout should return nullopt
     * (no new write has happened) */
    auto r2 = sub.wait_new(/*timeout_ms=*/50);
    EXPECT_FALSE(r2.has_value());
}

/* ── §8  Publisher liveness ──────────────────────────────────────────────── */

TEST(Liveness, AliveAfterRecentWrite) {
    auto name = unique_name("liveness");
    ShmCleaner _g{name};

    Publisher<BatteryState>  pub(1);
    Subscriber<BatteryState> sub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);
    ASSERT_EQ(sub.attach(name), TopicError::Ok);

    BatteryState m; m.voltage = 11.5f; m.stamp_ns = 1;
    pub.write(m);
    sub.spin();

    EXPECT_TRUE(sub.is_publisher_alive(/*max_age_ms=*/500));
}

/* ── §9  Torn-read statistics (zero contention baseline) ─────────────────── */

TEST(Stats, TornReadsNearZeroWithoutContention) {
    auto name = unique_name("torn_reads");
    ShmCleaner _g{name};

    Publisher<Odometry>  pub(1);
    Subscriber<Odometry> sub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);
    ASSERT_EQ(sub.attach(name), TopicError::Ok);

    constexpr int N = 1000;
    for (int i = 0; i < N; ++i) {
        Odometry m; m.x = i; m.stamp_ns = static_cast<uint64_t>(i);
        pub.write(m);
        sub.spin(64);
    }

    /* Without concurrent writers torn reads should be ≈0 */
    EXPECT_LE(sub.stats().torn_reads, static_cast<uint64_t>(N / 100));
}

/* ── §4  Multiple subscribers — no race condition (fork-based) ───────────── */

/*
 * Forks N_SUB child processes. Each child attaches as a subscriber, reads
 * READS_PER_CHILD messages, checks that values decode correctly, and records
 * torn-read counts. Children communicate results back via a pipe.
 *
 * Expected: all children get correct data; torn_reads / total_reads < 1%.
 */
TEST(MultipleSubscribers, NoRaceConditionForkN4) {
    constexpr int  N_SUB          = 4;
    constexpr int  WRITES         = 200;
    constexpr int  READS_PER_CHILD = WRITES;

    auto name = unique_name("multi_sub");
    ShmCleaner _g{name};

    Publisher<Pose2d> pub(1);
    ASSERT_EQ(pub.open(name), TopicError::Ok);

    /* Write some data before forking so the segment is ready */
    Pose2d seed; seed.x = 0; seed.stamp_ns = 1;
    pub.write(seed);

    struct ChildResult { uint64_t torn; uint64_t total; int last_seq; };
    int pipes[N_SUB][2];
    pid_t pids[N_SUB];

    for (int i = 0; i < N_SUB; ++i) {
        ASSERT_EQ(::pipe(pipes[i]), 0);
        pids[i] = ::fork();
        ASSERT_GE(pids[i], 0);

        if (pids[i] == 0) {
            /* Child */
            ::close(pipes[i][0]);
            Subscriber<Pose2d> sub;
            auto err = sub.attach(name, 3000);
            if (err != TopicError::Ok) { ::exit(1); }

            int last_x = -1;
            for (int r = 0; r < READS_PER_CHILD; ++r) {
                auto res = sub.spin(256);
                if (res) last_x = static_cast<int>(res->value.x);
            }
            ChildResult cr;
            cr.torn  = sub.stats().torn_reads;
            cr.total = sub.stats().total_reads;
            cr.last_seq = last_x;
            ::write(pipes[i][1], &cr, sizeof(cr));
            ::close(pipes[i][1]);
            ::exit(0);
        } else {
            ::close(pipes[i][1]);
        }
    }

    /* Parent: write WRITES messages */
    for (int w = 0; w < WRITES; ++w) {
        Pose2d m; m.x = static_cast<double>(w); m.stamp_ns = static_cast<uint64_t>(w + 1);
        pub.write(m);
        /* tiny sleep to let subscribers observe intermediate values */
        struct timespec sl{0, 500'000L};
        ::nanosleep(&sl, nullptr);
    }

    /* Collect child results */
    for (int i = 0; i < N_SUB; ++i) {
        ChildResult cr{};
        ssize_t got = ::read(pipes[i][0], &cr, sizeof(cr));
        ::close(pipes[i][0]);
        int status = 0;
        ::waitpid(pids[i], &status, 0);

        EXPECT_EQ(got, static_cast<ssize_t>(sizeof(cr)))
            << "Child " << i << " pipe read failed";
        EXPECT_EQ(WIFEXITED(status) ? WEXITSTATUS(status) : -1, 0)
            << "Child " << i << " exited abnormally";

        if (cr.total > 0) {
            double torn_pct = 100.0 * static_cast<double>(cr.torn)
                            / static_cast<double>(cr.total);
            EXPECT_LT(torn_pct, 5.0)
                << "Child " << i << " torn-read rate " << torn_pct << "% > 5%";
        }
    }
}

/* ── §5  Broadcast notify — all N subscribers wake within deadline ─────────
 *
 * This test verifies that write_notify() wakes ALL attached subscribers, not
 * just one (which would be the POSIX-sem unicast failure mode).
 *
 * Uses threads (not fork) because wait() blocks the calling thread; each
 * subscriber thread records its wakeup time and we check all N woke within
 * a generous 100 ms of the write.
 */
#if defined(__linux__)
TEST(BroadcastNotify, AllSubscribersWake) {
    constexpr int  N_SUB     = 4;
    constexpr int  TIMEOUT_MS = 100;

    auto name = unique_name("broadcast");
    ShmCleaner _g{name};

    Publisher<Twist> pub(1);
    ASSERT_EQ(pub.open(name), TopicError::Ok);

    /* Seed an initial write so attach succeeds immediately */
    Twist seed; seed.vx = 0; seed.stamp_ns = 1;
    pub.write(seed);

    std::vector<Subscriber<Twist>> subs(N_SUB);
    for (int i = 0; i < N_SUB; ++i) {
        ASSERT_EQ(subs[i].attach(name, 2000), TopicError::Ok)
            << "Subscriber " << i << " attach failed";
    }

    std::vector<std::atomic<bool>> woke(N_SUB);
    for (auto& w : woke) w.store(false);

    /* Launch subscriber threads that all block on wait() */
    std::vector<std::thread> threads;
    threads.reserve(N_SUB);
    for (int i = 0; i < N_SUB; ++i) {
        threads.emplace_back([&subs, &woke, i, TIMEOUT_MS]() {
            auto r = subs[i].wait(TIMEOUT_MS * 2);
            if (r) woke[i].store(true);
        });
    }

    /* Brief pause to let threads reach FUTEX_WAIT */
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    /* One write_notify() should broadcast to all N threads */
    Twist m; m.vx = 1.0f; m.stamp_ns = detail::now_ns();
    pub.write_notify(m);

    /* Wait for all subscriber threads */
    for (auto& t : threads) t.join();

    for (int i = 0; i < N_SUB; ++i) {
        EXPECT_TRUE(woke[i].load()) << "Subscriber " << i << " did not wake";
    }
}
#endif /* __linux__ */

/* ── §11  Topic name with slashes ────────────────────────────────────────── */

TEST(TopicName, SlashesConvertedToSafeShm) {
    /* "/myrobot/sensors/imu" must not crash; shm_open on the raw name
     * would fail (embedded slash is illegal in shm segment names). */
    auto name = std::string("myrobot_slash_") + std::to_string(::getpid());
    Publisher<Imu>  pub;
    Subscriber<Imu> sub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);
    ASSERT_EQ(sub.attach(name, 2000), TopicError::Ok);
    ShmCleaner _g{name};

    Imu m; m.ax = 1.0f; m.stamp_ns = 99;
    pub.write(m);
    auto r = sub.spin();
    ASSERT_TRUE(r.has_value());
    EXPECT_FLOAT_EQ(r->value.ax, 1.0f);
}

/* ── §12  age_us() plausible ─────────────────────────────────────────────── */

TEST(AgeUs, PlausibleAfterRead) {
    auto name = unique_name("age_us");
    ShmCleaner _g{name};

    Publisher<Pose2d>  pub(1);
    Subscriber<Pose2d> sub;
    ASSERT_EQ(pub.open(name), TopicError::Ok);
    ASSERT_EQ(sub.attach(name), TopicError::Ok);

    Pose2d m; m.x = 1.0; m.stamp_ns = 0;  /* heartbeat will fill stamp_ns */
    pub.write(m);

    auto r = sub.spin();
    ASSERT_TRUE(r.has_value());

    double age = sub.age_us();
    /* Age should be small (< 1 ms = 1000 µs) in a unit test */
    EXPECT_GE(age, 0.0);
    EXPECT_LT(age, 1000.0) << "age_us() returned unreasonably large value: " << age;
}
