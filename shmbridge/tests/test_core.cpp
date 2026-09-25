/*
 * test_core.cpp — unit tests for shmbridge core.hpp ShmPublisher/ShmSubscriber.
 *
 * Tests cover:
 *  1. Publisher open / close / is_open
 *  2. write_state seqlock round-trip (ShmPublisher → ShmSubscriber)
 *  3. read_cmd / write_cmd round-trip
 *  4. read_best_cmd picks highest-seq consumer
 *  5. Liveness: is_writer_alive / is_controller_alive
 *  6. ShmSubscriber::attach() retries and succeeds when publisher starts late
 *  7. attach() times out cleanly when publisher never starts
 *  8. Multi-robot state writes
 *
 * Build:
 *   cmake -DSHMBRIDGE_BUILD_TESTS=ON -S shmbridge -B shmbridge/build
 *   cmake --build shmbridge/build
 *   ctest --test-dir shmbridge/build -V
 */

#include <gtest/gtest.h>
#include <shmbridge/core.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

using namespace shmbridge;

/* ── helpers ─────────────────────────────────────────────────────────────── */

static std::string unique_name(const char* base) {
    return std::string("/sbcore_") + base + "_" + std::to_string(::getpid());
}

static RobotState make_state(int id) {
    RobotState s;
    s.x        = static_cast<double>(id) * 1.1;
    s.y        = static_cast<double>(id) * -2.2;
    s.heading  = static_cast<double>(id) * 0.3;
    s.vx       = static_cast<float>(id) * 0.5f;
    s.goal_x   = static_cast<double>(id) * 3.0;
    s.step     = static_cast<uint64_t>(id) * 10;
    s.sim_time = static_cast<double>(id) * 0.1;
    s.reached  = (id % 2 == 0);
    return s;
}

/* ── §1  Publisher lifecycle ─────────────────────────────────────────────── */

TEST(Publisher, OpenCloseIsOpen) {
    auto name = unique_name("lifecycle");
    ShmPublisher pub(name);
    EXPECT_FALSE(pub.is_open());
    pub.open();
    EXPECT_TRUE(pub.is_open());
    pub.close();
    EXPECT_FALSE(pub.is_open());
}

TEST(Publisher, DoubleCloseIsIdempotent) {
    auto name = unique_name("double_close");
    ShmPublisher pub(name);
    pub.open();
    pub.close();
    EXPECT_NO_THROW(pub.close());
}

/* ── §2  State round-trip ────────────────────────────────────────────────── */

TEST(StateRoundTrip, BasicWriteRead) {
    auto name = unique_name("state_rt");
    ShmPublisher pub(name, 1, 1);
    pub.open();

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));

    RobotState written = make_state(7);
    pub.write_state(0, written);

    auto r = sub.read_state_spin(0, 64);
    ASSERT_TRUE(r.has_value()) << "read_state_spin returned nullopt";
    EXPECT_NEAR(r->x,        written.x,       1e-9);
    EXPECT_NEAR(r->y,        written.y,       1e-9);
    EXPECT_NEAR(r->heading,  written.heading, 1e-9);
    EXPECT_EQ  (r->step,     written.step);
    EXPECT_NEAR(r->sim_time, written.sim_time, 1e-9);
    EXPECT_EQ  (r->reached,  written.reached);
    sub.detach();
}

TEST(StateRoundTrip, SeqlockConsistency) {
    auto name = unique_name("seqlock");
    ShmPublisher pub(name, 1, 1);
    pub.open();

    RobotState s = make_state(3);
    pub.write_state(0, s);
    pub.write_state(0, s);

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));
    auto r1 = sub.read_state_spin(0);
    auto r2 = sub.read_state_spin(0);
    ASSERT_TRUE(r1 && r2);
    EXPECT_EQ(r1->step, r2->step);
    sub.detach();
}

/* ── §3  Command round-trip ──────────────────────────────────────────────── */

TEST(CmdRoundTrip, WriteReadCmd) {
    auto name = unique_name("cmd_rt");
    ShmPublisher pub(name, 1, 1);
    pub.open();

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));

    sub.write_cmd(0, 0, 0.75f, -0.4f);
    auto cmd = pub.read_cmd(0, 0);
    ASSERT_TRUE(cmd.has_value());
    EXPECT_NEAR(cmd->linear,  0.75f, 1e-5f);
    EXPECT_NEAR(cmd->angular, -0.4f, 1e-5f);

    sub.detach();
}

/* ── §4  read_best_cmd ───────────────────────────────────────────────────── */

TEST(CmdRoundTrip, ReadBestCmdPicksHighestSeq) {
    auto name = unique_name("best_cmd");
    ShmPublisher pub(name, 1, 2);
    pub.open();

    ShmSubscriber sub0(name);
    ASSERT_NO_THROW(sub0.attach(2000));
    ShmSubscriber sub1(name);
    ASSERT_NO_THROW(sub1.attach(2000));

    sub0.write_cmd(0, 0, 0.5f, 0.0f);
    sub1.write_cmd(0, 1, 1.0f, 0.1f);
    sub1.write_cmd(0, 1, 1.0f, 0.1f);  /* consumer 1 now has higher seq */

    auto best = pub.read_best_cmd(0);
    ASSERT_TRUE(best.has_value());
    EXPECT_NEAR(best->linear, 1.0f, 1e-5f);

    sub0.detach();
    sub1.detach();
}

/* ── §5  Liveness ────────────────────────────────────────────────────────── */

TEST(Liveness, WriterAlive) {
    auto name = unique_name("liveness_w");
    ShmPublisher pub(name, 1, 1, /*heartbeat_every=*/1);
    pub.open();

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));

    EXPECT_TRUE(sub.is_publisher_alive(1000));  /* no write yet — ts==0 → alive */
    pub.write_state(0, make_state(1));
    EXPECT_TRUE(sub.is_publisher_alive(1000));
    EXPECT_FALSE(sub.is_publisher_alive(0));    /* immediately stale */

    sub.detach();
}

TEST(Liveness, ControllerAlive) {
    auto name = unique_name("liveness_c");
    ShmPublisher pub(name, 1, 1);
    pub.open();

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));

    EXPECT_TRUE(pub.is_controller_alive(1000, 0, 0));
    sub.write_cmd(0, 0, 0.1f, 0.0f);
    EXPECT_TRUE(pub.is_controller_alive(1000, 0, 0));
    EXPECT_FALSE(pub.is_controller_alive(0, 0, 0));

    sub.detach();
}

/* ── §6  Subscriber starts before publisher ──────────────────────────────── */

TEST(SubscriberOrdering, SubStartsBeforePublisher) {
    auto name = unique_name("late_pub");
    std::atomic<bool> pub_opened{false};

    /* Publisher opens after 200 ms. */
    std::thread pub_thread([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ShmPublisher pub(name, 1, 1);
        pub.open();
        pub_opened.store(true, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
    });

    ShmSubscriber sub(name);
    auto t0 = std::chrono::steady_clock::now();
    EXPECT_NO_THROW(sub.attach(5000));
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();

    pub_thread.join();

    EXPECT_TRUE(pub_opened.load());
    EXPECT_GE(ms, 100) << "attach() returned suspiciously fast";
    EXPECT_LT(ms, 2000) << "attach() waited too long";
    EXPECT_TRUE(sub.is_attached());
    sub.detach();
}

TEST(SubscriberOrdering, AttachTimeoutWhenNoPublisher) {
    auto name = unique_name("no_pub");
    ShmSubscriber sub(name);

    auto t0 = std::chrono::steady_clock::now();
    EXPECT_THROW(sub.attach(300), std::runtime_error);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();

    EXPECT_GE(ms, 200) << "timeout fired too early";
    EXPECT_LT(ms, 1500) << "timeout fired too late";
    EXPECT_FALSE(sub.is_attached());
}

/* ── §7  Publisher restart / reconnect loop ─────────────────────────────── */

TEST(PublisherRestart, ReattachAfterPublisherClose) {
    auto name = unique_name("restart");

    /* Round 1 */
    {
        ShmPublisher pub(name, 1, 1);
        pub.open();

        ShmSubscriber sub(name);
        ASSERT_NO_THROW(sub.attach(2000));

        pub.write_state(0, make_state(1));
        auto r = sub.read_state_spin(0);
        ASSERT_TRUE(r.has_value());
        EXPECT_EQ(r->step, 10u);

        sub.detach();
        /* pub closes and unlinks the segment here */
    }

    /* Round 2: publisher restarts with a fresh segment */
    {
        ShmPublisher pub(name, 1, 1);
        pub.open();

        ShmSubscriber sub(name);
        ASSERT_NO_THROW(sub.attach(2000));  /* re-attach to new segment */

        pub.write_state(0, make_state(2));
        auto r = sub.read_state_spin(0);
        ASSERT_TRUE(r.has_value());
        EXPECT_EQ(r->step, 20u);

        sub.detach();
    }
}

TEST(PublisherRestart, AttachAutoDetachesPreviousMapping) {
    auto name = unique_name("auto_detach");

    ShmPublisher pub(name, 1, 1);
    pub.open();

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));
    EXPECT_TRUE(sub.is_attached());

    /* Call attach() again without an explicit detach() — must not leak or throw. */
    ASSERT_NO_THROW(sub.attach(2000));
    EXPECT_TRUE(sub.is_attached());

    pub.write_state(0, make_state(7));
    auto r = sub.read_state_spin(0);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->step, make_state(7).step);

    sub.detach();
}

TEST(PublisherRestart, ReconnectLoopTwoRuns) {
    auto name = unique_name("reconnect_loop");
    std::vector<uint64_t> steps;

    for (int run = 0; run < 2; ++run) {
        ShmPublisher pub(name, 1, 1);
        pub.open();

        ShmSubscriber sub(name);
        sub.attach(2000);

        RobotState s;
        s.step = static_cast<uint64_t>((run + 1) * 10);
        pub.write_state(0, s);

        auto r = sub.read_state_spin(0);
        ASSERT_TRUE(r.has_value()) << "run " << run;
        steps.push_back(r->step);

        sub.detach();
        /* pub closes; segment is unlinked before the next iteration */
    }

    ASSERT_EQ(steps.size(), 2u);
    EXPECT_EQ(steps[0], 10u);
    EXPECT_EQ(steps[1], 20u);
}

TEST(PublisherRestart, SubscriberReattachesWhenPublisherRestartsLate) {
    auto name = unique_name("late_restart");
    std::atomic<bool> done{false};

    /* Publisher: run, pause 300 ms, restart. */
    std::thread pub_thread([&] {
        for (int run = 0; run < 2; ++run) {
            ShmPublisher pub(name, 1, 1);
            pub.open();
            RobotState s;
            s.step = static_cast<uint64_t>((run + 1) * 10);
            pub.write_state(0, s);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            /* pub closes and unlinks here */
            if (run == 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        done.store(true);
    });

    std::vector<uint64_t> steps;
    ShmSubscriber sub(name);
    for (int i = 0; i < 2; ++i) {
        /* attach() auto-detaches any previous mapping */
        ASSERT_NO_THROW(sub.attach(5000)) << "iteration " << i;
        auto r = sub.read_state_spin(0, 128);
        ASSERT_TRUE(r.has_value()) << "iteration " << i;
        steps.push_back(r->step);

        /* wait for publisher to go away */
        while (sub.is_publisher_alive(500)) {
            platform::sleep_ns(50'000'000LL);
        }
        sub.detach();
    }

    pub_thread.join();
    ASSERT_EQ(steps.size(), 2u);
    EXPECT_EQ(steps[0], 10u);
    EXPECT_EQ(steps[1], 20u);
}

/* ── §8  Multi-robot ─────────────────────────────────────────────────────── */

TEST(MultiRobot, WriteReadFourRobots) {
    auto name = unique_name("multirobot");
    ShmPublisher pub(name, 4, 1);
    pub.open();

    ShmSubscriber sub(name);
    ASSERT_NO_THROW(sub.attach(2000));
    EXPECT_EQ(sub.n_robots(), 4u);

    for (unsigned i = 0; i < 4; ++i)
        pub.write_state(i, make_state(static_cast<int>(i + 1)));

    for (unsigned i = 0; i < 4; ++i) {
        auto r = sub.read_state_spin(i);
        ASSERT_TRUE(r.has_value()) << "robot " << i;
        EXPECT_NEAR(r->x, (i + 1) * 1.1, 1e-9) << "robot " << i;
        EXPECT_EQ(r->step, (i + 1) * 10u) << "robot " << i;
    }

    sub.detach();
}
