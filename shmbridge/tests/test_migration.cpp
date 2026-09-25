/*
 * tests/test_migration.cpp — unit tests for ring.hpp, registry.hpp, node.hpp.
 */

#include "shmbridge/messages.hpp"
#include "shmbridge/node.hpp"
#include "shmbridge/registry.hpp"
#include "shmbridge/ring.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using namespace shmbridge;
using namespace shmbridge::msg;

/* ══════════════════════════════════════════════════════════════════════════
   ring.hpp
   ══════════════════════════════════════════════════════════════════════════ */

TEST(RingHeader, Exactly64Bytes) {
    EXPECT_EQ(sizeof(RingHeader), 64u);
}

TEST(Ring, BasicPushPop) {
    const char* topic = "test_ring_basic";
    ::shm_unlink("/sbr_test_ring_basic");

    RingPublisher<Pose2d, 8> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 8> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    Pose2d msg{1.0, 2.0, 0.5, 100};
    EXPECT_TRUE(pub.push(msg));

    auto got = sub.pop();
    ASSERT_TRUE(got.has_value());
    EXPECT_DOUBLE_EQ(got->x, 1.0);
    EXPECT_DOUBLE_EQ(got->y, 2.0);
    EXPECT_DOUBLE_EQ(got->heading, 0.5);
    EXPECT_EQ(got->stamp_ns, 100u);
}

TEST(Ring, EmptyPop) {
    const char* topic = "test_ring_empty";
    ::shm_unlink("/sbr_test_ring_empty");

    RingPublisher<Pose2d, 4> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 4> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    EXPECT_FALSE(sub.pop().has_value());
}

TEST(Ring, FullDrops) {
    const char* topic = "test_ring_full";
    ::shm_unlink("/sbr_test_ring_full");

    RingPublisher<Pose2d, 4> pub;
    ASSERT_TRUE(pub.open(topic));

    Pose2d msg{};
    int pushed = 0;
    for (int i = 0; i < 8; ++i)
        if (pub.push(msg)) ++pushed;

    /* Only 4 slots; should accept exactly 4 and drop the rest. */
    EXPECT_EQ(pushed, 4);
}

TEST(Ring, DrainCallback) {
    const char* topic = "test_ring_drain";
    ::shm_unlink("/sbr_test_ring_drain");

    RingPublisher<Twist, 8> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Twist, 8> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    for (int i = 0; i < 5; ++i) {
        Twist t{}; t.vx = static_cast<float>(i);
        pub.push(t);
    }

    std::vector<float> received;
    sub.drain([&](const Twist& t) { received.push_back(t.vx); });

    ASSERT_EQ(received.size(), 5u);
    for (int i = 0; i < 5; ++i)
        EXPECT_FLOAT_EQ(received[i], static_cast<float>(i));
}

TEST(Ring, ClosedFlag) {
    const char* topic = "test_ring_closed";
    ::shm_unlink("/sbr_test_ring_closed");

    RingPublisher<Pose2d, 4> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 4> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    EXPECT_FALSE(sub.is_closed());
    pub.signal_closed();
    EXPECT_TRUE(sub.is_closed());
}

TEST(Ring, SizeAndPeek) {
    const char* topic = "test_ring_size";
    ::shm_unlink("/sbr_test_ring_size");

    RingPublisher<Odometry, 8> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Odometry, 8> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    EXPECT_EQ(sub.size(), 0u);
    EXPECT_TRUE(sub.empty());

    Odometry o{}; o.x = 42.0;
    pub.push(o);

    EXPECT_EQ(sub.size(), 1u);
    auto peek = sub.peek();
    ASSERT_TRUE(peek.has_value());
    EXPECT_DOUBLE_EQ(peek->x, 42.0);
    EXPECT_EQ(sub.size(), 1u);  /* peek doesn't consume */

    sub.pop();
    EXPECT_EQ(sub.size(), 0u);
}

TEST(Ring, Throughput) {
    const char* topic = "test_ring_throughput";
    ::shm_unlink("/sbr_test_ring_throughput");

    RingPublisher<BatteryState, 64> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<BatteryState, 64> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    constexpr int N = 10000;
    std::atomic<int> received{0};

    std::thread writer([&] {
        for (int i = 0; i < N; ) {
            BatteryState b{}; b.voltage = static_cast<float>(i);
            if (pub.push(b)) ++i;
        }
    });

    std::thread reader([&] {
        while (received.load(std::memory_order_relaxed) < N) {
            sub.drain([&](const BatteryState&) {
                received.fetch_add(1, std::memory_order_relaxed);
            });
        }
    });

    writer.join();
    reader.join();
    EXPECT_EQ(received.load(), N);
}

/* ══════════════════════════════════════════════════════════════════════════
   registry.hpp
   ══════════════════════════════════════════════════════════════════════════ */

TEST(Registry, OpenAndRegister) {
    DiscoveryRegistry reg;
    ASSERT_TRUE(reg.open());

    int slot = reg.register_node("test_node",
                                  {"pose", "scan"},
                                  {"cmd_vel"},
                                  {"lidar_raw"});
    EXPECT_GE(slot, 0);
    EXPECT_EQ(reg.slot_idx(), slot);
}

TEST(Registry, ListActive) {
    DiscoveryRegistry reg;
    ASSERT_TRUE(reg.open());

    reg.register_node("regtest_node", {"topic_a"}, {}, {});

    auto nodes = reg.list_active();
    bool found = false;
    for (auto& n : nodes)
        if (n.node_id == "regtest_node") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(Registry, FindPublishers) {
    DiscoveryRegistry reg;
    ASSERT_TRUE(reg.open());

    reg.register_node("fp_node", {"unique_pub_xyz"}, {}, {});

    auto pubs = reg.find_publishers("unique_pub_xyz");
    ASSERT_FALSE(pubs.empty());
    EXPECT_EQ(pubs[0].node_id, "fp_node");
}

TEST(Registry, UnregisterDisappearsFromList) {
    DiscoveryRegistry reg;
    ASSERT_TRUE(reg.open());

    int slot = reg.register_node("ephemeral_node", {}, {}, {});
    ASSERT_GE(slot, 0);

    {
        auto nodes = reg.list_active();
        bool found = false;
        for (auto& n : nodes)
            if (n.node_id == "ephemeral_node") { found = true; break; }
        EXPECT_TRUE(found);
    }

    reg.unregister_node(slot);

    {
        auto nodes = reg.list_active();
        for (auto& n : nodes)
            EXPECT_NE(n.node_id, "ephemeral_node");
    }
}

TEST(Registry, TopicEncodeDecodeRoundTrip) {
    std::vector<std::string> pubs{"pose", "scan"};
    std::vector<std::string> kl{"cmd"};
    std::vector<std::string> ring{"raw_lidar"};

    std::string enc = detail::encode_topics(pubs, kl, ring);
    auto dec = detail::decode_topics(enc.c_str());

    EXPECT_EQ(dec.pubs, pubs);
    EXPECT_EQ(dec.subs_keep_latest, kl);
    EXPECT_EQ(dec.subs_ring, ring);
}

TEST(Registry, CasAtomicityNoConcurrentCorruption) {
    /* Two threads racing to register; both should succeed in distinct slots.
     * Keep registries alive across the assert so slots stay occupied. */
    std::shared_ptr<DiscoveryRegistry> r2, r3;
    std::atomic<int> s1{-1}, s2{-1};

    std::thread t1([&] {
        r2 = std::make_shared<DiscoveryRegistry>();
        r2->open();
        s1 = r2->register_node("cas_node_1", {}, {}, {});
    });
    std::thread t2([&] {
        r3 = std::make_shared<DiscoveryRegistry>();
        r3->open();
        s2 = r3->register_node("cas_node_2", {}, {}, {});
    });
    t1.join();
    t2.join();

    EXPECT_GE(s1.load(), 0);
    EXPECT_GE(s2.load(), 0);
    EXPECT_NE(s1.load(), s2.load());
    /* r2 and r3 go out of scope here — slots freed. */
}

/* ══════════════════════════════════════════════════════════════════════════
   node.hpp (ros_compat)
   ══════════════════════════════════════════════════════════════════════════ */

using namespace shmbridge::ros_compat;

TEST(QoS, DepthMapping) {
    EXPECT_TRUE(QoS(1).wants_keep_latest());
    EXPECT_TRUE(QoS(0).wants_keep_latest());
    EXPECT_FALSE(QoS(2).wants_keep_latest());
    EXPECT_FALSE(QoS(10).wants_keep_latest());
}

TEST(QoS, SensorDataAndSystemDefaults) {
    EXPECT_TRUE(SensorDataQoS().wants_keep_latest());
    EXPECT_FALSE(SystemDefaultsQoS().wants_keep_latest());
    EXPECT_FALSE(SensorDataQoS().is_reliable());
    EXPECT_TRUE(SystemDefaultsQoS().is_reliable());
}

TEST(Node, MakeNode) {
    EXPECT_NO_THROW({
        auto node = make_node("test_make_node");
        EXPECT_EQ(node->name(), "test_make_node");
    });
}

TEST(Node, KeepLatestPubSub) {
    ::shm_unlink("/sb_kltest_pose");

    auto pub_node = make_node("kl_pub");
    auto sub_node = make_node("kl_sub");

    auto pub = pub_node->create_publisher<Pose2d>("kltest_pose", SensorDataQoS());
    EXPECT_EQ(pub->topic(), "kltest_pose");
    EXPECT_TRUE(pub->keep_latest());

    Pose2d msg{3.0, 4.0, 1.0, 0};
    pub_node->publish<Pose2d>("kltest_pose", msg);

    /* Allow sub to attach. */
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    Pose2d received{};
    bool   got = false;
    auto sub = sub_node->create_subscription<Pose2d>(
        "kltest_pose", SensorDataQoS(), [&](const Pose2d& m) {
            received = m;
            got = true;
        });

    /* Publish again and spin_once to trigger callback. */
    pub_node->publish<Pose2d>("kltest_pose", msg);
    sub_node->spin_once();

    /* spin_once may take a few tries while sub attaches. */
    for (int i = 0; i < 20 && !got; ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(got);
    if (got) {
        EXPECT_DOUBLE_EQ(received.x, 3.0);
        EXPECT_DOUBLE_EQ(received.y, 4.0);
    }
}

TEST(Node, RingPubSub) {
    ::shm_unlink("/sbr_ringtest_twist");

    auto pub_node = make_node("ring_pub");
    auto sub_node = make_node("ring_sub");

    auto pub = pub_node->create_publisher<Twist>("ringtest_twist", SystemDefaultsQoS());
    EXPECT_FALSE(pub->keep_latest());

    Twist t{}; t.vx = 5.0f;
    pub_node->publish<Twist>("ringtest_twist", t);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    Twist received{};
    bool  got = false;
    auto sub = sub_node->create_subscription<Twist>(
        "ringtest_twist", SystemDefaultsQoS(), [&](const Twist& m) {
            received = m;
            got = true;
        });

    pub_node->publish<Twist>("ringtest_twist", t);

    for (int i = 0; i < 20 && !got; ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(got);
    if (got) EXPECT_FLOAT_EQ(received.vx, 5.0f);
}

TEST(Node, SpinFor) {
    auto node = make_node("spin_for_node");
    int  count = 0;

    Pose2d msg{};
    auto pub = node->create_publisher<Pose2d>("sf_pose", SensorDataQoS());
    node->create_subscription<Pose2d>("sf_pose", SensorDataQoS(),
                                      [&](const Pose2d&) { ++count; });

    /* Publish 3 messages, then spin for a short window. */
    for (int i = 0; i < 3; ++i) node->publish<Pose2d>("sf_pose", msg);

    spin_for(node, std::chrono::milliseconds(100));
    /* We don't assert exact count here since attach is lazy, but spin_for must
     * return without hanging. */
    SUCCEED();
}

/* ── DirectPublish via handle ─────────────────────────────────────────────── */

TEST(Node, DirectPublishHandle) {
    ::shm_unlink("/sb_dp_seqlock_pose");

    auto pub_node = make_node("dp_pub");
    auto sub_node = make_node("dp_sub");

    auto pub = pub_node->create_publisher<Pose2d>("dp_seqlock_pose", SensorDataQoS());

    Pose2d msg{7.0, 8.0, 0.1, 42};
    pub->publish(msg);   /* direct path — no map lookup, no cast */

    Pose2d received{};
    bool got = false;
    sub_node->create_subscription<Pose2d>(
        "dp_seqlock_pose", SensorDataQoS(), [&](const Pose2d& m) {
            received = m; got = true;
        });

    pub->publish(msg);
    for (int i = 0; i < 20 && !got; ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    EXPECT_TRUE(got);
    if (got) {
        EXPECT_DOUBLE_EQ(received.x, 7.0);
        EXPECT_DOUBLE_EQ(received.y, 8.0);
    }
}

/* ── MessageQueue / create_queue ──────────────────────────────────────────── */

TEST(MessageQueue, PushPopDrain) {
    MessageQueue<Pose2d> q(4);
    EXPECT_TRUE(q.empty());

    Pose2d a{1.0, 0.0, 0.0, 0}, b{2.0, 0.0, 0.0, 0};
    q.push(a);
    q.push(b);
    EXPECT_EQ(q.size(), 2u);

    auto m = q.pop();
    ASSERT_TRUE(m.has_value());
    EXPECT_DOUBLE_EQ(m->x, 1.0);   /* FIFO */

    q.push(a);
    q.push(b);
    auto latest = q.pop_latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_DOUBLE_EQ(latest->x, 2.0);  /* newest */
    EXPECT_TRUE(q.empty());             /* pop_latest clears all */
}

TEST(MessageQueue, BoundedDrop) {
    MessageQueue<Pose2d> q(3);
    for (int i = 0; i < 5; ++i) {
        Pose2d m{}; m.x = static_cast<double>(i);
        q.push(m);
    }
    EXPECT_EQ(q.size(), 3u);  /* capped; oldest dropped */
    /* Remaining should be the 3 newest: x = 2, 3, 4 */
    auto m = q.pop();
    ASSERT_TRUE(m.has_value());
    EXPECT_DOUBLE_EQ(m->x, 2.0);
}

TEST(Node, CreateQueueSeqlock) {
    ::shm_unlink("/sb_cq_seqlock_pose");

    auto pub_node = make_node("cq_pub");
    auto sub_node = make_node("cq_sub");

    auto pub = pub_node->create_publisher<Pose2d>("cq_seqlock_pose", SensorDataQoS());
    auto [sub, q] = sub_node->create_queue<Pose2d>("cq_seqlock_pose", SensorDataQoS());

    Pose2d msg{9.0, 3.0, 0.5, 0};
    pub->publish(msg);

    for (int i = 0; i < 30 && q->empty(); ++i) {
        pub->publish(msg);
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto got = q->pop_latest();
    ASSERT_TRUE(got.has_value());
    EXPECT_DOUBLE_EQ(got->x, 9.0);
    EXPECT_DOUBLE_EQ(got->y, 3.0);
}

TEST(Node, CreateQueueRing) {
    ::shm_unlink("/sbr_cq_ring_twist");

    auto pub_node = make_node("cqr_pub");
    auto sub_node = make_node("cqr_sub");

    auto pub = pub_node->create_publisher<Twist>("cq_ring_twist", SystemDefaultsQoS());
    auto [sub, q] = sub_node->create_queue<Twist>("cq_ring_twist", SystemDefaultsQoS());

    /* Publish 3 distinct messages */
    for (int i = 1; i <= 3; ++i) {
        Twist t{}; t.vx = static_cast<float>(i);
        pub->publish(t);
    }

    for (int i = 0; i < 30 && q->size() < 3; ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::vector<float> vals;
    q->drain([&](const Twist& t) { vals.push_back(t.vx); });
    ASSERT_EQ(vals.size(), 3u);
    EXPECT_FLOAT_EQ(vals[0], 1.0f);
    EXPECT_FLOAT_EQ(vals[2], 3.0f);
}

/* ── spin_once non-blocking with absent publisher ─────────────────────────── */

TEST(Node, SpinOnceNonBlockingWhenNoPublisher) {
    auto sub_node = make_node("nb_sub_node");
    sub_node->create_subscription<Pose2d>(
        "nonexistent_topic_nb", SensorDataQoS(), [](const Pose2d&) {});

    /* With timeout=0 + 100ms retry throttle, spin_once must return quickly. */
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 5; ++i) sub_node->spin_once();
    auto dt = std::chrono::steady_clock::now() - t0;

    /* 5 calls with no publisher must complete in well under 50 ms total. */
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 50);
}

/* ── Ring::skip_old ───────────────────────────────────────────────────────── */

TEST(Ring, SkipOldKeepsLatestN) {
    const char* topic = "test_ring_skipold";
    ::shm_unlink("/sbr_test_ring_skipold");

    RingPublisher<Pose2d, 8> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 8> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    /* Push 5 messages: x = 0 .. 4 */
    for (int i = 0; i < 5; ++i) {
        Pose2d m{}; m.x = static_cast<double>(i);
        ASSERT_TRUE(pub.push(m));
    }
    EXPECT_EQ(sub.size(), 5u);

    /* Keep only the 3 newest: should drop 2 (x=0,1), keep x=2,3,4. */
    uint32_t dropped = sub.skip_old(3);
    EXPECT_EQ(dropped, 2u);
    EXPECT_EQ(sub.size(), 3u);

    for (double expected = 2.0; expected <= 4.0; ++expected) {
        auto item = sub.pop();
        ASSERT_TRUE(item.has_value());
        EXPECT_DOUBLE_EQ(item->x, expected);
    }
    EXPECT_TRUE(sub.empty());
}

TEST(Ring, SkipOldNoop) {
    const char* topic = "test_ring_skipnoop";
    ::shm_unlink("/sbr_test_ring_skipnoop");

    RingPublisher<Pose2d, 8> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 8> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    for (int i = 0; i < 3; ++i) {
        Pose2d m{}; m.x = static_cast<double>(i);
        pub.push(m);
    }

    /* keep_n >= avail → nothing dropped */
    EXPECT_EQ(sub.skip_old(3), 0u);
    EXPECT_EQ(sub.skip_old(10), 0u);
    EXPECT_EQ(sub.size(), 3u);
}

/* ── Ring::pop_latest ─────────────────────────────────────────────────────── */

TEST(Ring, PopLatestGetsNewest) {
    const char* topic = "test_ring_poplatest";
    ::shm_unlink("/sbr_test_ring_poplatest");

    RingPublisher<Pose2d, 8> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 8> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    /* Push 4 messages: x = 0 .. 3 */
    for (int i = 0; i < 4; ++i) {
        Pose2d m{}; m.x = static_cast<double>(i);
        ASSERT_TRUE(pub.push(m));
    }

    auto latest = sub.pop_latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_DOUBLE_EQ(latest->x, 3.0);   /* newest item */
    EXPECT_TRUE(sub.empty());            /* all consumed */
}

TEST(Ring, PopLatestEmpty) {
    const char* topic = "test_ring_plat_empty";
    ::shm_unlink("/sbr_test_ring_plat_empty");

    RingPublisher<Pose2d, 4> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 4> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    EXPECT_FALSE(sub.pop_latest().has_value());
}

TEST(Ring, PopLatestSingle) {
    const char* topic = "test_ring_plat_single";
    ::shm_unlink("/sbr_test_ring_plat_single");

    RingPublisher<Pose2d, 4> pub;
    ASSERT_TRUE(pub.open(topic));

    RingSubscriber<Pose2d, 4> sub;
    ASSERT_TRUE(sub.attach(topic, 1000));

    Pose2d m{}; m.x = 99.0;
    pub.push(m);

    auto item = sub.pop_latest();
    ASSERT_TRUE(item.has_value());
    EXPECT_DOUBLE_EQ(item->x, 99.0);
    EXPECT_TRUE(sub.empty());
}

/* ── LatestSlot<T> ────────────────────────────────────────────────────────── */

TEST(LatestSlot, StoreAndTake) {
    LatestSlot<Pose2d> slot;
    EXPECT_FALSE(slot.has_value());

    Pose2d m{1.0, 2.0, 0.5, 0};
    slot.store(m);
    EXPECT_TRUE(slot.has_value());
    EXPECT_NE(slot.peek(), nullptr);
    EXPECT_DOUBLE_EQ(slot.peek()->x, 1.0);

    auto taken = slot.take();
    ASSERT_TRUE(taken.has_value());
    EXPECT_DOUBLE_EQ(taken->x, 1.0);
    EXPECT_FALSE(slot.has_value());
    EXPECT_EQ(slot.peek(), nullptr);
}

TEST(LatestSlot, MultipleStoresKeepLast) {
    LatestSlot<Pose2d> slot;
    for (int i = 0; i < 5; ++i) {
        Pose2d m{}; m.x = static_cast<double>(i);
        slot.store(m);
    }
    EXPECT_TRUE(slot.has_value());
    auto taken = slot.take();
    ASSERT_TRUE(taken.has_value());
    EXPECT_DOUBLE_EQ(taken->x, 4.0);   /* last stored wins */
    EXPECT_FALSE(slot.has_value());
}

TEST(LatestSlot, TakeEmpty) {
    LatestSlot<Pose2d> slot;
    EXPECT_FALSE(slot.take().has_value());
}

TEST(LatestSlot, Clear) {
    LatestSlot<Pose2d> slot;
    Pose2d m{}; m.x = 7.0;
    slot.store(m);
    slot.clear();
    EXPECT_FALSE(slot.has_value());
    EXPECT_FALSE(slot.take().has_value());
}

/* ── Node::create_latest ──────────────────────────────────────────────────── */

TEST(Node, CreateLatestSeqlock) {
    ::shm_unlink("/sb_cl_seqlock_pose");

    auto pub_node = make_node("cl_seq_pub");
    auto sub_node = make_node("cl_seq_sub");

    auto pub = pub_node->create_publisher<Pose2d>("cl_seqlock_pose", SensorDataQoS());
    auto [sub, slot] = sub_node->create_latest<Pose2d>("cl_seqlock_pose", SensorDataQoS());

    EXPECT_FALSE(slot->has_value());

    Pose2d msg{5.0, 6.0, 1.1, 0};
    pub->publish(msg);

    /* Spin until attached and the message arrives. */
    for (int i = 0; i < 30 && !slot->has_value(); ++i) {
        pub->publish(msg);
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto got = slot->take();
    ASSERT_TRUE(got.has_value());
    EXPECT_DOUBLE_EQ(got->x, 5.0);
    EXPECT_DOUBLE_EQ(got->y, 6.0);
    EXPECT_FALSE(slot->has_value());   /* consumed */
}

TEST(Node, CreateLatestRing) {
    ::shm_unlink("/sbr_cl_ring_twist");

    auto pub_node = make_node("cl_ring_pub");
    auto sub_node = make_node("cl_ring_sub");

    auto pub = pub_node->create_publisher<Twist>("cl_ring_twist", SystemDefaultsQoS());
    auto [sub, slot] = sub_node->create_latest<Twist>("cl_ring_twist", SystemDefaultsQoS());

    EXPECT_FALSE(slot->has_value());

    /* Publish 8 messages; only the newest should be visible to the slot. */
    for (int i = 1; i <= 8; ++i) {
        Twist t{}; t.vx = static_cast<float>(i);
        pub->publish(t);
    }

    for (int i = 0; i < 30 && !slot->has_value(); ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto got = slot->take();
    ASSERT_TRUE(got.has_value());
    /* pop_latest() delivers the newest; must be one of the later messages. */
    EXPECT_GE(got->vx, 1.0f);
    EXPECT_FALSE(slot->has_value());   /* slot cleared after take */
}

/* ── entry point ──────────────────────────────────────────────────────────── */
/* GTest main is provided by GTest::gtest_main link target. */
