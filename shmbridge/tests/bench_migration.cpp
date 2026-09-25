/*
 * bench_migration.cpp — performance benchmark for ring.hpp, registry.hpp, node.hpp.
 *
 * Metrics collected:
 *   - Latency (ns): p50 / p95 / p99 / p999 / max
 *   - Throughput (Mmsg/s)
 *   - CPU efficiency (spin vs block)
 *   - Memory footprint (bytes per slot)
 *   - Node overhead vs direct pub/sub
 *
 * Output: JSON to stdout (redirect to bench_migration.json).
 * Run:
 *   g++ -std=c++17 -O3 -march=native -I include -o /tmp/bench_migration \
 *       tests/bench_migration.cpp -lpthread -lrt
 *   /tmp/bench_migration > bench_migration.json
 */

#include "shmbridge/messages.hpp"
#include "shmbridge/node.hpp"
#include "shmbridge/registry.hpp"
#include "shmbridge/ring.hpp"
#include "shmbridge/topic.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <thread>
#include <vector>

using namespace shmbridge;
using namespace shmbridge::msg;

/* ── timing helpers ───────────────────────────────────────────────────────── */

static inline uint64_t now_ns() noexcept {
    struct timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

struct Stats {
    double mean_ns;
    double p50_ns, p95_ns, p99_ns, p999_ns, max_ns;
    uint64_t n;
};

static Stats compute(std::vector<uint64_t>& v) {
    std::sort(v.begin(), v.end());
    Stats s{};
    s.n = v.size();
    if (s.n == 0) return s;
    s.max_ns = static_cast<double>(v.back());
    auto pct = [&](double p) -> double {
        std::size_t idx = static_cast<std::size_t>(p * (s.n - 1));
        return static_cast<double>(v[idx]);
    };
    s.p50_ns  = pct(0.50);
    s.p95_ns  = pct(0.95);
    s.p99_ns  = pct(0.99);
    s.p999_ns = pct(0.999);
    uint64_t sum = std::accumulate(v.begin(), v.end(), uint64_t{0});
    s.mean_ns = static_cast<double>(sum) / s.n;
    return s;
}

/* ── JSON helpers ─────────────────────────────────────────────────────────── */

static void print_stats(const char* label, const Stats& s, bool comma = true) {
    printf(
        "  \"%s\": {\"n\":%llu, \"mean\":%.1f, \"p50\":%.1f, "
        "\"p95\":%.1f, \"p99\":%.1f, \"p999\":%.1f, \"max\":%.1f}%s\n",
        label,
        (unsigned long long)s.n,
        s.mean_ns, s.p50_ns, s.p95_ns, s.p99_ns, s.p999_ns, s.max_ns,
        comma ? "," : "");
}

/* ── Ring benchmarks ──────────────────────────────────────────────────────── */

static Stats bench_ring_push_pop_inprocess() {
    /* Single-threaded push immediately followed by pop — measures combined
     * slot write + read cost in the same process (L1 cache hot). */
    constexpr int WARMUP = 2000, N = 50000;
    ::shm_unlink("/sbr_bm_ring_lat");

    RingPublisher<Pose2d, 256> pub;
    pub.open("bm_ring_lat");
    RingSubscriber<Pose2d, 256> sub;
    sub.attach("bm_ring_lat", 1000);

    Pose2d msg{1.0, 2.0, 0.5, 0};
    for (int i = 0; i < WARMUP; ++i) {
        pub.push(msg);
        sub.pop();
    }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub.push(msg);
        sub.pop();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

static Stats bench_ring_push_inprocess() {
    /* Push-only cost (ring full → drop path excluded). */
    constexpr int WARMUP = 2000, N = 50000;
    ::shm_unlink("/sbr_bm_ring_push");

    RingPublisher<Pose2d, 256> pub;
    pub.open("bm_ring_push");
    RingSubscriber<Pose2d, 256> sub;
    sub.attach("bm_ring_push", 1000);

    Pose2d msg{};
    for (int i = 0; i < WARMUP; ++i) { pub.push(msg); sub.pop(); }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        /* Keep ring below half-full to avoid the full-drop path. */
        if (i % 4 == 3) { sub.pop(); sub.pop(); }
        uint64_t t0 = now_ns();
        pub.push(msg);
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

static Stats bench_ring_pop_inprocess() {
    /* Pop-only cost. */
    constexpr int WARMUP = 2000, N = 50000;
    ::shm_unlink("/sbr_bm_ring_pop");

    RingPublisher<Pose2d, 256> pub;
    pub.open("bm_ring_pop");
    RingSubscriber<Pose2d, 256> sub;
    sub.attach("bm_ring_pop", 1000);

    Pose2d msg{};
    for (int i = 0; i < WARMUP; ++i) { pub.push(msg); sub.pop(); }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        pub.push(msg);
        uint64_t t0 = now_ns();
        sub.pop();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

/* SPSC throughput: producer thread pushes N items, consumer thread drains.
 * Returns messages per second. */
static double bench_ring_spsc_throughput() {
    constexpr uint64_t N = 2'000'000;
    ::shm_unlink("/sbr_bm_ring_thr");

    RingPublisher<Pose2d, 1024> pub;
    pub.open("bm_ring_thr");

    std::atomic<bool> go{false};
    std::atomic<uint64_t> producer_ns{0}, consumer_ns{0};

    std::thread producer([&] {
        Pose2d msg{};
        while (!go.load(std::memory_order_relaxed));  /* sync start */
        uint64_t t0 = now_ns();
        uint64_t pushed = 0;
        while (pushed < N) {
            if (pub.push(msg)) ++pushed;
        }
        producer_ns.store(now_ns() - t0, std::memory_order_relaxed);
    });

    std::thread consumer([&] {
        RingSubscriber<Pose2d, 1024> sub;
        sub.attach("bm_ring_thr", 2000);
        while (!go.load(std::memory_order_relaxed));
        uint64_t t0 = now_ns();
        uint64_t consumed = 0;
        while (consumed < N) {
            if (auto r = sub.pop()) { ++consumed; (void)r; }
        }
        consumer_ns.store(now_ns() - t0, std::memory_order_relaxed);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); /* let sub attach */
    go.store(true, std::memory_order_release);
    producer.join();
    consumer.join();

    /* Throughput = N / max(producer_time, consumer_time) */
    uint64_t wall = std::max(producer_ns.load(), consumer_ns.load());
    return static_cast<double>(N) / (static_cast<double>(wall) / 1e9) / 1e6; /* Mmsg/s */
}

/* ── Seqlock benchmarks (for comparison) ─────────────────────────────────── */

static Stats bench_seqlock_write_inprocess() {
    constexpr int WARMUP = 2000, N = 50000;
    ::shm_unlink("/sb_bm_seq_write");

    Publisher<Pose2d> pub;
    pub.open("bm_seq_write");

    Pose2d msg{1.0, 2.0, 0.5, 0};
    for (int i = 0; i < WARMUP; ++i) pub.write(msg);

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub.write(msg);
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

static Stats bench_seqlock_read_inprocess() {
    constexpr int WARMUP = 2000, N = 50000;
    ::shm_unlink("/sb_bm_seq_read");

    Publisher<Pose2d> pub;
    pub.open("bm_seq_read");
    Subscriber<Pose2d> sub;
    sub.attach("bm_seq_read", 1000);

    Pose2d msg{1.0, 2.0, 0.5, 0};
    for (int i = 0; i < WARMUP; ++i) {
        pub.write(msg);
        sub.latest();
    }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        pub.write(msg);
        uint64_t t0 = now_ns();
        sub.latest();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

static Stats bench_seqlock_write_read_inprocess() {
    constexpr int WARMUP = 2000, N = 50000;
    ::shm_unlink("/sb_bm_seq_wr");

    Publisher<Pose2d> pub;
    pub.open("bm_seq_wr");
    Subscriber<Pose2d> sub;
    sub.attach("bm_seq_wr", 1000);

    Pose2d msg{};
    for (int i = 0; i < WARMUP; ++i) { pub.write(msg); sub.latest(); }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub.write(msg);
        sub.latest();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

/* ── Seqlock MRSW: 4 concurrent readers vs 1 writer ─────────────────────── */

static Stats bench_seqlock_read_contended(int n_readers) {
    ::shm_unlink("/sb_bm_seq_cont");
    Publisher<Pose2d> pub;
    pub.open("bm_seq_cont");

    std::atomic<bool> go{false}, done{false};
    std::vector<std::thread> readers;
    for (int i = 0; i < n_readers; ++i) {
        readers.emplace_back([&] {
            Subscriber<Pose2d> sub;
            sub.attach("bm_seq_cont", 2000);
            while (!go.load(std::memory_order_relaxed));
            while (!done.load(std::memory_order_relaxed)) sub.latest();
        });
    }

    constexpr int WARMUP = 1000, N = 20000;
    Pose2d msg{};
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    go.store(true, std::memory_order_release);
    for (int i = 0; i < WARMUP; ++i) pub.write(msg);

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub.write(msg);
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    done.store(true, std::memory_order_release);
    for (auto& t : readers) t.join();
    return compute(samples);
}

/* ── Registry benchmarks ──────────────────────────────────────────────────── */

static Stats bench_registry_register() {
    DiscoveryRegistry reg;
    reg.open();

    constexpr int WARMUP = 200, N = 5000;
    std::vector<uint64_t> samples;
    samples.reserve(N + WARMUP);

    for (int i = 0; i < WARMUP + N; ++i) {
        /* Unregister from last iteration. */
        int slot = reg.slot_idx();
        if (slot >= 0) reg.unregister_node(slot);

        uint64_t t0 = now_ns();
        reg.register_node("bench_node", {"pose"}, {"cmd"}, {"scan"});
        uint64_t t1 = now_ns();
        if (i >= WARMUP) samples.push_back(t1 - t0);
    }
    reg.unregister_node(reg.slot_idx());
    return compute(samples);
}

static Stats bench_registry_list_active(int n_nodes) {
    /* Pre-populate N nodes, then benchmark list_active(). */
    std::vector<DiscoveryRegistry> regs(n_nodes);
    for (auto& r : regs) r.open();
    for (int i = 0; i < n_nodes; ++i) {
        char name[32]; snprintf(name, sizeof(name), "bench_node_%d", i);
        regs[i].register_node(name, {"topic"}, {}, {});
    }

    constexpr int WARMUP = 200, N = 5000;
    DiscoveryRegistry qreg; qreg.open();

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < WARMUP; ++i) qreg.list_active();
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        qreg.list_active();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    for (auto& r : regs) r.unregister_node(r.slot_idx());
    return compute(samples);
}

static Stats bench_registry_heartbeat() {
    DiscoveryRegistry reg;
    reg.open();
    reg.register_node("hb_node", {}, {}, {});
    int slot = reg.slot_idx();

    constexpr int WARMUP = 500, N = 20000;
    for (int i = 0; i < WARMUP; ++i) reg.heartbeat(slot);

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        reg.heartbeat(slot);
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    reg.unregister_node(slot);
    return compute(samples);
}

/* ── Node overhead benchmarks ─────────────────────────────────────────────── */

/* Direct seqlock: pub.write_notify() → sub.read_if_new() */
static Stats bench_direct_seqlock_roundtrip() {
    ::shm_unlink("/sb_bm_direct_rt");
    Publisher<Pose2d> pub;
    pub.open("bm_direct_rt");
    Subscriber<Pose2d> sub;
    sub.attach("bm_direct_rt", 1000);

    Pose2d msg{1.0, 2.0, 0.5, 0};
    constexpr int WARMUP = 1000, N = 20000;
    for (int i = 0; i < WARMUP; ++i) { pub.write(msg); sub.read_if_new(); }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub.write(msg);
        sub.read_if_new();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

/* Node wrapper: node.publish() → spin_once() callback */
static Stats bench_node_publish_spin() {
    ::shm_unlink("/sb_bm_node_rt");

    using namespace ros_compat;
    auto pub_node = make_node("bm_pub");
    auto sub_node = make_node("bm_sub");

    auto pub = pub_node->create_publisher<Pose2d>("bm_node_rt", SensorDataQoS());

    std::atomic<uint64_t> cb_time{0};
    auto sub_h = sub_node->create_subscription<Pose2d>(
        "bm_node_rt", SensorDataQoS(),
        [&](const Pose2d&) { cb_time.store(now_ns(), std::memory_order_relaxed); });

    /* Let subscriber attach. */
    for (int i = 0; i < 50; ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    Pose2d msg{1.0, 2.0, 0.5, 0};
    constexpr int WARMUP = 500, N = 10000;
    for (int i = 0; i < WARMUP; ++i) {
        pub_node->publish<Pose2d>("bm_node_rt", msg);
        sub_node->spin_once();
    }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub_node->publish<Pose2d>("bm_node_rt", msg);
        sub_node->spin_once();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

/* Direct ring: push() → pop() vs Node ring path. */
static Stats bench_direct_ring_roundtrip() {
    ::shm_unlink("/sbr_bm_direct_ring");
    RingPublisher<Twist, 64> pub;
    pub.open("bm_direct_ring");
    RingSubscriber<Twist, 64> sub;
    sub.attach("bm_direct_ring", 1000);

    Twist msg{};
    constexpr int WARMUP = 1000, N = 20000;
    for (int i = 0; i < WARMUP; ++i) { pub.push(msg); sub.pop(); }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub.push(msg);
        sub.pop();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

static Stats bench_node_ring_spin() {
    ::shm_unlink("/sbr_bm_node_ring");

    using namespace ros_compat;
    auto pub_node = make_node("bm_ring_pub");
    auto sub_node = make_node("bm_ring_sub");

    auto pub = pub_node->create_publisher<Twist>("bm_node_ring", SystemDefaultsQoS());

    std::atomic<uint64_t> cb_time{0};
    sub_node->create_subscription<Twist>(
        "bm_node_ring", SystemDefaultsQoS(),
        [&](const Twist&) { cb_time.store(now_ns(), std::memory_order_relaxed); });

    for (int i = 0; i < 50; ++i) {
        sub_node->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    Twist msg{};
    constexpr int WARMUP = 500, N = 10000;
    for (int i = 0; i < WARMUP; ++i) {
        pub_node->publish<Twist>("bm_node_ring", msg);
        sub_node->spin_once();
    }

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        pub_node->publish<Twist>("bm_node_ring", msg);
        sub_node->spin_once();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

/* ── Memory footprint ─────────────────────────────────────────────────────── */

static void print_memory_footprint() {
    printf("  \"memory\": {\n");
    printf("    \"TopicHeader_bytes\": %zu,\n", sizeof(TopicHeader));
    printf("    \"SeqlockSlot_Pose2d_bytes\": %zu,\n", sizeof(SeqlockSlot<Pose2d>));
    printf("    \"SeqlockSlot_LaserScan2d_bytes\": %zu,\n", sizeof(SeqlockSlot<LaserScan2d>));
    printf("    \"RingHeader_bytes\": %zu,\n", sizeof(RingHeader));
    printf("    \"NodeSlot_bytes\": %zu,\n", sizeof(NodeSlot));
    printf("    \"NodeSlot_64_bytes\": %zu,\n", sizeof(NodeSlot) * MAX_NODES);
    /* Segment sizes */
    printf("    \"seqlock_segment_Pose2d_bytes\": %zu,\n",
           sizeof(TopicHeader) + sizeof(SeqlockSlot<Pose2d>));
    printf("    \"ring_segment_Pose2d_N64_bytes\": %zu,\n",
           sizeof(RingHeader) + 64 * sizeof(Pose2d));
    printf("    \"ring_segment_Pose2d_N1024_bytes\": %zu,\n",
           sizeof(RingHeader) + 1024 * sizeof(Pose2d));
    printf("    \"discovery_segment_bytes\": %zu\n",
           sizeof(NodeSlot) * MAX_NODES);
    printf("  },\n");
}

/* ── Spin-once overhead: empty loop cost ─────────────────────────────────── */

static Stats bench_spin_once_overhead() {
    using namespace ros_compat;
    auto node = make_node("bm_spin");
    /* Subscribe to a topic that is never published to — measures bare
     * spin_once() cost: heartbeat + one failed try_attach. */
    node->create_subscription<Pose2d>("nonexistent_topic", SensorDataQoS(),
                                      [](const Pose2d&) {});

    constexpr int WARMUP = 500, N = 20000;
    for (int i = 0; i < WARMUP; ++i) node->spin_once();

    std::vector<uint64_t> samples;
    samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        uint64_t t0 = now_ns();
        node->spin_once();
        uint64_t t1 = now_ns();
        samples.push_back(t1 - t0);
    }
    return compute(samples);
}

/* ── main ─────────────────────────────────────────────────────────────────── */

int main() {
    fprintf(stderr, "shmbridge migration benchmark starting...\n");

    printf("{\n");

    /* ── memory ── */
    print_memory_footprint();

    /* ── ring latency ── */
    fprintf(stderr, "  ring push+pop in-process...\n");
    auto r_pp = bench_ring_push_pop_inprocess();
    fprintf(stderr, "  ring push-only...\n");
    auto r_push = bench_ring_push_inprocess();
    fprintf(stderr, "  ring pop-only...\n");
    auto r_pop = bench_ring_pop_inprocess();

    printf("  \"ring\": {\n");
    print_stats("push_pop_ns", r_pp);
    print_stats("push_ns", r_push);
    print_stats("pop_ns", r_pop, false);
    printf("  },\n");

    /* ── ring throughput ── */
    fprintf(stderr, "  ring SPSC throughput...\n");
    double thr = bench_ring_spsc_throughput();
    printf("  \"ring_throughput_mmsg_per_s\": %.2f,\n", thr);

    /* ── seqlock latency ── */
    fprintf(stderr, "  seqlock write in-process...\n");
    auto s_w = bench_seqlock_write_inprocess();
    fprintf(stderr, "  seqlock read in-process...\n");
    auto s_r = bench_seqlock_read_inprocess();
    fprintf(stderr, "  seqlock write+read in-process...\n");
    auto s_wr = bench_seqlock_write_read_inprocess();
    fprintf(stderr, "  seqlock write contended (4 readers)...\n");
    auto s_cont = bench_seqlock_read_contended(4);

    printf("  \"seqlock\": {\n");
    print_stats("write_ns", s_w);
    print_stats("read_ns", s_r);
    print_stats("write_read_ns", s_wr);
    print_stats("write_4readers_ns", s_cont, false);
    printf("  },\n");

    /* ── registry ── */
    fprintf(stderr, "  registry register...\n");
    auto reg_r = bench_registry_register();
    fprintf(stderr, "  registry list_active (8 nodes)...\n");
    auto reg_l = bench_registry_list_active(8);
    fprintf(stderr, "  registry heartbeat...\n");
    auto reg_h = bench_registry_heartbeat();

    printf("  \"registry\": {\n");
    print_stats("register_ns", reg_r);
    print_stats("list_active_8nodes_ns", reg_l);
    print_stats("heartbeat_ns", reg_h, false);
    printf("  },\n");

    /* ── node overhead ── */
    fprintf(stderr, "  direct seqlock roundtrip...\n");
    auto d_seq = bench_direct_seqlock_roundtrip();
    fprintf(stderr, "  node seqlock roundtrip...\n");
    auto n_seq = bench_node_publish_spin();
    fprintf(stderr, "  direct ring roundtrip...\n");
    auto d_ring = bench_direct_ring_roundtrip();
    fprintf(stderr, "  node ring roundtrip...\n");
    auto n_ring = bench_node_ring_spin();
    fprintf(stderr, "  spin_once overhead...\n");
    auto spin_oh = bench_spin_once_overhead();

    printf("  \"node_overhead\": {\n");
    printf("    \"note\": \"publish+spin_once vs direct write+read, inprocess\",\n");
    print_stats("direct_seqlock_ns", d_seq);
    print_stats("node_seqlock_ns", n_seq);
    printf("    \"seqlock_overhead_pct\": %.1f,\n",
           (n_seq.p50_ns - d_seq.p50_ns) / d_seq.p50_ns * 100.0);
    print_stats("direct_ring_ns", d_ring);
    print_stats("node_ring_ns", n_ring);
    printf("    \"ring_overhead_pct\": %.1f,\n",
           (n_ring.p50_ns - d_ring.p50_ns) / d_ring.p50_ns * 100.0);
    print_stats("spin_once_empty_ns", spin_oh, false);
    printf("  }\n");

    printf("}\n");
    fprintf(stderr, "Done.\n");
    return 0;
}
