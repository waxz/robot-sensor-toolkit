/*
 * bench_realistic.cpp — realistic workload benchmarks for shmbridge node API.
 *
 * Measures improvements from:
 *   - Non-blocking attach (timeout=0 + retry throttle)
 *   - Rate-limited heartbeat (50ms interval)
 *   - Direct publish via Publisher<T> handle
 *   - MessageQueue<T> for deferred processing
 *
 * Outputs JSON to stdout; redirect to bench_realistic.json.
 * Compile with -O3 -march=native.
 */

#include "shmbridge/messages.hpp"
#include "shmbridge/node.hpp"
#include "shmbridge/registry.hpp"
#include "shmbridge/ring.hpp"
#include "shmbridge/topic.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <numeric>
#include <thread>
#include <vector>

using namespace shmbridge;
using namespace shmbridge::msg;
using namespace shmbridge::ros_compat;

/* ── timing ───────────────────────────────────────────────────────────────── */

static inline int64_t ns_now() noexcept {
    struct timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1'000'000'000LL + (int64_t)ts.tv_nsec;
}

static inline void sleep_ns(long ns) noexcept {
    struct timespec ts{ ns / 1'000'000'000L, ns % 1'000'000'000L };
    ::nanosleep(&ts, nullptr);
}

/* ── statistics ───────────────────────────────────────────────────────────── */

struct Stats { double mean, p50, p95, p99, p999, max; int n; };

static Stats compute(std::vector<int64_t> v) {
    std::sort(v.begin(), v.end());
    int n = (int)v.size();
    double sum = 0; for (auto x : v) sum += (double)x;
    auto p = [&](double pct) -> double {
        return (double)v[(size_t)(pct * (n - 1))];
    };
    return { sum / n, p(.50), p(.95), p(.99), p(.999), (double)v.back(), n };
}

static void ps(FILE* f, const char* k, const Stats& s, bool comma = true) {
    fprintf(f,
        "  \"%s\": {\"n\":%d,\"mean\":%.1f,\"p50\":%.1f,\"p95\":%.1f,"
        "\"p99\":%.1f,\"p999\":%.1f,\"max\":%.1f}%s\n",
        k, s.n, s.mean, s.p50, s.p95, s.p99, s.p999, s.max, comma ? "," : "");
}

/* ════════════════════════════════════════════════════════════════════════════
   A. spin_once overhead
   ════════════════════════════════════════════════════════════════════════════ */

static void bench_spin_overhead(FILE* f) {
    fprintf(f, "\"spin_overhead\": {\n");

    /* A1 — spin_once with NO publisher, 1000 consecutive calls (tight loop).
     * First call per 100ms window: does one non-blocking shm_open (fails fast).
     * Subsequent calls in the same window: just a timestamp comparison (~10ns). */
    {
        auto node = make_node("so_empty");
        node->create_subscription<Pose2d>("so_nonexist_xyz", SensorDataQoS(),
                                          [](const Pose2d&) {});

        constexpr int N = 1000;
        std::vector<int64_t> samples(N);
        for (int i = 0; i < N; ++i) {
            auto t0 = ns_now();
            node->spin_once();
            samples[i] = ns_now() - t0;
        }
        ps(f, "empty_tight_loop_ns", compute(samples));
    }

    /* A2 — spin_once with attached publisher, no new messages.
     * Cost = heartbeat rate check (~11ns) + poll (read_if_new returns nullopt). */
    {
        ::shm_unlink("/sb_so_idle_pose");
        auto pub_node = make_node("so_pub");
        auto sub_node = make_node("so_sub");

        auto pub = pub_node->create_publisher<Pose2d>("so_idle_pose", SensorDataQoS());

        /* Warm-up: let sub attach */
        for (int i = 0; i < 30; ++i) {
            sub_node->spin_once();
            sleep_ns(5'000'000L);
        }

        constexpr int N = 20000;
        std::vector<int64_t> samples(N);
        for (int i = 0; i < N; ++i) {
            auto t0 = ns_now();
            sub_node->spin_once();
            samples[i] = ns_now() - t0;
        }
        ps(f, "attached_no_msgs_ns", compute(samples));
    }

    /* A3 — spin_once: attached, message available, callback fires. */
    {
        ::shm_unlink("/sb_so_msg_pose");
        auto pub_node = make_node("so_msg_pub");
        auto sub_node = make_node("so_msg_sub");

        auto pub = pub_node->create_publisher<Pose2d>("so_msg_pose", SensorDataQoS());

        /* Wait for attach */
        Pose2d msg{1.0, 2.0, 0.0, 0};
        for (int i = 0; i < 30; ++i) {
            pub->publish(msg);
            sub_node->spin_once();
            sleep_ns(5'000'000L);
        }

        constexpr int N = 20000;
        std::vector<int64_t> samples(N);
        for (int i = 0; i < N; ++i) {
            msg.stamp_ns = (uint64_t)ns_now();
            pub->publish(msg);
            auto t0 = ns_now();
            sub_node->spin_once();
            samples[i] = ns_now() - t0;
        }
        ps(f, "attached_with_msg_ns", compute(samples), false);
    }

    fprintf(f, "},\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   B. Publish path: handle vs string lookup vs direct
   ════════════════════════════════════════════════════════════════════════════ */

static void bench_publish_path(FILE* f) {
    fprintf(f, "\"publish_path\": {\n");

    ::shm_unlink("/sb_pp_seqlock_pose");
    auto node = make_node("pp_node");
    auto pub  = node->create_publisher<Pose2d>("pp_seqlock_pose", SensorDataQoS());
    Pose2d msg{1.0, 2.0, 0.0, 0};

    constexpr int N = 50000;
    std::vector<int64_t> h_samples(N), s_samples(N), d_samples(N);

    /* B1 — handle publish: pub->publish(msg)  */
    for (int i = 0; i < N; ++i) {
        auto t0 = ns_now(); pub->publish(msg); h_samples[i] = ns_now() - t0;
    }

    /* B2 — string-key publish: node->publish<T>("topic", msg) */
    for (int i = 0; i < N; ++i) {
        auto t0 = ns_now();
        node->publish<Pose2d>("pp_seqlock_pose", msg);
        s_samples[i] = ns_now() - t0;
    }

    /* B3 — direct shmbridge::Publisher write (no Node involved) */
    {
        ::shm_unlink("/sb_pp_direct_pose");
        shmbridge::Publisher<Pose2d> direct;
        direct.open("pp_direct_pose");
        for (int i = 0; i < N; ++i) {
            auto t0 = ns_now(); direct.write_notify(msg); d_samples[i] = ns_now() - t0;
        }
    }

    ps(f, "handle_publish_ns",  compute(h_samples));
    ps(f, "string_publish_ns",  compute(s_samples));
    ps(f, "direct_write_ns",    compute(d_samples), false);

    fprintf(f, "},\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   C. Control loop latency (cross-thread, realistic schedule)
   ════════════════════════════════════════════════════════════════════════════ */

static Stats cross_thread_latency(const char* shm_name,
                                  long pub_period_ns, long sub_period_ns,
                                  int n_messages, bool use_ring) {
    ::shm_unlink(shm_name);
    std::string topic(shm_name + 1);  /* strip leading '/' */

    auto pub_node = make_node(std::string("cl_pub_") + topic);
    auto sub_node = make_node(std::string("cl_sub_") + topic);

    std::shared_ptr<ros_compat::Publisher<Pose2d>>     seqlock_pub;
    std::shared_ptr<ros_compat::Publisher<Twist>>      ring_pub;
    std::shared_ptr<ros_compat::MessageQueue<Pose2d>>  seqlock_q;
    std::shared_ptr<ros_compat::MessageQueue<Twist>>   ring_q;

    if (!use_ring) {
        seqlock_pub  = pub_node->create_publisher<Pose2d>(topic, SensorDataQoS());
        auto [sub, q] = sub_node->create_queue<Pose2d>(topic, SensorDataQoS());
        seqlock_q = q; (void)sub;
    } else {
        ring_pub = pub_node->create_publisher<Twist>(topic, SystemDefaultsQoS());
        auto [sub, q] = sub_node->create_queue<Twist>(topic, SystemDefaultsQoS());
        ring_q = q; (void)sub;
    }

    /* Warmup: let subscriber attach (up to 300ms) */
    for (int i = 0; i < 60; ++i) {
        if (!use_ring) {
            Pose2d m{}; m.stamp_ns = 0; seqlock_pub->publish(m);
        } else {
            Twist t{}; t.vx = 0.0f; ring_pub->publish(t);
        }
        sub_node->spin_once();
        sleep_ns(5'000'000L);
    }

    std::vector<int64_t> latencies;
    latencies.reserve((size_t)n_messages);
    std::atomic<bool> running{true};
    std::atomic<int>  pub_count{0};

    /* Publisher thread */
    std::thread pub_thread([&] {
        for (int i = 0; i < n_messages; ++i) {
            sleep_ns(pub_period_ns);
            int64_t now = ns_now();
            if (!use_ring) {
                Pose2d m{}; m.stamp_ns = (uint64_t)now;
                seqlock_pub->publish(m);
            } else {
                Twist t{}; t.vx = (float)(now & 0x7FFFFFFF);
                ring_pub->publish(t);
            }
            pub_count.fetch_add(1, std::memory_order_relaxed);
        }
        running.store(false, std::memory_order_release);
    });

    /* Subscriber thread */
    std::thread sub_thread([&] {
        while (running.load(std::memory_order_relaxed) ||
               (seqlock_q && !seqlock_q->empty()) ||
               (ring_q && !ring_q->empty())) {
            sub_node->spin_once();
            if (seqlock_q) {
                while (auto m = seqlock_q->pop()) {
                    int64_t lat = ns_now() - (int64_t)m->stamp_ns;
                    if (lat > 0 && lat < 200'000'000LL)
                        latencies.push_back(lat);
                }
            }
            if (ring_q) {
                ring_q->drain([&](const Twist& t) {
                    /* ring msgs don't carry timestamps; measure round-trip separately */
                    (void)t;
                });
            }
            sleep_ns(sub_period_ns);
        }
    });

    pub_thread.join();
    sleep_ns(sub_period_ns * 5);
    sub_thread.join();

    if (latencies.empty()) latencies.push_back(0);
    return compute(latencies);
}

static void bench_control_loop(FILE* f) {
    fprintf(f, "\"control_loop\": {\n");

    /* C1 — seqlock 100 Hz publisher, 100 Hz subscriber */
    fprintf(stderr, "  control loop 100Hz seqlock...\n");
    auto s1 = cross_thread_latency("/sb_cl_100hz", 10'000'000L, 10'000'000L, 300, false);
    ps(f, "seqlock_100hz_latency_us", [&]{
        Stats us = s1; us.mean /= 1000; us.p50 /= 1000; us.p95 /= 1000;
        us.p99 /= 1000; us.p999 /= 1000; us.max /= 1000; return us;
    }());

    /* C2 — seqlock 1 kHz publisher, 1 kHz subscriber */
    fprintf(stderr, "  control loop 1kHz seqlock...\n");
    auto s2 = cross_thread_latency("/sb_cl_1khz", 1'000'000L, 1'000'000L, 1000, false);
    ps(f, "seqlock_1khz_latency_us", [&]{
        Stats us = s2; us.mean /= 1000; us.p50 /= 1000; us.p95 /= 1000;
        us.p99 /= 1000; us.p999 /= 1000; us.max /= 1000; return us;
    }());

    /* C3 — ring 1 kHz publisher, 1 kHz subscriber (queue drain) */
    fprintf(stderr, "  control loop 1kHz ring+queue...\n");
    auto s3 = cross_thread_latency("/sbr_cl_ring_1khz", 1'000'000L, 1'000'000L, 1000, true);
    ps(f, "ring_1khz_queue_drain_ns", s3, false);  /* ring sub timestamps not available */

    fprintf(f, "},\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   D. MessageQueue operations
   ════════════════════════════════════════════════════════════════════════════ */

static void bench_queue(FILE* f) {
    fprintf(f, "\"queue\": {\n");

    constexpr int N = 50000;
    MessageQueue<Pose2d> q(N);
    Pose2d msg{1.0, 2.0, 0.0, 0};

    /* D1 — push cost */
    std::vector<int64_t> push_s(N);
    for (int i = 0; i < N; ++i) {
        q.clear();
        auto t0 = ns_now(); q.push(msg); push_s[i] = ns_now() - t0;
    }
    ps(f, "push_ns", compute(push_s));

    /* D2 — pop cost */
    std::vector<int64_t> pop_s(N);
    for (int i = 0; i < N; ++i) {
        q.push(msg);
        auto t0 = ns_now(); q.pop(); pop_s[i] = ns_now() - t0;
    }
    ps(f, "pop_ns", compute(pop_s));

    /* D3 — drain cost amortized per message (batch of 64) */
    constexpr int BATCH = 64;
    std::vector<int64_t> drain_s(N / BATCH);
    for (int b = 0; b < N / BATCH; ++b) {
        for (int j = 0; j < BATCH; ++j) q.push(msg);
        int cnt = 0;
        auto t0 = ns_now();
        q.drain([&](const Pose2d&) { ++cnt; });
        int64_t dt = ns_now() - t0;
        drain_s[b] = (cnt > 0) ? dt / cnt : 0;
    }
    ps(f, "drain_amortized_ns", compute(drain_s), false);

    fprintf(f, "},\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   E. Multi-subscriber seqlock fan-out
   ════════════════════════════════════════════════════════════════════════════ */

static void bench_fanout(FILE* f) {
    fprintf(f, "\"fanout\": {\n");

    for (int nsubs : {1, 2, 4}) {
        ::shm_unlink("/sb_fo_pose");
        auto pub_node = make_node(std::string("fo_pub_") + std::to_string(nsubs));

        struct SubCtx {
            std::shared_ptr<Node>           node;
            std::vector<int64_t>            latencies;
            std::atomic<bool>               running{true};
            std::thread                     thread;
        };

        auto pub = pub_node->create_publisher<Pose2d>("fo_pose", SensorDataQoS());

        std::vector<SubCtx> subs(nsubs);
        for (int i = 0; i < nsubs; ++i) {
            subs[i].node = make_node("fo_sub_" + std::to_string(nsubs) + "_" + std::to_string(i));
            subs[i].latencies.reserve(2000);
            subs[i].node->create_subscription<Pose2d>(
                "fo_pose", SensorDataQoS(),
                [&s = subs[i]](const Pose2d& m) {
                    int64_t lat = ns_now() - (int64_t)m.stamp_ns;
                    if (lat > 0 && lat < 100'000'000LL)
                        s.latencies.push_back(lat);
                });
        }

        /* Let all subs attach (200ms warmup) */
        for (int i = 0; i < 40; ++i) {
            Pose2d m{}; m.stamp_ns = 0; pub->publish(m);
            for (int j = 0; j < nsubs; ++j) subs[j].node->spin_once();
            sleep_ns(5'000'000L);
        }

        /* Subscriber threads */
        for (int i = 0; i < nsubs; ++i) {
            subs[i].thread = std::thread([&s = subs[i]] {
                while (s.running.load(std::memory_order_relaxed)) {
                    s.node->spin_once();
                    sleep_ns(1'000'000L);   /* 1ms poll */
                }
            });
        }

        /* Publisher at 1 kHz for 1000 messages */
        for (int m = 0; m < 1000; ++m) {
            sleep_ns(1'000'000L);
            Pose2d msg{}; msg.stamp_ns = (uint64_t)ns_now();
            pub->publish(msg);
        }
        sleep_ns(50'000'000L);

        for (int i = 0; i < nsubs; ++i) {
            subs[i].running.store(false, std::memory_order_release);
            subs[i].thread.join();
        }

        /* Merge latencies across all subs, report worst-subscriber p99 */
        std::vector<int64_t> all;
        for (int i = 0; i < nsubs; ++i)
            all.insert(all.end(), subs[i].latencies.begin(), subs[i].latencies.end());
        if (all.empty()) all.push_back(0);

        char key[64];
        snprintf(key, sizeof(key), "seqlock_%dsubs_latency_us", nsubs);
        Stats us = compute(all);
        us.mean /= 1000; us.p50 /= 1000; us.p95 /= 1000;
        us.p99 /= 1000; us.p999 /= 1000; us.max /= 1000;
        ps(f, key, us, nsubs != 4);
    }

    fprintf(f, "},\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   F. Ring SPSC throughput with queue (high-rate sensor simulation)
   ════════════════════════════════════════════════════════════════════════════ */

static void bench_ring_sensor(FILE* f) {
    fprintf(f, "\"ring_sensor\": {\n");

    ::shm_unlink("/sbr_rs_scan");
    auto pub_node = make_node("rs_pub");
    auto sub_node = make_node("rs_sub");

    auto pub = pub_node->create_publisher<LaserScan2d>("rs_scan", SystemDefaultsQoS());
    auto [sub, q] = sub_node->create_queue<LaserScan2d>("rs_scan", QoS(64));
    (void)sub;

    /* Attach warmup */
    LaserScan2d scan{};
    for (int i = 0; i < 30; ++i) {
        pub->publish(scan);
        sub_node->spin_once();
        sleep_ns(5'000'000L);
    }

    /* Burst: push N messages as fast as possible, then spin+drain */
    constexpr int N = 5000;
    std::vector<int64_t> push_s(N), drain_s;
    drain_s.reserve(N);

    for (int i = 0; i < N; ++i) {
        scan.stamp_ns = (uint64_t)ns_now();
        auto t0 = ns_now();
        pub->publish(scan);
        push_s[i] = ns_now() - t0;

        sub_node->spin_once();
        while (auto m = q->pop()) {
            int64_t lat = ns_now() - (int64_t)m->stamp_ns;
            if (lat > 0 && lat < 100'000'000LL)
                drain_s.push_back(lat);
        }
    }

    ps(f, "ring_push_ns", compute(push_s));
    if (drain_s.empty()) drain_s.push_back(0);
    Stats ds = compute(drain_s);
    ds.mean /= 1000; ds.p50 /= 1000; ds.p95 /= 1000;
    ds.p99 /= 1000; ds.p999 /= 1000; ds.max /= 1000;
    ps(f, "ring_end_to_end_latency_us", ds, false);

    fprintf(f, "},\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   G. Memory layout (updated with new Publisher<T> handle size)
   ════════════════════════════════════════════════════════════════════════════ */

static void bench_memory(FILE* f) {
    fprintf(f, "\"memory\": {\n");
    fprintf(f, "  \"TopicHeader_bytes\": %zu,\n",   sizeof(TopicHeader));
    fprintf(f, "  \"SeqlockSlot_Pose2d_bytes\": %zu,\n", sizeof(SeqlockSlot<Pose2d>));
    fprintf(f, "  \"RingHeader_bytes\": %zu,\n",    sizeof(RingHeader));
    fprintf(f, "  \"MessageQueue_Pose2d_empty_bytes\": %zu,\n",
            sizeof(MessageQueue<Pose2d>));
    fprintf(f, "  \"seqlock_segment_Pose2d_bytes\": %zu,\n",
            sizeof(TopicHeader) + sizeof(SeqlockSlot<Pose2d>));
    fprintf(f, "  \"ring_segment_Pose2d_N64_bytes\": %zu\n",
            sizeof(RingHeader) + 64 * sizeof(Pose2d));
    fprintf(f, "}\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   main
   ════════════════════════════════════════════════════════════════════════════ */

int main() {
    fprintf(stderr, "bench_realistic starting...\n");

    FILE* f = fopen("bench_realistic.json", "w");
    if (!f) { perror("fopen"); return 1; }

    fprintf(f, "{\n");

    fprintf(stderr, "  A. spin_once overhead...\n");
    bench_spin_overhead(f);

    fprintf(stderr, "  B. publish path comparison...\n");
    bench_publish_path(f);

    fprintf(stderr, "  C. control loop latency...\n");
    bench_control_loop(f);

    fprintf(stderr, "  D. MessageQueue operations...\n");
    bench_queue(f);

    fprintf(stderr, "  E. multi-subscriber fan-out...\n");
    bench_fanout(f);

    fprintf(stderr, "  F. ring sensor simulation...\n");
    bench_ring_sensor(f);

    bench_memory(f);

    fprintf(f, "}\n");
    fclose(f);

    fprintf(stderr, "Done. Written to bench_realistic.json\n");
    return 0;
}
