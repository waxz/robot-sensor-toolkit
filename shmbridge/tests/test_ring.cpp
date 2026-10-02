/*
 * tests/test_ring.cpp — ring.hpp unit tests (platform-independent).
 *
 * Covers design_ring_zero_copy.md's phase 1 (build system + notify wiring)
 * and phase 2 (overwrite semantics, local cursors, config, type/schema
 * checks, producer-conflict detection, latest-only read) exit criteria,
 * §11.1's test table.
 *
 * Single-process, thread-based (not fork-based, so this builds on Windows):
 * publisher and subscriber each hold their own RingPublisher<T>/
 * RingSubscriber<T> instance attached to the same named segment, exactly as
 * two separate processes would. The one exception is the producer-crash
 * simulation tests (stale-PID takeover, concurrent takeover race), which
 * need a genuinely separate OS process to obtain a PID that is verifiably
 * dead -- platform::spawn_and_reap_process() (platform.hpp) provides that;
 * this file has no OS-specific code of its own, by design (platform.hpp is
 * where OS-conditional code belongs in this codebase, not scattered into
 * test files that happen to need one OS-specific capability).
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "shmbridge/messages.hpp"
#include "shmbridge/ring.hpp"

/*
 * Two tests below (stale-PID takeover, concurrent takeover race) deliberately
 * leak a RingPublisher via unique_ptr::release() to simulate a crash that
 * never reaches the destructor's close() call -- the whole point of those
 * tests. AddressSanitizer's LeakSanitizer correctly flags that leak as a
 * leak; it has no way to know it's an intentional simulation rather than a
 * bug. __lsan_ignore_object() is LSan's own documented mechanism for
 * exactly this situation (an intentionally-retained allocation), available
 * only in ASan/LSan builds -- a no-op everywhere else so this file still
 * compiles cleanly on MinGW/MSVC/plain GCC without sanitizers.
 */
#if defined(__SANITIZE_ADDRESS__)
#  define SHMBRIDGE_TEST_HAS_LSAN 1
#elif defined(__has_feature)
#  if __has_feature(address_sanitizer)
#    define SHMBRIDGE_TEST_HAS_LSAN 1
#  endif
#endif
#ifndef SHMBRIDGE_TEST_HAS_LSAN
#  define SHMBRIDGE_TEST_HAS_LSAN 0
#endif
#if SHMBRIDGE_TEST_HAS_LSAN
#  include <sanitizer/lsan_interface.h>
#endif

namespace {
void mark_intentional_test_leak(const void* ptr) {
#if SHMBRIDGE_TEST_HAS_LSAN
    __lsan_ignore_object(ptr);
#else
    (void)ptr;
#endif
}
} // namespace

using shmbridge::RingConfig;
using shmbridge::RingCursorMode;
using shmbridge::RingPublisher;
using shmbridge::RingSubscriber;
using shmbridge::Result;
using shmbridge::TimeoutError;

namespace {

/* Unique topic name per test so parallel/repeated runs never collide. */
std::string unique_topic(const char* test_name) {
    static std::atomic<uint32_t> counter{0};
    return std::string("test_ring_") + test_name + "_" +
           std::to_string(counter.fetch_add(1));
}

/*
 * Records a microsecond-precision measurement both as a GTest test
 * property (so it lands in `--gtest_output=json:...`'s per-test
 * "properties" array, the mechanism this file relies on to put real
 * numbers in shmbridge/tests/phase1_ring_build_report.md instead of a
 * coarse ms-granularity duration or a "<10ms" placeholder) and as a
 * readable assertion-failure message if the associated bound is violated.
 */
void record_latency_us(const char* key, double us) {
    std::ostringstream oss;
    oss.precision(1);
    oss << std::fixed << us;
    ::testing::Test::RecordProperty(key, oss.str());
}

/* Nanoseconds since an arbitrary steady_clock epoch -- plain int64_t so it
 * can live in a std::atomic and be shared between a producer/closer thread
 * and the main thread without a lock, for a precise "time from the actual
 * wake-causing call to pop_wait() returning" measurement that excludes
 * each test's own deliberate pre-action delay. */
int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/*
 * A single sample, or even six (the earlier revision of this file), says
 * nothing about the tail -- a realtime budget is blown by the rare slow
 * call, not the typical one. This computes the distribution a budget
 * decision actually needs: mean plus p50/p90/p99/max, linearly
 * interpolated between the bracketing order statistics (the standard
 * "nearest-rank with interpolation" definition), over whatever sample
 * count the caller collected.
 */
struct LatencyDistribution {
    double mean_us = 0.0;
    double p50_us  = 0.0;
    double p90_us  = 0.0;
    double p99_us  = 0.0;
    double max_us  = 0.0;
    size_t n       = 0;
};

double percentile(const std::vector<double>& sorted_us, double p) {
    if (sorted_us.empty()) return 0.0;
    if (sorted_us.size() == 1) return sorted_us[0];
    double rank = (p / 100.0) * static_cast<double>(sorted_us.size() - 1);
    size_t lo = static_cast<size_t>(rank);
    size_t hi = std::min(lo + 1, sorted_us.size() - 1);
    double frac = rank - static_cast<double>(lo);
    return sorted_us[lo] + frac * (sorted_us[hi] - sorted_us[lo]);
}

LatencyDistribution compute_distribution(std::vector<double> samples_us) {
    LatencyDistribution d;
    d.n = samples_us.size();
    if (samples_us.empty()) return d;
    std::sort(samples_us.begin(), samples_us.end());
    double sum = 0.0;
    for (double v : samples_us) sum += v;
    d.mean_us = sum / static_cast<double>(samples_us.size());
    d.p50_us  = percentile(samples_us, 50);
    d.p90_us  = percentile(samples_us, 90);
    d.p99_us  = percentile(samples_us, 99);
    d.max_us  = samples_us.back();
    return d;
}

void record_distribution(const char* prefix, const LatencyDistribution& d) {
    record_latency_us((std::string(prefix) + "_mean_us").c_str(), d.mean_us);
    record_latency_us((std::string(prefix) + "_p50_us").c_str(), d.p50_us);
    record_latency_us((std::string(prefix) + "_p90_us").c_str(), d.p90_us);
    record_latency_us((std::string(prefix) + "_p99_us").c_str(), d.p99_us);
    record_latency_us((std::string(prefix) + "_max_us").c_str(), d.max_us);
    ::testing::Test::RecordProperty((std::string(prefix) + "_n").c_str(),
                                     static_cast<int>(d.n));
}

constexpr int kDistributionTrials = 200;

/*
 * A freshly started process's *first* short kernel wait can return well
 * before its requested timeout elapses on Windows -- confirmed directly
 * with a minimal repro using nothing but CreateEventA/WaitForSingleObject
 * (no shmbridge code at all): a fresh process's first
 * `WaitForSingleObject(h, 10)` call returned in as little as 753us against
 * a 10ms request, every subsequent call in the same process was accurate
 * to within normal timer-resolution rounding. This is a Windows process
 * cold-start characteristic, not a bug in ring.hpp/platform.hpp's wait
 * logic -- but it would make the very first timing-sensitive assertion in
 * this binary flaky for a reason that has nothing to do with what this
 * file is testing. One throwaway wait here, before any TEST runs, absorbs
 * that one-time cost so the real assertions measure steady-state behavior
 * -- which is also what a robotics process should do in practice: warm up
 * its own timing-critical paths at startup rather than trust the very
 * first tight-timeout wait of its life.
 */
class WarmUpWaitSubsystem : public ::testing::Environment {
public:
    void SetUp() override {
        RingConfig cfg; cfg.capacity = 8;
        RingPublisher<int> pub;
        pub.open("test_ring_warmup", cfg);
        RingSubscriber<int> sub;
        sub.try_attach("test_ring_warmup", cfg);
        sub.pop_wait(5);
    }
};

const ::testing::Environment* const warm_up_env =
    ::testing::AddGlobalTestEnvironment(new WarmUpWaitSubsystem);

} // namespace

/* ── basic correctness (F-1, F-2) ─────────────────────────────────────────── */

TEST(RingBasic, PushPopRoundTrip) {
    auto topic = unique_topic("push_pop");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));

    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    ASSERT_TRUE(pub.push(42));
    auto item = sub.pop_ex();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value, 42);
    EXPECT_FALSE(sub.pop_ex().has_value());
}

/*
 * Overflow/eviction (F-1, §5.2, §5.7): pushing past capacity without
 * reading advances start_idx correctly, and a subsequent read resyncs to
 * the oldest still-valid slot rather than returning corrupted data or
 * failing (unlike the old reject-on-full ring this design replaces).
 */
TEST(RingBasic, OverflowEvictionResyncsToOldestLive) {
    auto topic = unique_topic("overflow");
    RingConfig cfg; cfg.capacity = 4;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    /* Push 10 items into a 4-slot ring without reading -- every push must
     * still succeed (F-1), and only the newest 4 remain live. */
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(pub.push(i));
    }

    std::vector<int> received;
    while (auto item = sub.pop_ex()) received.push_back(item->value);

    EXPECT_EQ(received, (std::vector<int>{6, 7, 8, 9}));
}

/*
 * Multi-consumer independence (F-2): two subscribers at different poll
 * rates never interfere with each other's cursor -- each sees every
 * message still live when it reads, independent of the other's progress.
 */
TEST(RingBasic, MultiConsumerIndependence) {
    auto topic = unique_topic("multi_consumer");
    RingConfig cfg; cfg.capacity = 16;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> fast_sub, slow_sub;
    ASSERT_TRUE(fast_sub.try_attach(topic.c_str(), cfg));
    ASSERT_TRUE(slow_sub.try_attach(topic.c_str(), cfg));

    for (int i = 0; i < 5; ++i) pub.push(i);

    std::vector<int> fast_received;
    while (auto item = fast_sub.pop_ex()) fast_received.push_back(item->value);
    EXPECT_EQ(fast_received, (std::vector<int>{0, 1, 2, 3, 4}));

    for (int i = 5; i < 10; ++i) pub.push(i);

    std::vector<int> slow_received;
    while (auto item = slow_sub.pop_ex()) slow_received.push_back(item->value);
    EXPECT_EQ(slow_received, (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));

    std::vector<int> fast_received2;
    while (auto item = fast_sub.pop_ex()) fast_received2.push_back(item->value);
    EXPECT_EQ(fast_received2, (std::vector<int>{5, 6, 7, 8, 9}));
}

/*
 * Boundary-value correctness (R-4): OverflowEvictionResyncsToOldestLive
 * above only exercises the cursor-far-behind-start resync path (next_read_
 * well below start_idx after a burst of unread pushes). This test instead
 * pins next_read_ exactly at start_idx (the oldest still-live slot -- must
 * succeed, not be treated as evicted) and exactly one below it (must
 * resync, not read one-past-evicted data), directly exercising the
 * `next_read_ < start` comparison's off-by-one edges rather than a
 * scenario loose enough to pass even with `<=` swapped for `<` by mistake.
 */
TEST(RingBasic, PopExBoundaryAtExactlyStartIdx) {
    auto topic = unique_topic("boundary_start_idx");
    RingConfig cfg; cfg.capacity = 4;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    /* Fill exactly to capacity: start_idx=0, end_idx=4, nothing evicted yet. */
    for (int i = 0; i < 4; ++i) EXPECT_TRUE(pub.push(i));

    /* Consume one: next_read_ becomes 1. */
    auto first = sub.pop_ex();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->value, 0);

    /* One more push evicts index 0: start_idx becomes 1, end_idx becomes 5.
     * next_read_ (1) now sits EXACTLY at start_idx (1) -- the live boundary,
     * not one below it. This must read index 1 directly, not resync. */
    ASSERT_TRUE(pub.push(4));
    auto exact_boundary = sub.pop_ex();
    ASSERT_TRUE(exact_boundary.has_value());
    EXPECT_EQ(exact_boundary->value, 1)
        << "cursor exactly at start_idx must be read directly, not treated as evicted";

    /* Two more pushes advance start_idx to 3 (evicting index 2, which
     * next_read_ still points at). next_read_ (2) now sits exactly ONE
     * BELOW start_idx (3) -- this must resync to start_idx, not read the
     * already-evicted index 2. */
    ASSERT_TRUE(pub.push(5));
    ASSERT_TRUE(pub.push(6));
    auto resynced = sub.pop_ex();
    ASSERT_TRUE(resynced.has_value());
    EXPECT_EQ(resynced->value, 3)
        << "cursor one below start_idx must resync to start_idx, not read evicted data";
}

/* ── zero-copy write/read (F-4, F-5) ──────────────────────────────────────── */

TEST(RingZeroCopy, ReserveCommitMatchesPush) {
    auto topic = unique_topic("reserve_commit");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    int* slot = pub.reserve();
    ASSERT_NE(slot, nullptr);
    *slot = 123;
    pub.commit();

    auto item = sub.pop_ex();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value, 123);
}

TEST(RingZeroCopy, BorrowEndBorrowMatchesPopEx) {
    auto topic = unique_topic("borrow");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    ASSERT_TRUE(pub.push(77));
    uint64_t write_ns = 0;
    const int* borrowed = sub.borrow(&write_ns);
    ASSERT_NE(borrowed, nullptr);
    EXPECT_EQ(*borrowed, 77);
    EXPECT_GT(write_ns, 0u);
    EXPECT_TRUE(sub.end_borrow());
    EXPECT_FALSE(sub.pop_ex().has_value()); /* cursor advanced by end_borrow() */
}

TEST(RingMisuseGuards, ReserveCommitMisuseIsSafeNoOp) {
    auto topic = unique_topic("reserve_misuse");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    pub.commit(); /* no prior reserve() -- documented no-op, never publishes garbage */
    EXPECT_FALSE(sub.pop_ex().has_value());

    int* first = pub.reserve();
    ASSERT_NE(first, nullptr);
    *first = 1;
    EXPECT_EQ(pub.reserve(), nullptr); /* second reserve() before commit() -- rejected (F-14) */
    pub.commit();

    auto item = sub.pop_ex();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value, 1); /* the first reservation was never corrupted */
}

TEST(RingMisuseGuards, BorrowEndBorrowMisuseIsSafeNoOp) {
    auto topic = unique_topic("borrow_misuse");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    EXPECT_FALSE(sub.end_borrow()); /* no prior borrow() -- no-op, cursor unchanged */

    ASSERT_TRUE(pub.push(5));
    ASSERT_NE(sub.borrow(), nullptr);
    EXPECT_EQ(sub.borrow(), nullptr); /* second borrow() before end_borrow() -- rejected (F-14) */
    EXPECT_TRUE(sub.end_borrow());
}

/* ── freshness (F-6) ──────────────────────────────────────────────────────── */

TEST(RingFreshness, IsStaleBoundary) {
    Result<int> r;
    r.write_ns = shmbridge::platform::now_ns();
    EXPECT_FALSE(r.is_stale(1000.0)); /* just written: not stale against a generous 1s bound */

    r.write_ns = shmbridge::platform::now_ns() - static_cast<uint64_t>(50e6); /* 50ms ago */
    EXPECT_TRUE(r.is_stale(10.0));   /* 50ms old vs. a 10ms bound: stale */
    EXPECT_FALSE(r.is_stale(1000.0)); /* 50ms old vs. a 1s bound: not stale */
}

/* ── config resolution (F-8) ──────────────────────────────────────────────── */

TEST(RingConfigResolution, TopicOverridesDefaultOverridesBuiltin) {
    std::string path = "test_ring_config_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".toml";
    {
        std::ofstream f(path);
        f << "[ring.default]\n"
             "capacity = 128\n"
             "max_retries = 7\n"
             "\n"
             "[ring.\"/robot/special\"]\n"
             "capacity = 256\n"
             "cursor_mode = \"drain_backlog\"\n";
    }

    RingConfig default_topic = shmbridge::resolve_ring_config("/robot/other", path);
    EXPECT_EQ(default_topic.capacity, 128u);     /* from [ring.default] */
    EXPECT_EQ(default_topic.max_retries, 7u);    /* from [ring.default] */
    EXPECT_EQ(default_topic.cursor_mode, RingCursorMode::StartNow); /* built-in default */

    RingConfig special_topic = shmbridge::resolve_ring_config("/robot/special", path);
    EXPECT_EQ(special_topic.capacity, 256u);     /* overridden by [ring."/robot/special"] */
    EXPECT_EQ(special_topic.cursor_mode, RingCursorMode::DrainBacklog);
    EXPECT_EQ(special_topic.max_retries, 7u);    /* still inherited from [ring.default] */

    std::remove(path.c_str());
}

TEST(RingConfigResolution, MissingFileFallsBackToBuiltinDefaults) {
    RingConfig cfg = shmbridge::resolve_ring_config("/anything", "this_file_does_not_exist.toml");
    EXPECT_EQ(cfg.capacity, 64u);
    EXPECT_EQ(cfg.max_retries, 4u);
}

/* ── type/schema safety (F-7, F-10) ───────────────────────────────────────── */

TEST(RingTypeSafety, MismatchedTypeThrowsInvalidArgument) {
    auto topic = unique_topic("type_mismatch");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));

    RingSubscriber<double> wrong_sub; /* different T -- different type_hash */
    EXPECT_THROW(wrong_sub.try_attach(topic.c_str(), cfg), std::invalid_argument);
}

/* ── resilient attach / reattach (F-9) ────────────────────────────────────── */

TEST(RingResilience, TryAttachReturnsFalseBeforePublisherExists) {
    auto topic = unique_topic("resilient_attach");
    RingConfig cfg; cfg.capacity = 8;
    RingSubscriber<int> sub;
    EXPECT_FALSE(sub.try_attach(topic.c_str(), cfg)); /* not found yet -- never throws */

    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    EXPECT_TRUE(sub.try_attach(topic.c_str(), cfg)); /* now succeeds */
}

TEST(RingResilience, AttachThrowsTimeoutErrorWhenNoPublisherAppears) {
    auto topic = unique_topic("attach_timeout");
    RingConfig cfg; cfg.capacity = 8;
    RingSubscriber<int> sub;
    EXPECT_THROW(sub.attach(topic.c_str(), 20.0, cfg), TimeoutError);
}

TEST(RingResilience, ReattachAfterCloseGetsFreshCursor) {
    auto topic = unique_topic("reattach_after_close");
    RingConfig cfg; cfg.capacity = 8;

    auto pub1 = std::make_unique<RingPublisher<int>>();
    ASSERT_TRUE(pub1->open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));
    ASSERT_TRUE(pub1->push(1));
    ASSERT_TRUE(sub.pop_ex().has_value());

    pub1->close();
    EXPECT_TRUE(sub.is_closed());
    sub.detach(); /* mandatory full detach()/re-attach() cycle (§5.8) */

    auto pub2 = std::make_unique<RingPublisher<int>>();
    ASSERT_TRUE(pub2->open(topic.c_str(), cfg));
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));
    ASSERT_TRUE(pub2->push(2));
    auto item = sub.pop_ex();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value, 2);
}

/* ── pop_latest (F-13) ────────────────────────────────────────────────────── */

TEST(RingLatestOnly, PopLatestJumpsToNewestDiscardingBetween) {
    auto topic = unique_topic("pop_latest");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    for (int i = 0; i < 5; ++i) pub.push(i);

    auto item = sub.pop_latest();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value, 4); /* newest, not oldest */
    EXPECT_FALSE(sub.pop_ex().has_value()); /* everything in between was discarded */
}

/* ── producer conflict detection (F-11, §5.10) ────────────────────────────── */

TEST(RingProducerConflict, SecondOpenByLiveProducerThrows) {
    auto topic = unique_topic("producer_conflict");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub1;
    ASSERT_TRUE(pub1.open(topic.c_str(), cfg));

    RingPublisher<int> pub2;
    EXPECT_THROW(pub2.open(topic.c_str(), cfg), std::runtime_error);
}

TEST(RingProducerConflict, OpenAfterCloseSucceedsCleanly) {
    auto topic = unique_topic("producer_reopen");
    RingConfig cfg; cfg.capacity = 8;
    auto pub1 = std::make_unique<RingPublisher<int>>();
    ASSERT_TRUE(pub1->open(topic.c_str(), cfg));
    pub1->close();

    RingPublisher<int> pub2;
    EXPECT_TRUE(pub2.open(topic.c_str(), cfg));
}

/*
 * Stale-PID takeover (R-14/R-17, §5.10): a producer whose process is
 * verifiably dead (not a mock -- a real spawned-and-reaped child process,
 * see platform::spawn_and_reap_process()) allows a new open() to take over
 * rather than throwing, since claim_producer_slot() checks process_alive()
 * before concluding "conflict."
 */
TEST(RingProducerConflict, StalePidFromDeadProcessIsTakenOver) {
    auto topic = unique_topic("stale_pid_takeover");
    RingConfig cfg; cfg.capacity = 8;

    /* Simulate "a producer crashed without close()": open and push once,
     * then forge producer_pid to a real, now-dead PID without calling
     * close() (which would clear producer_active and defeat the point). */
    auto pub1 = std::make_unique<RingPublisher<int>>();
    ASSERT_TRUE(pub1->open(topic.c_str(), cfg));
    ASSERT_TRUE(pub1->push(1));

    uint32_t dead_pid = shmbridge::platform::spawn_and_reap_process();
    ASSERT_NE(dead_pid, 0u);
    EXPECT_FALSE(shmbridge::platform::process_alive(dead_pid));

    mark_intentional_test_leak(pub1.get());
    pub1.release(); /* leak on purpose: skip the destructor's close() call
                        to simulate a crash that never reaches it */

    /* Forge producer_pid via a second raw attach to the same segment --
     * white-box, test-only: no public API exposes a mutable header
     * reference, by design (production code has no reason to forge this). */
    const std::size_t seg_size =
        sizeof(shmbridge::RingHeader) + static_cast<std::size_t>(cfg.capacity) * sizeof(shmbridge::Slot<int>);
    void* raw = shmbridge::platform::shm_attach("/sbr_" + topic, seg_size);
    auto* hdr = static_cast<shmbridge::RingHeader*>(raw);
    ASSERT_EQ(hdr->producer_active.load(), 1u) << "precondition: still marked active, as a crash would leave it";
    hdr->producer_pid.store(dead_pid, std::memory_order_release);
    shmbridge::platform::shm_unmap(raw, seg_size);

    RingPublisher<int> pub2;
    EXPECT_TRUE(pub2.open(topic.c_str(), cfg)); /* takes over instead of throwing */
    EXPECT_TRUE(pub2.push(2)); /* fully functional afterward */
}

/*
 * Concurrent takeover race (R-17): two threads racing claim_producer_slot()
 * against the same stale (simulated-crashed) producer slot -- exactly one
 * must succeed, the other must observe it as a live conflict and throw,
 * never both silently "succeeding" (which would reintroduce the
 * dual-producer corruption F-11 exists to prevent).
 */
TEST(RingProducerConflict, ConcurrentTakeoverRaceHasExactlyOneWinner) {
    auto topic = unique_topic("concurrent_takeover");
    RingConfig cfg; cfg.capacity = 8;

    auto pub1 = std::make_unique<RingPublisher<int>>();
    ASSERT_TRUE(pub1->open(topic.c_str(), cfg));
    uint32_t dead_pid = shmbridge::platform::spawn_and_reap_process();
    ASSERT_NE(dead_pid, 0u);
    mark_intentional_test_leak(pub1.get());
    pub1.release(); /* simulate crash: skip close() */

    const std::size_t seg_size =
        sizeof(shmbridge::RingHeader) + static_cast<std::size_t>(cfg.capacity) * sizeof(shmbridge::Slot<int>);
    void* raw = shmbridge::platform::shm_attach("/sbr_" + topic, seg_size);
    static_cast<shmbridge::RingHeader*>(raw)->producer_pid.store(dead_pid, std::memory_order_release);
    shmbridge::platform::shm_unmap(raw, seg_size);

    std::atomic<int> successes{0};
    std::atomic<int> failures{0};
    std::atomic<int> ready_count{0};
    std::mutex winners_mtx;
    std::vector<std::unique_ptr<RingPublisher<int>>> winners;

    /*
     * A winner calling close() immediately (inside the race lambda, before
     * the other thread even attempts open()) would let the second thread
     * legitimately win a *fresh* claim against a since-cleared
     * producer_active -- a correct outcome for that (no longer racing)
     * situation, but not a test of the actual concurrent-takeover race
     * this test exists to exercise. Spin-waiting until both threads are
     * ready to call open(), and deferring close() until after both have
     * finished and been asserted on, keeps the two open() calls genuinely
     * overlapping.
     */
    auto race = [&] {
        auto pub = std::make_unique<RingPublisher<int>>();
        ready_count.fetch_add(1, std::memory_order_release);
        while (ready_count.load(std::memory_order_acquire) < 2) { /* spin */ }
        try {
            if (pub->open(topic.c_str(), cfg)) {
                successes.fetch_add(1);
                std::lock_guard<std::mutex> lk(winners_mtx);
                winners.push_back(std::move(pub));
            }
        } catch (const std::runtime_error&) {
            failures.fetch_add(1);
        }
    };

    std::thread t1(race), t2(race);
    t1.join();
    t2.join();

    EXPECT_EQ(successes.load(), 1) << "exactly one racer should win the takeover";
    EXPECT_EQ(failures.load(), 1) << "the loser should observe the winner as a live conflict, not also succeed";

    for (auto& w : winners) w->close();
}

/* ── blocking wait (F-3) ──────────────────────────────────────────────────── */

/*
 * The 500ms bound this test used to assert was loose enough that it
 * couldn't tell "returned immediately via the pop() fast path" apart from
 * "woke after one Windows notify_wait slice" (platform.hpp's kSliceMs is
 * 15ms) -- a real but much smaller-magnitude regression than "blocked for
 * (most of) the timeout". A 10-second timeout plus a tight ~50ms bound
 * (still generous for scheduler/CI jitter around an operation that is, on
 * the fast path, O(1) memcpy and atomic loads with no syscall at all) makes
 * both classes of regression fail this test instead of only the gross one.
 */
TEST(RingBlockingWait, PopWaitReturnsImmediatelyWhenDataAlreadyPresent) {
    auto topic = unique_topic("wait_immediate");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    ASSERT_TRUE(pub.push(7));
    auto start = std::chrono::steady_clock::now();
    auto item = sub.pop_wait(/*timeout_ms=*/10000);
    auto elapsed = std::chrono::steady_clock::now() - start;
    double elapsed_us =
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() / 1e3;
    record_latency_us("immediate_read_us", elapsed_us);

    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value, 7);
    EXPECT_LT(elapsed, std::chrono::milliseconds(50))
        << "pop_wait() took " << elapsed_us
        << "us to return data that was already available before it was called -- "
           "it should have taken the pop_ex() fast path, not gone anywhere near notify_wait()";
}

/*
 * The single-call case above could in principle pass by luck (one fast
 * notify_wait slice). Calling pop_wait() back-to-back for N already-published
 * items is a much stronger signal: if any call actually blocked instead of
 * taking the fast path, the cumulative elapsed time across N calls would
 * show it immediately, and the items must still come back in publish order.
 */
TEST(RingBlockingWait, PopWaitReturnsImmediatelyForEveryAlreadyPublishedItem) {
    auto topic = unique_topic("wait_immediate_multi");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    constexpr int kCount = 5;
    for (int i = 0; i < kCount; ++i) ASSERT_TRUE(pub.push(i * 10));

    auto start = std::chrono::steady_clock::now();
    std::vector<int> received;
    for (int i = 0; i < kCount; ++i) {
        auto item = sub.pop_wait(/*timeout_ms=*/10000);
        ASSERT_TRUE(item.has_value()) << "call #" << i;
        received.push_back(item->value);
    }
    auto elapsed = std::chrono::steady_clock::now() - start;
    double elapsed_us =
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() / 1e3;
    record_latency_us("immediate_read_5_calls_total_us", elapsed_us);
    record_latency_us("immediate_read_5_calls_mean_us", elapsed_us / kCount);

    EXPECT_EQ(received, (std::vector<int>{0, 10, 20, 30, 40}));
    EXPECT_LT(elapsed, std::chrono::milliseconds(50))
        << kCount << " pop_wait() calls against already-published data took "
        << elapsed_us << "us total";
}

/*
 * A robotics control loop can't tolerate a 100ms-class timeout anywhere
 * near its hot path, so this is deliberately a short, realistic one
 * (10ms) rather than the comfortably-slow 100ms this test used to request
 * -- if `pop_wait()` can't honor a 10ms timeout reasonably tightly, that's
 * exactly the kind of thing a robotics-facing design plan needs to know
 * about, not hide behind a loose request nothing would ever actually use.
 */
TEST(RingBlockingWait, PopWaitTimesOutWhenNoDataArrives) {
    auto topic = unique_topic("wait_timeout");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    auto start = std::chrono::steady_clock::now();
    auto item = sub.pop_wait(/*timeout_ms=*/10);
    auto elapsed = std::chrono::steady_clock::now() - start;
    record_latency_us("timeout_10ms_actual_us",
                       std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() / 1e3);

    EXPECT_FALSE(item.has_value());
    /* Finite: returns at/after roughly the requested timeout, not forever,
     * and never wildly short of it. Bounds are deliberately not tight
     * against the nominal 10ms on either side: Windows' own timer/wait
     * subsystem has observed, OS-level imprecision here, independent of
     * this design's own logic -- see design_ring_zero_copy.md's R-18 and
     * NFR-6 for the (more severe, process-cold-start) version of this same
     * category of finding from phase 1, confirmed there with a bare
     * CreateEventA/WaitForSingleObject repro having nothing to do with
     * shmbridge. This test has shown a milder instance of the same
     * category after heavy thread churn from the producer-conflict-race
     * tests immediately before it (observed as low as ~7ms against the
     * 10ms request, vs. phase 1's <1ms-on-a-fresh-process cases) -- a
     * [5ms, 200ms) window absorbs that jitter without turning a real
     * regression (e.g. not waiting at all) into a silent pass. */
    EXPECT_GE(elapsed, std::chrono::milliseconds(5));
    EXPECT_LT(elapsed, std::chrono::milliseconds(200));
}

/*
 * R-3: a publish landing between the subscriber's data-check and its wait
 * call must still be observed promptly, not missed until the timeout. This
 * is exactly what the notify_seq-snapshot-before-check ordering in
 * pop_wait() exists to guarantee.
 *
 * push() never destroys anything, so the publisher, the subscriber, and
 * the signaling thread are all created ONCE and reused across every trial
 * -- deliberately, to isolate the ring's own wake-path latency from the
 * OS overhead of creating a fresh std::thread 200 times over.
 */
TEST(RingLatencyDistribution, PushToWakeLatencyDistribution) {
    auto topic = unique_topic("dist_push_wake");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    std::mutex mtx;
    std::condition_variable cv;
    int go_trial = -1;
    bool stop = false;
    std::atomic<int64_t> push_time_ns{0};

    std::thread producer([&] {
        std::unique_lock<std::mutex> lk(mtx);
        int last_seen = -1;
        for (;;) {
            cv.wait(lk, [&] { return stop || go_trial != last_seen; });
            if (stop) return;
            last_seen = go_trial;
            lk.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            push_time_ns.store(now_ns(), std::memory_order_release);
            pub.push(last_seen);
            lk.lock();
        }
    });

    std::vector<double> samples_us;
    samples_us.reserve(kDistributionTrials);
    for (int i = 0; i < kDistributionTrials; ++i) {
        {
            std::lock_guard<std::mutex> lk(mtx);
            go_trial = i;
        }
        cv.notify_one();

        auto item = sub.pop_wait(/*timeout_ms=*/5000);
        int64_t wake_time_ns = now_ns();
        ASSERT_TRUE(item.has_value()) << "trial #" << i;

        samples_us.push_back(static_cast<double>(
            wake_time_ns - push_time_ns.load(std::memory_order_acquire)) / 1e3);
    }
    {
        std::lock_guard<std::mutex> lk(mtx);
        stop = true;
    }
    cv.notify_one();
    producer.join();

    auto dist = compute_distribution(samples_us);
    record_distribution("push_to_wake", dist);
    /* 1ms: comfortably inside a 1kHz control loop's own period, the
     * tightest of the three cases evaluated in the build report. */
    EXPECT_LT(dist.p99_us, 1000.0)
        << "p99 push()-to-wake latency was " << dist.p99_us
        << "us over " << dist.n << " trials -- exceeds a 1ms realtime budget";
}

/* ── close() wakes a blocked reader (R-5) ─────────────────────────────────── */

TEST(RingBlockingWait, CloseWakesBlockedReaderNearImmediately) {
    auto topic = unique_topic("close_wakes");
    RingConfig cfg; cfg.capacity = 8;
    auto pub = std::make_unique<RingPublisher<int>>();
    ASSERT_TRUE(pub->open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    std::atomic<int64_t> close_time_ns{0};
    std::thread closer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        close_time_ns.store(now_ns(), std::memory_order_release);
        pub->close();
    });

    /* Long timeout: if close() didn't wake the reader, this test would take
     * the full timeout to pass instead of failing — that's the R-5 bug this
     * guards against. */
    auto item = sub.pop_wait(/*timeout_ms=*/5000);
    int64_t wake_time_ns = now_ns();
    closer.join();

    EXPECT_FALSE(item.has_value()); /* closed with nothing published */
    EXPECT_TRUE(sub.is_closed());
    double wake_latency_us =
        static_cast<double>(wake_time_ns - close_time_ns.load(std::memory_order_acquire)) / 1e3;
    record_latency_us("close_to_wake_latency_us", wake_latency_us);
    EXPECT_LT(wake_latency_us, 10'000.0)
        << "close()-to-wake latency was " << (wake_latency_us / 1e3)
        << "ms -- too slow for a realtime robotics control loop";
}

/*
 * close() destroys the publisher's segment, so (unlike push-to-wake above)
 * a fresh RingPublisher/RingSubscriber pair is unavoidable each trial --
 * but the signaling thread is still created once and reused.
 */
TEST(RingLatencyDistribution, CloseToWakeLatencyDistribution) {
    std::mutex mtx;
    std::condition_variable cv;       /* signals "go" to the closer thread */
    std::condition_variable done_cv;  /* signals "close() fully returned" back */
    int go_trial = -1;
    int done_trial = -1;
    bool stop = false;
    std::atomic<int64_t> close_time_ns{0};
    RingPublisher<int>* pub_to_close = nullptr;

    std::thread closer([&] {
        int last_seen = -1;
        for (;;) {
            RingPublisher<int>* target;
            {
                std::unique_lock<std::mutex> lk(mtx);
                cv.wait(lk, [&] { return stop || go_trial != last_seen; });
                if (stop) return;
                last_seen = go_trial;
                target = pub_to_close;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            close_time_ns.store(now_ns(), std::memory_order_release);
            /*
             * close()'s own signal_closed() call wakes the main thread's
             * pop_wait() the instant the "closed" flag is set -- well
             * before close() itself returns (shm_unmap/shm_destroy still
             * run after). The main thread must not let `pub` be destroyed
             * (its unique_ptr going out of scope) until this call has
             * fully returned, or it's a use-after-free in the test
             * harness (not in ring.hpp) -- done_cv below is what the main
             * thread waits on instead.
             */
            target->close();
            {
                std::lock_guard<std::mutex> lk(mtx);
                done_trial = last_seen;
            }
            done_cv.notify_one();
        }
    });

    RingConfig cfg; cfg.capacity = 8;
    std::vector<double> samples_us;
    samples_us.reserve(kDistributionTrials);
    for (int i = 0; i < kDistributionTrials; ++i) {
        auto topic = unique_topic("dist_close_wake");
        auto pub = std::make_unique<RingPublisher<int>>();
        ASSERT_TRUE(pub->open(topic.c_str(), cfg));
        RingSubscriber<int> sub;
        ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

        {
            std::lock_guard<std::mutex> lk(mtx);
            pub_to_close = pub.get();
            go_trial = i;
        }
        cv.notify_one();

        auto item = sub.pop_wait(/*timeout_ms=*/5000);
        int64_t wake_time_ns = now_ns();
        ASSERT_FALSE(item.has_value()) << "trial #" << i;

        samples_us.push_back(static_cast<double>(
            wake_time_ns - close_time_ns.load(std::memory_order_acquire)) / 1e3);

        std::unique_lock<std::mutex> lk(mtx);
        done_cv.wait(lk, [&] { return done_trial == i; });
    }
    {
        std::lock_guard<std::mutex> lk(mtx);
        stop = true;
    }
    cv.notify_one();
    closer.join();

    auto dist = compute_distribution(samples_us);
    record_distribution("close_to_wake", dist);
    EXPECT_LT(dist.p99_us, 1000.0)
        << "p99 close()-to-wake latency was " << dist.p99_us
        << "us over " << dist.n << " trials -- exceeds a 1ms realtime budget";
}

/* ── drain_ex ─────────────────────────────────────────────────────────────── */

TEST(RingBasic, DrainExConsumesEverythingAvailable) {
    auto topic = unique_topic("drain_ex");
    RingConfig cfg; cfg.capacity = 8;
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    ASSERT_TRUE(pub.push(1));
    ASSERT_TRUE(pub.push(2));
    ASSERT_TRUE(pub.push(3));

    std::vector<int> received;
    uint32_t count = sub.drain_ex([&](const int& v, uint64_t) { received.push_back(v); });

    EXPECT_EQ(count, 3u);
    EXPECT_EQ(received, (std::vector<int>{1, 2, 3}));
}

/* ── latency distributions (p50/p90/p99/max, not a single sample) ───────────
 *
 * A single measurement -- even six or seven of them -- says nothing about
 * the tail, and a realtime budget is blown by the rare slow call, not the
 * typical one. These tests run kDistributionTrials (200) independent
 * trials of each timing-sensitive path and record the full distribution,
 * so headroom against a stated budget can be computed from p99 (what a
 * control loop running at some rate will actually see almost all the
 * time) rather than from a mean that a slow tail can hide behind.
 *
 * Budgets asserted here are deliberately loose (not hair-trigger), same
 * as the single-sample tests above -- see
 * shmbridge/tests/phase1_ring_build_report.md for the real p99 numbers
 * and the headroom computed from them against several concrete control-
 * loop rates (1kHz/100Hz/10Hz), which is the number that actually matters
 * for a capacity decision, not the pass/fail bound.
 */

TEST(RingLatencyDistribution, ImmediateReadDistribution) {
    RingConfig cfg; cfg.capacity = 8;
    std::vector<double> samples_us;
    samples_us.reserve(kDistributionTrials);
    for (int i = 0; i < kDistributionTrials; ++i) {
        auto topic = unique_topic("dist_immediate");
        RingPublisher<int> pub;
        ASSERT_TRUE(pub.open(topic.c_str(), cfg));
        RingSubscriber<int> sub;
        ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

        ASSERT_TRUE(pub.push(i));
        auto start = std::chrono::steady_clock::now();
        auto item = sub.pop_wait(/*timeout_ms=*/10000);
        auto elapsed = std::chrono::steady_clock::now() - start;
        ASSERT_TRUE(item.has_value()) << "trial #" << i;
        samples_us.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() / 1e3);
    }

    auto dist = compute_distribution(samples_us);
    record_distribution("immediate_read", dist);
    EXPECT_LT(dist.p99_us, 50.0)
        << "p99 immediate-read latency was " << dist.p99_us
        << "us over " << dist.n << " trials -- the pop_ex() fast path should never "
           "approach this regardless of OS scheduling jitter";
}

/* ── torn-read retry under real contention (R-1, R-4) ─────────────────────── */

/*
 * The boundary test above exercises pop_ex()'s start_idx comparison with
 * controlled, single-threaded timing. This test instead forces the retry
 * path (§5.2, R-1) to fire for real: a tiny 2-slot ring with a writer
 * thread pushing as fast as possible gives every pop_ex() call a good
 * chance of racing an eviction mid-copy.
 *
 * What this can and can't prove: R-2 (§7) explicitly accepts that the
 * bulk `memcpy` of a slot the writer may concurrently overwrite is
 * undefined behavior in the strict C++ memory-model sense, relying on the
 * start_idx before/after check to discard the overwhelming majority of
 * torn reads -- "empirically low... never observed to corrupt beyond
 * recognition," not formally proven impossible. A hard "values must
 * always be strictly increasing" assertion is therefore testing a
 * stronger guarantee than the design actually makes: at this
 * deliberately adversarial capacity (2, chosen to maximize eviction
 * races), an occasional non-monotonic value slipping past the
 * before/after check is R-2 itself showing up, not a new defect this
 * test discovered. (It did show up at least once while writing this
 * test, at the default max_retries=4 and this adversarial capacity --
 * recorded as direct evidence R-2's rarity claim, not proof of zero.)
 * What this test actually gates on: the reader makes real progress and
 * finishes promptly (R-1's retry bound doesn't stall it indefinitely),
 * and any non-monotonic anomaly stays rare -- matching R-2's own
 * "empirically low" characterization with a real measured rate instead
 * of leaving that claim unverified.
 */
TEST(RingTornReadRetry, HighContentionStaysLiveWithRareR2Anomalies) {
    auto topic = unique_topic("torn_read_contention");
    RingConfig cfg; cfg.capacity = 2; /* deliberately tiny: maximizes eviction races */
    RingPublisher<int> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));
    RingSubscriber<int> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    constexpr int N = 20000;
    std::thread writer([&] {
        for (int i = 0; i < N; ++i) pub.push(i);
    });

    std::vector<int> received;
    received.reserve(N);
    auto start = std::chrono::steady_clock::now();
    while (received.empty() || received.back() != N - 1) {
        if (auto item = sub.pop_ex()) received.push_back(item->value);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) break;
    }
    writer.join();

    ASSERT_FALSE(received.empty());
    EXPECT_EQ(received.back(), N - 1) << "reader never caught up to the last pushed value";

    std::size_t anomalies = 0;
    for (std::size_t i = 1; i < received.size(); ++i) {
        if (received[i - 1] >= received[i]) ++anomalies;
    }
    const double anomaly_pct =
        received.size() > 1 ? 100.0 * static_cast<double>(anomalies) / static_cast<double>(received.size() - 1) : 0.0;
    record_latency_us("torn_read_anomaly_pct", anomaly_pct);
    EXPECT_LT(anomaly_pct, 1.0)
        << anomalies << "/" << received.size() << " (" << anomaly_pct << "%) non-monotonic "
           "reads -- R-2 accepts this as a rare possibility, not this common";

    const auto& stats = sub.stats();
    EXPECT_GT(stats.total_reads, 0u);
    /* Not asserting retried_reads/exhausted_reads > 0: real but
     * non-deterministic under this test's timing, recorded for
     * NFR-4/the performance report rather than gated on here (a
     * scheduler that never interleaves the writer mid-copy on a given
     * run would make a > 0 assertion flaky, not meaningfully stronger). */
    RecordProperty("torn_read_total_reads", static_cast<int>(stats.total_reads));
    RecordProperty("torn_read_retried_reads", static_cast<int>(stats.retried_reads));
    RecordProperty("torn_read_exhausted_reads", static_cast<int>(stats.exhausted_reads));
}

/* ── RingConfig TOML field coverage (R-10, R-11, R-12) ────────────────────── */

/*
 * TopicOverridesDefaultOverridesBuiltin (above) only exercises 3 of
 * RingConfig's 7 TOML-resolvable fields (capacity, cursor_mode,
 * max_retries). This test covers the remaining 4 -- max_age_ms (R-11/R-12's
 * per-topic staleness knob), warn_every_ms (R-10's log-rate-limit knob),
 * max_takeover_attempts (F-11/R-17's takeover-retry bound), and
 * attach_retry_ms (§5.8's resilient-attach poll cadence) -- so every
 * field `detail::apply_ring_config_table()` recognizes has at least one
 * test confirming it actually resolves from a TOML file, not just that it
 * exists as a struct member with the right compiled-in default.
 */
TEST(RingConfigResolution, RemainingFieldsResolveFromToml) {
    std::string path = "test_ring_config_remaining_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".toml";
    {
        std::ofstream f(path);
        f << "[ring.default]\n"
             "max_age_ms = 250.5\n"
             "warn_every_ms = 1000.0\n"
             "max_takeover_attempts = 9\n"
             "attach_retry_ms = 42.0\n";
    }

    RingConfig cfg = shmbridge::resolve_ring_config("/any/topic", path);
    EXPECT_DOUBLE_EQ(cfg.max_age_ms, 250.5);
    EXPECT_DOUBLE_EQ(cfg.warn_every_ms, 1000.0);
    EXPECT_EQ(cfg.max_takeover_attempts, 9u);
    EXPECT_DOUBLE_EQ(cfg.attach_retry_ms, 42.0);

    std::remove(path.c_str());
}

/* ── NFR-1: blocked pop_wait() vs. busy-spin CPU usage ───────────────────── */

/*
 * NFR-1 claims a subscriber blocked in pop_wait() with no new data uses
 * ~0% CPU, against a busy-spin baseline of ~100% of one core. This measures
 * both directly via platform::thread_cpu_time_ns() over the same
 * wall-clock window rather than asserting the qualitative claim untested:
 * a blocked pop_wait() loop and a tight non-blocking pop_ex() loop, each
 * run for the same duration with no publisher ever providing data, should
 * show a large, unambiguous gap in CPU time consumed.
 */
TEST(RingPerformance, BlockedSubscriberUsesFarLessCpuThanBusySpin) {
    RingConfig cfg; cfg.capacity = 8;
    constexpr auto kWindow = std::chrono::milliseconds(200);

    double blocked_cpu_pct = 0.0, busyspin_cpu_pct = 0.0;

    {
        auto topic = unique_topic("perf_cpu_blocked");
        RingPublisher<int> pub;
        ASSERT_TRUE(pub.open(topic.c_str(), cfg));
        RingSubscriber<int> sub;
        ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

        uint64_t cpu0 = shmbridge::platform::thread_cpu_time_ns();
        auto wall0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - wall0 < kWindow) {
            sub.pop_wait(10); /* always returns nullopt -- nothing is ever published */
        }
        uint64_t cpu1 = shmbridge::platform::thread_cpu_time_ns();
        blocked_cpu_pct = 100.0 * static_cast<double>(cpu1 - cpu0) /
                          static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(kWindow).count());
    }
    {
        auto topic = unique_topic("perf_cpu_busyspin");
        RingPublisher<int> pub;
        ASSERT_TRUE(pub.open(topic.c_str(), cfg));
        RingSubscriber<int> sub;
        ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

        uint64_t cpu0 = shmbridge::platform::thread_cpu_time_ns();
        auto wall0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - wall0 < kWindow) {
            sub.pop_ex(); /* non-blocking; tight loop pins this core */
        }
        uint64_t cpu1 = shmbridge::platform::thread_cpu_time_ns();
        busyspin_cpu_pct = 100.0 * static_cast<double>(cpu1 - cpu0) /
                           static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(kWindow).count());
    }

    record_latency_us("nfr1_blocked_cpu_pct", blocked_cpu_pct);
    record_latency_us("nfr1_busyspin_cpu_pct", busyspin_cpu_pct);

    EXPECT_LT(blocked_cpu_pct, 10.0)
        << "pop_wait() with no data should use near-0% CPU, measured " << blocked_cpu_pct << "%";
    EXPECT_GT(busyspin_cpu_pct, 50.0)
        << "a tight non-blocking poll loop should pin the core, measured " << busyspin_cpu_pct << "%";
    EXPECT_GT(busyspin_cpu_pct, blocked_cpu_pct * 5.0)
        << "blocked (" << blocked_cpu_pct << "%) vs busy-spin (" << busyspin_cpu_pct
        << "%) should differ by a wide, unambiguous margin";
}

/* ── NFR-3: zero-copy vs. copy latency at large payload sizes ─────────────── */

/*
 * NFR-3 claims zero-copy write/read measurably reduces latency relative to
 * the bulk-copy path at large payload sizes. PointCloud4096 (~64 KiB) is
 * large enough that an extra full-payload memcpy each way should be
 * measurable against push()/pop_ex()'s two memcpys (publisher writes into
 * the slot, subscriber copies out of it) versus reserve()/commit()+
 * borrow()/end_borrow()'s zero -- the caller constructs directly into
 * shared memory and reads directly out of it.
 */
TEST(RingPerformance, ZeroCopyFasterThanCopyAtLargePayload) {
    using shmbridge::msg::PointCloud4096;
    constexpr int N = 200;
    RingConfig cfg; cfg.capacity = 4;

    LatencyDistribution copy_dist, zerocopy_dist;
    {
        auto topic = unique_topic("perf_large_copy");
        RingPublisher<PointCloud4096> pub;
        ASSERT_TRUE(pub.open(topic.c_str(), cfg));
        RingSubscriber<PointCloud4096> sub;
        ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));
        PointCloud4096 msg{}; msg.n_points = 4096;

        std::vector<double> samples_us;
        samples_us.reserve(N);
        for (int i = 0; i < N; ++i) {
            auto t0 = std::chrono::steady_clock::now();
            ASSERT_TRUE(pub.push(msg));
            auto item = sub.pop_ex();
            auto t1 = std::chrono::steady_clock::now();
            ASSERT_TRUE(item.has_value());
            samples_us.push_back(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1e3);
        }
        copy_dist = compute_distribution(samples_us);
        record_distribution("large_payload_copy", copy_dist);
    }
    {
        auto topic = unique_topic("perf_large_zerocopy");
        RingPublisher<PointCloud4096> pub;
        ASSERT_TRUE(pub.open(topic.c_str(), cfg));
        RingSubscriber<PointCloud4096> sub;
        ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

        std::vector<double> samples_us;
        samples_us.reserve(N);
        for (int i = 0; i < N; ++i) {
            auto t0 = std::chrono::steady_clock::now();
            PointCloud4096* slot = pub.reserve();
            ASSERT_NE(slot, nullptr);
            slot->n_points = 4096;
            pub.commit();
            const PointCloud4096* item = sub.borrow();
            auto t1 = std::chrono::steady_clock::now();
            ASSERT_NE(item, nullptr);
            sub.end_borrow();
            samples_us.push_back(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1e3);
        }
        zerocopy_dist = compute_distribution(samples_us);
        record_distribution("large_payload_zerocopy", zerocopy_dist);
    }

    EXPECT_LT(zerocopy_dist.p50_us, copy_dist.p50_us)
        << "zero-copy median (" << zerocopy_dist.p50_us << "us) should beat "
           "copy median (" << copy_dist.p50_us << "us) at a ~64KiB payload";
}

/*
 * Fixed-size point-cloud message family (F-12, §5.11): each
 * PointCloudFixed<MaxPoints> instantiation is a complete, trivially-
 * copyable T usable directly as RingPublisher<T>/RingSubscriber<T>. A
 * round trip must preserve n_points and point data exactly, and a
 * subscriber attaching with a different MaxPoints than the publisher
 * must be rejected via type_hash (F-7) -- the same mechanism
 * RingTypeSafety.MismatchedTypeThrowsInvalidArgument already exercises
 * for unrelated types, here exercised across two instantiations of the
 * same template.
 */

TEST(RingPointCloud, RoundTripPreservesNPointsAndData) {
    using shmbridge::msg::PointCloud64;

    auto topic = unique_topic("pointcloud_roundtrip");
    RingConfig cfg; cfg.capacity = 4;
    RingPublisher<PointCloud64> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));

    RingSubscriber<PointCloud64> sub;
    ASSERT_TRUE(sub.try_attach(topic.c_str(), cfg));

    PointCloud64 cloud{};
    cloud.n_points = 3;
    cloud.points[0][0] = 1.0f; cloud.points[0][1] = 2.0f;
    cloud.points[0][2] = 3.0f; cloud.points[0][3] = 0.5f;
    cloud.points[1][0] = -1.0f; cloud.points[1][1] = -2.0f;
    cloud.points[1][2] = -3.0f; cloud.points[1][3] = 1.0f;
    cloud.points[2][0] = 100.0f; cloud.points[2][1] = 200.0f;
    cloud.points[2][2] = 300.0f; cloud.points[2][3] = 0.25f;
    /* points beyond n_points are left at their default-initialized 0 --
     * a real publisher only fills n_points of the full MaxPoints array. */

    ASSERT_TRUE(pub.push(cloud));
    auto item = sub.pop_ex();
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->value.n_points, 3u);
    for (uint32_t p = 0; p < item->value.n_points; ++p) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_FLOAT_EQ(item->value.points[p][c], cloud.points[p][c])
                << "point " << p << " component " << c;
        }
    }
}

TEST(RingPointCloud, DifferentMaxPointsVariantThrowsOnAttach) {
    using shmbridge::msg::PointCloud64;
    using shmbridge::msg::PointCloud256;

    auto topic = unique_topic("pointcloud_mismatch");
    RingConfig cfg; cfg.capacity = 2;
    RingPublisher<PointCloud64> pub;
    ASSERT_TRUE(pub.open(topic.c_str(), cfg));

    /* PointCloudFixed<64> and PointCloudFixed<256> are different, same-
     * sizeof-pattern-but-different-size types -- type_id<T>() hashes
     * __PRETTY_FUNCTION__/__FUNCSIG__, which includes MaxPoints, so these
     * get distinct type_hash values despite being the same template. */
    RingSubscriber<PointCloud256> wrong_sub;
    EXPECT_THROW(wrong_sub.try_attach(topic.c_str(), cfg), std::invalid_argument);
}
