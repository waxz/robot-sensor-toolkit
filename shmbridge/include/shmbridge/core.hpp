/*
 * shmbridge/core.hpp  -  C++17 publisher/subscriber classes for shmbridge v2.
 *
 * Header-only. Include this in any C++ project; no build-system changes needed.
 *
 *   #include <shmbridge/core.hpp>
 *   using namespace shmbridge;
 *
 *   // Simulator (publisher) side
 *   ShmPublisher pub("/shmbridge_v2", 1, 1);
 *   pub.open();
 *   pub.write_state(0, state);
 *   auto cmd = pub.read_cmd(0, 0);
 *
 *   // Controller (subscriber) side
 *   ShmSubscriber sub("/shmbridge_v2");
 *   sub.attach();
 *   auto state = sub.read_state(0);
 *   sub.write_cmd(0, 0, 0.5f, 0.1f);
 *
 * Roles are symmetric: a Python script can be the publisher, a C++ process
 * the subscriber - or vice versa.
 */

#pragma once

#include "platform.hpp"
#include "types.h"

#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

/* ── Memory-order fences ──────────────────────────────────────────────────────
 * Split writer / reader fences instead of seq_cst.
 * On x86 (TSO) these compile to compiler barriers only — no mfence (~30 ns).
 * On AArch64 they emit dmb ishst / dmb ish, still lighter than dmb sy.
 *
 * The outer #ifndef _SB_FENCE_W guard lets this coexist with topic.hpp in
 * the same translation unit (e.g. ext_core.hpp + ext_topics.hpp both
 * included from py_bindings.cpp) — see topic.hpp's matching comment for why.
 * ─────────────────────────────────────────────────────────────────────────── */
#ifndef _SB_FENCE_W
#ifdef __cplusplus
#  include <atomic>
/* StoreStore: seals the write window before and after payload stores. */
#  define _SB_FENCE_W() std::atomic_thread_fence(std::memory_order_release)
/* LoadLoad: ensures seq is read before payload, and payload before seq2. */
#  define _SB_FENCE_R() std::atomic_thread_fence(std::memory_order_acquire)
#elif defined(__aarch64__) || defined(__ARM_ARCH_8A__)
#  define _SB_FENCE_W() __asm__ volatile("dmb ishst" ::: "memory")
#  define _SB_FENCE_R() __asm__ volatile("dmb ish"   ::: "memory")
#else
#  define _SB_FENCE_W() __asm__ volatile("" ::: "memory")
#  define _SB_FENCE_R() __asm__ volatile("" ::: "memory")
#endif

/* Spin-wait hint — cuts memory-bus contention and pipeline stalls in retry loops. */
#if defined(__x86_64__) || defined(__i386__)
#  define _SB_PAUSE() __builtin_ia32_pause()
#elif defined(_M_X64) || defined(_M_IX86)
#  define _SB_PAUSE() _mm_pause()
#elif defined(__aarch64__) || defined(__ARM_ARCH_8A__)
#  define _SB_PAUSE() __asm__ volatile("yield" ::: "memory")
#else
#  define _SB_PAUSE() ((void)0)
#endif
#endif /* _SB_FENCE_W */

#ifdef __cplusplus
#  include <atomic>
#endif

namespace shmbridge {

/* ── Wire-level data types ────────────────────────────────────────────────── */

struct RobotState {
    double   x = 0, y = 0, heading = 0;
    float    vx = 0, vy = 0, omega = 0;
    float    goal_x = 0, goal_y = 0, goal_dist = 0;
    uint64_t step     = 0;
    double   sim_time = 0;
    bool     reached  = false;
    bool     collision = false;
    /* System-monotonic nanoseconds captured inside the seqlock at write time.
     * Zero when heartbeat_every==0 (timestamp disabled). Use for staleness
     * checks and usable-latency measurement: (now_ns() - write_ns). */
    uint64_t write_ns = 0;
};

struct RobotCmd {
    float    linear  = 0;
    float    angular = 0;
    uint32_t seq     = 0;
};

/* ── Internal helpers ─────────────────────────────────────────────────────── */

namespace detail {

/* Guarded so this coexists with topic.hpp's identical detail::now_ns() in
 * the same translation unit (e.g. ext_core.hpp + ext_topics.hpp both
 * included from py_bindings.cpp) — a plain `inline` doesn't protect against
 * two definitions in the *same* TU, only across separate ones. */
#ifndef SHMBRIDGE_DETAIL_NOW_NS_DEFINED
#define SHMBRIDGE_DETAIL_NOW_NS_DEFINED
inline uint64_t now_ns() noexcept { return platform::now_ns(); }
#endif

inline size_t raw_size(unsigned n, unsigned nc) noexcept {
    return 128u + 128u * n + 128u * n * nc;
}

inline size_t aligned_size(unsigned n, unsigned nc) noexcept {
    return (raw_size(n, nc) + 4095u) & ~4095u;
}

/* Pointer to state slot r (avoids UB of indexing past states[1]). */
inline ShmStateSlot* state_ptr(void* mem, unsigned r) noexcept {
    return reinterpret_cast<ShmStateSlot*>(
        static_cast<char*>(mem) + 128 + static_cast<size_t>(r) * 128);
}

/* Pointer to cmd slot (robot r, consumer c). */
inline ShmCmdSlot* cmd_ptr(void* mem, unsigned r, unsigned c,
                              unsigned nc) noexcept {
    size_t off = 128 + 128u * 1 /* placeholder, recalc below */;
    // header(128) + states(128 * n_robots, but we need n_robots from header)
    // Store n_robots in caller; this helper is called after open/attach.
    (void)off;
    ShmHeader* hdr = static_cast<ShmHeader*>(mem);
    unsigned n = hdr->n_robots;
    size_t offset = 128u + 128u * n + (static_cast<size_t>(r) * nc + c) * 128u;
    return reinterpret_cast<ShmCmdSlot*>(static_cast<char*>(mem) + offset);
}

/* Seqlock write of robot state to a StateSlot.
 * When heartbeat_every > 0 the system-monotonic timestamp is captured
 * INSIDE the seqlock window so readers get a consistent (state, write_ns)
 * pair. Set heartbeat_every=0 to skip the clock call for minimum latency. */
inline void write_state_slot(ShmStateSlot* slot, const RobotState& s,
                              uint64_t& seq_counter, unsigned& write_count,
                              unsigned heartbeat_every) noexcept {
    slot->seq = ++seq_counter;  /* even → odd */
    _SB_FENCE_W();
    ShmState& d = slot->state;
    d.x         = s.x;
    d.y         = s.y;
    d.heading   = s.heading;
    d.vx        = s.vx;
    d.vy        = s.vy;
    d.omega     = s.omega;
    d.goal_x    = s.goal_x;
    d.goal_y    = s.goal_y;
    d.goal_dist = s.goal_dist;
    d.step      = s.step;
    d.sim_time  = s.sim_time;
    d.reached   = s.reached ? 1u : 0u;
    d.collision = s.collision ? 1u : 0u;
    /* Timestamp inside the window — consistent with the state payload above. */
    if (heartbeat_every > 0) {
        (void)write_count;
        slot->writer_ts_ns = now_ns();
    }
    _SB_FENCE_W();
    slot->seq = ++seq_counter;  /* odd → even */
    slot->seq2 = seq_counter;
}

/* Seqlock read of robot state; returns false on torn read.
 * out.write_ns is populated from slot->writer_ts_ns (zero if never written). */
inline bool read_state_slot(const ShmStateSlot* slot, RobotState& out) noexcept {
    uint64_t s1 = slot->seq;
    _SB_FENCE_R();
    const ShmState& d = slot->state;
    out.x         = d.x;
    out.y         = d.y;
    out.heading   = d.heading;
    out.vx        = d.vx;
    out.vy        = d.vy;
    out.omega     = d.omega;
    out.goal_x    = d.goal_x;
    out.goal_y    = d.goal_y;
    out.goal_dist = d.goal_dist;
    out.step      = d.step;
    out.sim_time  = d.sim_time;
    out.reached   = d.reached != 0;
    out.collision = d.collision != 0;
    out.write_ns  = slot->writer_ts_ns;  /* consistent with payload above */
    _SB_FENCE_R();
    uint64_t s2 = slot->seq2;
    return s1 == s2 && !(s1 & 1u);
}

/* Seqlock write of a velocity command to a CmdSlot. */
inline void write_cmd_slot(ShmCmdSlot* slot, float linear, float angular,
                            uint64_t& seq_counter) noexcept {
    slot->seq = ++seq_counter;
    _SB_FENCE_W();
    slot->cmd.linear  = linear;
    slot->cmd.angular = angular;
    slot->cmd.seq     = static_cast<uint32_t>(seq_counter);
    slot->cmd.valid   = 1;
    _SB_FENCE_W();
    slot->seq = ++seq_counter;
    slot->seq2 = seq_counter;
    slot->writer_ts_ns = now_ns();
}

/* Seqlock read of a cmd; returns nullopt on torn read or no valid cmd. */
inline std::optional<RobotCmd> read_cmd_slot(const ShmCmdSlot* slot) noexcept {
    uint64_t s1 = slot->seq;
    _SB_FENCE_R();
    float    lin   = slot->cmd.linear;
    float    ang   = slot->cmd.angular;
    uint32_t seq   = slot->cmd.seq;
    uint32_t valid = slot->cmd.valid;
    _SB_FENCE_R();
    uint64_t s2 = slot->seq2;
    if (s1 != s2 || (s1 & 1u) || !valid) return std::nullopt;
    return RobotCmd{lin, ang, seq};
}

} /* namespace detail */

/* ── ShmPublisher ─────────────────────────────────────────────────────────── */

/**
 * Creates and owns the shm segment; writes robot state, reads velocity cmds.
 *
 * Typical use: the simulator (Python or C++) creates a ShmPublisher, then
 * one or more ShmSubscriber instances (C++ controllers or Python agents)
 * attach to it.
 */
class ShmPublisher {
public:
    /**
     * @param name           POSIX shm name, e.g. "/shmbridge_v2"
     * @param n_robots       Number of robot slots (default 1)
     * @param n_consumers    Independent cmd writers per robot (default 1)
     * @param heartbeat_every Update writer_ts_ns every N writes (default 1)
     */
    explicit ShmPublisher(std::string name       = SHMBRIDGE_SHM_NAME,
                          unsigned    n_robots    = 1,
                          unsigned    n_consumers = 1,
                          unsigned    heartbeat_every = 1)
        : name_(std::move(name))
        , n_(n_robots)
        , nc_(n_consumers)
        , heartbeat_every_(heartbeat_every)
        , size_(detail::aligned_size(n_robots, n_consumers))
        , state_seqs_(n_robots, 0)
        , write_counts_(n_robots, 0)
    {
        if (n_robots < 1)    throw std::invalid_argument("n_robots must be >= 1");
        if (n_consumers < 1) throw std::invalid_argument("n_consumers must be >= 1");
        /* heartbeat_every == 0 means never update writer_ts_ns (lowest-latency mode). */
    }

    ~ShmPublisher() { close(); }

    ShmPublisher(const ShmPublisher&)            = delete;
    ShmPublisher& operator=(const ShmPublisher&) = delete;

    /** Create and zero-init the shm segment. */
    void open(bool do_mlock = false) {
        mem_ = platform::shm_create(name_, size_);
        if (do_mlock) platform::mem_lock(mem_, size_);
        ShmHeader* hdr = static_cast<ShmHeader*>(mem_);
        hdr->magic          = SHMBRIDGE_MAGIC;
        hdr->schema_version = SHMBRIDGE_VERSION;
        hdr->n_robots       = static_cast<uint8_t>(n_);
        hdr->n_consumers    = static_cast<uint8_t>(nc_);
        hdr->ready          = 1;  /* signal subscribers */
    }

    /** Unmap and unlink the segment. */
    void close() noexcept {
        if (mem_) { platform::shm_unmap(mem_, size_); mem_ = nullptr; }
        platform::shm_destroy(name_);
    }

    bool is_open() const noexcept { return mem_ != nullptr; }

    /** Write robot state via seqlock. */
    void write_state(unsigned robot_idx, const RobotState& s) {
        detail::write_state_slot(detail::state_ptr(mem_, robot_idx), s,
                                 state_seqs_[robot_idx],
                                 write_counts_[robot_idx],
                                 heartbeat_every_);
    }

    /**
     * Read cmd from a specific consumer slot.
     * Returns nullopt if no valid cmd has been posted or slot is mid-write.
     */
    std::optional<RobotCmd> read_cmd(unsigned robot_idx    = 0,
                                     unsigned consumer_idx = 0) const noexcept {
        return detail::read_cmd_slot(detail::cmd_ptr(mem_, robot_idx,
                                                     consumer_idx, nc_));
    }

    /**
     * Block until a valid cmd arrives or timeout_ms elapses.
     *
     * Between poll attempts nanosleep(poll_sleep_ns) is issued — a direct
     * OS syscall that yields the CPU without going through Python or the
     * spin_sleep Welford estimator.  This drops idle CPU from ~100 % to
     * ~5-10 % at negligible extra latency cost.
     *
     * poll_sleep_ns = 0  →  pure busy-poll (lowest latency, 100 % CPU).
     * poll_sleep_ns = 500'000  →  500 µs sleep between polls (default).
     *
     * Returns nullopt on timeout.
     */
    std::optional<RobotCmd> read_cmd_blocking(
            double   timeout_ms    = 10.0,
            int64_t  poll_sleep_ns = 500'000LL,
            unsigned robot_idx     = 0,
            unsigned consumer_idx  = 0) const noexcept
    {
        uint64_t deadline = detail::now_ns() +
                            static_cast<uint64_t>(timeout_ms * 1e6);
        while (detail::now_ns() < deadline) {
            auto cmd = detail::read_cmd_slot(
                    detail::cmd_ptr(mem_, robot_idx, consumer_idx, nc_));
            if (cmd) return cmd;
            if (poll_sleep_ns > 0) {
                platform::sleep_ns(poll_sleep_ns);
            } else {
                _SB_PAUSE();
            }
        }
        return std::nullopt;
    }

    /**
     * Read across all consumer slots; return the one with the highest seq
     * (most recently posted valid command).
     */
    std::optional<RobotCmd> read_best_cmd(unsigned robot_idx = 0) const noexcept {
        std::optional<RobotCmd> best;
        for (unsigned c = 0; c < nc_; ++c) {
            auto cmd = detail::read_cmd_slot(
                detail::cmd_ptr(mem_, robot_idx, c, nc_));
            if (cmd && (!best || cmd->seq > best->seq))
                best = cmd;
        }
        return best;
    }

    /** True if consumer c posted a cmd within max_age_ms milliseconds. */
    bool is_controller_alive(double   max_age_ms   = 100.0,
                             unsigned robot_idx    = 0,
                             unsigned consumer_idx = 0) const noexcept {
        uint64_t ts = detail::cmd_ptr(mem_, robot_idx, consumer_idx,
                                      nc_)->writer_ts_ns;
        if (ts == 0) return true;
        return (detail::now_ns() - ts) < static_cast<uint64_t>(max_age_ms * 1e6);
    }

    unsigned n_robots()    const noexcept { return n_; }
    unsigned n_consumers() const noexcept { return nc_; }

private:
    std::string           name_;
    unsigned              n_, nc_, heartbeat_every_;
    size_t                size_;
    void*                 mem_ = nullptr;
    std::vector<uint64_t> state_seqs_;
    std::vector<unsigned> write_counts_;
};

/* ── ShmSubscriber ────────────────────────────────────────────────────────── */

/**
 * Attaches to an existing shm segment; reads robot state, writes velocity cmds.
 *
 * Any number of ShmSubscribers can attach to the same segment.  Each uses its
 * own consumer_idx so the publisher can arbitrate commands via read_best_cmd().
 */
class ShmSubscriber {
public:
    /**
     * @param name         POSIX shm name (must match the publisher's)
     * @param n_robots     Hint; overridden by the segment header on attach()
     */
    explicit ShmSubscriber(std::string name    = SHMBRIDGE_SHM_NAME,
                           unsigned    n_robots = 1)
        : name_(std::move(name)), n_(n_robots), nc_(1)
        , size_(detail::aligned_size(n_robots, 1))
    {}

    ~ShmSubscriber() { detach(); }

    ShmSubscriber(const ShmSubscriber&)            = delete;
    ShmSubscriber& operator=(const ShmSubscriber&) = delete;

    /**
     * Attach to an existing segment.  Retries until the publisher creates the
     * segment, then blocks until header.ready == 1, or timeout_ms elapses.
     *
     * Safe to call before the publisher has started: the subscriber will wait
     * up to timeout_ms for the segment to appear, then another full timeout_ms
     * for the ready flag.  Pass timeout_ms=0 for a non-blocking single attempt.
     *
     * Calling attach() when already attached silently detaches first so the
     * caller can re-attach after a publisher restart without an explicit
     * detach() call:
     *
     *   while (true) {
     *     sub.attach(30000);          // waits up to 30 s for publisher
     *     while (sub.is_publisher_alive(500)) { ... read ... }
     *     // publisher gone — loop back; attach() will detach the old mapping
     *   }
     *
     * @param timeout_ms  Maximum wait in milliseconds (float-friendly for Python).
     */
    void attach(double timeout_ms = 30000.0) {
        detach();  /* release old mapping before (re-)attaching */
        uint64_t deadline = detail::now_ns() +
                            static_cast<uint64_t>(timeout_ms * 1e6);

        /* Probe header with retry — wait for publisher to create the segment. */
        for (;;) {
            try {
                void* hdr_page = platform::shm_attach(name_, 4096);
                ShmHeader* h = static_cast<ShmHeader*>(hdr_page);
                if (h->n_robots    > 0) n_  = h->n_robots;
                if (h->n_consumers > 0) nc_ = h->n_consumers;
                platform::shm_unmap(hdr_page, 4096);
                break;  /* probe succeeded */
            } catch (...) {
                if (detail::now_ns() > deadline)
                    throw std::runtime_error(
                        "attach timeout: segment '" + name_ + "' not found — "
                        "is the publisher running? (waited "
                        + std::to_string(static_cast<int>(timeout_ms)) + " ms)");
                platform::sleep_ns(50'000'000LL);  /* 50 ms between retries */
            }
        }

        size_ = detail::aligned_size(n_, nc_);
        mem_ = platform::shm_attach(name_, size_);

        /* Wait for ready signal. */
        ShmHeader* hdr = static_cast<ShmHeader*>(mem_);
        while (!hdr->ready) {
            if (detail::now_ns() > deadline) {
                platform::shm_unmap(mem_, size_); mem_ = nullptr;
                throw std::runtime_error("attach timeout: publisher not ready");
            }
            platform::sleep_ns(500'000LL);
        }
        if (hdr->magic != 0 && hdr->magic != SHMBRIDGE_MAGIC) {
            platform::shm_unmap(mem_, size_); mem_ = nullptr;
            throw std::runtime_error("schema mismatch: unexpected magic");
        }
        /* Re-read final n_robots / n_consumers in case they changed. */
        if (hdr->n_robots    > 0) n_  = hdr->n_robots;
        if (hdr->n_consumers > 0) nc_ = hdr->n_consumers;
        cmd_seqs_.assign(static_cast<size_t>(n_) * nc_, 0);
    }

    void detach() noexcept {
        if (mem_) { platform::shm_unmap(mem_, size_); mem_ = nullptr; }
    }

    bool is_attached() const noexcept { return mem_ != nullptr; }

    /**
     * Seqlock read of robot state.
     * Returns nullopt on torn read (caller should retry immediately).
     */
    std::optional<RobotState> read_state(unsigned robot_idx = 0) const noexcept {
        RobotState s;
        if (detail::read_state_slot(detail::state_ptr(mem_, robot_idx), s))
            return s;
        return std::nullopt;
    }

    /**
     * Spin until a clean read or max_retries attempts.
     * Returns nullopt only if every attempt was torn (should be very rare).
     */
    std::optional<RobotState> read_state_spin(unsigned robot_idx = 0,
                                              unsigned max_retries = 64) const noexcept {
        for (unsigned i = 0; i < max_retries; ++i) {
            if (auto s = read_state(robot_idx)) return s;
            _SB_PAUSE();
        }
        return std::nullopt;
    }

    /**
     * Write a velocity command to a specific consumer slot.
     * @param consumer_idx  Which of the publisher's n_consumers slots to write.
     */
    void write_cmd(unsigned robot_idx, unsigned consumer_idx,
                   float linear, float angular) {
        size_t idx = static_cast<size_t>(robot_idx) * nc_ + consumer_idx;
        detail::write_cmd_slot(
            detail::cmd_ptr(mem_, robot_idx, consumer_idx, nc_),
            linear, angular, cmd_seqs_[idx]);
    }

    /** True if the publisher has written a state within max_age_ms ms. */
    bool is_publisher_alive(double   max_age_ms = 100.0,
                            unsigned robot_idx  = 0) const noexcept {
        uint64_t ts = detail::state_ptr(mem_, robot_idx)->writer_ts_ns;
        if (ts == 0) return true;
        return (detail::now_ns() - ts) < static_cast<uint64_t>(max_age_ms * 1e6);
    }

    unsigned n_robots()    const noexcept { return n_; }
    unsigned n_consumers() const noexcept { return nc_; }

private:
    std::string           name_;
    unsigned              n_, nc_;
    size_t                size_;
    void*                 mem_ = nullptr;
    std::vector<uint64_t> cmd_seqs_;  /* per (robot, consumer) seqlock counter */
};

} /* namespace shmbridge */
