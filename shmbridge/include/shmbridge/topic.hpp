/*
 * shmbridge/topic.hpp — generic typed Publisher<T> / Subscriber<T> for shmbridge v3.
 *
 * Each topic is one POSIX shared-memory segment:
 *
 *   [TopicHeader 128B][SeqlockSlot<T> 128B]
 *   Total = 256 B for small messages (sizeof(T) ≤ 104 B)
 *
 * Multiple subscribers are safe by construction: the seqlock is MRSW — readers
 * never modify seq/seq2, so N concurrent readers have zero race condition.
 * The futex-based broadcaster wakes all N subscribers in one syscall (O(1) cost
 * regardless of subscriber count), resolving the unicast bottleneck of POSIX sems.
 *
 * Type safety:
 *   type_id<T>() produces a constexpr FNV-1a hash of __PRETTY_FUNCTION__.
 *   Subscriber::attach() compares the stored hash and refuses mismatched types.
 *
 * Thread safety:
 *   Publisher — one writer at a time (call from one thread only).
 *   Subscriber — one reader per instance; create multiple Subscriber instances
 *   for multiple readers sharing the same topic.
 *
 * Requires C++17 and POSIX (Linux preferred; macOS supported without futex).
 */

#pragma once

#include "platform.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

/* ── arch fences (same as core.hpp) ─────────────────────────────────────── */
#if defined(__x86_64__) || defined(_M_X64)
#  define _SB_FENCE_W()  __asm__ volatile("" ::: "memory")
#  define _SB_FENCE_R()  __asm__ volatile("" ::: "memory")
#  define _SB_PAUSE()    __asm__ volatile("pause" ::: "memory")
#elif defined(_M_X64) || defined(_M_IX86)
#  define _SB_FENCE_W()  _mm_sfence()
#  define _SB_FENCE_R()  _mm_lfence()
#  define _SB_PAUSE()    _mm_pause()
#elif defined(__aarch64__)
#  define _SB_FENCE_W()  __asm__ volatile("dmb ishst" ::: "memory")
#  define _SB_FENCE_R()  __asm__ volatile("dmb ish"   ::: "memory")
#  define _SB_PAUSE()    __asm__ volatile("yield"      ::: "memory")
#else
#  define _SB_FENCE_W()  std::atomic_thread_fence(std::memory_order_release)
#  define _SB_FENCE_R()  std::atomic_thread_fence(std::memory_order_acquire)
#  define _SB_PAUSE()    ((void)0)
#endif

namespace shmbridge {

/* ── compile-time type fingerprint ──────────────────────────────────────── */

namespace detail {

/* FNV-1a over a compile-time string.  __PRETTY_FUNCTION__ includes the
 * full template argument so distinct instantiations produce distinct hashes. */
constexpr uint64_t fnv1a_const(const char* s, uint64_t h = 0xcbf29ce484222325ULL) {
    return (*s == '\0') ? h : fnv1a_const(s + 1,
        (h ^ static_cast<uint64_t>(static_cast<unsigned char>(*s))) * 0x100000001b3ULL);
}

inline uint64_t now_ns() noexcept { return platform::now_ns(); }

} /* namespace detail */

/* Returns a stable 64-bit hash that identifies type T within a compilation
 * unit. Use this to detect publisher/subscriber type mismatches at attach(). */
template<typename T>
constexpr uint64_t type_id() noexcept {
    return detail::fnv1a_const(__PRETTY_FUNCTION__);
}

/* ── shared-memory layout ────────────────────────────────────────────────── */

/*
 * TopicHeader — 128 bytes, always at offset 0 of the shm segment.
 *
 *   ready        set to 1 by Publisher after all fields are written.
 *   magic        0x53484D43 ("SHMC")
 *   schema_ver   3
 *   type_hash    type_id<T>() — checked by Subscriber::attach()
 *   sizeof_T     sizeof(T) — extra guard against sizeof mismatch
 *   notify_seq   futex word; Publisher increments + FUTEX_WAKE to broadcast
 *   name         null-terminated topic name (max 87 chars)
 */
struct TopicHeader {
    volatile uint64_t ready          = 0;
    uint32_t          magic          = 0;
    uint32_t          schema_version = 0;
    uint64_t          type_hash      = 0;
    uint32_t          sizeof_T       = 0;
    uint32_t          _pad0          = 0;
    volatile uint32_t notify_seq     = 0;
    uint32_t          _pad1          = 0;
    uint8_t           name[88]       = {};
};
static_assert(sizeof(TopicHeader) == 128, "TopicHeader must be 128 bytes");

/*
 * SeqlockSlot<T> — generic dual-sequence seqlock for any trivially-copyable T.
 *
 * Always exactly 128 bytes:  [seq:8][_storage:104][seq2:8][write_ns:8]
 *   seq       even = stable, odd = write in progress
 *   _storage  104-byte region; first sizeof(T) bytes hold the payload
 *             (accessed via memcpy so no alignment constraints on T)
 *   seq2      copy of seq after write; reader checks seq == seq2
 *   write_ns  CLOCK_MONOTONIC ns captured inside the seqlock window
 *
 * Constraint: sizeof(T) ≤ 104.  Larger types (LaserScan, PointCloud, Map)
 * carry only their header struct here; bulk payload lives in a separate
 * region (future BulkTopic).
 */
template<typename T>
struct SeqlockSlot {
    static_assert(sizeof(T) <= 104,
        "T too large for a single seqlock slot (max 104 B). "
        "Use BulkTopic for large payloads.");
    static_assert(__is_trivially_copyable(T),
        "T must be trivially copyable for safe seqlock use.");

    volatile uint64_t seq        = 0;
    uint8_t           _storage[104]{};   /* always 104 B — no template-size array */
    volatile uint64_t seq2       = 0;
    volatile uint64_t write_ns   = 0;
    /* 8 + 104 + 8 + 8 = 128 exactly */
};
static_assert(sizeof(SeqlockSlot<uint8_t>) == 128);

/* ── seqlock helpers ─────────────────────────────────────────────────────── */

namespace detail {

template<typename T>
inline void write_slot(SeqlockSlot<T>* slot, const T& msg,
                       uint64_t& seq_ctr, bool do_ts) noexcept {
    slot->seq = ++seq_ctr;          /* even → odd */
    _SB_FENCE_W();
    std::memcpy(slot->_storage, &msg, sizeof(T));
    if (do_ts) slot->write_ns = now_ns();
    _SB_FENCE_W();
    slot->seq  = ++seq_ctr;         /* odd → even */
    slot->seq2 = seq_ctr;
}

template<typename T>
inline bool read_slot(const SeqlockSlot<T>* slot, T& out, uint64_t& wns) noexcept {
    uint64_t s1 = slot->seq;
    _SB_FENCE_R();
    std::memcpy(&out, slot->_storage, sizeof(T));
    wns = slot->write_ns;
    _SB_FENCE_R();
    uint64_t s2 = slot->seq2;
    return (s1 == s2) && !(s1 & 1u);
}

} /* namespace detail */

/* ── segment size ────────────────────────────────────────────────────────── */

template<typename T>
constexpr std::size_t topic_shm_size() {
    return sizeof(TopicHeader) + sizeof(SeqlockSlot<T>);
}

/* ── error codes ─────────────────────────────────────────────────────────── */

enum class TopicError {
    Ok          = 0,
    ShmFailed,      /* shm_open / ftruncate / mmap failed */
    TypeMismatch,   /* type_hash stored ≠ type_id<T>() */
    SizeMismatch,   /* sizeof_T stored ≠ sizeof(T) */
    NotReady,       /* publisher not yet ready */
    Timeout,        /* attach timed out waiting for publisher */
};

/* ── subscriber result ───────────────────────────────────────────────────── */

template<typename T>
struct ReadResult {
    T        value{};
    uint64_t write_ns = 0;   /* CLOCK_MONOTONIC ns of the write */
    uint64_t read_ns  = 0;   /* ns when the read completed */
    bool     ok       = false;

    uint64_t age_ns() const noexcept { return (read_ns > write_ns) ? read_ns - write_ns : 0; }
    double   age_us() const noexcept { return static_cast<double>(age_ns()) / 1e3; }
};

/* ── Publisher<T> ────────────────────────────────────────────────────────── */

/*
 * Publisher creates the shm segment and owns the sole writer slot.
 *
 *   pub.open("/myrobot/pose");          // creates + zeros segment
 *   shmbridge::msg::Pose2d p{1.0, 0.0, 0.0, 0};
 *   pub.write(p);                       // seqlock-write, no notify
 *   pub.write_notify(p);                // seqlock-write + futex broadcast
 *   pub.close();                        // unmap + unlink
 *
 * heartbeat_every: capture write_ns every N writes (0 = never, 1 = always).
 * CLOCK_MONOTONIC call costs ~25 ns (VDSO); set 0 for minimum-latency paths.
 */
template<typename T>
class Publisher {
public:
    explicit Publisher(uint32_t heartbeat_every = 1) noexcept
        : heartbeat_every_(heartbeat_every) {}

    ~Publisher() { close(); }

    /* Non-copyable, movable */
    Publisher(const Publisher&)            = delete;
    Publisher& operator=(const Publisher&) = delete;
    Publisher(Publisher&&)                 = default;
    Publisher& operator=(Publisher&&)      = default;

    TopicError open(const std::string& name, bool do_mlock = false) noexcept {
        name_ = name;
        const std::size_t sz = topic_shm_size<T>();
        try {
            base_ = platform::shm_create(to_shm_name(name), sz);
        } catch (...) { return TopicError::ShmFailed; }
        if (do_mlock) platform::mem_lock(base_, sz);

        /* Populate header */
        auto* hdr            = header();
        hdr->magic           = 0x53484D43u;
        hdr->schema_version  = 3;
        hdr->type_hash       = type_id<T>();
        hdr->sizeof_T        = static_cast<uint32_t>(sizeof(T));
        std::size_t nlen     = std::min(name.size(), static_cast<std::size_t>(87u));
        std::memcpy(hdr->name, name.data(), nlen);
        hdr->name[nlen]      = '\0';

        _SB_FENCE_W();
        hdr->ready = 1;

        seq_ctr_ = 0;
        return TopicError::Ok;
    }

    void close() noexcept {
        if (!base_) return;
        platform::shm_unmap(base_, topic_shm_size<T>());
        base_ = nullptr;
        platform::shm_destroy(to_shm_name(name_));
        name_.clear();
    }

    /* Seqlock-write without notification */
    void write(const T& msg) noexcept {
        bool do_ts = (heartbeat_every_ > 0) && ((++write_count_ % heartbeat_every_) == 0);
        detail::write_slot(slot(), msg, seq_ctr_, do_ts);
    }

    /* Seqlock-write then broadcast to all waiting subscribers */
    void write_notify(const T& msg) noexcept {
        write(msg);
        platform::notify_wake_all(&header()->notify_seq);
    }

    bool      is_open()     const noexcept { return base_ != nullptr; }
    TopicHeader* header()   const noexcept { return static_cast<TopicHeader*>(base_); }

private:
    SeqlockSlot<T>* slot() const noexcept {
        return reinterpret_cast<SeqlockSlot<T>*>(
            static_cast<uint8_t*>(base_) + sizeof(TopicHeader));
    }

    static std::string to_shm_name(const std::string& n) {
        /* Convert "/myrobot/pose" → "/sb_myrobot_pose" (single-component POSIX name) */
        std::string out = "/sb_";
        for (char c : n) out += (c == '/' ? '_' : c);
        return out;
    }

    void*       base_            = nullptr;
    std::string name_;
    uint64_t    seq_ctr_         = 0;
    uint64_t    write_count_     = 0;
    uint32_t    heartbeat_every_ = 1;
};

/* ── Subscriber<T> ───────────────────────────────────────────────────────── */

/*
 * Subscriber attaches to an existing segment and provides:
 *
 *   latest()        — single seqlock read; nullopt on torn read
 *   spin(retries)   — retry until clean; nullopt only on persistent failure
 *   read_if_new()   — returns nullopt if write_ns unchanged (de-duplicates)
 *   wait(ms)        — block via futex until publisher calls write_notify
 *   wait_new(ms)    — block until a write_ns *newer* than last seen arrives
 *   age_us()        — µs age of the last successfully read value
 *   stats()         — cumulative read statistics
 *
 * Multiple Subscriber instances on the same topic are safe: readers never
 * write to the seqlock fields, so concurrent reads create no race condition.
 */

template<typename T>
struct SubscriberStats {
    uint64_t total_reads  = 0;
    uint64_t torn_reads   = 0;
    uint64_t wait_wakeups = 0;  /* how many futex wakeups received */
};

template<typename T>
class Subscriber {
public:
    Subscriber() = default;
    ~Subscriber() { detach(); }

    Subscriber(const Subscriber&)            = delete;
    Subscriber& operator=(const Subscriber&) = delete;
    Subscriber(Subscriber&&)                 = default;
    Subscriber& operator=(Subscriber&&)      = default;

    /*
     * Attach to an existing segment.
     * Waits up to timeout_ms for the publisher to mark ready.
     * Returns TopicError::TypeMismatch / SizeMismatch if the stored type
     * does not match T (catches publisher/subscriber type bugs at startup).
     */
    TopicError attach(const std::string& name, int timeout_ms = 5000) noexcept {
        name_ = name;
        const std::size_t sz = topic_shm_size<T>();
        long waited = 0;
        long limit_ns = static_cast<long>(timeout_ms) * 1'000'000L;

        /* Poll until the segment appears or timeout. */
        while (true) {
            try { base_ = platform::shm_attach(to_shm_name(name), sz); break; }
            catch (...) {}
            if (waited >= limit_ns) return TopicError::Timeout;
            platform::sleep_ns(5'000'000LL);
            waited += 5'000'000L;
        }

        /* Poll until publisher sets ready. */
        if (header()->ready == 0) {
            waited = 0;
            while (header()->ready == 0 && waited < limit_ns) {
                platform::sleep_ns(1'000'000LL);
                waited += 1'000'000L;
            }
            if (header()->ready == 0) return TopicError::NotReady;
        }

        /* Type guards */
        if (header()->type_hash != type_id<T>()) return TopicError::TypeMismatch;
        if (header()->sizeof_T  != sizeof(T))    return TopicError::SizeMismatch;

        last_write_ns_ = 0;
        return TopicError::Ok;
    }

    void detach() noexcept {
        if (!base_) return;
        platform::shm_unmap(base_, topic_shm_size<T>());
        base_ = nullptr;
        name_.clear();
    }

    /* ── read methods ──────────────────────────────────────────────────── */

    /* Non-blocking single seqlock attempt. Returns nullopt on torn read. */
    std::optional<ReadResult<T>> latest() noexcept {
        if (!base_) return std::nullopt;
        stats_.total_reads++;
        ReadResult<T> r;
        if (!detail::read_slot(slot(), r.value, r.write_ns)) {
            stats_.torn_reads++;
            return std::nullopt;
        }
        r.read_ns      = detail::now_ns();
        last_write_ns_ = r.write_ns;
        return r;
    }

    /* Spin up to max_retries times, returning nullopt only on total failure. */
    std::optional<ReadResult<T>> spin(int max_retries = 64) noexcept {
        for (int i = 0; i < max_retries; ++i) {
            auto r = latest();
            if (r) return r;
            _SB_PAUSE();
        }
        return std::nullopt;
    }

    /*
     * Returns nullopt if write_ns has not changed since the last read
     * (publisher has not produced a new message).  Useful for zero-copy
     * "is there anything new?" checks at a controller's own rate.
     */
    std::optional<ReadResult<T>> read_if_new() noexcept {
        if (!base_) return std::nullopt;
        /* Peek at write_ns without a full seqlock — just a quick check */
        uint64_t wns = slot()->write_ns;
        if (wns == last_write_ns_ || wns == 0) return std::nullopt;
        return spin();
    }

    /*
     * Block until the publisher calls write_notify() (futex broadcast).
     * Returns the first clean read after waking.  timeout_ms < 0 = infinite.
     * Falls back to a 1ms-poll loop on non-Linux platforms.
     */
    std::optional<ReadResult<T>> wait(int timeout_ms = -1) noexcept {
        if (!base_) return std::nullopt;
        volatile uint32_t* word = &header()->notify_seq;
        uint32_t val = reinterpret_cast<const std::atomic<uint32_t>*>(
            const_cast<const uint32_t*>(word))->load(std::memory_order_acquire);
        platform::notify_wait(word, val, timeout_ms);
        stats_.wait_wakeups++;
        return spin();
    }

    /*
     * Like wait(), but skips the result if write_ns is not newer than the
     * last seen value — safe to call in a tight loop even if the publisher
     * wrote a burst of messages while this subscriber was busy.
     */
    std::optional<ReadResult<T>> wait_new(int timeout_ms = -1) noexcept {
        auto r = wait(timeout_ms);
        if (!r) return std::nullopt;
        if (r->write_ns <= last_write_ns_) return std::nullopt;  /* already seen */
        return r;
    }

    /* ── diagnostics ───────────────────────────────────────────────────── */

    /* µs age of the most-recent successfully-read message (0 if none). */
    double age_us() const noexcept {
        if (last_write_ns_ == 0) return 0.0;
        return static_cast<double>(detail::now_ns() - last_write_ns_) / 1e3;
    }

    bool is_publisher_alive(uint64_t max_age_ms = 500) const noexcept {
        if (!base_ || last_write_ns_ == 0) return false;
        uint64_t age = detail::now_ns() - last_write_ns_;
        return age < max_age_ms * 1'000'000ULL;
    }

    const SubscriberStats<T>& stats() const noexcept { return stats_; }

    bool         is_attached() const noexcept { return base_ != nullptr; }
    TopicHeader* header()      const noexcept { return static_cast<TopicHeader*>(base_); }

private:
    SeqlockSlot<T>* slot() const noexcept {
        return reinterpret_cast<SeqlockSlot<T>*>(
            static_cast<uint8_t*>(base_) + sizeof(TopicHeader));
    }

    static std::string to_shm_name(const std::string& n) {
        std::string out = "/sb_";
        for (char c : n) out += (c == '/' ? '_' : c);
        return out;
    }

    void*              base_          = nullptr;
    std::string        name_;
    uint64_t           last_write_ns_ = 0;
    SubscriberStats<T> stats_{};
};

} /* namespace shmbridge */
