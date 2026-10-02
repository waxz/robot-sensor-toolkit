/*
 * shmbridge/ring.hpp — SPMC (single-producer, multi-consumer) overwrite-on-
 * full ring buffer over shared memory, zero-copy capable.
 *
 * See docs/design_ring_zero_copy.md for the full design plan this
 * implements. Summary:
 *
 *   RingPublisher<T>  — single writer per topic (enforced, F-11/§5.10);
 *                        push()/reserve()+commit() never fail or block
 *                        (F-1) -- the oldest unread message is overwritten
 *                        once the ring is full.
 *   RingSubscriber<T> — any number of independent readers per topic (F-2),
 *                        each with its own process-local read cursor; no
 *                        reader-coordination state is shared at all.
 *
 * Memory layout (one shared-memory segment named "<topic>_ring"):
 *   RingHeader    (cache-line aligned, 64 bytes, §5.1)
 *   Slot<T>[N]    (immediately follows the header; N = RingConfig::capacity)
 *
 * The live message window is always [start_idx, end_idx), both writer-
 * owned and published explicitly (§5.2) -- a reader only ever compares its
 * own cursor against these two atomics, never re-derives validity from
 * capacity arithmetic, which is what makes adding another independent
 * reader free (no new shared state).
 *
 * Capacity is a runtime RingConfig value (§5.5), not a compile-time
 * template parameter -- resolved from a TOML file (resolve_ring_config())
 * and stamped into the header once, by whichever process creates the
 * segment (the publisher); every other process reads it back from the
 * header rather than from its own config, so publisher and subscriber can
 * never disagree about the ring's size.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "shmbridge/platform.hpp"
#include "shmbridge/topic.hpp" /* reuses type_id<T>() rather than a second mechanism (§5.6) */

#include "third_party/toml.hpp"

namespace shmbridge {

/* Thrown by RingSubscriber::attach() when the publisher doesn't appear
 * within timeout_ms (§3). A real type/schema incompatibility instead
 * throws std::invalid_argument -- a different, non-retriable failure
 * class (§5.6/§5.8, R-7/R-8). */
class TimeoutError : public std::runtime_error {
public:
    explicit TimeoutError(const std::string& what) : std::runtime_error(what) {}
};

/* ── RingConfig (§5.5) ────────────────────────────────────────────────────── */

enum class RingCursorMode {
    StartNow,     /* default: subscriber only sees messages published after attach */
    DrainBacklog, /* subscriber starts at start_idx: catches up on the current backlog */
};

struct RingConfig {
    uint32_t       capacity              = 64;    /* power of two; fixed for the segment's lifetime */
    RingCursorMode cursor_mode           = RingCursorMode::StartNow;
    double         max_age_ms            = 0.0;   /* 0 = staleness check disabled by default */
    double         warn_every_ms         = 2000.0; /* rate limit for resilient-attach warnings (§5.8) */
    uint32_t       max_retries           = 4;     /* pop_ex() torn-read retry bound (§8, R-1) */
    uint32_t       max_takeover_attempts = 4;     /* producer stale-takeover retry bound (§5.10, R-17) */
    double         attach_retry_ms       = 100.0; /* resilient-loop attach-probe cadence (§5.8) */
};

namespace detail {

constexpr uint32_t ring_next_pow2(uint32_t v) {
    if (v == 0) return 1;
    v--;
    v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    return v + 1;
}
static_assert(ring_next_pow2(1) == 1);
static_assert(ring_next_pow2(3) == 4);
static_assert(ring_next_pow2(64) == 64);

inline std::string to_ring_shm_name(const std::string& topic) {
    std::string out = "/sbr_";
    for (char c : topic)
        out += (c == '/' || c == ' ') ? '_' : c;
    return out;
}

/* Applies every recognized key in a TOML table to cfg, leaving any key not
 * present untouched (so [ring.default] followed by [ring."<topic>"] each
 * only override what they actually specify -- §5.5's resolution order). */
inline void apply_ring_config_table(const toml::table* section, RingConfig& cfg) {
    if (!section) return;
    if (auto v = section->get("capacity"))
        if (auto i = v->value<int64_t>()) cfg.capacity = static_cast<uint32_t>(*i);
    if (auto v = section->get("cursor_mode"))
        if (auto s = v->value<std::string>())
            cfg.cursor_mode = (*s == "drain_backlog") ? RingCursorMode::DrainBacklog
                                                       : RingCursorMode::StartNow;
    if (auto v = section->get("max_age_ms"))
        if (auto d = v->value<double>()) cfg.max_age_ms = *d;
    if (auto v = section->get("warn_every_ms"))
        if (auto d = v->value<double>()) cfg.warn_every_ms = *d;
    if (auto v = section->get("max_retries"))
        if (auto i = v->value<int64_t>()) cfg.max_retries = static_cast<uint32_t>(*i);
    if (auto v = section->get("max_takeover_attempts"))
        if (auto i = v->value<int64_t>()) cfg.max_takeover_attempts = static_cast<uint32_t>(*i);
    if (auto v = section->get("attach_retry_ms"))
        if (auto d = v->value<double>()) cfg.attach_retry_ms = *d;
}

} /* namespace detail */

/*
 * Resolution order (first match wins): [ring."<topic_name>"] overrides
 * [ring.default] overrides RingConfig{}'s built-in defaults (§5.5).
 * `config_path` empty means "use the SHMBRIDGE_RING_CONFIG environment
 * variable"; if that's unset too, or the file can't be parsed, this
 * returns the built-in defaults rather than throwing -- config tuning is
 * optional, not a required deployment step (F-8).
 */
inline RingConfig resolve_ring_config(const std::string& topic_name,
                                       const std::string& config_path = {}) {
    RingConfig cfg{};

    std::string path = config_path;
    if (path.empty()) {
        if (const char* env = std::getenv("SHMBRIDGE_RING_CONFIG")) path = env;
    }
    if (path.empty()) return cfg;

    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (...) {
        return cfg; /* malformed/missing config file: fall back to defaults */
    }

    const toml::node* ring_node = tbl.get("ring");
    if (!ring_node || !ring_node->is_table()) return cfg;
    const toml::table& ring = *ring_node->as_table();

    detail::apply_ring_config_table(ring.get("default") ? ring.get("default")->as_table() : nullptr, cfg);
    detail::apply_ring_config_table(ring.get(topic_name) ? ring.get(topic_name)->as_table() : nullptr, cfg);
    return cfg;
}

/* ── RingHeader (§5.1) ────────────────────────────────────────────────────── */

/*
 * Lives at offset 0 of the shm segment, one cache line (64 bytes).
 * `magic` is the cross-process "header fully initialized" signal: the
 * creating publisher writes every other field first, then stores `magic`
 * last with release ordering; a subscriber's probe attach (§5.8) spins on
 * `magic` with acquire ordering before trusting anything else in the
 * header -- the same "publish via a flag" pattern phase 1's `capacity`
 * field used, generalized to the whole header now that there's more to
 * publish atomically than one value.
 */
struct alignas(64) RingHeader {
    static constexpr uint32_t kMagic         = 0x53425247u; /* "SBRG" */
    static constexpr uint32_t kSchemaVersion = 1;

    std::atomic<uint32_t> magic{0};                 /* offset  0 */
    uint32_t              schema_version{0};        /* offset  4 */
    uint64_t              type_hash{0};              /* offset  8 */
    uint32_t               capacity_n{0};             /* offset 16 */
    uint8_t                _pad0[4]{};                /* offset 20 -> 24 */
    std::atomic<uint64_t> start_idx{0};              /* offset 24 */
    std::atomic<uint64_t> end_idx{0};                /* offset 32 */
    volatile uint32_t     notify_seq{0};              /* offset 40 */
    std::atomic<uint8_t>  closed{0};                  /* offset 44 */
    std::atomic<uint8_t>  producer_active{0};         /* offset 45 */
    uint8_t                _pad1[2]{};                /* offset 46 */
    std::atomic<uint32_t> producer_pid{0};            /* offset 48 */
    uint8_t                _pad2[12]{};               /* offset 52 -> 64 */
};
static_assert(sizeof(RingHeader) == 64);
static_assert(alignof(RingHeader) == 64);
static_assert(std::is_trivially_destructible_v<RingHeader>);

/* One ring element: the monotonic publish timestamp (§5.4) plus the
 * payload. Lives in the backing array immediately after RingHeader. */
template <typename T>
struct Slot {
    uint64_t write_ns{0};
    T        value{};
};

/* ── Result<T> (§5.4) ─────────────────────────────────────────────────────── */

template <typename T>
struct Result {
    T        value{};
    uint64_t write_ns{0};

    bool is_stale(double max_age_ms) const noexcept {
        return (platform::now_ns() - write_ns) > static_cast<uint64_t>(max_age_ms * 1e6);
    }
};

/* ── RingSubscriberStats (NFR-4) ──────────────────────────────────────────── */

/*
 * Torn-read retry instrumentation: a sustained non-zero retried_reads/
 * exhausted_reads rate is a signal that this topic's configured capacity
 * (RingConfig::capacity) is undersized for its actual consumer pattern
 * (R-11), not a correctness defect in itself (pop_ex() never returns
 * corrupted data regardless) -- see design_ring_zero_copy.md §9 NFR-4.
 * Mirrors topic.hpp's existing SubscriberStats<T> naming/shape rather than
 * inventing a new instrumentation convention.
 */
struct RingSubscriberStats {
    uint64_t total_reads      = 0; /* every pop_ex() call that found new data */
    uint64_t retried_reads    = 0; /* ...of which needed >=1 torn-read retry  */
    uint64_t exhausted_reads  = 0; /* ...of which exhausted max_retries and gave up without data */
};

/* ── RingPublisher<T> ─────────────────────────────────────────────────────── */

template <typename T>
class RingPublisher {
    static_assert(std::is_trivially_copyable_v<T>,
                  "shmbridge ring requires trivially copyable T");
public:
    RingPublisher() = default;
    ~RingPublisher() { close(); }

    RingPublisher(const RingPublisher&) = delete;
    RingPublisher& operator=(const RingPublisher&) = delete;

    /*
     * Open (or recover) the ring segment and claim producer ownership.
     * Throws std::invalid_argument if `cfg.capacity` isn't a power of two
     * >= 2, or if an existing segment under this name has an incompatible
     * type/schema/capacity. Throws std::runtime_error if another process
     * is verified still alive and already holds this topic (F-11, §5.10),
     * or if stale-producer takeover doesn't converge within
     * cfg.max_takeover_attempts. Returns false only on a lower-level
     * shm/mmap failure.
     */
    bool open(const char* topic, RingConfig cfg = RingConfig{}) {
        if ((cfg.capacity & (cfg.capacity - 1)) != 0 || cfg.capacity < 2) {
            throw std::invalid_argument(
                std::string("ring '") + topic + "': capacity must be a power of two >= 2");
        }
        name_     = topic;
        cfg_      = cfg;
        shm_name_ = detail::to_ring_shm_name(topic);
        const std::size_t seg_size = sizeof(RingHeader) + static_cast<std::size_t>(cfg.capacity) * sizeof(Slot<T>);

        void* p = nullptr;
        bool  created_fresh = false;
        try {
            /* Attach to an existing segment first, if there is one -- NOT
             * shm_create(), which always zero-fills (platform.hpp) and
             * would destroy exactly the evidence claim_producer_slot()
             * needs to tell a crash-without-close() apart from a live
             * producer (R-17/R-18, §5.10/§13). */
            p = platform::shm_attach(shm_name_, seg_size);
        } catch (const std::system_error&) {
            p = platform::shm_create(shm_name_, seg_size);
            created_fresh = true;
        }

        hdr_  = static_cast<RingHeader*>(p);
        data_ = reinterpret_cast<Slot<T>*>(static_cast<char*>(p) + sizeof(RingHeader));

        if (created_fresh) {
            hdr_->schema_version = RingHeader::kSchemaVersion;
            hdr_->type_hash      = type_id<T>();
            hdr_->capacity_n     = cfg.capacity;
        } else {
            /* An existing segment under this name must already agree with
             * what this publisher expects -- a mismatch means the name
             * collides with an incompatible prior use, not a producer to
             * recover from. */
            if (hdr_->schema_version != RingHeader::kSchemaVersion ||
                hdr_->type_hash != type_id<T>() ||
                hdr_->capacity_n != cfg.capacity) {
                platform::shm_unmap(hdr_, seg_size);
                hdr_ = nullptr; data_ = nullptr;
                throw std::invalid_argument(
                    "ring '" + name_ + "': existing segment has incompatible type/schema/capacity");
            }
        }
        capacity_n_ = cfg.capacity;

        claim_producer_slot(); /* F-11, §5.10 -- throws on genuine conflict/non-convergence */

        hdr_->magic.store(RingHeader::kMagic, std::memory_order_release);
        platform::notify_bind(shm_name_, &hdr_->notify_seq);
        return true;
    }

    /* Write one item. Always succeeds (F-1): the oldest unread slot is
     * overwritten once the ring is full. */
    bool push(const T& value) noexcept {
        if (!hdr_) return false;
        const uint64_t idx = hdr_->end_idx.load(std::memory_order_relaxed);
        data_[idx % capacity_n_].write_ns = platform::now_ns();
        std::memcpy(&data_[idx % capacity_n_].value, &value, sizeof(T));
        publish_index(idx);
        return true;
    }

    /*
     * Zero-copy write (F-4). Returns a pointer to construct the next
     * message directly in shared memory; never null while open and
     * unreserved. A second reserve() before the matching commit() is
     * rejected (F-14, §5.9) -- the slot at end_idx is never published
     * until commit() advances the index, so no reader can ever observe a
     * half-written reservation; the guard exists purely to catch a caller
     * bug, not to close a race.
     */
    T* reserve() noexcept {
        if (!hdr_ || reserved_) return nullptr;
        reserved_   = true;
        reserve_idx_ = hdr_->end_idx.load(std::memory_order_relaxed);
        return &data_[reserve_idx_ % capacity_n_].value;
    }

    /* commit() with no prior reserve() is a documented no-op -- never
     * publishes garbage (F-14, §5.9). */
    void commit() noexcept {
        if (!hdr_ || !reserved_) return;
        reserved_ = false;
        data_[reserve_idx_ % capacity_n_].write_ns = platform::now_ns();
        publish_index(reserve_idx_);
    }

    /* Signal to subscribers that no more data will come. Wakes any
     * subscriber currently blocked in pop_wait()/drain_wait() immediately
     * rather than leaving it parked until its own timeout independently
     * expires (R-5). */
    void signal_closed() noexcept {
        if (!hdr_) return;
        hdr_->closed.store(1, std::memory_order_release);
        platform::notify_wake_all(&hdr_->notify_seq);
    }

    bool is_open() const noexcept { return hdr_ != nullptr; }

    void close() noexcept {
        if (!hdr_) return;
        signal_closed();
        hdr_->producer_active.store(0, std::memory_order_release); /* F-11: clean shutdown clears the flag */
        platform::notify_unbind(&hdr_->notify_seq);
        const std::size_t seg_size = sizeof(RingHeader) + static_cast<std::size_t>(capacity_n_) * sizeof(Slot<T>);
        platform::shm_unmap(hdr_, seg_size);
        platform::shm_destroy(shm_name_);
        hdr_  = nullptr;
        data_ = nullptr;
    }

private:
    /* Shared by push()/commit(): publish idx+1 as the new end_idx, evict
     * from the front if that exceeds capacity, and wake blocked readers
     * (§5.2). */
    void publish_index(uint64_t idx) noexcept {
        hdr_->end_idx.store(idx + 1, std::memory_order_release);
        const uint64_t start = hdr_->start_idx.load(std::memory_order_relaxed);
        if (idx + 1 - start > capacity_n_) {
            hdr_->start_idx.store(idx + 1 - capacity_n_, std::memory_order_release);
        }
        platform::notify_wake_all(&hdr_->notify_seq);
    }

    /*
     * Claims this topic for this process. See
     * docs/design_ring_zero_copy.md §5.10 for the full reasoning (PID-
     * reuse race accepted as R-14; concurrent-takeover race closed by the
     * producer_pid CAS, R-17).
     */
    void claim_producer_slot() {
        for (uint32_t attempt = 0; attempt < cfg_.max_takeover_attempts; ++attempt) {
            uint8_t expected = 0;
            if (hdr_->producer_active.compare_exchange_strong(expected, 1, std::memory_order_acq_rel)) {
                hdr_->producer_pid.store(platform::current_pid(), std::memory_order_release);
                return;
            }
            uint32_t other_pid = hdr_->producer_pid.load(std::memory_order_acquire);
            if (platform::process_alive(other_pid)) {
                throw std::runtime_error(
                    "ring '" + name_ + "' already has an active producer (pid=" +
                    std::to_string(other_pid) + ")");
            }
            uint32_t expected_pid = other_pid;
            if (hdr_->producer_pid.compare_exchange_strong(expected_pid, platform::current_pid(),
                                                             std::memory_order_acq_rel)) {
                hdr_->producer_active.store(1, std::memory_order_release);
                return;
            }
            /* Lost the takeover race; loop and re-evaluate from scratch. */
        }
        throw std::runtime_error(
            "ring '" + name_ + "': producer takeover did not converge after " +
            std::to_string(cfg_.max_takeover_attempts) + " attempts");
    }

    RingHeader* hdr_         = nullptr;
    Slot<T>*    data_        = nullptr;
    std::string shm_name_;
    std::string name_;
    RingConfig  cfg_{};
    uint32_t    capacity_n_  = 0;
    bool        reserved_    = false;
    uint64_t    reserve_idx_ = 0;
};

/* ── RingSubscriber<T> ────────────────────────────────────────────────────── */

template <typename T>
class RingSubscriber {
    static_assert(std::is_trivially_copyable_v<T>,
                  "shmbridge ring requires trivially copyable T");
public:
    RingSubscriber() = default;
    ~RingSubscriber() { detach(); }

    RingSubscriber(const RingSubscriber&) = delete;
    RingSubscriber& operator=(const RingSubscriber&) = delete;

    /*
     * Non-blocking attach probe (§5.8). Returns false for "not found yet"
     * (expected, transient, never thrown); throws std::invalid_argument on
     * a type/schema mismatch -- a permanent incompatibility, never
     * silently retried (R-7/R-8).
     */
    bool try_attach(const char* topic, const RingConfig& cfg = RingConfig{}) {
        shm_name_ = detail::to_ring_shm_name(topic);

        void* probe = nullptr;
        try {
            probe = platform::shm_attach(shm_name_, sizeof(RingHeader));
        } catch (const std::system_error&) {
            return false; /* segment doesn't exist yet */
        }
        auto* phdr = static_cast<RingHeader*>(probe);
        uint32_t magic = phdr->magic.load(std::memory_order_acquire);
        if (magic == 0) {
            platform::shm_unmap(probe, sizeof(RingHeader));
            return false; /* segment exists but publisher hasn't finished open() yet */
        }
        if (magic != RingHeader::kMagic ||
            phdr->schema_version != RingHeader::kSchemaVersion ||
            phdr->type_hash != type_id<T>()) {
            platform::shm_unmap(probe, sizeof(RingHeader));
            throw std::invalid_argument(
                std::string("ring '") + topic + "': type/schema mismatch");
        }
        const uint32_t capacity_n = phdr->capacity_n;
        platform::shm_unmap(probe, sizeof(RingHeader));

        const std::size_t seg_size = sizeof(RingHeader) + static_cast<std::size_t>(capacity_n) * sizeof(Slot<T>);
        void* p = nullptr;
        try {
            p = platform::shm_attach(shm_name_, seg_size);
        } catch (const std::system_error&) {
            return false; /* vanished between the probe and the full attach -- treat as "not found yet" */
        }
        hdr_        = static_cast<RingHeader*>(p);
        data_       = reinterpret_cast<const Slot<T>*>(static_cast<const char*>(p) + sizeof(RingHeader));
        capacity_n_ = capacity_n;
        cfg_        = cfg;

        const uint64_t end   = hdr_->end_idx.load(std::memory_order_acquire);
        const uint64_t start = hdr_->start_idx.load(std::memory_order_acquire);
        next_read_ = (cfg.cursor_mode == RingCursorMode::DrainBacklog) ? start : end;

        platform::notify_bind(shm_name_, &hdr_->notify_seq);
        return true;
    }

    /*
     * Blocking attach: polls try_attach() until it succeeds or timeout_ms
     * elapses. Throws TimeoutError on timeout; propagates try_attach()'s
     * std::invalid_argument on a type/schema mismatch unchanged.
     */
    void attach(const char* topic, double timeout_ms, const RingConfig& cfg = RingConfig{}) {
        if (try_attach(topic, cfg)) return;
        /* Poll cadence is cfg.attach_retry_ms (§5.5/§5.8), not a hardcoded
         * constant -- a 1ms floor keeps a pathological 0/negative config
         * value from busy-spinning this loop. */
        const int64_t slice_ns = std::max<int64_t>(
            static_cast<int64_t>(cfg.attach_retry_ms * 1e6), 1'000'000LL);
        const int64_t deadline_ns = static_cast<int64_t>(timeout_ms * 1e6);
        int64_t waited_ns = 0;
        while (waited_ns < deadline_ns) {
            platform::sleep_ns(slice_ns);
            waited_ns += slice_ns;
            if (try_attach(topic, cfg)) return;
        }
        throw TimeoutError(
            std::string("ring '") + topic + "': no publisher within " +
            std::to_string(timeout_ms) + "ms");
    }

    /*
     * Pop one item, retrying a torn read (producer wrapped mid-copy) up to
     * cfg.max_retries times (§5.2, R-1). Returns nullopt if there's no new
     * data, or if retries are exhausted -- never corrupted data.
     */
    std::optional<Result<T>> pop_ex() noexcept {
        if (!hdr_) return std::nullopt;
        {
            const uint64_t end = hdr_->end_idx.load(std::memory_order_acquire);
            if (next_read_ >= end) return std::nullopt; /* no new data -- not a read attempt, not counted */
        }
        stats_.total_reads++;
        for (uint32_t attempt = 0; attempt < cfg_.max_retries; ++attempt) {
            const uint64_t end = hdr_->end_idx.load(std::memory_order_acquire);
            if (next_read_ >= end) return std::nullopt;
            const uint64_t start = hdr_->start_idx.load(std::memory_order_acquire);
            if (next_read_ < start) next_read_ = start; /* fell behind; resync silently (§5.7) */

            const Slot<T>* slot = &data_[next_read_ % capacity_n_];
            Result<T> result;
            result.write_ns = slot->write_ns;
            std::memcpy(&result.value, &slot->value, sizeof(T));

            const uint64_t start_after = hdr_->start_idx.load(std::memory_order_acquire);
            if (start_after > next_read_) {
                if (attempt == 0) stats_.retried_reads++; /* NFR-4: count once per read that needed any retry */
                continue; /* torn: producer evicted this slot mid-copy -- retry */
            }

            next_read_ += 1;
            return result;
        }
        stats_.exhausted_reads++;
        const uint64_t start = hdr_->start_idx.load(std::memory_order_acquire);
        if (next_read_ < start) next_read_ = start;
        return std::nullopt; /* retries exhausted: resync and report "no data" this cycle, not corrupted data */
    }

    /* Torn-read retry instrumentation (NFR-4, R-11) -- see RingSubscriberStats. */
    const RingSubscriberStats& stats() const noexcept { return stats_; }

    /*
     * Block (OS wait primitive, not a spin loop) until an item is
     * available or timeout_ms elapses, then pop it. timeout_ms is always
     * finite -- this call always returns, never blocks forever (F-3).
     */
    std::optional<Result<T>> pop_wait(int timeout_ms) noexcept {
        if (!hdr_) return std::nullopt;
        if (timeout_ms < 0) timeout_ms = 0;
        const uint32_t seq = notify_seq_load();
        if (auto item = pop_ex()) return item;
        platform::notify_wait(&hdr_->notify_seq, seq, timeout_ms);
        return pop_ex();
    }

    /*
     * Jumps straight to the newest message, discarding everything between
     * (F-13). Never retries a torn read -- the cursor still jumps to
     * end_idx either way, since re-delivering a stale skipped message
     * would defeat the "what's current" intent.
     *
     * Stack-size note for large T: this returns Result<T> *by value*,
     * which means a full T as a stack-local inside this function (same
     * for pop_ex()). For a small T that's free; for a large one (e.g.
     * msg::PointCloud65536, ~1 MiB) it is NOT -- it fits Linux's default
     * 8 MiB thread stack but can overflow Windows' default 1 MiB one, a
     * crash with no exception and no Python traceback if called through
     * pybind11 (found migrating ext_topics.hpp's PointCloudSubscriber
     * onto this type; see design_ring_zero_copy.md §16/§17). For T sizes
     * near or above a few hundred KiB, prefer borrow()/end_borrow()
     * (zero-copy, no T-sized local ever) and copy only the valid
     * sub-range (e.g. T::n_points worth) into caller-owned storage.
     */
    std::optional<Result<T>> pop_latest() noexcept {
        if (!hdr_) return std::nullopt;
        const uint64_t end = hdr_->end_idx.load(std::memory_order_acquire);
        if (next_read_ >= end) return std::nullopt;
        const uint64_t target = end - 1;

        const Slot<T>* slot = &data_[target % capacity_n_];
        Result<T> result;
        result.write_ns = slot->write_ns;
        std::memcpy(&result.value, &slot->value, sizeof(T));

        const uint64_t start_after = hdr_->start_idx.load(std::memory_order_acquire);
        next_read_ = end;
        if (start_after > target) return std::nullopt; /* torn; no retry (F-13) */
        return result;
    }

    /* Drains everything available at call time into f(value, write_ns),
     * bounded by a snapshot of end_idx so a producer publishing during the
     * drain can't make this loop unbounded (§5.2/§3). */
    template <typename F>
    uint32_t drain_ex(F&& f) noexcept(
        noexcept(f(std::declval<const T&>(), std::declval<uint64_t>()))) {
        if (!hdr_) return 0;
        const uint64_t end_snapshot = hdr_->end_idx.load(std::memory_order_acquire);
        uint32_t count = 0;
        while (next_read_ < end_snapshot) {
            auto item = pop_ex();
            if (!item) break;
            f(item->value, item->write_ns);
            ++count;
        }
        return count;
    }

    /*
     * Zero-copy read (F-5): a direct pointer into shared memory, no copy.
     * A second borrow() before the matching end_borrow() is rejected
     * (F-14, §5.9).
     */
    const T* borrow(uint64_t* write_ns_out = nullptr) noexcept {
        if (!hdr_ || borrowed_) return nullptr;
        const uint64_t end = hdr_->end_idx.load(std::memory_order_acquire);
        if (next_read_ >= end) return nullptr;
        const uint64_t start = hdr_->start_idx.load(std::memory_order_acquire);
        if (next_read_ < start) next_read_ = start;

        const Slot<T>* slot = &data_[next_read_ % capacity_n_];
        if (write_ns_out) *write_ns_out = slot->write_ns;
        borrowed_   = true;
        borrow_idx_ = next_read_;
        return &slot->value;
    }

    /* end_borrow() with no prior successful borrow() is a documented
     * no-op -- never advances the cursor speculatively (F-14, §5.9). The
     * cursor only advances if the boundary re-check passes (not torn). */
    bool end_borrow() noexcept {
        if (!hdr_ || !borrowed_) return false;
        borrowed_ = false;
        const uint64_t start_after = hdr_->start_idx.load(std::memory_order_acquire);
        if (start_after > borrow_idx_) return false; /* torn while borrowed -- do not advance */
        next_read_ = borrow_idx_ + 1;
        return true;
    }

    bool is_closed() const noexcept {
        return hdr_ && hdr_->closed.load(std::memory_order_acquire);
    }
    bool is_attached() const noexcept { return hdr_ != nullptr; }

    void detach() noexcept {
        if (!hdr_) return;
        platform::notify_unbind(&hdr_->notify_seq);
        const std::size_t seg_size = sizeof(RingHeader) + static_cast<std::size_t>(capacity_n_) * sizeof(Slot<T>);
        platform::shm_unmap(hdr_, seg_size);
        hdr_      = nullptr;
        data_     = nullptr;
        borrowed_ = false;
    }

private:
    /* Acquire-load of notify_seq, matching topic.hpp's wait()/wait_new(). */
    uint32_t notify_seq_load() const noexcept {
        return reinterpret_cast<const std::atomic<uint32_t>*>(
            const_cast<const uint32_t*>(&hdr_->notify_seq))
            ->load(std::memory_order_acquire);
    }

    RingHeader*    hdr_        = nullptr;
    const Slot<T>* data_       = nullptr;
    std::string    shm_name_;
    RingConfig     cfg_{};
    uint32_t       capacity_n_ = 0;
    uint64_t       next_read_  = 0;
    bool           borrowed_   = false;
    uint64_t       borrow_idx_ = 0;
    RingSubscriberStats stats_{};
};

} /* namespace shmbridge */
