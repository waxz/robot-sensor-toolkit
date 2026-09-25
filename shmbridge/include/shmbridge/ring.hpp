/*
 * shmbridge/ring.hpp — SPSC ring buffer over shared memory.
 *
 * RingPublisher<T,N>  — single writer; N must be a power of two.
 * RingSubscriber<T,N> — single reader; attach after publisher opens.
 *
 * Each slot is a plain T (no sequence number needed: SPSC guarantee means the
 * reader only advances after the writer has fully committed a slot).
 *
 * Memory layout (one POSIX shm segment named "<name>_ring"):
 *   RingHeader  (cache-line aligned, 64 bytes)
 *   T data[N]   (immediately follows header, naturally aligned)
 *
 * Indices use the doubled-modulus scheme from commsys:
 *   - empty:  write_idx == read_idx
 *   - full:   (write_idx - read_idx) % (2*N) == N
 *   - used:   (write_idx - read_idx) % (2*N)
 * This avoids ABA ambiguity without wasting one slot.
 *
 * Memory ordering:
 *   Producer: acquire-reads read_idx, then release-stores write_idx.
 *   Consumer: acquire-reads write_idx, then release-stores read_idx.
 */

#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <type_traits>

#if defined(__APPLE__) || defined(__linux__)
#  include <fcntl.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <time.h>   /* nanosleep */
#  include <unistd.h>
#endif

namespace shmbridge {

/* ── helpers ──────────────────────────────────────────────────────────────── */

namespace detail {

/* Convert a human-readable topic name to a safe POSIX shm name with suffix. */
inline std::string to_ring_shm_name(const char* topic) {
    std::string out = "/sbr_";
    for (const char* p = topic; *p; ++p)
        out += (*p == '/' || *p == ' ') ? '_' : *p;
    return out;
}

/* Smallest N that is a power of two and >= requested capacity. */
constexpr uint32_t next_pow2(uint32_t v) {
    if (v == 0) return 1;
    v--;
    v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    return v + 1;
}

static_assert(next_pow2(1) == 1);
static_assert(next_pow2(3) == 4);
static_assert(next_pow2(64) == 64);

} /* namespace detail */

/* ── RingHeader ───────────────────────────────────────────────────────────── */

/*
 * Lives at offset 0 of the shm segment.  Padded to 64 bytes so data[] starts
 * on its own cache line.
 */
struct alignas(64) RingHeader {
    std::atomic<uint64_t> write_idx{0};   /* next slot to write into; offset  0 */
    std::atomic<uint64_t> read_idx{0};    /* next slot to consume;    offset  8 */
    std::atomic<uint8_t>  closed{0};      /* 1 = publisher gone;      offset 16 */
    uint8_t               _pad0[3]{};     /*                          offset 17 */
    uint32_t              capacity{0};    /* N (power of two);        offset 20 */
    uint8_t               _pad1[40]{};   /*                          offset 24 → total 64 */
};
static_assert(sizeof(RingHeader) == 64);
static_assert(alignof(RingHeader) == 64);
static_assert(std::is_trivially_destructible_v<RingHeader>);

/* ── RingPublisher<T,N> ───────────────────────────────────────────────────── */

template <typename T, uint32_t N = 64>
class RingPublisher {
    static_assert(std::is_trivially_copyable_v<T>,
                  "shmbridge ring requires trivially copyable T");
    static_assert((N & (N - 1)) == 0 && N >= 2,
                  "N must be a power of two >= 2");
public:
    RingPublisher() = default;
    ~RingPublisher() { close(); }

    RingPublisher(const RingPublisher&) = delete;
    RingPublisher& operator=(const RingPublisher&) = delete;

    /* Open (create) the ring segment.  Returns true on success. */
    bool open(const char* topic) noexcept {
        shm_name_ = detail::to_ring_shm_name(topic);
        const std::size_t seg_size = sizeof(RingHeader) + N * sizeof(T);

        int fd = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0600);
        if (fd < 0) return false;
        if (::ftruncate(fd, static_cast<off_t>(seg_size)) < 0) {
            ::close(fd);
            return false;
        }
        void* p = ::mmap(nullptr, seg_size, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED) return false;

        /* Zero-init and publish capacity last so subscribers can detect ready. */
        std::memset(p, 0, seg_size);
        hdr_  = static_cast<RingHeader*>(p);
        data_ = reinterpret_cast<T*>(static_cast<char*>(p) + sizeof(RingHeader));
        /* Publish capacity atomically — subscribers peek this to detect init. */
        __atomic_store_n(&hdr_->capacity, N, __ATOMIC_RELEASE);
        return true;
    }

    /* Write one item.  Returns false if the ring is full (drop). */
    bool push(const T& item) noexcept {
        if (!hdr_) return false;
        const uint64_t w  = hdr_->write_idx.load(std::memory_order_relaxed);
        const uint64_t r  = hdr_->read_idx.load(std::memory_order_acquire);
        const uint64_t used = (w - r) % (2u * N);
        if (used >= N) return false;  /* full */

        data_[w % N] = item;
        hdr_->write_idx.store(w + 1, std::memory_order_release);
        return true;
    }

    /* Returns how many slots are currently available for writing. */
    uint32_t available() const noexcept {
        if (!hdr_) return 0;
        const uint64_t w    = hdr_->write_idx.load(std::memory_order_relaxed);
        const uint64_t r    = hdr_->read_idx.load(std::memory_order_acquire);
        const uint64_t used = (w - r) % (2u * N);
        return static_cast<uint32_t>(N - used);
    }

    /* Signal to subscribers that no more data will come. */
    void signal_closed() noexcept {
        if (hdr_) hdr_->closed.store(1, std::memory_order_release);
    }

    bool is_open() const noexcept { return hdr_ != nullptr; }

    void close() noexcept {
        if (!hdr_) return;
        signal_closed();
        const std::size_t seg_size = sizeof(RingHeader) + N * sizeof(T);
        ::munmap(hdr_, seg_size);
        ::shm_unlink(shm_name_.c_str());
        hdr_  = nullptr;
        data_ = nullptr;
    }

private:
    RingHeader* hdr_  = nullptr;
    T*          data_ = nullptr;
    std::string shm_name_;
};

/* ── RingSubscriber<T,N> ──────────────────────────────────────────────────── */

template <typename T, uint32_t N = 64>
class RingSubscriber {
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert((N & (N - 1)) == 0 && N >= 2);
public:
    RingSubscriber() = default;
    ~RingSubscriber() { detach(); }

    RingSubscriber(const RingSubscriber&) = delete;
    RingSubscriber& operator=(const RingSubscriber&) = delete;

    /*
     * Attach to an existing ring segment.  Polls until the publisher has
     * written the capacity field (init-race guard) or timeout_ms elapses.
     * N must match the publisher's N; returns false on mismatch or timeout.
     */
    bool attach(const char* topic, int timeout_ms = 5000) noexcept {
        shm_name_ = detail::to_ring_shm_name(topic);
        const std::size_t seg_size = sizeof(RingHeader) + N * sizeof(T);

        /* O_RDWR required: subscriber writes read_idx to advance the consumer cursor. */
        int fd = ::shm_open(shm_name_.c_str(), O_RDWR, 0);
        if (fd < 0) {
            /* Poll until the publisher creates the segment. */
            const long deadline_ns = static_cast<long>(timeout_ms) * 1'000'000L;
            long waited = 0;
            while (waited < deadline_ns) {
                struct timespec sl{0, 5'000'000L};
                ::nanosleep(&sl, nullptr);
                waited += 5'000'000L;
                fd = ::shm_open(shm_name_.c_str(), O_RDWR, 0);
                if (fd >= 0) break;
            }
            if (fd < 0) return false;
        }

        void* p = ::mmap(nullptr, seg_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED) return false;

        hdr_  = static_cast<RingHeader*>(p);
        data_ = reinterpret_cast<const T*>(static_cast<const char*>(p) + sizeof(RingHeader));

        /* Init-race guard: wait for publisher to commit capacity.
         * Always try once first (fast path for when publisher is already ready). */
        auto check_cap = [&](uint32_t cap) -> int {
            /* returns 1=ok, 0=not ready, -1=mismatch */
            if (cap == 0)   return 0;
            if (cap != N) { ::munmap(hdr_, seg_size); hdr_ = nullptr; data_ = nullptr; return -1; }
            return 1;
        };
        {
            uint32_t cap = __atomic_load_n(&hdr_->capacity, __ATOMIC_ACQUIRE);
            int r = check_cap(cap);
            if (r == 1) return true;
            if (r == -1) return false;
        }
        const long deadline_ns = static_cast<long>(timeout_ms) * 1'000'000L;
        long waited = 0;
        while (waited < deadline_ns) {
            struct timespec sl{0, 1'000'000L};
            ::nanosleep(&sl, nullptr);
            waited += 1'000'000L;
            uint32_t cap = __atomic_load_n(&hdr_->capacity, __ATOMIC_ACQUIRE);
            int r = check_cap(cap);
            if (r == 1) return true;
            if (r == -1) return false;
        }
        ::munmap(hdr_, seg_size);
        hdr_ = nullptr; data_ = nullptr;
        return false;
    }

    /*
     * Pop one item.  Returns nullopt if the ring is empty or the publisher
     * is gone and all items have been consumed.
     */
    std::optional<T> pop() noexcept {
        if (!hdr_) return std::nullopt;
        const uint64_t r = hdr_->read_idx.load(std::memory_order_relaxed);
        const uint64_t w = hdr_->write_idx.load(std::memory_order_acquire);
        if (w == r) return std::nullopt;  /* empty */

        T item;
        std::memcpy(&item, &data_[r % N], sizeof(T));
        hdr_->read_idx.store(r + 1, std::memory_order_release);
        return item;
    }

    /*
     * Drain all available items into a callback f(const T&).
     * Returns the number of items consumed.
     */
    template <typename F>
    uint32_t drain(F&& f) noexcept(noexcept(f(std::declval<const T&>()))) {
        uint32_t count = 0;
        while (auto item = pop()) { f(*item); ++count; }
        return count;
    }

    /*
     * Skip to the newest item, discarding all earlier ones.
     *
     * For SPSC safety the read_idx is advanced to (w-1) BEFORE copying the
     * slot so the publisher cannot recycle that slot while we read it.
     * Returns nullopt if the ring is empty.
     */
    std::optional<T> pop_latest() noexcept {
        if (!hdr_) return std::nullopt;
        const uint64_t r = hdr_->read_idx.load(std::memory_order_relaxed);
        const uint64_t w = hdr_->write_idx.load(std::memory_order_acquire);
        if (w == r) return std::nullopt;
        /* Advance past all but the newest — protects slot (w-1)%N from being
         * recycled by the publisher before we have a chance to read it. */
        if (w - 1 != r)
            hdr_->read_idx.store(w - 1, std::memory_order_release);
        T item;
        std::memcpy(&item, &data_[(w - 1) % N], sizeof(T));
        hdr_->read_idx.store(w, std::memory_order_release);
        return item;
    }

    /*
     * Drop oldest messages so that at most keep_n remain.
     * Advancing read_idx frees ring slots for the publisher immediately,
     * preventing silent message drops when the subscriber is slow.
     * Returns the number of messages discarded.
     */
    uint32_t skip_old(uint32_t keep_n) noexcept {
        if (!hdr_) return 0;
        const uint64_t r     = hdr_->read_idx.load(std::memory_order_relaxed);
        const uint64_t w     = hdr_->write_idx.load(std::memory_order_acquire);
        const uint64_t avail = w - r;
        if (avail <= keep_n) return 0;
        const uint64_t drop = avail - keep_n;
        hdr_->read_idx.store(r + drop, std::memory_order_release);
        return static_cast<uint32_t>(drop);
    }

    /* Peek at the next item without consuming it. */
    std::optional<T> peek() const noexcept {
        if (!hdr_) return std::nullopt;
        const uint64_t r = hdr_->read_idx.load(std::memory_order_relaxed);
        const uint64_t w = hdr_->write_idx.load(std::memory_order_acquire);
        if (w == r) return std::nullopt;
        T item;
        std::memcpy(&item, &data_[r % N], sizeof(T));
        return item;
    }

    /* Number of items currently in the ring. */
    uint32_t size() const noexcept {
        if (!hdr_) return 0;
        const uint64_t w = hdr_->write_idx.load(std::memory_order_acquire);
        const uint64_t r = hdr_->read_idx.load(std::memory_order_relaxed);
        return static_cast<uint32_t>((w - r) % (2u * N));
    }

    bool empty() const noexcept { return size() == 0; }
    bool is_closed() const noexcept {
        return hdr_ && hdr_->closed.load(std::memory_order_acquire);
    }
    bool is_attached() const noexcept { return hdr_ != nullptr; }

    void detach() noexcept {
        if (!hdr_) return;
        const std::size_t seg_size = sizeof(RingHeader) + N * sizeof(T);
        ::munmap(hdr_, seg_size);
        hdr_  = nullptr;
        data_ = nullptr;
    }

private:
    RingHeader* hdr_  = nullptr;   /* non-const: subscriber writes read_idx */
    const T*    data_ = nullptr;
    std::string       shm_name_;
};

} /* namespace shmbridge */
