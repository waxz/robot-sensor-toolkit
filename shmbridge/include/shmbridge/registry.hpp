/*
 * shmbridge/registry.hpp — same-host node discovery for shmbridge.
 *
 * A flat array of NodeSlot[MAX_NODES] lives in a single shm segment
 * ("/sb_discovery").  No central master is needed; any process can read the
 * full table.
 *
 * Slot lifecycle (atomic, avoids TOCTOU):
 *   0 = free
 *   2 = reserving (CAS 0→2, write fields, then 2→1)
 *   1 = active
 *
 * Liveness: dual check — heartbeat TTL AND kill(pid, 0) (EPERM means alive).
 *
 * Topic encoding in NodeSlot::topics[]:
 *   "pub=foo,bar|sub=~baz,qux"
 *   '~' prefix on a sub topic = wants keep_latest (seqlock slot).
 *   No prefix on a sub topic  = wants FIFO ring.
 */

#pragma once

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#  include <cerrno>
#  include <fcntl.h>
#  include <signal.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <unistd.h>
#endif

namespace shmbridge {

/* ── constants ────────────────────────────────────────────────────────────── */

constexpr int    MAX_NODES        = 64;
constexpr int    NODE_ID_LEN      = 64;
constexpr int    TOPICS_LEN       = 384;
constexpr double HEARTBEAT_TTL_S  = 5.0;

static const char* DISCOVERY_SHM = "/sb_discovery";

/* ── helpers ──────────────────────────────────────────────────────────────── */

/* File-scope anonymous namespace: avoids ODR conflict with topic.hpp's
 * shmbridge::reg_now_ns() when both headers appear in the same TU. */
namespace {
inline uint64_t reg_now_ns() noexcept {
    struct timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}
} /* anonymous namespace */

namespace detail {

/* Returns true if process pid is alive on this host. */
inline bool pid_alive(int pid) noexcept {
    if (pid <= 0) return false;
    int rc = ::kill(pid, 0);
    if (rc == 0)          return true;   /* success — process exists        */
    if (errno == EPERM)   return true;   /* exists but we lack permission   */
    /* ESRCH — process gone; any other error is treated as gone */
    return false;
}

/* Encode published / subscribed topics into NodeSlot::topics format. */
inline std::string encode_topics(const std::vector<std::string>& pubs,
                                  const std::vector<std::string>& subs_keep_latest,
                                  const std::vector<std::string>& subs_ring) {
    std::string out;
    if (!pubs.empty()) {
        out += "pub=";
        for (std::size_t i = 0; i < pubs.size(); ++i) {
            if (i) out += ',';
            out += pubs[i];
        }
    }
    std::vector<std::string> all_subs;
    for (auto& s : subs_keep_latest) all_subs.push_back("~" + s);
    for (auto& s : subs_ring)        all_subs.push_back(s);
    if (!all_subs.empty()) {
        if (!out.empty()) out += '|';
        out += "sub=";
        for (std::size_t i = 0; i < all_subs.size(); ++i) {
            if (i) out += ',';
            out += all_subs[i];
        }
    }
    return out;
}

struct TopicLists {
    std::vector<std::string> pubs;
    std::vector<std::string> subs_keep_latest;
    std::vector<std::string> subs_ring;
};

/* Parse NodeSlot::topics string into separate lists. */
inline TopicLists decode_topics(const char* topics_str) {
    TopicLists out;
    std::string s(topics_str);
    /* Split on '|'. */
    std::size_t pos = 0;
    while (pos <= s.size()) {
        std::size_t pipe = s.find('|', pos);
        if (pipe == std::string::npos) pipe = s.size();
        std::string part = s.substr(pos, pipe - pos);
        pos = pipe + 1;
        if (part.empty()) continue;

        bool is_pub = (part.rfind("pub=", 0) == 0);
        bool is_sub = (part.rfind("sub=", 0) == 0);
        std::string list_str = is_pub ? part.substr(4)
                             : is_sub ? part.substr(4) : "";
        if (list_str.empty()) continue;

        /* Split on ','. */
        std::size_t lpos = 0;
        while (lpos <= list_str.size()) {
            std::size_t comma = list_str.find(',', lpos);
            if (comma == std::string::npos) comma = list_str.size();
            std::string tok = list_str.substr(lpos, comma - lpos);
            lpos = comma + 1;
            if (tok.empty()) continue;
            if (is_pub) {
                out.pubs.push_back(tok);
            } else if (is_sub) {
                if (!tok.empty() && tok[0] == '~')
                    out.subs_keep_latest.push_back(tok.substr(1));
                else
                    out.subs_ring.push_back(tok);
            }
        }
    }
    return out;
}

} /* namespace detail */

/* ── NodeSlot ─────────────────────────────────────────────────────────────── */

/*
 * Natural alignment — no #pragma pack.  std::atomic requires it.
 * Each slot is padded to a cache line boundary (64 bytes * k).
 */
struct NodeSlot {
    std::atomic<uint8_t> active{0};           /* 0=free, 2=reserving, 1=active */
    uint8_t              _pad0[3]{};
    int32_t              pid{0};
    char                 node_id[NODE_ID_LEN]{};
    uint64_t             last_heartbeat_ns{0};
    char                 topics[TOPICS_LEN]{};
    uint8_t              _pad1[8]{};          /* round to 8-byte boundary */
};

static_assert(std::is_trivially_destructible_v<std::atomic<uint8_t>>);

/* ── DiscoveryRegistry ────────────────────────────────────────────────────── */

/*
 * Thin RAII wrapper around the "/sb_discovery" shared-memory segment.
 * Call open() once per process.  The segment is created on first open and
 * never unlinked (persists for the lifetime of the host session).
 */
class DiscoveryRegistry {
public:
    DiscoveryRegistry() = default;
    ~DiscoveryRegistry() { close(); }

    DiscoveryRegistry(const DiscoveryRegistry&)            = delete;
    DiscoveryRegistry& operator=(const DiscoveryRegistry&) = delete;

    bool open() noexcept {
        const std::size_t seg = sizeof(NodeSlot) * MAX_NODES;
        /* Try create first, fall back to attach. */
        int fd = ::shm_open(DISCOVERY_SHM, O_CREAT | O_RDWR, 0666);
        if (fd < 0) return false;
        if (::ftruncate(fd, static_cast<off_t>(seg)) < 0) { ::close(fd); return false; }
        void* p = ::mmap(nullptr, seg, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p == MAP_FAILED) return false;
        slots_ = static_cast<NodeSlot*>(p);
        seg_size_ = seg;
        return true;
    }

    bool is_open() const noexcept { return slots_ != nullptr; }

    /*
     * Claim a free slot for node_id.  Returns the slot index on success,
     * -1 if no free slot or already registered.
     *
     * Algorithm (TOCTOU-safe):
     *   for each slot:
     *     CAS active: 0 → 2 (reserve)
     *     if success: write fields, then store active = 1
     */
    int register_node(const char* node_id,
                      const std::vector<std::string>& pubs,
                      const std::vector<std::string>& subs_keep_latest,
                      const std::vector<std::string>& subs_ring) noexcept {
        if (!slots_) return -1;
        const std::string enc = detail::encode_topics(pubs, subs_keep_latest, subs_ring);

        for (int i = 0; i < MAX_NODES; ++i) {
            uint8_t expected = 0;
            if (!slots_[i].active.compare_exchange_strong(
                    expected, 2,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                continue;  /* slot taken or being reserved */
            }
            /* We own the slot exclusively now — write fields. */
            slots_[i].pid = static_cast<int32_t>(::getpid());
            std::strncpy(slots_[i].node_id, node_id, NODE_ID_LEN - 1);
            slots_[i].node_id[NODE_ID_LEN - 1] = '\0';
            slots_[i].last_heartbeat_ns = reg_now_ns();
            std::strncpy(slots_[i].topics, enc.c_str(), TOPICS_LEN - 1);
            slots_[i].topics[TOPICS_LEN - 1] = '\0';
            /* Publish — make slot visible. */
            slots_[i].active.store(1, std::memory_order_release);
            slot_idx_ = i;
            return i;
        }
        return -1;  /* table full */
    }

    /* Update heartbeat for a previously registered slot. */
    void heartbeat(int slot_idx) noexcept {
        if (!slots_ || slot_idx < 0 || slot_idx >= MAX_NODES) return;
        if (slots_[slot_idx].active.load(std::memory_order_relaxed) != 1) return;
        slots_[slot_idx].last_heartbeat_ns = reg_now_ns();
    }

    /* Update the topic list for a previously registered slot. */
    void update_topics(int slot_idx,
                       const std::vector<std::string>& pubs,
                       const std::vector<std::string>& subs_keep_latest,
                       const std::vector<std::string>& subs_ring) noexcept {
        if (!slots_ || slot_idx < 0 || slot_idx >= MAX_NODES) return;
        const std::string enc = detail::encode_topics(pubs, subs_keep_latest, subs_ring);
        std::strncpy(slots_[slot_idx].topics, enc.c_str(), TOPICS_LEN - 1);
        slots_[slot_idx].topics[TOPICS_LEN - 1] = '\0';
    }

    /* Free a previously registered slot. */
    void unregister_node(int slot_idx) noexcept {
        if (!slots_ || slot_idx < 0 || slot_idx >= MAX_NODES) return;
        slots_[slot_idx].active.store(0, std::memory_order_release);
        slot_idx_ = -1;
    }

    struct NodeInfo {
        int         slot;
        int32_t     pid;
        std::string node_id;
        double      age_s;          /* seconds since last heartbeat */
        detail::TopicLists topics;
    };

    /* Return all live nodes.  ttl_sec defaults to HEARTBEAT_TTL_S. */
    std::vector<NodeInfo> list_active(double ttl_sec = HEARTBEAT_TTL_S,
                                       int exclude_slot = -1) const noexcept {
        if (!slots_) return {};
        std::vector<NodeInfo> out;
        const uint64_t now = reg_now_ns();
        for (int i = 0; i < MAX_NODES; ++i) {
            if (i == exclude_slot) continue;
            if (slots_[i].active.load(std::memory_order_acquire) != 1) continue;
            int32_t  pid = slots_[i].pid;
            uint64_t hb  = slots_[i].last_heartbeat_ns;
            double   age = static_cast<double>(now - hb) / 1e9;
            /* Dual liveness: TTL AND kill(pid,0). */
            if (age > ttl_sec) continue;
            if (!detail::pid_alive(pid)) continue;
            NodeInfo ni;
            ni.slot    = i;
            ni.pid     = pid;
            ni.node_id = slots_[i].node_id;
            ni.age_s   = age;
            ni.topics  = detail::decode_topics(slots_[i].topics);
            out.push_back(std::move(ni));
        }
        return out;
    }

    /* Convenience: find all publishers of a given topic across live nodes. */
    std::vector<NodeInfo> find_publishers(const std::string& topic) const noexcept {
        std::vector<NodeInfo> out;
        for (auto& n : list_active()) {
            for (auto& p : n.topics.pubs)
                if (p == topic) { out.push_back(n); break; }
        }
        return out;
    }

    int  slot_idx() const noexcept { return slot_idx_; }

    void close() noexcept {
        if (!slots_) return;
        if (slot_idx_ >= 0) unregister_node(slot_idx_);
        ::munmap(slots_, seg_size_);
        slots_    = nullptr;
        seg_size_ = 0;
    }

private:
    NodeSlot*   slots_    = nullptr;
    std::size_t seg_size_ = 0;
    int         slot_idx_ = -1;
};

} /* namespace shmbridge */
