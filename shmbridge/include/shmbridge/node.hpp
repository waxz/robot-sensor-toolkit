/*
 * shmbridge/node.hpp — ROS 2-style Node API built on shmbridge.
 *
 * Mirrors the rclcpp surface closely enough that porting is mechanical:
 *
 *   // rclcpp                              // shmbridge::ros_compat
 *   rclcpp::init(argc, argv);              shmbridge::ros_compat::init();
 *   auto node = rclcpp::Node("cam");       auto node = ros_compat::make_node("cam");
 *   auto pub  = node->create_publisher     auto pub  = node->create_publisher
 *                 <Pose2d>("pose", 10);                <Pose2d>("pose", 10);
 *   auto sub  = node->create_subscription  auto sub  = node->create_subscription
 *                 <Pose2d>("pose", 10, cb);            <Pose2d>("pose", 10, cb);
 *   rclcpp::spin(node);                    ros_compat::spin(node);
 *   rclcpp::shutdown();                    ros_compat::shutdown();
 *
 * Differences from rclcpp worth noting:
 *
 *   - Subscription<T> is a stateless handle.  The callback fires from
 *     spin_once() in the same thread — no executor thread pool.
 *   - QoS(depth<=1) maps to a seqlock (keep-latest) topic; depth>1 maps to a
 *     SPSC ring.  Reliable vs BestEffort is currently treated the same.
 *   - create_publisher() opens the seqlock / ring segment immediately.
 *     create_subscription() attaches at spin_once() once a publisher is
 *     found via the discovery registry.  This matches the commsys design.
 *   - make_node() registers the node in the discovery registry and starts it.
 *     spin_once() sends a heartbeat (rate-limited to 50 ms), polls for new
 *     publishers (at most every 100 ms per sub), and drains all attached subs.
 *
 * Queue-based subscription (deferred callback pattern):
 *
 *   auto [sub, q] = node->create_queue<Pose2d>("pose", SensorDataQoS());
 *   // spin_once() accumulates messages into q; caller drains at their own rate:
 *   node->spin_once();
 *   if (auto msg = q->pop_latest()) { ... }   // keep-latest read
 *   q->drain([](const Pose2d& m){ ... });      // process all queued
 *
 * Direct publish via handle (zero-overhead path):
 *
 *   auto pub = node->create_publisher<Pose2d>("pose", SensorDataQoS());
 *   pub->publish(msg);          // no map lookup, no cast
 *   // or equivalently:
 *   node->publish(pub, msg);    // delegates to pub->publish
 */

#pragma once

#include "shmbridge/registry.hpp"
#include "shmbridge/ring.hpp"
#include "shmbridge/topic.hpp"

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace shmbridge::ros_compat {

/* ── QoS ──────────────────────────────────────────────────────────────────── */

class QoS {
public:
    explicit QoS(uint32_t depth = 10) : depth_(depth) {}

    /* depth <= 1 → seqlock (keep-latest); depth > 1 → SPSC ring. */
    bool wants_keep_latest() const noexcept { return depth_ <= 1; }
    uint32_t depth() const noexcept { return depth_; }

    QoS& best_effort() noexcept { reliable_ = false; return *this; }
    QoS& reliable()    noexcept { reliable_ = true;  return *this; }
    bool is_reliable() const noexcept { return reliable_; }

private:
    uint32_t depth_    = 10;
    bool     reliable_ = true;
};

inline QoS SensorDataQoS()     { return QoS(1).best_effort(); }
inline QoS SystemDefaultsQoS() { return QoS(10).reliable();   }

/* ── MessageQueue<T> — bounded deque for deferred callback processing ─────── */

/*
 * Accumulates messages pushed by spin_once() into a bounded circular deque.
 * The caller drains at their own pace, fully decoupling processing latency
 * from the spin cycle.  NOT thread-safe: push() and pop/drain() must be
 * called from the same thread (or guarded externally).
 */
template <typename T>
class MessageQueue {
public:
    explicit MessageQueue(std::size_t capacity = 32) : capacity_(capacity) {}

    /* Called by spin_once() — O(1).  Drops oldest if at capacity. */
    void push(const T& msg) noexcept {
        if (queue_.size() >= capacity_) queue_.pop_front();
        queue_.push_back(msg);
    }

    /* Pop the oldest message (FIFO).  Returns nullopt if empty. */
    std::optional<T> pop() noexcept {
        if (queue_.empty()) return std::nullopt;
        T v = std::move(queue_.front());
        queue_.pop_front();
        return v;
    }

    /* Pop the newest message and discard all others (keep-latest semantics). */
    std::optional<T> pop_latest() noexcept {
        if (queue_.empty()) return std::nullopt;
        T v = std::move(queue_.back());
        queue_.clear();
        return v;
    }

    /* Process all queued messages in arrival order, then clear. */
    template <typename CB>
    void drain(CB&& cb) noexcept {
        while (!queue_.empty()) {
            cb(queue_.front());
            queue_.pop_front();
        }
    }

    bool        empty()    const noexcept { return queue_.empty(); }
    std::size_t size()     const noexcept { return queue_.size(); }
    void        clear()    noexcept       { queue_.clear(); }

private:
    std::deque<T>  queue_;
    std::size_t    capacity_;
};

/* ── LatestSlot<T> — single-message keep-latest store, no heap allocation ── */

/*
 * Stores exactly one message: each store() overwrites the previous value.
 * take() returns the stored value and clears the slot in O(1).
 * No std::deque, no heap allocation in the hot path.
 *
 * NOT thread-safe: store() (called from spin_once()) and take()/peek()
 * (called by the user) must execute from the same thread, or be guarded.
 */
template <typename T>
class LatestSlot {
public:
    void store(const T& msg) noexcept { value_ = msg; has_value_ = true; }

    std::optional<T> take() noexcept {
        if (!has_value_) return std::nullopt;
        has_value_ = false;
        return value_;
    }

    const T* peek()      const noexcept { return has_value_ ? &value_ : nullptr; }
    bool     has_value() const noexcept { return has_value_; }
    void     clear()           noexcept { has_value_ = false; }

private:
    T    value_{};
    bool has_value_{false};
};

/* ── Subscription handle (stateless, user-visible) ───────────────────────── */

/*
 * Unlike rclcpp::Subscription, this is a thin handle.  The Node owns the
 * actual subscriber state; this object just carries the topic name.
 */
template <typename T>
class Subscription {
public:
    explicit Subscription(std::string topic) : topic_(std::move(topic)) {}
    const std::string& topic() const noexcept { return topic_; }
private:
    std::string topic_;
};

/* ── Publisher handle (carries the actual publisher — zero-overhead publish) ─ */

/*
 * Publisher<T> now owns the underlying shmbridge::Publisher<T> or
 * shmbridge::RingPublisher<T>.  Calling pub->publish(msg) incurs no map
 * lookup and no shared_ptr cast — it goes straight to write_notify/push.
 *
 * The legacy node->publish<T>("topic", msg) string-based path still works
 * for backward compatibility, but pub->publish(msg) is preferred.
 */
template <typename T>
class Publisher {
public:
    Publisher(std::string topic, bool keep_latest,
              std::shared_ptr<shmbridge::Publisher<T>>     seqlock_pub,
              std::shared_ptr<shmbridge::RingPublisher<T>> ring_pub)
        : topic_(std::move(topic)), keep_latest_(keep_latest),
          seqlock_pub_(std::move(seqlock_pub)),
          ring_pub_(std::move(ring_pub)) {}

    const std::string& topic()       const noexcept { return topic_; }
    bool               keep_latest() const noexcept { return keep_latest_; }

    /* Zero-overhead publish: no map lookup, no cast. */
    void publish(const T& msg) noexcept {
        if (seqlock_pub_) seqlock_pub_->write_notify(msg);
        else if (ring_pub_) ring_pub_->push(msg);
    }

private:
    std::string topic_;
    bool        keep_latest_;
    std::shared_ptr<shmbridge::Publisher<T>>     seqlock_pub_;
    std::shared_ptr<shmbridge::RingPublisher<T>> ring_pub_;
};

/* ── Internal type-erased subscriber slot ─────────────────────────────────── */

namespace detail {

struct SubSlot {
    std::string             topic;
    bool                    keep_latest;
    bool                    attached{false};
    /* Non-zero after first attempt; retried at most every ATTACH_RETRY_NS. */
    int64_t                 last_attach_try_ns{0};
    std::function<void()>   poll_fn;      /* drain or read_if_new; calls user cb */
    std::function<bool()>   try_attach;   /* non-blocking attempt (timeout=0)   */
};

} /* namespace detail */

/* ── Node ─────────────────────────────────────────────────────────────────── */

class Node {
    /* Heartbeat sent at most once every 50 ms. */
    static constexpr int64_t HEARTBEAT_INTERVAL_NS = 50'000'000LL;
    /* Attach attempt retried at most once every 100 ms per subscriber. */
    static constexpr int64_t ATTACH_RETRY_NS       = 100'000'000LL;

public:
    explicit Node(std::string name) : name_(std::move(name)) {}
    ~Node() { stop(); }

    Node(const Node&)            = delete;
    Node& operator=(const Node&) = delete;

    const std::string& name() const noexcept { return name_; }

    /* Called by make_node(); do not call directly. */
    bool start() noexcept {
        return registry_.open();
    }

    /* ── create_publisher ─────────────────────────────────────────────────── */

    template <typename T>
    std::shared_ptr<Publisher<T>>
    create_publisher(const std::string& topic, uint32_t depth = 10) {
        return create_publisher<T>(topic, QoS(depth));
    }

    template <typename T>
    std::shared_ptr<Publisher<T>>
    create_publisher(const std::string& topic, const QoS& qos) {
        bool kl = qos.wants_keep_latest();
        std::shared_ptr<shmbridge::Publisher<T>>     sp;
        std::shared_ptr<shmbridge::RingPublisher<T>> rp;

        if (kl) {
            sp = std::make_shared<shmbridge::Publisher<T>>();
            if (sp->open(topic) != TopicError::Ok)
                throw std::runtime_error("shmbridge: failed to open publisher for " + topic);
            seqlock_pubs_[topic] = sp;   /* keep alive */
        } else {
            rp = std::make_shared<shmbridge::RingPublisher<T>>();
            if (!rp->open(topic.c_str()))
                throw std::runtime_error("shmbridge: failed to open ring publisher for " + topic);
            ring_pubs_[topic] = rp;      /* keep alive */
        }

        pub_topics_.push_back(topic);
        refresh_registry();

        /* Publisher handle owns the actual publisher — zero-overhead path. */
        return std::make_shared<Publisher<T>>(topic, kl, sp, rp);
    }

    /* ── publish ─────────────────────────────────────────────────────────── */

    /* Fast path: delegates directly to the handle's embedded publisher. */
    template <typename T>
    void publish(const std::shared_ptr<Publisher<T>>& pub, const T& msg) {
        pub->publish(msg);
    }

    /* Legacy string-based path (backward-compatible; incurs map lookup). */
    template <typename T>
    void publish(const std::string& topic, const T& msg) {
        {
            auto it = seqlock_pubs_.find(topic);
            if (it != seqlock_pubs_.end()) {
                auto pub = std::static_pointer_cast<shmbridge::Publisher<T>>(it->second);
                pub->write_notify(msg);
                return;
            }
        }
        {
            auto it = ring_pubs_.find(topic);
            if (it != ring_pubs_.end()) {
                auto pub = std::static_pointer_cast<shmbridge::RingPublisher<T>>(it->second);
                pub->push(msg);
            }
        }
    }

    /* ── create_subscription ──────────────────────────────────────────────── */

    template <typename T, typename CB>
    std::shared_ptr<Subscription<T>>
    create_subscription(const std::string& topic, uint32_t depth, CB&& cb) {
        return create_subscription<T>(topic, QoS(depth), std::forward<CB>(cb));
    }

    template <typename T, typename CB>
    std::shared_ptr<Subscription<T>>
    create_subscription(const std::string& topic, const QoS& qos, CB&& cb) {
        detail::SubSlot slot;
        slot.topic        = topic;
        slot.keep_latest  = qos.wants_keep_latest();

        if (slot.keep_latest) {
            auto sub     = std::make_shared<shmbridge::Subscriber<T>>();
            auto cb_copy = std::function<void(const T&)>(std::forward<CB>(cb));
            slot.try_attach = [sub, topic]() -> bool {
                return sub->attach(topic.c_str(), 0) == TopicError::Ok;
            };
            slot.poll_fn = [sub, cb_copy]() {
                if (auto r = sub->read_if_new()) cb_copy(r->value);
            };
            seqlock_subs_[topic] = sub;
        } else {
            auto sub     = std::make_shared<shmbridge::RingSubscriber<T>>();
            auto cb_copy = std::function<void(const T&)>(std::forward<CB>(cb));
            slot.try_attach = [sub, topic]() -> bool {
                return sub->attach(topic.c_str(), 0);
            };
            /* Enforce backlog limit before draining: if the subscriber is
             * slower than the publisher, skip_old() advances read_idx past
             * stale messages so the ring never stays full and the publisher
             * never silently drops new writes due to overflow. */
            slot.poll_fn = [sub, cb_copy, keep_n = qos.depth()]() {
                sub->skip_old(keep_n);
                sub->drain([&](const T& msg) { cb_copy(msg); });
            };
            ring_subs_[topic] = sub;
        }

        sub_topics_.push_back(topic);
        refresh_registry();
        sub_slots_.push_back(std::move(slot));
        return std::make_shared<Subscription<T>>(topic);
    }

    /* ── create_queue ─────────────────────────────────────────────────────── */

    /*
     * Queue-based subscription: spin_once() accumulates messages into the
     * returned MessageQueue<T> instead of firing a callback inline.
     * The caller drains the queue at their own pace between spin cycles.
     *
     *   auto [sub, q] = node->create_queue<Pose2d>("pose", SensorDataQoS());
     *   while (running) {
     *       node->spin_once();
     *       if (auto m = q->pop_latest()) process(*m);
     *   }
     */
    template <typename T>
    std::pair<std::shared_ptr<Subscription<T>>, std::shared_ptr<MessageQueue<T>>>
    create_queue(const std::string& topic, const QoS& qos = QoS(10)) {
        std::size_t cap = qos.depth() > 0 ? static_cast<std::size_t>(qos.depth()) : 32u;
        auto q   = std::make_shared<MessageQueue<T>>(cap);
        auto sub = create_subscription<T>(topic, qos,
                       [q](const T& msg) { q->push(msg); });
        return {sub, q};
    }

    /* ── create_latest ───────────────────────────────────────────────────── */

    /*
     * Single-message subscription backed by LatestSlot<T>.
     * spin_once() overwrites the slot with the newest available message;
     * the caller reads it with slot->take() or slot->peek().
     *
     *   auto [sub, slot] = node->create_latest<Pose2d>("pose");
     *   while (running) {
     *       node->spin_once();
     *       if (auto m = slot->take()) process(*m);
     *   }
     *
     * For seqlock topics (depth <= 1) the callback approach is used — the
     * seqlock already gives keep-latest semantics via read_if_new().
     * For ring topics (depth > 1) pop_latest() is used inside poll_fn,
     * skipping all but the newest item in O(1) — no drain loop, no deque.
     */
    template <typename T>
    std::pair<std::shared_ptr<Subscription<T>>, std::shared_ptr<LatestSlot<T>>>
    create_latest(const std::string& topic, const QoS& qos = SensorDataQoS()) {
        auto slot_ptr = std::make_shared<LatestSlot<T>>();

        if (qos.wants_keep_latest()) {
            /* Seqlock path: delegate to create_subscription, which already
             * calls read_if_new() — inherently keep-latest. */
            auto sub = create_subscription<T>(topic, qos,
                           [slot_ptr](const T& msg) { slot_ptr->store(msg); });
            return {sub, slot_ptr};
        }

        /* Ring path: custom poll_fn uses pop_latest() to skip the backlog
         * and take only the single newest item in O(1). */
        auto raw_sub = std::make_shared<shmbridge::RingSubscriber<T>>();

        detail::SubSlot sub_slot;
        sub_slot.topic       = topic;
        sub_slot.keep_latest = false;
        sub_slot.try_attach  = [raw_sub, topic]() -> bool {
            return raw_sub->attach(topic.c_str(), 0);
        };
        sub_slot.poll_fn = [raw_sub, slot_ptr]() {
            if (auto item = raw_sub->pop_latest()) slot_ptr->store(*item);
        };
        ring_subs_[topic] = raw_sub;

        sub_topics_.push_back(topic);
        refresh_registry();
        sub_slots_.push_back(std::move(sub_slot));

        return {std::make_shared<Subscription<T>>(topic), slot_ptr};
    }

    /* ── spin_once ────────────────────────────────────────────────────────── */

    /*
     * 1. Send heartbeat (rate-limited: at most once every 50 ms).
     * 2. For each un-attached subscriber, retry attach at most every 100 ms
     *    using a non-blocking (timeout=0) attempt — never blocks.
     * 3. Poll every attached subscriber and fire callbacks / push to queues.
     */
    void spin_once() {
        /* Single clock read shared by heartbeat check and attach-retry throttle. */
        const int64_t now = static_cast<int64_t>(shmbridge::detail::now_ns());

        if (registry_.is_open() && now - last_heartbeat_ns_ >= HEARTBEAT_INTERVAL_NS) {
            registry_.heartbeat(registry_.slot_idx());
            last_heartbeat_ns_ = now;
        }

        for (auto& slot : sub_slots_) {
            if (!slot.attached) {
                if (now - slot.last_attach_try_ns >= ATTACH_RETRY_NS) {
                    slot.last_attach_try_ns = now;
                    slot.attached = slot.try_attach();
                }
            }
            if (slot.attached) {
                slot.poll_fn();
            }
        }
    }

    void stop() noexcept {
        registry_.close();
    }

private:
    /* Re-register with updated topic lists after each create_* call. */
    void refresh_registry() {
        if (!registry_.is_open()) return;
        int idx = registry_.slot_idx();
        std::vector<std::string> subs_kl, subs_ring;
        for (auto& s : sub_slots_)
            (s.keep_latest ? subs_kl : subs_ring).push_back(s.topic);

        if (idx < 0)
            registry_.register_node(name_.c_str(), pub_topics_, subs_kl, subs_ring);
        else
            registry_.update_topics(idx, pub_topics_, subs_kl, subs_ring);
    }

    std::string name_;
    DiscoveryRegistry registry_;

    int64_t last_heartbeat_ns_{0};

    /* Internal publisher storage (type-erased; kept alive here). */
    std::unordered_map<std::string, std::shared_ptr<void>> seqlock_pubs_;
    std::unordered_map<std::string, std::shared_ptr<void>> ring_pubs_;

    /* Internal subscriber storage (type-erased). */
    std::unordered_map<std::string, std::shared_ptr<void>> seqlock_subs_;
    std::unordered_map<std::string, std::shared_ptr<void>> ring_subs_;

    /* Subscriber slots — drive spin_once(). */
    std::vector<detail::SubSlot> sub_slots_;

    /* Topic lists for registry encoding. */
    std::vector<std::string> pub_topics_;
    std::vector<std::string> sub_topics_;
};

/* ── free functions ───────────────────────────────────────────────────────── */

inline void init(int /*argc*/ = 0, char** /*argv*/ = nullptr) noexcept {}
inline void shutdown() noexcept {}

inline std::shared_ptr<Node> make_node(const std::string& name) {
    auto n = std::make_shared<Node>(name);
    if (!n->start())
        throw std::runtime_error("shmbridge: failed to open discovery registry");
    return n;
}

/*
 * spin() — run until the process receives SIGINT / SIGTERM.
 * The caller is responsible for setting up the signal handler; spin() just
 * calls spin_once() + sleep in a loop, checking the flag each iteration.
 */
inline std::atomic<bool> g_ok{true};  /* set false by signal handler */

inline void spin(std::shared_ptr<Node> node,
                 std::chrono::milliseconds period = std::chrono::milliseconds(10)) {
    while (g_ok.load(std::memory_order_relaxed)) {
        node->spin_once();
        std::this_thread::sleep_for(period);
    }
}

inline void spin_some(std::shared_ptr<Node> node) {
    node->spin_once();
}

inline void spin_for(std::shared_ptr<Node> node,
                     std::chrono::milliseconds duration,
                     std::chrono::milliseconds period = std::chrono::milliseconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline && g_ok.load()) {
        node->spin_once();
        std::this_thread::sleep_for(period);
    }
}

} /* namespace shmbridge::ros_compat */
