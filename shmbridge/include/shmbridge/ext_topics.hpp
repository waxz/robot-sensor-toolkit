/*
 * shmbridge/ext_topics.hpp  -  Independent, per-topic publish/subscribe
 * primitives, so a totally separate node/process can publish (or
 * subscribe to) just one topic, without the combined ExtShmBridge segment
 * (ext_core.hpp) existing at all.
 *
 * This is the missing piece ExtShmBridge doesn't provide: it bundles
 * state+cmd+imu+encoder+pointcloud+user-channels into ONE segment that
 * only a single process can own (see its own header comment) -- fine for
 * a monolithic simulator like urdf_tools' 04_pub_sensors.py, but it can't
 * model a realistic multi-node robot where, say, an IMU driver, a wheel
 * encoder reader, and a lidar node are separate executables. Those should
 * each be able to publish (and a controller subscribe to) their own topic
 * independently, exactly like shmbridge.h's ShmBridgeClient and the
 * examples/cpp_pipeline nodes do for state+cmd (each of
 * sensor_node/planner_node/control_node owns its own named segment and
 * composes whichever others it needs) -- this header gives every message
 * that same freedom.
 *
 * ROS-style: write a message definition, not a new publisher/subscriber.
 * ------------------------------------------------------------------
 * A fixed-size, trivially-copyable message (IMU, encoder, a pose, whatever
 * else you need) doesn't get its own class here -- it's just a struct,
 * published via the ONE generic `Publisher<T>`/`Subscriber<T>` from
 * topic.hpp, the same way rospy.Publisher(topic, MsgType) takes the
 * message type as an argument instead of every message type needing its
 * own Publisher subclass:
 *
 *   shmbridge::Publisher<ShmImu> pub;
 *   pub.open("robot1_imu");             // -> shm segment "/sb_robot1_imu"
 *   pub.write(ShmImu{ax, ay, az, ...});
 *
 *   shmbridge::Subscriber<ShmImu> sub;
 *   sub.attach("robot1_imu");
 *   auto sample = sub.spin();           // ReadResult<ShmImu>, or nullopt
 *
 * ext_messages.hpp's ShmImu/ShmEncoder are exactly such definitions
 * (reused as-is here, so they produce byte-identical payloads to
 * ExtShmBridge's write_imu()/write_encoder() -- just carried in a
 * different segment layout/protocol). Earlier revisions of this header
 * also defined type aliases (ImuPublisher = Publisher<ShmImu>, etc.) for
 * each one, which is exactly the anti-pattern this section warns against:
 * every one of those aliases was pure duplication of a class that already
 * exists generically, and nothing in this codebase ever ended up using
 * them (urdf_tools' Python layer went straight to the generic
 * RawChannelPublisher/Subscriber + its own message definitions instead --
 * see shmbridge.message.Publisher/Subscriber and urdf_tools.pubsub). Don't
 * add new ones; add a message struct and instantiate Publisher<T>/
 * Subscriber<T> directly, in C++ or (via RawMsg88 below) in Python.
 *
 * Custom messages ride the same way via RawMsg88, a generic 88-byte
 * payload matching ExtShmBridge's write_channel/read_channel payload size
 * -- unlike that fixed 8-slot pool multiplexed into one segment, each
 * RawChannelPublisher/Subscriber pair here is its own segment, so the
 * number of distinct custom topics isn't capped and doesn't require a
 * single owning process to have claimed a slot for it up front. This is
 * also the generic "any message" primitive Python message definitions
 * (dataclasses with a struct.pack format string, see shmbridge.message)
 * are packed into, since Python has no compile-time T to hand
 * Publisher<T> the way C++ does.
 *
 * Point clouds don't fit topic.hpp's SeqlockSlot<T> (capped at 104 bytes;
 * a cloud can be up to 1 MiB) -- but they fit ring.hpp's RingPublisher<T>/
 * RingSubscriber<T> fine (that's exactly the problem F-12's
 * PointCloudFixed<MaxPoints> family in messages.hpp exists to solve; see
 * design_ring_zero_copy.md §5.11). PointCloudPublisher/Subscriber below
 * used to be a bespoke hand-rolled seqlock-over-a-dedicated-segment pair,
 * written before PointCloudFixed existed; it's now a thin wrapper over
 * RingPublisher<msg::PointCloud65536>/RingSubscriber<msg::PointCloud65536>
 * (65536 points was this pair's original cap, preserved exactly) instead
 * of a second, independently-maintained bulk-transport implementation —
 * see docs/design_ring_zero_copy.md §16 for the redundancy this closes.
 * A small capacity (2) gives seqlock-like "always current" semantics over
 * the ring while still getting ring.hpp's own torn-read handling (R-1)
 * and stale-producer takeover (R-17) for free, neither of which the old
 * bespoke pair had.
 *
 * Header-only, no pybind11 dependency (consistent with core.hpp and
 * topic.hpp) — Python bindings live in csrc/py_bindings.cpp.
 */

#pragma once

#include "ext_messages.hpp"
#include "messages.hpp"
#include "platform.hpp"
#include "ring.hpp"
#include "topic.hpp"

#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

namespace shmbridge {

/* ── Custom messages: independent topics, unlimited distinct names ───────── */
/*
 * A generic 88-byte payload (matching ExtShmBridge's
 * EXT_USER_CHANNEL_PAYLOAD_BYTES) for callers that want a custom message
 * without defining their own POD struct in C++ -- e.g. the Python side's
 * shmbridge.message.Channel packs/unpacks arbitrary dataclasses into
 * exactly this many bytes. A C++ node with its own real message struct
 * (<=104 bytes, trivially copyable) should prefer `Publisher<MyStruct>`/
 * `Subscriber<MyStruct>` directly over RawMsg88 -- topic.hpp's type_hash
 * guard then catches a publisher/subscriber struct mismatch at attach()
 * time instead of silently misinterpreting bytes.
 */
struct RawMsg88 {
    uint8_t data[88] = {};
};
static_assert(sizeof(RawMsg88) == 88, "RawMsg88 must be 88 B");

using RawChannelPublisher = Publisher<RawMsg88>;
using RawChannelSubscriber = Subscriber<RawMsg88>;

/* ── Point cloud: independent topic, now riding ring.hpp (F-12) ───────────── */

constexpr size_t PC_TOPIC_MAX_POINTS = 65536; /* matches msg::PointCloud65536 */
constexpr size_t PC_TOPIC_POINT_BYTES = 16; /* x, y, z, intensity as float32 */

/** open()/write() over RingPublisher<msg::PointCloud65536> (§5.11) — a
 * capacity-2 ring gives the same "always current" semantics the old
 * bespoke seqlock had, plus ring.hpp's torn-read retry (R-1) and
 * stale-producer takeover (R-17) for free. */
class PointCloudPublisher {
public:
    PointCloudPublisher() = default;
    ~PointCloudPublisher() { close(); }
    PointCloudPublisher(const PointCloudPublisher&) = delete;
    PointCloudPublisher& operator=(const PointCloudPublisher&) = delete;
    PointCloudPublisher(PointCloudPublisher&&) = default;
    PointCloudPublisher& operator=(PointCloudPublisher&&) = default;

    /** Throws std::runtime_error if another live process already
     * publishes this topic (F-11, §5.10) — the old bespoke pair had no
     * such check; two publishers racing on the same name silently
     * clobbered each other instead. */
    void open(const std::string& name) {
        RingConfig cfg;
        cfg.capacity = 2; /* bulk payload, latest-only -- a small ring is plenty */
        ring_.open(name.c_str(), cfg);
    }

    void close() noexcept { ring_.close(); }

    bool is_open() const noexcept { return ring_.is_open(); }

    /** Zero-copy write of N points (x,y,z,intensity float32 each) from a
     * raw buffer, via reserve()/commit() (F-4) straight into the ring
     * slot — no intermediate copy. */
    void write(const float* data, size_t n_points, double ts = 0) {
        if (n_points > PC_TOPIC_MAX_POINTS) {
            throw std::invalid_argument(
                "Too many points: " + std::to_string(n_points) + " > " +
                std::to_string(PC_TOPIC_MAX_POINTS));
        }
        msg::PointCloud65536* slot = ring_.reserve();
        slot->n_points = static_cast<uint32_t>(n_points);
        slot->ts = ts;
        std::memcpy(slot->points, data, n_points * PC_TOPIC_POINT_BYTES);
        ring_.commit();
    }

private:
    RingPublisher<msg::PointCloud65536> ring_;
};

/** attach()/read()/read_header()/is_publisher_alive() over
 * RingSubscriber<msg::PointCloud65536>. ring.hpp's pop_latest() (F-13) is
 * consume-once (returns nullopt once there's nothing new since the last
 * call) — the opposite of the old seqlock's "always re-readable" model,
 * which callers (this class's own Python ctypes mirror in topic.py, and
 * anything built against this API) rely on. refresh() bridges the two:
 * every call opportunistically advances to the newest published cloud and
 * caches it, so repeated reads with no new publish keep returning the
 * last known cloud instead of flipping to "no data". */
class PointCloudSubscriber {
public:
    PointCloudSubscriber() = default;
    ~PointCloudSubscriber() { detach(); }
    PointCloudSubscriber(const PointCloudSubscriber&) = delete;
    PointCloudSubscriber& operator=(const PointCloudSubscriber&) = delete;
    PointCloudSubscriber(PointCloudSubscriber&&) = default;
    PointCloudSubscriber& operator=(PointCloudSubscriber&&) = default;

    /** Attach to an existing point-cloud topic; retries until it appears
     * or times out. Raises std::invalid_argument on a schema/type
     * mismatch, shmbridge::TimeoutError (a std::runtime_error) on
     * timeout. */
    void attach(const std::string& name, double timeout_ms = 30000.0) {
        detach();
        has_cache_ = false;
        ring_.attach(name.c_str(), timeout_ms);
    }

    void detach() noexcept {
        ring_.detach();
        has_cache_ = false;
    }

    bool is_attached() const noexcept { return ring_.is_attached(); }

    /** Copy the latest cloud into out_buf (caller-sized for
     * PC_TOPIC_MAX_POINTS points); returns the number of points copied,
     * or 0 if nothing has ever been published yet. */
    size_t read(float* out_buf) const noexcept {
        refresh();
        if (!has_cache_) return 0;
        uint32_t n = cache_.n_points;
        if (n > PC_TOPIC_MAX_POINTS) n = 0;
        if (n > 0) std::memcpy(out_buf, cache_.points, n * PC_TOPIC_POINT_BYTES);
        return n;
    }

    std::optional<PcHeaderSample> read_header() const noexcept {
        refresh();
        if (!has_cache_) return std::nullopt;
        return PcHeaderSample{cache_.n_points,
                               static_cast<uint32_t>(PC_TOPIC_MAX_POINTS), cache_.ts};
    }

    bool is_publisher_alive(double max_age_ms = 100.0) const noexcept {
        refresh();
        if (!has_cache_) return true; /* nothing published yet -- benefit of the doubt */
        return (platform::now_ns() - cache_write_ns_) < static_cast<uint64_t>(max_age_ms * 1e6);
    }

private:
    /** Pull the newest cloud off the ring into cache_ if one has arrived
     * since the last call; a no-op (cache_ left as-is) when there's
     * nothing new, which is what gives read()/read_header() their
     * "always returns latest known" behavior instead of ring.hpp's
     * default "nullopt once drained" one. mutable + const: this is a
     * caching lookup, not an observable state change from callers' point
     * of view (same const-despite-internal-state shape the old seqlock
     * version had, just with real mutable state behind it now).
     *
     * Deliberately uses borrow()/end_borrow() (F-5), not pop_latest()/
     * pop_ex() -- both of those return a `Result<T>` *by value*, which
     * means a full `T` (~1 MiB for PointCloud65536) as a stack-local
     * inside ring.hpp itself. That's well within Linux's default 8 MiB
     * thread stack but overflows Windows' default 1 MiB one -- a crash
     * (observed as a silent access violation, no exception, no Python
     * traceback) caught only by actually running this on Windows, not by
     * the WSL/Linux build this was first verified against. borrow()
     * returns a pointer straight into shared memory instead, so nothing
     * here is ever stack-sized by T. The loop drains every item
     * currently available so a call still reaches the true newest cloud
     * (matching pop_latest()'s F-13 "jump to latest" semantics) rather
     * than stopping at the oldest unread one. scratch_ is a second
     * persistent member (not a stack local) so a torn read's possibly-
     * corrupted bytes land there and get discarded, never overwriting
     * cache_ (the one callers read from) until end_borrow() confirms the
     * copy wasn't torn -- same "never return corrupted data" guarantee
     * the old seqlock's seq/seq2 check gave, via a different mechanism. */
    void refresh() const noexcept {
        for (;;) {
            uint64_t write_ns = 0;
            const msg::PointCloud65536* slot = ring_.borrow(&write_ns);
            if (!slot) return;
            scratch_ = *slot;
            if (ring_.end_borrow()) {
                cache_ = scratch_;
                cache_write_ns_ = write_ns;
                has_cache_ = true;
            }
        }
    }

    mutable RingSubscriber<msg::PointCloud65536> ring_;
    mutable msg::PointCloud65536 cache_{};
    mutable msg::PointCloud65536 scratch_{};
    mutable uint64_t cache_write_ns_ = 0;
    mutable bool has_cache_ = false;
};

} /* namespace shmbridge */
