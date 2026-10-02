/*
 * shmbridge/ext_core.hpp  -  C++ ExtShmBridge: the transport class that
 * maps/seqlocks state+cmd (from core.hpp) plus the IMU / encoder /
 * point-cloud / generic-user-channel messages defined in ext_messages.hpp,
 * wire-compatible with the pure-Python ExtShmBridge in
 * shmbridge/src/shmbridge/bridge_ext.py and its ctypes layout in
 * shmbridge/src/shmbridge/_types.py.
 *
 * The message/slot *type definitions* live in ext_messages.hpp, deliberately
 * separate from this file: this header only has ShmExtBlock (the specific
 * memory layout this specific bridge maps) and the ExtShmBridge class
 * itself (the seqlock read/write logic + lifecycle), so "what a message
 * looks like on the wire" and "how a segment full of them gets mapped and
 * synchronized" don't have to be read, or changed, together.
 *
 * Header-only, no pybind11 dependency (consistent with core.hpp) — Python
 * bindings live in csrc/py_bindings.cpp.
 *
 * Deliberately reuses core.hpp's seqlock helpers (detail::write_state_slot,
 * read_state_slot, write_cmd_slot, read_cmd_slot) for the state/cmd channels
 * instead of re-implementing that protocol a second time — this is the same
 * protocol ShmPublisher/ShmSubscriber use, just addressed at this segment's
 * single state/cmd slot pair.
 *
 * Layout (matches _types.py's _ShmExtBlock exactly):
 *   offset   0  ShmHeader        (128 B, from types.h)
 *   offset 128  ShmStateSlot     (128 B, from types.h)
 *   offset 256  ShmCmdSlot       (128 B, from types.h)
 *   offset 384  ShmImuSlot       (128 B, from ext_messages.hpp)
 *   offset 512  ShmEncoderSlot   (128 B, from ext_messages.hpp)
 *   offset 640  ShmPcSlot        (128 B, from ext_messages.hpp)
 *   offset 768  ShmUserSlot[N]   (128 B each, from ext_messages.hpp)
 *   ...         raw point-cloud data (PC_MAX_POINTS * 16 B)
 */

#pragma once

#include "core.hpp"
#include "ext_messages.hpp"
#include "platform.hpp"
#include "types.h"

#include <cstring>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace shmbridge {

constexpr const char* EXT_SHM_NAME_DEFAULT = "/shmbridge_ext_v2";

struct ShmExtBlock {
    ShmHeader      header;
    ShmStateSlot   state;
    ShmCmdSlot     cmd;
    ShmImuSlot     imu;
    ShmEncoderSlot encoder;
    ShmPcSlot      pc_slot;
    ShmUserSlot    user_slots[EXT_N_USER_CHANNELS];
    /* raw point-cloud payload follows immediately in the mapped segment */
}; /* sizeof == 768 + 128*EXT_N_USER_CHANNELS */
static_assert(sizeof(ShmHeader) == 128, "ShmHeader must stay 128 B");
static_assert(sizeof(ShmExtBlock) == 768 + 128 * EXT_N_USER_CHANNELS,
              "ShmExtBlock size must match _types.py's EXT_HEADER_BYTES");

constexpr size_t EXT_SHM_SIZE = sizeof(ShmExtBlock) + EXT_PC_DATA_BYTES;

/* ── ExtShmBridge ─────────────────────────────────────────────────────────── */

/**
 * State + cmd (as core.hpp's ShmPublisher/ShmSubscriber) plus IMU, encoder,
 * and point-cloud channels in one shm segment. Single robot, single
 * consumer — matches the pure-Python ExtShmBridge this mirrors.
 *
 * Unlike ShmPublisher/ShmSubscriber (which split writer/reader roles into
 * two classes), this is one class used by both roles, exactly like the
 * Python original: whichever side calls open() creates the segment;
 * whichever calls attach() maps an existing one. Either side may write or
 * read any channel — e.g. the simulator writes state/imu/encoder/scan and
 * reads cmd, while a teleop viewer writes cmd and reads everything else.
 */
class ExtShmBridge {
public:
    explicit ExtShmBridge(std::string name = EXT_SHM_NAME_DEFAULT)
        : name_(std::move(name)) {}

    ~ExtShmBridge() { close(); }

    ExtShmBridge(const ExtShmBridge&)            = delete;
    ExtShmBridge& operator=(const ExtShmBridge&) = delete;

    /* ── lifecycle ────────────────────────────────────────────────────────── */

    void open() {
        mem_ = platform::shm_create(name_, EXT_SHM_SIZE);
        blk_ = static_cast<ShmExtBlock*>(mem_);
        blk_->header.magic              = SHMBRIDGE_MAGIC;
        blk_->header.schema_version     = SHMBRIDGE_VERSION;
        blk_->header.ext_schema_version = EXT_SCHEMA_VERSION;
        blk_->header.n_robots           = 1;
        blk_->header.n_consumers        = 1;
        blk_->header.ready              = 1;
    }

    void close() noexcept {
        if (mem_) {
            platform::shm_unmap(mem_, EXT_SHM_SIZE);
            mem_ = nullptr;
            blk_ = nullptr;
        }
        platform::shm_destroy(name_);
    }

    bool is_open() const noexcept { return mem_ != nullptr; }

    /** Attach to an existing segment; retries until it appears or times out.
     *
     * Checks BOTH magic and schema_version (unlike a since-fixed gap in the
     * pure-Python ExtShmBridge that only checked magic) — a stale/mismatched
     * publisher is a real mismatched-pub/sub bug, not a "not ready yet".
     */
    void attach(double timeout_ms = 30000.0) {
        detach();
        uint64_t deadline =
            detail::now_ns() + static_cast<uint64_t>(timeout_ms * 1e6);

        for (;;) {
            try {
                mem_ = platform::shm_attach(name_, EXT_SHM_SIZE);
                break;
            } catch (...) {
                if (detail::now_ns() > deadline) {
                    throw std::runtime_error(
                        "attach timeout: segment '" + name_ +
                        "' not found — is the publisher running?");
                }
                platform::sleep_ns(50'000'000LL);
            }
        }

        blk_ = static_cast<ShmExtBlock*>(mem_);
        while (!blk_->header.ready) {
            if (detail::now_ns() > deadline) {
                platform::shm_unmap(mem_, EXT_SHM_SIZE);
                mem_ = nullptr;
                blk_ = nullptr;
                throw std::runtime_error("attach timeout: publisher not ready");
            }
            platform::sleep_ns(500'000LL);
        }
        if (blk_->header.magic != 0 && blk_->header.magic != SHMBRIDGE_MAGIC) {
            platform::shm_unmap(mem_, EXT_SHM_SIZE);
            mem_ = nullptr;
            blk_ = nullptr;
            throw std::invalid_argument(
                "Schema mismatch: magic=0x" + std::to_string(blk_->header.magic));
        }
        if (blk_->header.schema_version != 0 &&
            blk_->header.schema_version != SHMBRIDGE_VERSION) {
            uint32_t got = blk_->header.schema_version;
            platform::shm_unmap(mem_, EXT_SHM_SIZE);
            mem_ = nullptr;
            blk_ = nullptr;
            throw std::invalid_argument(
                "Schema version mismatch: got " + std::to_string(got) +
                ", expected " + std::to_string(SHMBRIDGE_VERSION) +
                " — publisher and subscriber are running different "
                "shmbridge versions");
        }
        if (blk_->header.ext_schema_version != 0 &&
            blk_->header.ext_schema_version != EXT_SCHEMA_VERSION) {
            uint32_t got = blk_->header.ext_schema_version;
            platform::shm_unmap(mem_, EXT_SHM_SIZE);
            mem_ = nullptr;
            blk_ = nullptr;
            throw std::invalid_argument(
                "Extended schema mismatch: got " + std::to_string(got) +
                ", expected " + std::to_string(EXT_SCHEMA_VERSION) +
                " — publisher and subscriber are running different "
                "shmbridge versions");
        }
    }

    void detach() noexcept {
        if (mem_) {
            platform::shm_unmap(mem_, EXT_SHM_SIZE);
            mem_ = nullptr;
            blk_ = nullptr;
        }
    }

    bool is_attached() const noexcept { return mem_ != nullptr; }

    /* ── state / cmd (delegates to core.hpp's seqlock helpers) ─────────────── */

    void write_state(const RobotState& s) {
        detail::write_state_slot(&blk_->state, s, state_seq_, write_count_, 1);
    }

    std::optional<RobotState> read_state() const noexcept {
        RobotState s;
        if (detail::read_state_slot(&blk_->state, s)) return s;
        return std::nullopt;
    }

    void write_cmd(float linear, float angular) {
        detail::write_cmd_slot(&blk_->cmd, linear, angular, cmd_seq_);
    }

    std::optional<RobotCmd> read_cmd() const noexcept {
        return detail::read_cmd_slot(&blk_->cmd);
    }

    /* ── IMU ─────────────────────────────────────────────────────────────── */

    void write_imu(float ax, float ay, float az, float gx, float gy, float gz,
                   float mx = 0, float my = 0, float mz = 0, float ts = 0) {
        ShmImuSlot* slot = &blk_->imu;
        slot->seq = ++imu_seq_;
        _SB_FENCE_W();
        slot->imu = ShmImu{ax, ay, az, gx, gy, gz, mx, my, mz, ts};
        slot->writer_ts_ns = detail::now_ns();
        _SB_FENCE_W();
        slot->seq = ++imu_seq_;
        slot->seq2 = imu_seq_;
    }

    std::optional<ImuSample> read_imu() const noexcept {
        const ShmImuSlot* slot = &blk_->imu;
        uint64_t s1 = slot->seq;
        _SB_FENCE_R();
        ShmImu d = slot->imu;
        _SB_FENCE_R();
        uint64_t s2 = slot->seq2;
        if (s1 != s2 || (s1 & 1u)) return std::nullopt;
        return ImuSample{d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.mx, d.my, d.mz, d.ts};
    }

    /* ── Encoder ─────────────────────────────────────────────────────────── */

    void write_encoder(const std::array<int32_t, 4>& ticks,
                        const std::array<float, 4>& speeds, float ts = 0) {
        ShmEncoderSlot* slot = &blk_->encoder;
        slot->seq = ++encoder_seq_;
        _SB_FENCE_W();
        ShmEncoder& e = slot->encoder;
        for (int i = 0; i < 4; ++i) {
            e.ticks[i] = ticks[i];
            e.speed[i] = speeds[i];
        }
        e.ts = ts;
        slot->writer_ts_ns = detail::now_ns();
        _SB_FENCE_W();
        slot->seq = ++encoder_seq_;
        slot->seq2 = encoder_seq_;
    }

    std::optional<EncoderSample> read_encoder() const noexcept {
        const ShmEncoderSlot* slot = &blk_->encoder;
        uint64_t s1 = slot->seq;
        _SB_FENCE_R();
        ShmEncoder d = slot->encoder;
        _SB_FENCE_R();
        uint64_t s2 = slot->seq2;
        if (s1 != s2 || (s1 & 1u)) return std::nullopt;
        EncoderSample out;
        for (int i = 0; i < 4; ++i) {
            out.ticks[i] = d.ticks[i];
            out.speed[i] = d.speed[i];
        }
        out.ts = d.ts;
        return out;
    }

    /* ── Point cloud ─────────────────────────────────────────────────────── */

    /** Write N points (x,y,z,intensity float32 each) from a raw buffer. */
    void write_pointcloud(const float* data, size_t n_points, double ts = 0) {
        if (n_points > EXT_PC_MAX_POINTS) {
            throw std::invalid_argument("Too many points: " +
                                         std::to_string(n_points) + " > " +
                                         std::to_string(EXT_PC_MAX_POINTS));
        }
        ShmPcSlot* slot = &blk_->pc_slot;
        slot->seq = ++pc_seq_;
        _SB_FENCE_W();
        slot->hdr.n_points = static_cast<uint32_t>(n_points);
        slot->hdr.max_pts  = static_cast<uint32_t>(EXT_PC_MAX_POINTS);
        slot->hdr.ts       = ts;
        std::memcpy(pc_data(), data, n_points * EXT_PC_POINT_BYTES);
        slot->writer_ts_ns = detail::now_ns();
        _SB_FENCE_W();
        slot->seq = ++pc_seq_;
        slot->seq2 = pc_seq_;
    }

    /** Copy the latest cloud into out_buf (caller-sized for EXT_PC_MAX_POINTS
     * points); returns the number of points copied, or 0 on a torn read /
     * no data. */
    size_t read_pointcloud(float* out_buf) const noexcept {
        const ShmPcSlot* slot = &blk_->pc_slot;
        uint64_t s1 = slot->seq;
        _SB_FENCE_R();
        uint32_t n = slot->hdr.n_points;
        if (n > EXT_PC_MAX_POINTS) n = 0;
        if (n > 0) std::memcpy(out_buf, pc_data(), n * EXT_PC_POINT_BYTES);
        _SB_FENCE_R();
        uint64_t s2 = slot->seq2;
        if (s1 != s2 || (s1 & 1u) || s1 == 0) return 0;
        return n;
    }

    std::optional<PcHeaderSample> read_pointcloud_header() const noexcept {
        const ShmPcSlot* slot = &blk_->pc_slot;
        uint64_t s1 = slot->seq;
        _SB_FENCE_R();
        PcHeaderSample out{slot->hdr.n_points, slot->hdr.max_pts, slot->hdr.ts};
        _SB_FENCE_R();
        uint64_t s2 = slot->seq2;
        if (s1 != s2 || (s1 & 1u) || s1 == 0) return std::nullopt;
        return out;
    }

    /* ── Generic user-defined channels ──────────────────────────────────────
     *
     * A fixed pool of name-addressed slots for custom message types — a
     * custom message needs zero changes to this file: define your own POD
     * struct on the C++ side (or a struct.pack format string in Python via
     * shmbridge.message.Channel) and reinterpret its bytes through these
     * two methods.
     */

    /** Claim (if requested and not already claimed) or find the slot for
     * *name*. Returns nullptr if not found and not claiming. */
    ShmUserSlot* find_user_slot(const std::string& name, bool claim) {
        auto it = user_slot_cache_.find(name);
        if (it != user_slot_cache_.end()) return &blk_->user_slots[it->second];
        for (size_t i = 0; i < EXT_N_USER_CHANNELS; ++i) {
            ShmUserSlot& slot = blk_->user_slots[i];
            if (std::strncmp(slot.name, name.c_str(), EXT_USER_CHANNEL_NAME_BYTES) == 0 &&
                slot.name[0] != '\0') {
                user_slot_cache_[name] = i;
                return &slot;
            }
        }
        if (!claim) return nullptr;
        for (size_t i = 0; i < EXT_N_USER_CHANNELS; ++i) {
            ShmUserSlot& slot = blk_->user_slots[i];
            if (slot.name[0] == '\0') {
                std::memset(slot.name, 0, EXT_USER_CHANNEL_NAME_BYTES);
                std::strncpy(slot.name, name.c_str(), EXT_USER_CHANNEL_NAME_BYTES - 1);
                user_slot_cache_[name] = i;
                return &slot;
            }
        }
        throw std::invalid_argument(
            "no free user-channel slot for '" + name +
            "' (all " + std::to_string(EXT_N_USER_CHANNELS) +
            " slots claimed by other channel names)");
    }

    /** Seqlock-write raw bytes to a generic named user channel. Claims a
     * free slot the first time *name* is used (first-fit); only the
     * segment's single writer role for a given name should ever do this —
     * a concurrent first-write of the *same new* name from multiple
     * processes is a race. */
    void write_channel(const std::string& name, const void* data, size_t len) {
        if (name.size() > EXT_USER_CHANNEL_NAME_BYTES - 1) {
            throw std::invalid_argument(
                "channel name too long: '" + name + "' (max " +
                std::to_string(EXT_USER_CHANNEL_NAME_BYTES - 1) + " chars)");
        }
        if (len > EXT_USER_CHANNEL_PAYLOAD_BYTES) {
            throw std::invalid_argument(
                "channel payload too large: " + std::to_string(len) +
                " bytes > " + std::to_string(EXT_USER_CHANNEL_PAYLOAD_BYTES));
        }
        ShmUserSlot* slot = find_user_slot(name, /*claim=*/true);
        uint64_t& seq = user_seqs_[name];
        slot->seq = ++seq;
        _SB_FENCE_W();
        std::memcpy(slot->payload, data, len);
        slot->writer_ts_ns = detail::now_ns();
        _SB_FENCE_W();
        slot->seq = ++seq;
        slot->seq2 = seq;
    }

    /** Seqlock read of a generic named user channel's raw payload (always
     * EXT_USER_CHANNEL_PAYLOAD_BYTES long; the caller's message format
     * determines how many bytes are meaningful). Returns false if the
     * channel doesn't exist (yet) or the read was torn. */
    bool read_channel(const std::string& name, uint8_t* out_buf) {
        ShmUserSlot* slot = find_user_slot(name, /*claim=*/false);
        if (slot == nullptr) return false;
        uint64_t s1 = slot->seq;
        _SB_FENCE_R();
        std::memcpy(out_buf, slot->payload, EXT_USER_CHANNEL_PAYLOAD_BYTES);
        _SB_FENCE_R();
        uint64_t s2 = slot->seq2;
        return s1 == s2 && !(s1 & 1u) && s1 != 0;
    }

    /* ── liveness / topic discovery ──────────────────────────────────────── */

    bool is_imu_alive(double max_age_ms = 100.0) const noexcept {
        uint64_t ts = blk_->imu.writer_ts_ns;
        if (ts == 0) return true;
        return (detail::now_ns() - ts) < static_cast<uint64_t>(max_age_ms * 1e6);
    }

    bool is_pointcloud_alive(double max_age_ms = 100.0) const noexcept {
        uint64_t ts = blk_->pc_slot.writer_ts_ns;
        if (ts == 0) return true;
        return (detail::now_ns() - ts) < static_cast<uint64_t>(max_age_ms * 1e6);
    }

    /** Enumerate every channel, with liveness — mirrors the Python
     * ExtShmBridge.list_topics() this was ported from, including its
     * "never written" != "alive" distinction (see there for rationale). */
    std::vector<TopicInfo> list_topics(double max_age_ms = 300.0) const {
        uint64_t now = detail::now_ns();
        auto topic = [&](const char* name, uint64_t writer_ts_ns) {
            TopicInfo t;
            t.name = name;
            if (writer_ts_ns == 0) {
                t.alive = false;
                t.has_age = false;
            } else {
                double age_ms = static_cast<double>(now - writer_ts_ns) / 1e6;
                t.has_age = true;
                t.age_ms = age_ms;
                t.alive = age_ms < max_age_ms;
            }
            return t;
        };
        std::vector<TopicInfo> out = {
            topic("state", blk_->state.writer_ts_ns),
            topic("cmd", blk_->cmd.writer_ts_ns),
            topic("imu", blk_->imu.writer_ts_ns),
            topic("encoder", blk_->encoder.writer_ts_ns),
            topic("pointcloud", blk_->pc_slot.writer_ts_ns),
        };
        for (const auto& slot : blk_->user_slots) {
            if (slot.name[0] != '\0') out.push_back(topic(slot.name, slot.writer_ts_ns));
        }
        return out;
    }

private:
    float* pc_data() const noexcept {
        return reinterpret_cast<float*>(static_cast<char*>(mem_) + sizeof(ShmExtBlock));
    }

    std::string  name_;
    void*        mem_ = nullptr;
    ShmExtBlock* blk_ = nullptr;
    uint64_t     state_seq_ = 0;
    unsigned     write_count_ = 0;
    uint64_t     cmd_seq_ = 0;
    uint64_t     imu_seq_ = 0;
    uint64_t     encoder_seq_ = 0;
    std::unordered_map<std::string, size_t> user_slot_cache_;
    std::unordered_map<std::string, uint64_t> user_seqs_;
    uint64_t     pc_seq_ = 0;
};

} /* namespace shmbridge */
