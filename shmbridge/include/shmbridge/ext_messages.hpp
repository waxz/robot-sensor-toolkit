/*
 * shmbridge/ext_messages.hpp  -  Message/slot type definitions for the
 * extended segment (see ext_core.hpp's ExtShmBridge), split out from the
 * bridge/transport class itself so the wire-level schema for each specific
 * message (IMU, encoder, point cloud, plus the generic named user-channel
 * slot) lives in one place, independent of how ExtShmBridge maps and
 * seqlocks them.
 *
 * Wire-compatible with the pure-Python ExtShmBridge in
 * shmbridge/src/shmbridge/bridge_ext.py and its ctypes layout in
 * shmbridge/src/shmbridge/_types.py (_ShmImu, _ShmEncoder, _ShmPcHdr,
 * _ShmUserSlot). Header-only, no pybind11 dependency — Python bindings
 * live in csrc/py_bindings.cpp.
 */

#pragma once

#include "types.h"

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>

namespace shmbridge {

/* ── IMU (byte-for-byte match to _types.py's _ShmImu/_ShmImuSlot) ─────────── */

struct ShmImu {
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float mx = 0, my = 0, mz = 0;
    float ts = 0;
}; /* sizeof == 40 */
static_assert(sizeof(ShmImu) == 40, "ShmImu must match _ShmImu (40 B)");

struct ShmImuSlot {
    volatile uint64_t seq;
    ShmImu            imu;
    volatile uint64_t seq2;
    volatile uint64_t writer_ts_ns;
    uint8_t           _fill[64];
}; /* sizeof == 128 */
static_assert(sizeof(ShmImuSlot) == 128, "ShmImuSlot must be 128 B");

/* ── Encoder (matches _types.py's _ShmEncoder/_ShmEncoderSlot) ─────────────── */

struct ShmEncoder {
    int32_t ticks[4] = {0, 0, 0, 0};
    float   speed[4] = {0, 0, 0, 0};
    float   ts       = 0;
    uint8_t _pad[4]  = {0, 0, 0, 0};
}; /* sizeof == 40 */
static_assert(sizeof(ShmEncoder) == 40, "ShmEncoder must match _ShmEncoder (40 B)");

struct ShmEncoderSlot {
    volatile uint64_t seq;
    ShmEncoder        encoder;
    volatile uint64_t seq2;
    volatile uint64_t writer_ts_ns;
    uint8_t           _fill[64];
}; /* sizeof == 128 */
static_assert(sizeof(ShmEncoderSlot) == 128, "ShmEncoderSlot must be 128 B");

/* ── Point cloud (matches _types.py's _ShmPcHdr/_ShmPcSlot) ────────────────── */

struct ShmPcHdr {
    volatile uint64_t seq;
    uint32_t          n_points = 0;
    uint32_t          max_pts  = 0;
    double            ts       = 0;
    uint8_t           _fill[40] = {};
}; /* sizeof == 64 */
static_assert(sizeof(ShmPcHdr) == 64, "ShmPcHdr must match _ShmPcHdr (64 B)");

struct ShmPcSlot {
    volatile uint64_t seq;
    ShmPcHdr          hdr;
    volatile uint64_t seq2;
    volatile uint64_t writer_ts_ns;
    uint8_t           _fill[40];
}; /* sizeof == 128 */
static_assert(sizeof(ShmPcSlot) == 128, "ShmPcSlot must be 128 B");

constexpr size_t EXT_PC_MAX_POINTS = 65536;
constexpr size_t EXT_PC_POINT_BYTES = 16; /* x, y, z, intensity as float32 */
constexpr size_t EXT_PC_DATA_BYTES = EXT_PC_MAX_POINTS * EXT_PC_POINT_BYTES;

/* ── Generic user-defined channels ─────────────────────────────────────────
 *
 * A fixed pool of name-addressed slots for custom message types (mirrors
 * _types.py's _ShmUserSlot/N_USER_CHANNELS exactly) — a custom message
 * needs zero changes here: define your own POD struct, reinterpret_cast
 * its bytes through ExtShmBridge::write_channel/read_channel. See
 * shmbridge.message on the Python side for the equivalent struct.pack/
 * unpack layer.
 */
constexpr size_t EXT_USER_CHANNEL_NAME_BYTES = 16;
constexpr size_t EXT_USER_CHANNEL_PAYLOAD_BYTES = 88;
constexpr size_t EXT_N_USER_CHANNELS = 8;

struct ShmUserSlot {
    volatile uint64_t seq;
    char              name[EXT_USER_CHANNEL_NAME_BYTES];
    uint8_t           payload[EXT_USER_CHANNEL_PAYLOAD_BYTES];
    volatile uint64_t seq2;
    volatile uint64_t writer_ts_ns;
}; /* sizeof == 128 */
static_assert(sizeof(ShmUserSlot) == 128, "ShmUserSlot must be 128 B");

/* Versions ONLY the extended block's own layout (state/cmd + imu/encoder/
 * pointcloud/user channels) -- independent of SHMBRIDGE_VERSION (types.h),
 * which is the base state/cmd protocol every ShmBridge/ShmPublisher/
 * ShmSubscriber segment uses. Bump this -- and the identical constant in
 * shmbridge/src/shmbridge/_types.py -- whenever any struct in this file,
 * or ShmExtBlock's layout in ext_core.hpp, changes. */
constexpr uint32_t EXT_SCHEMA_VERSION = 1;

/* ── Python-facing value types ──────────────────────────────────────────────
 *
 * Plain data returned from ExtShmBridge's read_*() methods to Python via
 * pybind11 (see csrc/py_bindings.cpp) — deliberately separate from the
 * wire-level Shm*Slot structs above (those hold volatile seqlock fields and
 * raw padding that have no business leaking into a caller's hands).
 */

struct ImuSample {
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float mx = 0, my = 0, mz = 0;
    float ts = 0;
};

struct EncoderSample {
    std::array<int32_t, 4> ticks = {0, 0, 0, 0};
    std::array<float, 4>   speed = {0, 0, 0, 0};
    float                   ts    = 0;
};

struct PcHeaderSample {
    uint32_t n_points = 0;
    uint32_t max_pts  = 0;
    double   ts       = 0;
};

/** One discoverable channel, mirroring Python's ShmTopic/urdf_tools Topic. */
struct TopicInfo {
    std::string name;
    bool        alive  = false;
    /* has_age is false when the channel has never been written (age_ms is
     * then meaningless) — mirrors Python's Topic.age_ms being None. */
    bool        has_age = false;
    double      age_ms  = 0.0;
};

} /* namespace shmbridge */
