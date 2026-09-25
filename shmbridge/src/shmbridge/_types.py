"""
ctypes struct definitions for shmbridge schema v2.

Layout (per robot, 256 bytes; N robots = 128 + 256*N total):

  ShmHeader   (128 B)  offset   0  - magic, version, ready, n_robots
  ShmStateSlot(128 B)  offset 128  - robot state written by the sim
  ShmCmdSlot  (128 B)  offset 256  - velocity cmd written by C++

Wire format is identical to schema v1 for the first robot slot; v1 C++
controllers can read a v2 segment without modification.  New fields
(writer_ts_ns, magic, schema_version, n_robots) occupy bytes that were
padding in v1.
"""

from __future__ import annotations

import ctypes

MAGIC: int = 0x53484D42  # ASCII "SHMB"
SCHEMA_VERSION: int = 2
SHM_NAME_DEFAULT = "/shmbridge_v2"
EXT_SHM_NAME_DEFAULT = "/shmbridge_ext_v2"


def _shm_size(n_robots: int = 1, n_consumers: int = 1) -> int:
    """Segment byte size for *n_robots* robots and *n_consumers* cmd writers, page-aligned."""
    raw = 128 + 128 * n_robots + 128 * n_robots * n_consumers
    page = 4096
    return (raw + page - 1) & ~(page - 1)


# ── Core payload structs (unchanged from v1; C++ ABI stable) ──────────────


class _ShmState(ctypes.Structure):
    """72 bytes - robot pose/velocity snapshot written by the simulator."""

    _fields_ = [
        ("x", ctypes.c_double),  # offset  0   world-frame x (m)
        ("y", ctypes.c_double),  # offset  8   world-frame y (m)
        ("heading", ctypes.c_double),  # offset 16   orientation (rad)
        ("vx", ctypes.c_float),  # offset 24   velocity x (m/s)
        ("vy", ctypes.c_float),  # offset 28   velocity y (m/s)
        ("omega", ctypes.c_float),  # offset 32   angular velocity (rad/s)
        ("goal_x", ctypes.c_float),  # offset 36   goal position (m)
        ("goal_y", ctypes.c_float),  # offset 40
        ("goal_dist", ctypes.c_float),  # offset 44   distance to goal (m)
        ("step", ctypes.c_uint64),  # offset 48   sim step counter
        ("sim_time", ctypes.c_double),  # offset 56   simulated time (s)
        ("reached", ctypes.c_uint8),  # offset 64   1 = goal reached
        ("collision", ctypes.c_uint8),  # offset 65   1 = in collision
        ("_pad", ctypes.c_uint8 * 6),
    ]


assert ctypes.sizeof(_ShmState) == 72, ctypes.sizeof(_ShmState)


class _ShmCmd(ctypes.Structure):
    """16 bytes - velocity command written by the external controller."""

    _fields_ = [
        ("linear", ctypes.c_float),  # forward velocity (m/s)
        ("angular", ctypes.c_float),  # angular velocity (rad/s, CCW+)
        ("seq", ctypes.c_uint32),  # command counter (monotone)
        ("valid", ctypes.c_uint32),  # nonzero = command is fresh
    ]


assert ctypes.sizeof(_ShmCmd) == 16, ctypes.sizeof(_ShmCmd)


# ── Seqlock slot wrappers (128 bytes each, cache-line aligned) ────────────


class _ShmStateSlot(ctypes.Structure):
    """
    128 bytes.  v2 adds writer_ts_ns at offset 88 (was _fill in v1).
    Backward-compatible: a v1 C++ reader ignores bytes 88-95 silently.
    """

    _fields_ = [
        ("seq", ctypes.c_uint64),  # offset  0   odd while writing
        ("state", _ShmState),  # offset  8   72 bytes
        ("seq2", ctypes.c_uint64),  # offset 80   mirrors seq
        ("writer_ts_ns", ctypes.c_uint64),  # offset 88   monotonic ns (NEW)
        ("_fill", ctypes.c_uint8 * 32),  # offset 96   pad to 128
    ]


assert ctypes.sizeof(_ShmStateSlot) == 128, ctypes.sizeof(_ShmStateSlot)


class _ShmCmdSlot(ctypes.Structure):
    """
    128 bytes.  v2 adds writer_ts_ns at offset 40 (was _fill in v1).
    """

    _fields_ = [
        ("seq", ctypes.c_uint64),  # offset  0
        ("cmd", _ShmCmd),  # offset  8  16 bytes
        ("seq2", ctypes.c_uint64),  # offset 24
        ("writer_ts_ns", ctypes.c_uint64),  # offset 32  (NEW)
        ("_fill", ctypes.c_uint8 * 88),  # offset 40  pad to 128
    ]


assert ctypes.sizeof(_ShmCmdSlot) == 128, ctypes.sizeof(_ShmCmdSlot)


class _ShmHeader(ctypes.Structure):
    """
    128 bytes.  v2 layout (backward-compatible: ready is still at offset 0).

      offset  0  ready          (uint64) - 1 = segment initialized
      offset  8  magic          (uint32) - 0x53484D42 "SHMB"
      offset 12  schema_version (uint32) - 2
      offset 16  n_robots       (uint8)  - number of robot slots
      offset 17  _fill[111]
    """

    _fields_ = [
        ("ready", ctypes.c_uint64),  # offset  0  UNCHANGED from v1
        ("magic", ctypes.c_uint32),  # offset  8  NEW
        ("schema_version", ctypes.c_uint32),  # offset 12  NEW
        ("n_robots", ctypes.c_uint8),  # offset 16  NEW
        ("n_consumers", ctypes.c_uint8),  # offset 17  NEW - cmd writers per robot
        ("_fill", ctypes.c_uint8 * 110),  # offset 18  pad to 128
    ]


assert ctypes.sizeof(_ShmHeader) == 128, ctypes.sizeof(_ShmHeader)


def make_block_type(n_robots: int = 1, n_consumers: int = 1) -> type[ctypes.Structure]:
    """
    Build a ctypes Structure for *n_robots* robots and *n_consumers* cmd writers.

    Layout:
      ShmHeader (128)
      ShmStateSlot[n_robots]           (n_robots * 128)
      ShmCmdSlot[n_robots * n_consumers] (n_robots * n_consumers * 128)

    Cmd slot index for (robot r, consumer c): r * n_consumers + c.
    When n_consumers == 1 the layout is identical to schema v2.
    """
    n_cmd = n_robots * n_consumers

    class _ShmBlock(ctypes.Structure):
        _fields_ = [
            ("header", _ShmHeader),
            ("states", _ShmStateSlot * n_robots),
            ("cmds", _ShmCmdSlot * n_cmd),
        ]

    expected = 128 + 128 * n_robots + 128 * n_cmd
    assert ctypes.sizeof(_ShmBlock) == expected, (
        f"Block size {ctypes.sizeof(_ShmBlock)} != {expected}"
    )
    return _ShmBlock


# ── Extended segment (IMU / encoder / point cloud) ────────────────────────


class _ShmImu(ctypes.Structure):
    """40 bytes - accelerometer + gyroscope + magnetometer."""

    _fields_ = [
        ("ax", ctypes.c_float),
        ("ay", ctypes.c_float),
        ("az", ctypes.c_float),
        ("gx", ctypes.c_float),
        ("gy", ctypes.c_float),
        ("gz", ctypes.c_float),
        ("mx", ctypes.c_float),
        ("my", ctypes.c_float),
        ("mz", ctypes.c_float),
        ("ts", ctypes.c_float),
    ]


assert ctypes.sizeof(_ShmImu) == 40, ctypes.sizeof(_ShmImu)


class _ShmImuSlot(ctypes.Structure):
    """128 bytes.  8+40+8+8+64 = 128."""

    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("imu", _ShmImu),
        ("seq2", ctypes.c_uint64),
        ("writer_ts_ns", ctypes.c_uint64),
        ("_fill", ctypes.c_uint8 * 64),
    ]


assert ctypes.sizeof(_ShmImuSlot) == 128, ctypes.sizeof(_ShmImuSlot)


class _ShmEncoder(ctypes.Structure):
    """40 bytes - wheel encoders (up to 4 wheels).  4*4+4*4+4+4 = 40."""

    _fields_ = [
        ("ticks", ctypes.c_int32 * 4),
        ("speed", ctypes.c_float * 4),
        ("ts", ctypes.c_float),
        ("_pad", ctypes.c_uint8 * 4),
    ]


assert ctypes.sizeof(_ShmEncoder) == 40, ctypes.sizeof(_ShmEncoder)


class _ShmEncoderSlot(ctypes.Structure):
    """128 bytes.  8+40+8+8+64 = 128."""

    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("encoder", _ShmEncoder),
        ("seq2", ctypes.c_uint64),
        ("writer_ts_ns", ctypes.c_uint64),
        ("_fill", ctypes.c_uint8 * 64),
    ]


assert ctypes.sizeof(_ShmEncoderSlot) == 128, ctypes.sizeof(_ShmEncoderSlot)


class _ShmPcHdr(ctypes.Structure):
    """64 bytes - point cloud metadata header.  8+4+4+8+40 = 64."""

    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("n_points", ctypes.c_uint32),
        ("max_pts", ctypes.c_uint32),
        ("ts", ctypes.c_double),
        ("_fill", ctypes.c_uint8 * 40),
    ]


assert ctypes.sizeof(_ShmPcHdr) == 64, ctypes.sizeof(_ShmPcHdr)


class _ShmPcSlot(ctypes.Structure):
    """128 bytes.  8+64+8+8+40 = 128."""

    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("hdr", _ShmPcHdr),
        ("seq2", ctypes.c_uint64),
        ("writer_ts_ns", ctypes.c_uint64),
        ("_fill", ctypes.c_uint8 * 40),
    ]


assert ctypes.sizeof(_ShmPcSlot) == 128, ctypes.sizeof(_ShmPcSlot)

# Maximum points in a point-cloud payload (each point: x,y,z,intensity as float32)
PC_MAX_POINTS = 65_536
PC_POINT_BYTES = 16  # 4 * float32
PC_DATA_BYTES = PC_MAX_POINTS * PC_POINT_BYTES  # 1 MiB

EXT_SHM_SIZE = (
    768 + PC_DATA_BYTES
)  # header(128)+state(128)+cmd(128)+imu(128)+enc(128)+pc_slot(128)+data


class _ShmExtBlock(ctypes.Structure):
    """Top 768 bytes of the extended segment; raw PC data follows."""

    _fields_ = [
        ("header", _ShmHeader),
        ("state", _ShmStateSlot),
        ("cmd", _ShmCmdSlot),
        ("imu", _ShmImuSlot),
        ("encoder", _ShmEncoderSlot),
        ("pc_slot", _ShmPcSlot),
    ]


assert ctypes.sizeof(_ShmExtBlock) == 768, ctypes.sizeof(_ShmExtBlock)
