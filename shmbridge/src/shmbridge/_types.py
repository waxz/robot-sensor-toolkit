"""
Wire-protocol constants shared across shmbridge's Python public API.

These mirror literal values defined in the C++ headers (types.h,
ext_core.hpp, ext_topics.hpp) — not a parallel implementation of anything,
just the handful of numbers (magic, schema version, default segment names,
payload size limits) that Python-side callers need to know about (e.g. to
validate a payload size before calling into ``shmbridge._core``). All
actual shared-memory layout, seqlock logic, and I/O lives in the compiled
``shmbridge._core`` extension; see docs/design_ring_zero_copy.md and
shmbridge/README.md's "Build" section — the C++ extension is required,
there is no pure-Python fallback.
"""

from __future__ import annotations

MAGIC: int = 0x53484D42  # ASCII "SHMB" -- types.h's SHMBRIDGE_MAGIC
SCHEMA_VERSION: int = 2  # types.h's SHMBRIDGE_VERSION
SHM_NAME_DEFAULT = "/shmbridge_v2"
EXT_SHM_NAME_DEFAULT = "/shmbridge_ext_v2"  # ext_core.hpp's EXT_SHM_NAME_DEFAULT

# Versions ONLY the extended block's own layout (state/cmd + imu/encoder/
# pointcloud/user channels), independent of SCHEMA_VERSION above. Bump this
# -- and the identical constant in include/shmbridge/ext_core.hpp -- whenever
# ShmExtBlock's layout changes.
EXT_SCHEMA_VERSION: int = 1

USER_CHANNEL_NAME_BYTES = 16  # includes the NUL terminator; name <= 15 ASCII chars
USER_CHANNEL_PAYLOAD_BYTES = 88  # raw bytes available to struct.pack per message -- ext_core.hpp's EXT_USER_CHANNEL_PAYLOAD_BYTES
N_USER_CHANNELS = 8  # fixed pool size -- ext_core.hpp's EXT_N_USER_CHANNELS
RAW_PAYLOAD_BYTES = 88  # ext_topics.hpp's RawMsg88 size, used by shmbridge.topics' RawChannelPublisher/Subscriber
