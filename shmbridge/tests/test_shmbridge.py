"""Tests for shmbridge v2."""

import ctypes
import os
import time

import pytest

from shmbridge import MAGIC, SCHEMA_VERSION, ShmBridge
from shmbridge._types import (
    _shm_size,
    _ShmCmdSlot,
    _ShmHeader,
    _ShmStateSlot,
    make_block_type,
)


@pytest.fixture
def bridge(tmp_path):
    shm_name = f"/test_shmbridge_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name)
    b.open()
    yield b
    b.close()


# ── struct sizes ───────────────────────────────────────────────────────────


def test_state_slot_size():
    assert ctypes.sizeof(_ShmStateSlot) == 128


def test_cmd_slot_size():
    assert ctypes.sizeof(_ShmCmdSlot) == 128


def test_header_size():
    assert ctypes.sizeof(_ShmHeader) == 128


def test_block_size_single():
    blk = make_block_type(1)
    assert ctypes.sizeof(blk) == 384


def test_block_size_multi():
    for n in (2, 4, 8):
        blk = make_block_type(n)
        assert ctypes.sizeof(blk) == 128 + 256 * n


def test_shm_size_aligned():
    assert _shm_size(1) == 4096  # 384 < 4096, rounds up
    assert _shm_size(16) == 4096 * 2  # 128+256*16 = 4224 → 8192


# ── field offsets (must match C++ types.h) ────────────────────────────────


def test_state_slot_writer_ts_ns_offset():
    assert _ShmStateSlot.writer_ts_ns.offset == 88


def test_cmd_slot_writer_ts_ns_offset():
    assert _ShmCmdSlot.writer_ts_ns.offset == 32


def test_header_magic_offset():
    assert _ShmHeader.magic.offset == 8


def test_header_n_robots_offset():
    assert _ShmHeader.n_robots.offset == 16


# ── open / header ─────────────────────────────────────────────────────────


def test_header_magic_after_open(bridge):
    assert bridge._blk.header.magic == MAGIC


def test_header_version_after_open(bridge):
    assert bridge._blk.header.schema_version == SCHEMA_VERSION


def test_header_ready_after_open(bridge):
    assert bridge._blk.header.ready == 1


def test_header_n_robots_after_open(bridge):
    assert bridge._blk.header.n_robots == 1


# ── write_state seqlock ───────────────────────────────────────────────────


def test_write_state_sets_even_seqlock(bridge):
    bridge.write_state(1, 2, 3, 0, 0, 0, 5, 6, 1.2, step=1, sim_time=0.01)
    slot = bridge._blk.states[0]
    assert slot.seq % 2 == 0, "seq must be even (not mid-write)"
    assert slot.seq2 % 2 == 0, "seq2 must be even"
    assert slot.seq == slot.seq2, "seq must equal seq2 after write"


def test_write_state_values(bridge):
    bridge.write_state(
        1.1, 2.2, 3.3, 0.5, 0.0, 0.1, 4.0, 5.0, 1.5, step=7, sim_time=0.07
    )
    s = bridge._blk.states[0].state
    assert abs(s.x - 1.1) < 1e-9
    assert abs(s.y - 2.2) < 1e-9
    assert abs(s.heading - 3.3) < 1e-9
    assert abs(s.vx - 0.5) < 1e-5
    assert abs(s.goal_x - 4.0) < 1e-5
    assert abs(s.goal_dist - 1.5) < 1e-5
    assert s.step == 7
    assert abs(s.sim_time - 0.07) < 1e-9


def test_writer_ts_ns_updated(bridge):
    before = bridge._blk.states[0].writer_ts_ns
    bridge.write_state(0, 0, 0, 0, 0, 0, 0, 0, 0, step=1, sim_time=0)
    after = bridge._blk.states[0].writer_ts_ns
    assert after > before


# ── read_cmd ──────────────────────────────────────────────────────────────


def test_read_cmd_returns_none_before_write(bridge):
    assert bridge.read_cmd() is None


def test_read_cmd_after_write(bridge):
    # Manually write a cmd via ctypes
    slot = bridge._blk.cmds[0]
    slot.seq = 1  # odd: begin write
    slot.cmd.linear = 0.8
    slot.cmd.angular = -0.3
    slot.cmd.seq = 1
    slot.cmd.valid = 1
    slot.seq = 2  # even: done
    slot.seq2 = 2
    cmd = bridge.read_cmd()
    assert cmd is not None
    assert abs(cmd.linear - 0.8) < 1e-5
    assert abs(cmd.angular + 0.3) < 1e-5


# ── liveness ──────────────────────────────────────────────────────────────


def test_is_writer_alive_no_write(bridge):
    # ts == 0 → never written → considered alive
    assert bridge.is_writer_alive(max_age_ms=0) is True


def test_is_writer_alive_after_write(bridge):
    bridge.write_state(0, 0, 0, 0, 0, 0, 0, 0, 0, step=1, sim_time=0)
    assert bridge.is_writer_alive(max_age_ms=1000) is True
    # immediately stale
    assert bridge.is_writer_alive(max_age_ms=0) is False


def test_is_controller_alive_no_cmd(bridge):
    assert bridge.is_controller_alive(max_age_ms=0) is True


# ── multi-robot ───────────────────────────────────────────────────────────


def test_multi_robot_open():
    shm_name = f"/test_shmbridge_multi_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name, n_robots=4)
    b.open()
    try:
        assert b._blk.header.n_robots == 4
        assert len(b._state_slots) == 4
        assert len(b._cmd_slots) == 4
        for i in range(4):
            b.write_state(
                float(i), 0, 0, 0, 0, 0, 0, 0, 0, step=i, sim_time=0, robot_idx=i
            )
        # Read scalar values directly (no ctypes refs held across close()).
        xs = [float(b._blk.states[i].state.x) for i in range(4)]
    finally:
        b.close()
    for i, x in enumerate(xs):
        assert abs(x - float(i)) < 1e-9


# ── read_cmd_blocking ─────────────────────────────────────────────────────


def test_read_cmd_blocking_timeout(bridge):
    t0 = time.monotonic()
    result = bridge.read_cmd_blocking(timeout_ms=20.0)
    elapsed_ms = (time.monotonic() - t0) * 1000
    assert result is None
    assert 15 < elapsed_ms < 100, f"timeout elapsed: {elapsed_ms:.1f} ms"


# ── n_consumers header ────────────────────────────────────────────────────


def test_n_consumers_header_default():
    shm_name = f"/test_shmbridge_nc1_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name, n_consumers=1)
    b.open()
    try:
        assert b._blk.header.n_consumers == 1
    finally:
        b.close()


def test_n_consumers_header_multi():
    shm_name = f"/test_shmbridge_nc3_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name, n_consumers=3)
    b.open()
    try:
        assert b._blk.header.n_consumers == 3
    finally:
        b.close()


# ── multi-consumer block size ─────────────────────────────────────────────


def test_multi_consumer_block_size():
    # 2 robots, 3 consumers: 128 + 2*128 + 2*3*128 = 128 + 256 + 768 = 1152
    blk = make_block_type(2, 3)
    assert ctypes.sizeof(blk) == 1152


def test_shm_size_multi_consumer():
    assert _shm_size(2, 3) == 4096  # 1152 < 4096


# ── read_best_cmd arbiter ─────────────────────────────────────────────────


def test_read_best_cmd_picks_highest_seq():
    shm_name = f"/test_shmbridge_best_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name, n_consumers=2)
    b.open()
    try:
        # Write to consumer slot 0 with seq=1
        slot0 = b._blk.cmds[0]
        slot0.seq = 1
        slot0.cmd.linear = 0.5
        slot0.cmd.angular = 0.0
        slot0.cmd.seq = 1
        slot0.cmd.valid = 1
        slot0.seq = 2
        slot0.seq2 = 2
        # Write to consumer slot 1 with seq=5 (higher, should win)
        slot1 = b._blk.cmds[1]
        slot1.seq = 1
        slot1.cmd.linear = 1.0
        slot1.cmd.angular = 0.1
        slot1.cmd.seq = 5
        slot1.cmd.valid = 1
        slot1.seq = 2
        slot1.seq2 = 2
        best = b.read_best_cmd(robot_idx=0)
        # Release ctypes slot refs before close() to avoid BufferError
        del slot0, slot1
        assert best is not None
        assert best.seq == 5
        assert abs(best.linear - 1.0) < 1e-5
    finally:
        b.close()


def test_read_best_cmd_none_when_no_valid():
    shm_name = f"/test_shmbridge_best_none_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name, n_consumers=2)
    b.open()
    try:
        assert b.read_best_cmd(robot_idx=0) is None
    finally:
        b.close()


# ── write_state_fast ──────────────────────────────────────────────────────


def test_write_state_fast_values(bridge):
    bridge.write_state_fast(
        1.1,
        2.2,
        3.3,
        0.5,
        0.0,
        0.1,
        4.0,
        5.0,
        1.5,
        step=7,
        sim_time=0.07,
        reached=True,
        collision=False,
    )
    s = bridge._blk.states[0].state
    assert abs(s.x - 1.1) < 1e-9
    assert abs(s.y - 2.2) < 1e-9
    assert abs(s.heading - 3.3) < 1e-9
    assert abs(s.vx - 0.5) < 1e-5
    assert abs(s.goal_x - 4.0) < 1e-5
    assert abs(s.goal_dist - 1.5) < 1e-5
    assert s.step == 7
    assert abs(s.sim_time - 0.07) < 1e-9
    assert s.reached == 1
    assert s.collision == 0


def test_write_state_fast_seqlock(bridge):
    bridge.write_state_fast(0, 0, 0, 0, 0, 0, 0, 0, 0, step=1, sim_time=0)
    slot = bridge._blk.states[0]
    assert slot.seq % 2 == 0
    assert slot.seq2 % 2 == 0
    assert slot.seq == slot.seq2


def test_write_state_fast_ts_updated(bridge):
    before = bridge._blk.states[0].writer_ts_ns
    bridge.write_state_fast(0, 0, 0, 0, 0, 0, 0, 0, 0, step=1, sim_time=0)
    after = bridge._blk.states[0].writer_ts_ns
    assert after > before


# ── heartbeat_every batching ──────────────────────────────────────────────


def test_heartbeat_every_batching():
    shm_name = f"/test_shmbridge_hb_{os.getpid()}"
    b = ShmBridge(shm_name=shm_name, heartbeat_every=4)
    b.open()
    try:
        # First 3 writes: ts should stay 0 (no heartbeat yet)
        for _ in range(3):
            b.write_state(0, 0, 0, 0, 0, 0, 0, 0, 0, step=0, sim_time=0)
        ts_before = b._blk.states[0].writer_ts_ns
        # 4th write: triggers heartbeat
        b.write_state(0, 0, 0, 0, 0, 0, 0, 0, 0, step=0, sim_time=0)
        ts_after = b._blk.states[0].writer_ts_ns
        assert ts_after > ts_before
    finally:
        b.close()


def test_heartbeat_every_1_updates_immediately(bridge):
    # Default heartbeat_every=1: every write updates ts
    before = bridge._blk.states[0].writer_ts_ns
    bridge.write_state(0, 0, 0, 0, 0, 0, 0, 0, 0, step=1, sim_time=0)
    after = bridge._blk.states[0].writer_ts_ns
    assert after > before


# ── header n_consumers offset ─────────────────────────────────────────────


def test_header_n_consumers_offset():
    from shmbridge._types import _ShmHeader

    assert _ShmHeader.n_consumers.offset == 17
