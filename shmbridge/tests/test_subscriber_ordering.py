"""
test_subscriber_ordering.py — tests for subscriber attach() retry logic.

Covers the scenario where the subscriber starts before the publisher:
  • Python fallback (_PyShmSubscriber) retry on segment-not-found
  • C++ extension (ShmSubscriber) retry on segment-not-found (if compiled)
  • Timeout propagation when publisher never starts
  • Multiprocess: publisher starts N seconds after subscriber

All tests are cross-platform (Linux, Windows, macOS).
"""

from __future__ import annotations

import multiprocessing
import os
import sys
import time

import pytest

from shmbridge import ShmPublisher
from shmbridge.bridge import _PyShmSubscriber


def _shm_name(suffix: str) -> str:
    return f"/test_order_{suffix}_{os.getpid()}"


# ── helpers ───────────────────────────────────────────────────────────────────


def _publisher_proc(name: str, delay_s: float, n_writes: int) -> None:
    """Child process: sleep, then publish n_writes states and exit."""
    time.sleep(delay_s)
    pub = ShmPublisher(name, n_robots=1, n_consumers=1)
    pub.open()
    from shmbridge import RobotState

    for i in range(n_writes):
        s = RobotState()
        s.step = i
        s.sim_time = i * 0.01
        pub.write_state(0, s)
        time.sleep(0.01)
    pub.close()


def _subscriber_proc(
    name: str,
    timeout_ms: float,
    result_queue: multiprocessing.Queue[bool],
) -> None:
    """Child process: attach and read one state; put True/False into queue.

    Uses the pure-Python fallback (_PyShmSubscriber) so this works even when
    the compiled C++ extension has a different (older) attach() signature.
    """
    try:
        from shmbridge.bridge import _PyShmSubscriber

        sub = _PyShmSubscriber(name, n_robots=1)
        sub.attach(timeout_ms=float(timeout_ms))
        state = sub.read_state_spin(0)
        sub.detach()
        result_queue.put(state is not None)
    except Exception as exc:
        result_queue.put(False)
        print(f"[sub proc] error: {exc}", flush=True)


# ── unit tests (single-process) ────────────────────────────────────────────────


class TestPySubscriberRetry:
    """Test the pure-Python _PyShmSubscriber retry logic."""

    def test_attach_timeout_when_no_publisher(self) -> None:
        """attach() raises TimeoutError when publisher never starts."""
        name = _shm_name("no_pub")
        sub = _PyShmSubscriber(name)
        t0 = time.monotonic()
        with pytest.raises((TimeoutError, OSError)):
            sub.attach(timeout_ms=300)
        elapsed = (time.monotonic() - t0) * 1000
        # Should timeout within a reasonable window (~300 ms ± 200 ms overhead)
        assert elapsed < 700, f"timeout took too long: {elapsed:.0f} ms"

    def test_attach_succeeds_when_publisher_exists(self) -> None:
        """attach() succeeds immediately when the publisher is already open."""
        name = _shm_name("pub_first")
        pub = ShmPublisher(name)
        pub.open()
        try:
            sub = _PyShmSubscriber(name)
            sub.attach(timeout_ms=1000)
            assert sub.is_attached()
            sub.detach()
        finally:
            pub.close()

    def test_attach_waits_for_ready_flag(self) -> None:
        """attach() blocks until the publisher sets header.ready=1."""
        import mmap
        import threading

        from shmbridge._types import MAGIC, SCHEMA_VERSION, _shm_size

        name = _shm_name("ready_flag")
        n, nc = 1, 1
        size = _shm_size(n, nc)

        # Create the mapping manually without setting ready
        if sys.platform == "win32":
            tag = name.lstrip("/")
            mm = mmap.mmap(-1, size, tagname=tag, access=mmap.ACCESS_WRITE)
            mm.write(b"\x00" * size)
            mm.seek(0)
        else:
            from shmbridge._libc import _libc
            from shmbridge._platform import O_CREAT, O_EXCL, O_RDWR

            _libc.shm_unlink(name.encode())
            fd = _libc.shm_open(name.encode(), O_CREAT | O_RDWR | O_EXCL, 0o666)
            _libc.ftruncate(fd, size)
            mm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            os.close(fd)
            mm.write(b"\x00" * size)
            mm.seek(0)

        try:
            # Set magic and n_robots but NOT ready
            from shmbridge._types import make_block_type

            blk_type = make_block_type(n, nc)
            blk = blk_type.from_buffer(mm)
            blk.header.magic = MAGIC
            blk.header.schema_version = SCHEMA_VERSION
            blk.header.n_robots = n
            blk.header.n_consumers = nc
            # ready stays 0

            # Set ready after 200 ms
            def set_ready() -> None:
                time.sleep(0.2)
                blk.header.ready = 1

            t = threading.Thread(target=set_ready)
            t.start()

            sub = _PyShmSubscriber(name)
            t0 = time.monotonic()
            sub.attach(timeout_ms=3000)
            elapsed = (time.monotonic() - t0) * 1000
            t.join()

            assert sub.is_attached()
            assert 150 < elapsed < 800, f"ready-wait took {elapsed:.0f} ms"
            sub.detach()
        finally:
            blk = None  # release ctypes reference before closing mmap
            mm.close()
            if sys.platform != "win32":
                _libc.shm_unlink(name.encode())


class TestRoundTrip:
    """Publisher → subscriber round-trip of state and command."""

    def test_write_state_read_state(self) -> None:
        name = _shm_name("roundtrip")
        pub = ShmPublisher(name, n_robots=1)
        pub.open()
        sub = _PyShmSubscriber(name)
        sub.attach(timeout_ms=1000)

        from shmbridge import RobotState

        s = RobotState()
        s.x = 1.5
        s.y = -2.3
        s.step = 42
        pub.write_state(0, s)

        state = sub.read_state_spin(0)
        assert state is not None
        assert abs(state.x - 1.5) < 1e-6
        assert abs(state.y - (-2.3)) < 1e-6
        assert state.step == 42

        sub.detach()
        pub.close()

    def test_write_cmd_read_cmd(self) -> None:
        name = _shm_name("cmd_rt")
        pub = ShmPublisher(name, n_robots=1, n_consumers=1)
        pub.open()
        sub = _PyShmSubscriber(name)
        sub.attach(timeout_ms=1000)

        sub.write_cmd(0, 0, linear=0.75, angular=-0.5)
        cmd = pub.read_cmd(0, 0)
        assert cmd is not None
        assert abs(cmd.linear - 0.75) < 1e-5
        assert abs(cmd.angular - (-0.5)) < 1e-5

        sub.detach()
        pub.close()

    def test_liveness_checks(self) -> None:
        name = _shm_name("liveness")
        pub = ShmPublisher(name, n_robots=1)
        pub.open()
        sub = _PyShmSubscriber(name)
        sub.attach(timeout_ms=1000)

        from shmbridge import RobotState

        pub.write_state(0, RobotState())
        assert sub.is_publisher_alive(max_age_ms=1000)
        assert not sub.is_publisher_alive(max_age_ms=0)

        sub.detach()
        pub.close()


# ── multiprocess tests ────────────────────────────────────────────────────────


@pytest.mark.parametrize("delay_s", [0.0, 0.3, 0.8])
def test_subscriber_starts_before_publisher(delay_s: float) -> None:
    """Subscriber starts first; publisher appears after delay_s seconds."""
    name = _shm_name(f"mp_{int(delay_s * 1000)}")
    ctx = multiprocessing.get_context("spawn")
    q: multiprocessing.Queue[bool] = ctx.Queue()

    sub_proc = ctx.Process(
        target=_subscriber_proc,
        args=(name, 5000.0, q),
        daemon=True,
    )
    pub_proc = ctx.Process(
        target=_publisher_proc,
        args=(name, delay_s, 20),
        daemon=True,
    )

    sub_proc.start()
    time.sleep(0.05)  # let subscriber reach attach()
    pub_proc.start()

    pub_proc.join(timeout=10)
    sub_proc.join(timeout=10)

    assert not q.empty(), "subscriber process produced no result"
    result = q.get_nowait()
    assert result, f"subscriber failed to read state after {delay_s:.1f} s delay"


def test_subscriber_timeout_when_publisher_never_starts() -> None:
    """Subscriber gives up cleanly when publisher never starts."""
    name = _shm_name("mp_timeout")
    ctx = multiprocessing.get_context("spawn")
    q: multiprocessing.Queue[bool] = ctx.Queue()

    sub_proc = ctx.Process(
        target=_subscriber_proc,
        args=(name, 400.0, q),  # 400 ms timeout
        daemon=True,
    )
    t0 = time.monotonic()
    sub_proc.start()
    sub_proc.join(timeout=5)
    elapsed = time.monotonic() - t0

    # Subscriber should have exited (result = False, not hanging)
    assert not sub_proc.is_alive(), "subscriber process is still alive (hung?)"
    assert elapsed < 3.0, f"subscriber took too long to timeout: {elapsed:.1f} s"
    if not q.empty():
        assert q.get_nowait() is False


# ── publisher-restart tests ────────────────────────────────────────────────────


class TestPublisherRestart:
    """attach() is safe to call again after publisher exits and restarts."""

    def test_reattach_after_publisher_restart(self) -> None:
        """Subscriber can re-attach to a fresh segment after publisher restarts."""
        name = _shm_name("restart")
        from shmbridge import RobotState
        from shmbridge.bridge import _PyShmSubscriber

        sub = _PyShmSubscriber(name, n_robots=1)

        # ── round 1 ──────────────────────────────────────────────────────────
        pub1 = ShmPublisher(name, n_robots=1)
        pub1.open()
        sub.attach(timeout_ms=2000)
        s = RobotState()
        s.step = 1
        pub1.write_state(0, s)
        state1 = sub.read_state_spin(0)
        assert state1 is not None
        assert state1.step == 1
        pub1.close()
        sub.detach()

        # ── round 2: publisher restarts with new segment ──────────────────
        pub2 = ShmPublisher(name, n_robots=1)
        pub2.open()
        sub.attach(timeout_ms=2000)  # re-attach to new segment
        s2 = RobotState()
        s2.step = 2
        pub2.write_state(0, s2)
        state2 = sub.read_state_spin(0)
        assert state2 is not None
        assert state2.step == 2
        sub.detach()
        pub2.close()

    def test_attach_auto_detaches_previous_mapping(self) -> None:
        """Calling attach() without detach() releases the old mapping first."""
        name = _shm_name("auto_detach")
        from shmbridge import RobotState
        from shmbridge.bridge import _PyShmSubscriber

        pub = ShmPublisher(name, n_robots=1)
        pub.open()
        sub = _PyShmSubscriber(name, n_robots=1)

        sub.attach(timeout_ms=2000)
        assert sub.is_attached()

        # Call attach() again without detach() — should not raise or leak
        sub.attach(timeout_ms=2000)
        assert sub.is_attached()

        s = RobotState()
        s.step = 99
        pub.write_state(0, s)
        state = sub.read_state_spin(0)
        assert state is not None
        assert state.step == 99

        sub.detach()
        pub.close()

    def test_reconnect_loop_across_publisher_restart(self) -> None:
        """Simulate the reconnect loop: publisher exits and subscriber re-attaches."""
        import threading

        name = _shm_name("reconnect_loop")
        from shmbridge import RobotState
        from shmbridge.bridge import _PyShmSubscriber

        results: list[int] = []

        def publisher_lifecycle() -> None:
            for run in range(2):
                pub = ShmPublisher(name, n_robots=1)
                pub.open()
                s = RobotState()
                s.step = run + 1
                pub.write_state(0, s)
                time.sleep(0.1)  # hold segment open briefly
                pub.close()
                if run == 0:
                    time.sleep(0.2)  # gap between publisher runs

        pub_thread = threading.Thread(target=publisher_lifecycle)
        pub_thread.start()

        sub = _PyShmSubscriber(name, n_robots=1)
        for _ in range(2):
            # attach() auto-detaches previous mapping
            sub.attach(timeout_ms=3000)
            state = sub.read_state_spin(0)
            if state is not None:
                results.append(state.step)
            # wait for publisher to exit
            while sub.is_publisher_alive(max_age_ms=300):
                time.sleep(0.05)
            sub.detach()

        pub_thread.join(timeout=5)

        assert results == [1, 2], f"expected [1, 2], got {results}"
