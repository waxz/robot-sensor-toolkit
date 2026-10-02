"""
Behavioral tests for shmbridge's public Python API.

All of this goes through the compiled ``shmbridge._core`` extension (there
is no pure-Python fallback to test separately) — these are black-box tests
against the public classes, not implementation-internal ones. C++-level
coverage of the same transport lives in tests/test_core.cpp.
"""

import os

import pytest

from shmbridge import ExtShmBridge, RobotState, ShmPublisher, ShmSubscriber


def _name(suffix: str) -> str:
    return f"/test_shmbridge_{suffix}_{os.getpid()}"


# ── ShmPublisher / ShmSubscriber ─────────────────────────────────────────────


@pytest.fixture
def pub_sub():
    name = _name("pubsub")
    pub = ShmPublisher(name, n_robots=1)
    pub.open()
    sub = ShmSubscriber(name, n_robots=1)
    sub.attach(timeout_ms=2000)
    yield pub, sub
    sub.detach()
    pub.close()


def test_write_state_read_state(pub_sub):
    pub, sub = pub_sub
    s = RobotState()
    s.x, s.y, s.step = 1.5, -2.3, 42
    pub.write_state(0, s)
    state = sub.read_state(0)
    assert state is not None
    assert abs(state.x - 1.5) < 1e-6
    assert abs(state.y - (-2.3)) < 1e-6
    assert state.step == 42


def test_read_cmd_none_before_write(pub_sub):
    pub, _sub = pub_sub
    assert pub.read_cmd(0, 0) is None


def test_write_cmd_read_cmd(pub_sub):
    pub, sub = pub_sub
    sub.write_cmd(0, 0, linear=0.75, angular=-0.5)
    cmd = pub.read_cmd(0, 0)
    assert cmd is not None
    assert abs(cmd.linear - 0.75) < 1e-5
    assert abs(cmd.angular - (-0.5)) < 1e-5


def test_liveness_checks(pub_sub):
    pub, sub = pub_sub
    pub.write_state(0, RobotState())
    assert sub.is_publisher_alive(max_age_ms=1000)
    assert not sub.is_publisher_alive(max_age_ms=0)


def test_attach_timeout_when_no_publisher():
    sub = ShmSubscriber(_name("no_pub"))
    with pytest.raises((TimeoutError, OSError, RuntimeError)):
        sub.attach(timeout_ms=300)


def test_subscriber_reattaches_after_publisher_restart():
    name = _name("restart")
    sub = ShmSubscriber(name)

    pub1 = ShmPublisher(name)
    pub1.open()
    sub.attach(timeout_ms=2000)
    s = RobotState()
    s.step = 1
    pub1.write_state(0, s)
    assert sub.read_state(0).step == 1
    pub1.close()
    sub.detach()

    pub2 = ShmPublisher(name)
    pub2.open()
    sub.attach(timeout_ms=2000)
    s2 = RobotState()
    s2.step = 2
    pub2.write_state(0, s2)
    assert sub.read_state(0).step == 2
    sub.detach()
    pub2.close()


# ── ExtShmBridge ──────────────────────────────────────────────────────────────


@pytest.fixture
def ext_bridge():
    bridge = ExtShmBridge(_name("ext"))
    bridge.open()
    yield bridge
    bridge.close()


def test_ext_write_read_state(ext_bridge):
    ext_bridge.write_state(x=1.0, y=2.0, heading=0.5, vx=0.1, vy=0.2, omega=0.0,
                            goal_x=5.0, goal_y=5.0, goal_dist=3.0, step=1, sim_time=0.1)
    s = ext_bridge.read_state()
    assert s is not None
    assert abs(s.x - 1.0) < 1e-9
    assert s.step == 1


def test_ext_write_read_imu(ext_bridge):
    ext_bridge.write_imu(ax=1, ay=2, az=3, gx=4, gy=5, gz=6)
    imu = ext_bridge.read_imu()
    assert imu is not None


def test_ext_write_read_channel(ext_bridge):
    ext_bridge.write_channel("foo", b"bar" * 10)
    assert ext_bridge.read_channel("foo")[:9] == b"barbarbar"


def test_ext_list_topics_reports_written_channels(ext_bridge):
    ext_bridge.write_imu(ax=0, ay=0, az=0, gx=0, gy=0, gz=0)
    topics = {t.name: t.alive for t in ext_bridge.list_topics()}
    assert topics.get("imu") is True
    assert topics.get("cmd") is False  # never written


# ── shmbridge.message (generic Publisher/Subscriber) ─────────────────────────


def test_message_publisher_subscriber_round_trip():
    from dataclasses import dataclass

    from shmbridge.message import Message, Publisher, Subscriber

    @dataclass
    class BatteryState(Message):
        _format = "<ffB"
        voltage: float = 0.0
        current: float = 0.0
        charging: bool = False

    topic = _name("battery")
    pub = Publisher(topic, BatteryState)
    pub.open()
    sub = Subscriber(topic, BatteryState)
    try:
        pub.publish(BatteryState(voltage=12.1, current=0.4, charging=True))
        msg = sub.read()
        assert msg is not None
        assert abs(msg.voltage - 12.1) < 1e-5
        assert msg.charging is True
    finally:
        sub.detach()
        pub.close()


def test_message_subscriber_reads_none_before_publish():
    from dataclasses import dataclass

    from shmbridge.message import Message, Subscriber

    @dataclass
    class Empty(Message):
        _format = "<f"
        value: float = 0.0

    sub = Subscriber(_name("never_published"), Empty)
    assert sub.read() is None
