"""
shmbridge - cross-platform shared-memory seqlock bridge for Python and C++
robot control.

The Python API in this package is a thin wrapper around the compiled C++
extension (``shmbridge._core``, built from csrc/py_bindings.cpp against
include/shmbridge/*.hpp) — there is no pure-Python fallback. Installing
this package therefore requires a C++17 compiler and pybind11 at build
time (see shmbridge/README.md's "Build" section); `pip install -e .` /
`uv pip install -e .` handle this automatically via scikit-build-core.

Roles are symmetric: Python or C++ can be publisher (sim) or subscriber
(controller).

Quick start::

    # Python publisher (simulator)
    from shmbridge import ShmPublisher, RobotState
    pub = ShmPublisher("/shmbridge_v2")
    pub.open()
    pub.write_state(0, state)
    cmd = pub.read_best_cmd(0)

    # Python subscriber (controller)
    from shmbridge import ShmSubscriber
    sub = ShmSubscriber("/shmbridge_v2")
    sub.attach()
    state = sub.read_state_spin(0)
    sub.write_cmd(0, 0, linear=0.5, angular=0.1)

    // C++ publisher or subscriber: #include <shmbridge/core.hpp>
    shmbridge::ShmPublisher pub("/shmbridge_v2");
    pub.open();
    pub.write_state(0, state);

Custom message types (IMU/encoder/point-cloud cover the built-in sensors;
anything else — battery, a custom detector, whatever) are plain dataclasses,
ROS-style: define the message, not a new publisher/subscriber class::

    from dataclasses import dataclass
    from shmbridge.message import Message, Publisher, Subscriber

    @dataclass
    class BatteryState(Message):
        _format = "<ffB"  # voltage, current, charging
        voltage: float = 0.0
        current: float = 0.0
        charging: bool = False

    pub = Publisher("battery", BatteryState)
    pub.open()
    pub.publish(BatteryState(voltage=12.1, current=0.4, charging=True))

    sub = Subscriber("battery", BatteryState)
    msg = sub.read()  # BatteryState | None (attaches lazily)

See ``shmbridge.message`` for ``Channel``, an older alternative bound to one
``ExtShmBridge``'s fixed named-channel pool instead of an independent topic.
"""

try:
    from ._core import RobotCmd, RobotState, ShmPublisher, ShmSubscriber  # type: ignore[import]
except ImportError as e:
    raise ImportError(
        "shmbridge requires its compiled C++ extension (shmbridge._core), "
        "which failed to import. Build it with `pip install -e .` or "
        "`uv pip install -e .` from the shmbridge/ directory (requires a "
        "C++17 compiler and pybind11 — see README.md's Build section)."
    ) from e
from ._types import (
    EXT_SHM_NAME_DEFAULT,
    MAGIC,
    N_USER_CHANNELS,
    SCHEMA_VERSION,
    SHM_NAME_DEFAULT,
    USER_CHANNEL_PAYLOAD_BYTES,
)
from .bridge_ext import ExtShmBridge
from .message import Channel, Message, Publisher, Subscriber
from .topics import PointCloudPublisher, PointCloudSubscriber, TopicPublisher, TopicSubscriber

__version__ = "2.1.0"
__all__ = [
    "EXT_SHM_NAME_DEFAULT",
    "MAGIC",
    "N_USER_CHANNELS",
    "SCHEMA_VERSION",
    "SHM_NAME_DEFAULT",
    "USER_CHANNEL_PAYLOAD_BYTES",
    "Channel",
    "ExtShmBridge",
    "Message",
    "PointCloudPublisher",
    "PointCloudSubscriber",
    "Publisher",
    "RobotCmd",
    "RobotState",
    "ShmPublisher",
    "ShmSubscriber",
    "Subscriber",
    "TopicPublisher",
    "TopicSubscriber",
]
