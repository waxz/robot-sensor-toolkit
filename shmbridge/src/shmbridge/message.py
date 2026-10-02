"""
message.py — define message types as small dataclasses (like a ROS .msg),
not new publisher/subscriber classes.

A message is a dataclass with a ``struct`` format string; :class:`Publisher`/
:class:`Subscriber` take the message *type* as an argument the way
``rospy.Publisher(topic, MsgType)`` does, instead of needing a new class
per message the way ``ImuPublisher``/``EncoderPublisher`` briefly did in an
earlier revision of this package (removed — see ext_topics.hpp's header
comment for why that was the wrong shape). Adding a new message type is
purely a Python-side change: no shmbridge C++ code, no rebuild.

    from dataclasses import dataclass
    from shmbridge.message import Message, Publisher, Subscriber

    @dataclass
    class BatteryState(Message):
        _format = "<ffB"  # struct format: voltage(f), current(f), charging(B)
        voltage: float = 0.0
        current: float = 0.0
        charging: bool = False

    pub = Publisher("battery", BatteryState)
    pub.open()
    pub.publish(BatteryState(voltage=12.1, current=0.4, charging=True))

    sub = Subscriber("battery", BatteryState)
    sub.attach()             # or just start calling read(); see below
    msg = sub.read()          # BatteryState | None

:class:`Publisher`/:class:`Subscriber` are each their own shm segment (see
:mod:`shmbridge.topics`) — the same "own named topic" model
examples/cpp_pipeline's separate node executables use, so a completely
different process can publish (or subscribe to) just this one message.
:meth:`Subscriber.read` attaches lazily and non-blockingly on first call if
you haven't called :meth:`Subscriber.attach` yourself, so polling a
topic that may not have a publisher yet is a normal way to use this (it
just reads as ``None`` until one shows up).

There's also :class:`Channel`, the older API bound to one
:class:`~shmbridge.ExtShmBridge` instance's fixed 8-slot named-channel
pool instead of an independent topic — still supported, useful if you're
already using ``ExtShmBridge`` for other channels in the same process and
want one more without a new segment, but :class:`Publisher`/
:class:`Subscriber` is the one to reach for by default.
"""

from __future__ import annotations

import struct
from typing import Any, ClassVar

from ._types import USER_CHANNEL_PAYLOAD_BYTES
from .topics import TopicPublisher, TopicSubscriber


class Message:
    """Base class for a custom shmbridge user-channel payload.

    Subclass as a ``@dataclass`` with a ``_format`` struct format string
    (see the :mod:`struct` docs) whose field order matches the dataclass
    field declaration order. :meth:`pack`/:meth:`unpack` (de)serialize
    to/from the packed bytes stored in a user-channel slot.
    """

    _format: ClassVar[str] = ""

    def pack(self) -> bytes:
        """Serialize this message's dataclass fields to bytes."""
        values = [getattr(self, name) for name in self._field_names()]
        return struct.pack(self._format, *values)

    @classmethod
    def unpack(cls, data: bytes) -> Any:
        """Deserialize the leading ``cls.size()`` bytes of *data* into an instance.

        Fields declared ``bool`` are coerced from struct's plain ``int``
        (e.g. format code ``"B"``) back to an actual ``bool`` — everything
        else is passed through as whatever :func:`struct.unpack` returns.
        """
        size = cls.size()
        values = struct.unpack(cls._format, data[:size])
        import dataclasses

        fields = dataclasses.fields(cls)
        coerced = [
            bool(v) if f.type in (bool, "bool") else v
            for f, v in zip(fields, values)
        ]
        return cls(*coerced)

    @classmethod
    def size(cls) -> int:
        """Packed byte size of this message type."""
        return struct.calcsize(cls._format)

    @classmethod
    def _field_names(cls) -> tuple[str, ...]:
        import dataclasses

        if not dataclasses.is_dataclass(cls):
            raise TypeError(
                f"{cls.__name__} must be a @dataclass subclassing Message"
            )
        return tuple(f.name for f in dataclasses.fields(cls))

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if cls._format and cls.size() > USER_CHANNEL_PAYLOAD_BYTES:
            raise ValueError(
                f"{cls.__name__}._format {cls._format!r} packs to "
                f"{cls.size()} bytes > USER_CHANNEL_PAYLOAD_BYTES "
                f"({USER_CHANNEL_PAYLOAD_BYTES}) — shrink the format or "
                "split into multiple channels"
            )


class Channel:
    """Binds a :class:`Message` type to one named channel on an ExtShmBridge.

    ``bridge`` must already be open()'d or attach()'d. Works with either
    ExtShmBridge backend — both expose the same ``write_channel``/
    ``read_channel`` primitive this wraps.
    """

    def __init__(self, bridge: Any, name: str, message_type: type[Message]) -> None:
        self._bridge = bridge
        self._name = name
        self._type = message_type

    @property
    def name(self) -> str:
        return self._name

    def write(self, msg: Message) -> None:
        """Pack *msg* and seqlock-write it to this channel."""
        self._bridge.write_channel(self._name, msg.pack())

    def read(self) -> Any | None:
        """Seqlock-read this channel and unpack it, or None if unavailable."""
        raw = self._bridge.read_channel(self._name)
        if raw is None:
            return None
        return self._type.unpack(raw)


class Publisher:
    """Publishes one :class:`Message` type on its own named topic.

    ``Publisher(topic, MsgType)`` mirrors ``rospy.Publisher(topic, MsgType)``:
    the message type is an argument, not a subclass or a new wrapper class.
    Backed by :class:`shmbridge.topics.TopicPublisher` (its own shm segment,
    via the compiled C++ extension).
    """

    def __init__(self, topic: str, message_type: type[Message]) -> None:
        self._topic = topic
        self._type = message_type
        self._pub = TopicPublisher()

    @property
    def topic(self) -> str:
        return self._topic

    def open(self) -> None:
        self._pub.open(self._topic)

    def close(self) -> None:
        self._pub.close()

    def publish(self, msg: Message) -> None:
        """Pack *msg* and seqlock-write it to this topic."""
        self._pub.write(msg.pack())


class Subscriber:
    """Subscribes to one :class:`Message` type on its own named topic.

    Backed by :class:`shmbridge.topics.TopicSubscriber`. :meth:`read` attaches
    lazily on first call (non-blocking) if :meth:`attach` wasn't called
    first, so polling a topic before its publisher exists is normal usage.
    """

    def __init__(self, topic: str, message_type: type[Message]) -> None:
        self._topic = topic
        self._type = message_type
        self._sub = TopicSubscriber()

    @property
    def topic(self) -> str:
        return self._topic

    def attach(self, timeout_ms: float = 5000.0) -> None:
        """Attach to the publisher's segment, blocking up to *timeout_ms*.

        Raises ``TimeoutError`` if no publisher shows up in time, or
        ``ValueError`` on a real schema mismatch (a publisher is there, but
        running an incompatible message type/shmbridge build).
        """
        self._sub.attach(self._topic, timeout_ms)

    def detach(self) -> None:
        self._sub.detach()

    def is_attached(self) -> bool:
        return self._sub.is_attached()

    def read(self) -> Any | None:
        """Seqlock-read this topic and unpack it, or None if unavailable.

        Attaches lazily and non-blockingly on first call if :meth:`attach`
        wasn't called already — a topic with no publisher yet just reads as
        None, but a real schema mismatch still raises ``ValueError``.
        """
        if not self.is_attached():
            try:
                self._sub.attach(self._topic, 0.0)
            except TimeoutError:
                return None
        raw = self._sub.read()
        if raw is None:
            return None
        return self._type.unpack(raw)
