"""
topics.py - generic per-topic publish/subscribe classes.

Thin Python-facing adapters over ``shmbridge._core``'s compiled classes
(``RawChannelPublisher``/``RawChannelSubscriber``, a generic 88-byte
message own-segment pair, and ``PointCloudPublisher``/
``PointCloudSubscriber``, bulk payload own-segment pair) -- see
ext_topics.hpp. ``TopicPublisher``/``TopicSubscriber`` and
``PointCloudPublisher``/``PointCloudSubscriber`` are just those classes
under this package's own naming.

This is the layer urdf_tools.pubsub is built on for its per-sensor topics
(see its module docstring) — each sensor gets its own independent segment,
the way examples/cpp_pipeline's separate node executables each own theirs,
rather than one process owning a single combined segment.
"""

from __future__ import annotations

from ._core import PointCloudPublisher  # type: ignore[import]
from ._core import PointCloudSubscriber as _CppPcSubscriber  # type: ignore[import]
from ._core import RawChannelPublisher as TopicPublisher  # type: ignore[import]
from ._core import RawChannelSubscriber as _CppTopicSubscriber  # type: ignore[import]
from ._types import RAW_PAYLOAD_BYTES


class TopicSubscriber:
    """Wraps ``_core.RawChannelSubscriber`` so ``attach()`` raises
    TimeoutError (not a plain RuntimeError) on a timed-out attach, matching
    every other attach()/open() pair in this package. A schema/type
    mismatch already raises ValueError (pybind11 maps std::invalid_argument
    to ValueError automatically).
    """

    def __init__(self) -> None:
        self._sub = _CppTopicSubscriber()

    def attach(self, name: str, timeout_ms: float = 5000.0) -> None:
        try:
            self._sub.attach(name, int(timeout_ms))
        except RuntimeError as e:
            raise TimeoutError(str(e)) from e

    def __getattr__(self, name: str):
        return getattr(self._sub, name)


class PointCloudSubscriber:
    """Wraps ``_core.PointCloudSubscriber``; see :class:`TopicSubscriber`."""

    def __init__(self) -> None:
        self._sub = _CppPcSubscriber()

    def attach(self, name: str, timeout_ms: float = 30000.0) -> None:
        try:
            self._sub.attach(name, timeout_ms)
        except RuntimeError as e:
            raise TimeoutError(str(e)) from e

    def __getattr__(self, name: str):
        return getattr(self._sub, name)


__all__ = [
    "RAW_PAYLOAD_BYTES",
    "PointCloudPublisher",
    "PointCloudSubscriber",
    "TopicPublisher",
    "TopicSubscriber",
]
