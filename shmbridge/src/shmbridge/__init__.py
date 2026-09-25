"""
shmbridge - POSIX shared-memory seqlock bridge for Python and C++ robot control.

When the C++ extension (_core.so) is installed it is used automatically; the
pure-Python ctypes fallback is loaded otherwise, so the package works in any
environment without a compiler.

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
"""

from ._types import EXT_SHM_NAME_DEFAULT, MAGIC, SCHEMA_VERSION, SHM_NAME_DEFAULT
from .bridge import (  # always available
    RobotCmd,
    RobotState,
    ShmBridge,
    _PyShmSubscriber,
)
from .bridge_ext import ExtShmBridge

# ── C++ extension (preferred) or pure-Python fallback ────────────────────────
_BACKEND = "python"

try:
    from ._core import ShmPublisher, ShmSubscriber  # type: ignore[import]

    _BACKEND = "cpp"
    # Re-export C++ RobotState / RobotCmd so downstream code gets the fast types
    from ._core import RobotCmd, RobotState  # type: ignore[import]
except ImportError:
    # Fallback: ShmPublisher = ShmBridge (publisher role)
    #           ShmSubscriber = _PyShmSubscriber (subscriber role)
    ShmPublisher = ShmBridge  # type: ignore[misc,assignment]
    ShmSubscriber = _PyShmSubscriber  # type: ignore[misc,assignment]

__version__ = "2.1.0"
__all__ = [
    "EXT_SHM_NAME_DEFAULT",
    "MAGIC",
    "SCHEMA_VERSION",
    "SHM_NAME_DEFAULT",
    "ExtShmBridge",
    "RobotCmd",
    "RobotState",
    "ShmBridge",
    "ShmPublisher",
    "ShmSubscriber",
]
