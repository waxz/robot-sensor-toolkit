"""Protocol definitions for the shared world-model interface.

Sensors in this package accept any object that satisfies these lightweight
protocols — they are not tied to ir-sim's ObjectBase or Scene3D.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class GeometryObject2D(Protocol):
    """Minimal interface for a 2D obstacle/object used by Lidar2D and Encoder."""

    @property
    def geometry(self) -> Any:  # shapely.Geometry
        """2D Shapely geometry of the object in the world frame."""
        ...

    @property
    def obj_id(self) -> int:
        """Unique integer identifier for this object."""
        ...


@runtime_checkable
class Scene3DProtocol(Protocol):
    """Interface for a 3D ray-casting scene used by Lidar3D.

    Any object with a ``cast_3d_lidar`` method satisfying this signature
    can be assigned to ``Lidar3D.scene``.  The ir-sim ``Scene3D`` class
    satisfies this protocol without modification.
    """

    def cast_3d_lidar(
        self,
        origin: list[float],
        profile: str,
        range_max: float,
    ) -> np.ndarray:
        """Cast a full 3D LiDAR scan from ``origin``.

        Args:
            origin: Sensor origin ``[x, y, z]`` in metres.
            profile: Named beam pattern (e.g. ``"vlp16"``).
            range_max: Maximum detection range in metres.

        Returns:
            Array of shape ``(N, 4)`` — columns ``(x, y, z, distance)``.
        """
        ...
