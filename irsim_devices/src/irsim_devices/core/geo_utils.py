"""Lightweight 2-D geometry utilities used by the sensor modules.

Dependency-free (only numpy + shapely).
"""

from __future__ import annotations

from math import pi
from typing import Any

import numpy as np
import shapely


def ClipTo2Pi(rad: float) -> float:  # noqa: N802
    """Clip an angular span to [0, 2π].

    Unlike ``WrapTo2Pi``, a full circle stays ``2π`` instead of wrapping to 0.
    Non-finite input yields 0.0.
    """
    if not np.isfinite(rad):
        return 0.0
    return float(np.clip(rad, 0.0, 2 * pi))


def geometry_transform(geometry: Any, state: np.ndarray) -> Any:
    """Rigidly transform a Shapely geometry by *state* = [x, y, θ].

    Rotates by θ then translates by (x, y).
    """
    values = np.asarray(state, dtype=float).reshape(-1)
    x, y = values[0], values[1]
    theta = values[2] if values.size >= 3 else 0.0
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    def rotate_translate(coords: np.ndarray) -> np.ndarray:
        px, py = coords[:, 0], coords[:, 1]
        return np.column_stack(
            (px * cos_t - py * sin_t + x, px * sin_t + py * cos_t + y)
        )

    with np.errstate(all="ignore"):
        return shapely.transform(geometry, rotate_translate)


def _wrap_to_pi(rad: float) -> float:
    """Wrap angle to (−π, π]."""
    return float(np.arctan2(np.sin(rad), np.cos(rad)))


def _get_transform(
    state: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (translation, rotation_matrix) from a [x, y, θ] state."""
    s = np.asarray(state, dtype=float).reshape(-1)
    x, y = float(s[0]), float(s[1])
    theta = float(s[2]) if s.size >= 3 else 0.0
    trans = np.array([[x], [y]])
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return trans, rot


def transform_point_with_state(point: np.ndarray, state: np.ndarray) -> np.ndarray:
    """Transform a [x, y, θ] point by a [x, y, θ] state.

    Returns a (3, 1) array.
    """
    trans, rot = _get_transform(state)
    new_position = (
        rot @ np.asarray(point, dtype=float).reshape(-1)[:2].reshape(2, 1) + trans
    )
    new_theta = _wrap_to_pi(float(point[2]) + float(state[2]))
    return np.array([new_position[0, 0], new_position[1, 0], new_theta]).reshape((3, 1))
