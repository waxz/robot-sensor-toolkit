"""Standalone random-number utilities.

Provides a module-level ``rng`` proxy and a ``_generator()`` callable that
mirror the interface used inside ir-sim, so sensor modules can be ported
without change to their hot paths.

The generator is a plain ``numpy.random.default_rng()`` singleton.  Call
``set_seed(seed)`` to make sampling reproducible.
"""

from __future__ import annotations

import numpy as np

_default: np.random.Generator = np.random.default_rng()


def _generator() -> np.random.Generator:
    """Return the active random generator."""
    return _default


class _RNGProxy:
    """Proxy over ``_generator()`` so ``rng.uniform(...)`` works at module level."""

    def __getattr__(self, name: str):
        return getattr(_generator(), name)


rng: _RNGProxy = _RNGProxy()


def set_seed(seed: int | None = None) -> None:
    """Reseed the shared generator.

    Args:
        seed: Integer seed for deterministic sampling.  ``None`` creates a
            fresh non-deterministic generator.
    """
    global _default
    _default = np.random.default_rng(seed)
