"""irsim_devices — standalone sensors and actuators extracted from ir-sim."""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("irsim-devices")
except PackageNotFoundError:
    __version__ = "0.0.0.dev"

__all__ = ["__version__"]
