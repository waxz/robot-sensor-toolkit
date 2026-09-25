"""Workspace build script to build and install all packages locally, editable.

Delegates build configuration to each package's pyproject.toml / setup.py:
1. shmbridge: C++ extension via pyproject.toml (scikit-build-core)
2. irsim_devices: optional C++ extension (lidar_embree) via pyproject.toml + setup.py
   (setuptools + pybind11; downloads the Embree4 SDK automatically if missing)
3. urdf_tools: pure-Python CLI + library (hatchling)

Usage:
    python build_extensions.py
    python build_extensions.py --target shmbridge
    python build_extensions.py --target irsim_devices
    python build_extensions.py --target urdf_tools
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent


def _run_install(pkg_dir: Path, description: str) -> bool:
    print("\n" + "=" * 65)
    print(f" Building and installing {description}...")
    print("=" * 65)
    if shutil.which("uv"):
        cmd = ["uv", "pip", "install", "-e", str(pkg_dir)]
    else:
        cmd = [sys.executable, "-m", "pip", "install", "-e", str(pkg_dir)]

    res = subprocess.run(cmd, cwd=WORKSPACE_ROOT)
    if res.returncode == 0:
        print(f"\n[SUCCESS] {description} built and installed successfully.")
        return True
    print(f"\n[WARNING] Failed to build/install {description}.", file=sys.stderr)
    return False


def build_shmbridge() -> bool:
    return _run_install(WORKSPACE_ROOT / "shmbridge", "shmbridge")


def build_irsim_devices() -> bool:
    return _run_install(WORKSPACE_ROOT / "irsim_devices", "irsim_devices (lidar_embree)")


def build_urdf_tools() -> bool:
    return _run_install(WORKSPACE_ROOT / "urdf_tools", "urdf_tools")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build and install all robot-sensor-toolkit packages."
    )
    parser.add_argument(
        "--target",
        choices=["all", "shmbridge", "irsim_devices", "urdf_tools"],
        default="all",
        help="Target package to build (default: all)",
    )
    args = parser.parse_args()

    success = True
    if args.target in ("all", "shmbridge"):
        success = build_shmbridge() and success

    if args.target in ("all", "irsim_devices"):
        success = build_irsim_devices() and success

    if args.target in ("all", "urdf_tools"):
        success = build_urdf_tools() and success

    if success:
        print("\n" + "=" * 65)
        print(" [FINISHED] All requested packages built and installed!")
        print("=" * 65 + "\n")
    else:
        print(
            "\n[FINISHED] Some build steps encountered warnings/errors.",
            file=sys.stderr,
        )


if __name__ == "__main__":
    main()
