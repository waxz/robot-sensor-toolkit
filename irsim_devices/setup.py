"""Build script for irsim_devices C++ extensions (lidar_embree).

Compiles ``lidar_embree.cpp`` using pybind11 and Intel Embree4 SDK without requiring CMake.
The extension is optional and, by default, its ~35 MB SDK is NEVER downloaded
automatically -- a plain ``pip install -e .`` stays fast. Opt in with one of:

    IRSIM_DEVICES_BUILD_EMBREE=1 pip install -e ".[embree]"
    EMBREE_ROOT=/path/to/embree4 pip install -e ".[embree]"

If the SDK is already cached locally (``cpp/embree-sdk``, from a previous
opted-in build) it is reused automatically without re-downloading.
If Embree4 cannot be found/downloaded/compiled, setup completes cleanly.
"""

from __future__ import annotations

import io
import os
import shutil
import sys
import time
import urllib.request
import zipfile
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

HERE = Path(__file__).resolve().parent
CPP_DIR = HERE / "cpp"
LOCAL_SDK = CPP_DIR / "embree-sdk"

# Network download is opt-in: a bare `pip install -e .` must stay fast and
# never block on a ~35 MB fetch nobody asked for. Local caches / EMBREE_ROOT
# are still honoured unconditionally since they cost nothing to check.
_DOWNLOAD_OPT_IN = os.environ.get("IRSIM_DEVICES_BUILD_EMBREE") == "1"
_DOWNLOAD_TIMEOUT_S = 30


def _find_local_embree_sdk(embree_root_arg: str | None = None) -> Path | None:
    """Locate an already-available Embree 4 SDK (no network access)."""
    candidate = embree_root_arg or os.environ.get("EMBREE_ROOT")
    if candidate:
        p = Path(candidate)
        if (p / "include" / "embree4" / "rtcore.h").exists():
            return p
        print(
            f"[WARNING] EMBREE_ROOT set to {p} but include/embree4/rtcore.h not found.",
            flush=True,
        )

    if (LOCAL_SDK / "include" / "embree4" / "rtcore.h").exists():
        return LOCAL_SDK

    if sys.platform != "win32":
        for sys_path in (Path("/usr"), Path("/usr/local"), Path("/opt/homebrew")):
            if (sys_path / "include" / "embree4" / "rtcore.h").exists():
                return sys_path

    return None


def _download_embree_sdk() -> Path | None:
    """Download and unpack the Embree 4 SDK. Network access; opt-in only."""
    if sys.platform == "win32":
        url = "https://github.com/RenderKit/embree/releases/download/v4.3.3/embree-4.3.3.x64.windows.zip"
    elif sys.platform == "darwin":
        url = "https://github.com/RenderKit/embree/releases/download/v4.3.3/embree-4.3.3.x86_64.macosx.tar.gz"
    else:
        url = "https://github.com/RenderKit/embree/releases/download/v4.3.3/embree-4.3.3.x86_64.linux.tar.gz"

    try:
        print(f"[irsim_devices] Downloading Embree 4 SDK from {url} ...", flush=True)
        t0 = time.monotonic()
        req = urllib.request.urlopen(url, timeout=_DOWNLOAD_TIMEOUT_S)
        total = req.length or 0
        chunks: list[bytes] = []
        read = 0
        while True:
            chunk = req.read(1 << 20)  # 1 MiB per read -- keeps stdout live
            if not chunk:
                break
            chunks.append(chunk)
            read += len(chunk)
            pct = f"{100 * read // total}%" if total else f"{read // 1_000_000}MB"
            print(f"  ... {pct} ({read // 1_000_000} MB)", flush=True)
        data = b"".join(chunks)
        print(
            f"[irsim_devices] Download complete in {time.monotonic() - t0:.1f}s",
            flush=True,
        )

        temp_extract = CPP_DIR / "temp_embree"
        if temp_extract.exists():
            shutil.rmtree(temp_extract)

        if url.endswith(".zip"):
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                z.extractall(temp_extract)
        else:
            import tarfile

            with tarfile.open(fileobj=io.BytesIO(data), mode="r:*") as t:
                t.extractall(temp_extract)

        if (temp_extract / "include" / "embree4" / "rtcore.h").exists():
            if LOCAL_SDK.exists():
                shutil.rmtree(LOCAL_SDK)
            temp_extract.rename(LOCAL_SDK)
        else:
            extracted_dirs = list(temp_extract.glob("embree-*"))
            if not extracted_dirs:
                print(
                    "[ERROR] Failed to find extracted embree folder.",
                    file=sys.stderr,
                    flush=True,
                )
                return None
            if LOCAL_SDK.exists():
                shutil.rmtree(LOCAL_SDK)
            extracted_dirs[0].rename(LOCAL_SDK)
            shutil.rmtree(temp_extract, ignore_errors=True)

        print(
            f"[SUCCESS] Embree 4 SDK downloaded and unpacked to: {LOCAL_SDK}",
            flush=True,
        )
        return LOCAL_SDK
    except (TimeoutError, OSError) as exc:
        print(
            f"[ERROR] Failed to download Embree 4 SDK ({exc}). "
            "Set EMBREE_ROOT to a local install to skip the download.",
            file=sys.stderr,
            flush=True,
        )
        return None


def ensure_embree_sdk(embree_root_arg: str | None = None) -> Path | None:
    """Locate the Embree 4 SDK, downloading it only if explicitly opted in.

    A local cache (``cpp/embree-sdk``) or ``EMBREE_ROOT`` is always honoured
    since checking for those is instant. A fresh network download only
    happens when ``IRSIM_DEVICES_BUILD_EMBREE=1`` is set -- otherwise this
    returns ``None`` immediately so a plain ``pip install -e .`` never
    blocks on it.
    """
    found = _find_local_embree_sdk(embree_root_arg)
    if found is not None:
        return found

    if not _DOWNLOAD_OPT_IN:
        print(
            "[irsim_devices] Embree 4 SDK not found locally; skipping the optional "
            "lidar_embree extension (core package is unaffected). Set "
            "IRSIM_DEVICES_BUILD_EMBREE=1 to download it (~35 MB) and build it.",
            flush=True,
        )
        return None

    return _download_embree_sdk()


class EmbreeBuildExt(build_ext):
    """Custom build_ext for lidar_embree using pybind11 & Embree 4."""

    #: Populated by build_extension(); reported by the final summary in run().
    embree_status: str = "not attempted"

    def build_extension(self, ext: Extension) -> None:
        if ext.name == "irsim_devices.lidar_embree":
            embree_root = ensure_embree_sdk()
            if not embree_root:
                print(
                    "[WARNING] Skipping lidar_embree compilation (Embree SDK unavailable).",
                    file=sys.stderr,
                )
                self.embree_status = "skipped (Embree SDK unavailable)"
                return

            try:
                import pybind11
            except ImportError:
                print(
                    "[WARNING] Skipping lidar_embree compilation (pybind11 missing).",
                    file=sys.stderr,
                )
                self.embree_status = "skipped (pybind11 missing)"
                return

            ext.include_dirs = [
                str(CPP_DIR),
                str(embree_root / "include"),
                pybind11.get_include(),
            ]

            lib_dirs = []
            if (embree_root / "lib").exists():
                lib_dirs.append(str(embree_root / "lib"))
            if (embree_root / "lib64").exists():
                lib_dirs.append(str(embree_root / "lib64"))
            ext.library_dirs = lib_dirs
            ext.libraries = ["embree4"]

            if sys.platform == "win32":
                ext.extra_compile_args = [
                    "/O2",
                    "/arch:AVX2",
                    "/std:c++17",
                    "/D_USE_MATH_DEFINES",
                ]
                ext.extra_link_args = []
            elif sys.platform == "darwin":
                ext.extra_compile_args = [
                    "-O3",
                    "-std=c++17",
                    "-march=native",
                    "-ffast-math",
                ]
                ext.extra_link_args = [f"-Wl,-rpath,{embree_root}/lib"]
            else:
                ext.extra_compile_args = [
                    "-O3",
                    "-std=c++17",
                    "-march=native",
                    "-ffast-math",
                ]
                ext.extra_link_args = [f"-Wl,-rpath={embree_root}/lib"]

        try:
            super().build_extension(ext)
            # Copy runtime DLLs on Windows
            if sys.platform == "win32" and ext.name == "irsim_devices.lidar_embree":
                embree_root = ensure_embree_sdk()
                if embree_root:
                    bin_dir = embree_root / "bin"
                    out_dir = Path(self.get_ext_fullpath(ext.name)).parent
                    if bin_dir.exists():
                        for dll in bin_dir.glob("*.dll"):
                            dest = out_dir / dll.name
                            if dest.resolve() != dll.resolve():
                                shutil.copy2(dll, dest)
                                print(f"Copied {dll.name} -> {out_dir}")
            if ext.name == "irsim_devices.lidar_embree":
                self.embree_status = "built"
        except Exception as exc:
            print(
                f"[WARNING] Optional extension {ext.name} failed to build: {exc}",
                file=sys.stderr,
            )
            if ext.name == "irsim_devices.lidar_embree":
                self.embree_status = f"FAILED ({exc})"
            if not getattr(ext, "optional", False):
                raise

    def run(self) -> None:
        super().run()
        _print_build_summary(self.embree_status)


def _print_build_summary(embree_status: str) -> None:
    """Verify and report what actually compiled, so silent fallbacks are visible.

    Covers the C++/SIMD extensions this package can build or load, plus the
    pure-Python fallbacks (NumPy, open3d) each one degrades to when a native
    piece is unavailable.
    """
    sys.path.insert(0, str(HERE / "src"))

    try:
        from irsim_devices.core import ray_casting_2d_omp as _omp

        _omp.ensure_built(verbose=False)
        if _omp.is_avx2_f32_available():
            omp_status = "AVX2 float32 8-wide SIMD (fastest)"
        elif _omp.is_avx2_available():
            omp_status = "AVX2 float64 4-wide SIMD"
        elif _omp.is_omp_available():
            omp_status = "scalar OpenMP (no AVX2)"
        else:
            omp_status = "unavailable -- falling back to pure NumPy"
    except Exception as exc:
        omp_status = f"could not probe ({exc})"

    try:
        import open3d  # noqa: F401

        open3d_status = "available (Lidar3D can use the Embree-via-open3d backend)"
    except ImportError:
        open3d_status = "not installed (Lidar3D falls back to the native lidar_embree extension, if built)"

    try:
        import pybind11  # noqa: F401

        pybind11_status = "available"
    except ImportError:
        pybind11_status = "not installed"

    print("")
    print("==================== irsim_devices build summary ====================")
    print(f"  lidar_embree (C++/Embree4, pybind11) : {embree_status}")
    print(f"  ray_casting_2d_omp (C/OpenMP/SIMD)    : {omp_status}")
    print(f"  pybind11                              : {pybind11_status}")
    print(f"  open3d                                : {open3d_status}")
    print("=======================================================================")
    print("")


setup(
    ext_modules=[
        Extension(
            "irsim_devices.lidar_embree",
            sources=["cpp/lidar_embree.cpp"],
            optional=True,
        )
    ],
    cmdclass={"build_ext": EmbreeBuildExt},
)
