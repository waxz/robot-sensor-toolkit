"""Optional C+OpenMP-accelerated ray-casting kernels.

Drop-in replacement for :func:`~irsim_devices.core.ray_casting_2d.cast_ray_segments`
when a C compiler with OpenMP is available.  If compilation fails the module still
imports cleanly and :func:`cast_ray_segments_omp` falls back to the NumPy kernel.

Three kernels are available, in descending preference:

* :func:`cast_ray_segments_avx2_f32` — AVX2 SIMD **8-wide float32** OpenMP (SoA);
  fastest; requires ``cast_ray_segments_avx2_f32_soa`` in the library.
* :func:`cast_ray_segments_avx2` — AVX2 SIMD 4-wide float64 OpenMP (SoA);
  available when the library exposes ``cast_ray_segments_avx2_soa``.
* :func:`cast_ray_segments_omp` — scalar OpenMP (AoS layout); always available.

Build
-----
The preferred path is the pre-compiled ``_ray_casting_omp`` extension built by
setuptools / cibuildwheel and installed alongside the package.  When the extension
is absent (e.g. an sdist install on a machine without a compiler), the first call
to :func:`ensure_built` attempts a legacy ``gcc`` compile of ``ray_casting_omp.c``
into ``ray_casting_omp.so`` in the same directory.

Usage::

    from irsim_devices.core.ray_casting_2d_omp import (
        cast_ray_segments_avx2_f32,
        cast_ray_segments_avx2,
        cast_ray_segments_omp,
        is_avx2_f32_available,
        is_avx2_available,
        is_omp_available,
        ensure_built,
    )

    if is_avx2_f32_available():
        ranges, hits = cast_ray_segments_avx2_f32(
            origin, directions, seg_start, seg_end, max_range
        )
    elif is_avx2_available():
        ranges, hits = cast_ray_segments_avx2(
            origin, directions, seg_start, seg_end, max_range
        )
    elif is_omp_available():
        ranges, hits = cast_ray_segments_omp(
            origin, directions, seg_start, seg_end, max_range
        )
"""

from __future__ import annotations

import ctypes
import subprocess
from pathlib import Path

import numpy as np

_HERE = Path(__file__).parent
# C source lives in csrc/ sibling of the package root
_C_SRC = _HERE.parents[2] / "csrc" / "ray_casting_omp.c"
# Legacy output path (runtime gcc compile); kept for backward compatibility.
_SO_OUT = _HERE / "ray_casting_omp.so"

_lib: ctypes.CDLL | None = None
_OMP_AVAILABLE: bool | None = None  # None = not yet probed
_AVX2_AVAILABLE: bool = False
_F32_AVAILABLE: bool = False


def _find_compiled_ext() -> Path | None:
    """Locate the setuptools-compiled ``_ray_casting_omp`` extension.

    Checks the installed package first (via importlib), then looks for an
    in-tree build artifact next to this file.
    """
    import importlib.util

    spec = importlib.util.find_spec("irsim_devices.core._ray_casting_omp")
    if spec is not None and spec.origin:
        return Path(spec.origin)
    # In-tree / editable install: search for the compiled file beside this module.
    for pat in ("_ray_casting_omp*.so", "_ray_casting_omp*.pyd"):
        found = list(_HERE.glob(pat))
        if found:
            return found[0]
    return None


def _try_build(force: bool = False) -> bool:
    """Compile the C source via gcc (legacy fallback).  Returns True on success."""
    if not force and _SO_OUT.exists():
        return True
    if not _C_SRC.exists():
        return False
    cmd = [
        "gcc",
        "-O3",
        "-march=native",
        "-fopenmp",
        "-shared",
        "-fPIC",
        str(_C_SRC),
        "-o",
        str(_SO_OUT),
        "-lm",
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True, timeout=60)
        return True
    except (
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
        FileNotFoundError,
    ):
        return False


def _setup_lib(lib: ctypes.CDLL) -> ctypes.CDLL:
    """Attach argtypes/restype to *lib* and return it."""
    global _AVX2_AVAILABLE, _F32_AVAILABLE
    # Thread-count control (always present in the compiled library)
    try:
        lib.set_omp_num_threads.restype = None
        lib.set_omp_num_threads.argtypes = [ctypes.c_int]
    except AttributeError:
        pass
    lib.cast_ray_segments_omp.restype = None
    lib.cast_ray_segments_omp.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # origin
        ctypes.POINTER(ctypes.c_double),  # directions
        ctypes.POINTER(ctypes.c_double),  # seg_start
        ctypes.POINTER(ctypes.c_double),  # seg_end
        ctypes.c_int,  # N
        ctypes.c_int,  # M
        ctypes.c_double,  # max_range
        ctypes.POINTER(ctypes.c_double),  # out_ranges
        ctypes.POINTER(ctypes.c_int64),  # out_hit
    ]
    # Wire up AVX2 float64 SoA kernel
    try:
        fn = lib.cast_ray_segments_avx2_soa
        fn.restype = None
        fn.argtypes = [
            ctypes.POINTER(ctypes.c_double),  # origin
            ctypes.POINTER(ctypes.c_double),  # dir_dx
            ctypes.POINTER(ctypes.c_double),  # dir_dy
            ctypes.POINTER(ctypes.c_double),  # seg_sx
            ctypes.POINTER(ctypes.c_double),  # seg_sy
            ctypes.POINTER(ctypes.c_double),  # seg_ex
            ctypes.POINTER(ctypes.c_double),  # seg_ey
            ctypes.c_int,  # N
            ctypes.c_int,  # M
            ctypes.c_double,  # max_range
            ctypes.POINTER(ctypes.c_double),  # out_ranges
            ctypes.POINTER(ctypes.c_int64),  # out_hit
        ]
        _AVX2_AVAILABLE = True
    except AttributeError:
        _AVX2_AVAILABLE = False
    # Wire up AVX2 float32 8-wide SoA kernel
    try:
        fn32 = lib.cast_ray_segments_avx2_f32_soa
        fn32.restype = None
        fn32.argtypes = [
            ctypes.POINTER(ctypes.c_float),  # origin  float[2]
            ctypes.POINTER(ctypes.c_float),  # dir_dx  float[N]
            ctypes.POINTER(ctypes.c_float),  # dir_dy  float[N]
            ctypes.POINTER(ctypes.c_float),  # seg_sx  float[M]
            ctypes.POINTER(ctypes.c_float),  # seg_sy  float[M]
            ctypes.POINTER(ctypes.c_float),  # seg_ex  float[M]
            ctypes.POINTER(ctypes.c_float),  # seg_ey  float[M]
            ctypes.c_int,  # N
            ctypes.c_int,  # M
            ctypes.c_float,  # max_range
            ctypes.POINTER(ctypes.c_float),  # out_ranges float[N]
            ctypes.POINTER(ctypes.c_int32),  # out_hit    int32[N]
        ]
        _F32_AVAILABLE = True
    except AttributeError:
        _F32_AVAILABLE = False
    return lib


def _load_lib() -> ctypes.CDLL | None:
    """Load the compiled shared library and set up argtypes.

    Priority (highest first):
    1. Runtime-compiled ``ray_casting_omp.so`` next to this file — carries the
       latest kernels including AVX2 float32.
    2. The setuptools-compiled ``_ray_casting_omp`` extension — installed by
       pip/cibuildwheel; may not expose newer kernels.
    """
    # 1) Runtime .so (may expose newer kernels than the installed extension)
    if _SO_OUT.exists():
        try:
            return _setup_lib(ctypes.CDLL(str(_SO_OUT)))
        except OSError:
            pass
    # 2) Setuptools-compiled extension (installed package fallback)
    ext_path = _find_compiled_ext()
    if ext_path is not None:
        try:
            return _setup_lib(ctypes.CDLL(str(ext_path)))
        except OSError:
            pass
    return None


def build_omp_lib(force: bool = False, verbose: bool = False) -> bool:
    """Build (or rebuild) the OpenMP shared library.

    Attempts to use the setuptools-compiled extension first.  If unavailable,
    falls back to a runtime ``gcc`` compile.

    Args:
        force: Recompile the legacy gcc artefact even if it already exists.
               Has no effect on the setuptools extension.
        verbose: Print a one-line summary of which kernel(s) ended up
            available (or the NumPy fallback).  Off by default so importing
            this module in an application stays silent; ``setup.py`` passes
            ``verbose=True`` to surface this in the build log.

    Returns:
        ``True`` if the library is available (pre-compiled or just built).
    """
    global _lib, _OMP_AVAILABLE
    # Try to load an already-available library (setuptools ext or legacy .so)
    candidate = _load_lib()
    if candidate is not None:
        _lib = candidate
        _OMP_AVAILABLE = True
    else:
        # Fall back to runtime gcc compile (Linux only; ignored on Windows/macOS)
        ok = _try_build(force=force)
        if ok:
            _lib = _load_lib()
            _OMP_AVAILABLE = _lib is not None
        else:
            _OMP_AVAILABLE = False
        if not _OMP_AVAILABLE:
            global _AVX2_AVAILABLE, _F32_AVAILABLE
            _AVX2_AVAILABLE = False
            _F32_AVAILABLE = False

    if verbose:
        if _F32_AVAILABLE:
            kernel = "AVX2 float32 8-wide SIMD (fastest)"
        elif _AVX2_AVAILABLE:
            kernel = "AVX2 float64 4-wide SIMD"
        elif _OMP_AVAILABLE:
            kernel = "scalar OpenMP (no AVX2)"
        else:
            kernel = "none — falling back to pure NumPy (no C/OpenMP/SIMD)"
        print(f"[irsim_devices] ray_casting_2d_omp kernel: {kernel}")

    return bool(_OMP_AVAILABLE)


def ensure_built(verbose: bool = False) -> bool:
    """Build if not already done.  Returns availability."""
    global _OMP_AVAILABLE
    if _OMP_AVAILABLE is None:
        build_omp_lib(verbose=verbose)
    return bool(_OMP_AVAILABLE)


def is_omp_available() -> bool:
    """Return ``True`` when the OpenMP kernel is compiled and loaded."""
    if _OMP_AVAILABLE is None:
        ensure_built()
    return bool(_OMP_AVAILABLE)


def is_avx2_available() -> bool:
    """Return ``True`` when the AVX2 float64 SoA kernel is compiled and loaded."""
    if _OMP_AVAILABLE is None:
        ensure_built()
    return _AVX2_AVAILABLE


def is_avx2_f32_available() -> bool:
    """Return ``True`` when the AVX2 float32 8-wide SoA kernel is available."""
    if _OMP_AVAILABLE is None:
        ensure_built()
    return _F32_AVAILABLE


def _np_ptr(arr: np.ndarray):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


def _i64_ptr(arr: np.ndarray):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))


def _f32_ptr(arr: np.ndarray):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


def _i32_ptr(arr: np.ndarray):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int32))


def cast_ray_segments_omp(
    origin: np.ndarray,
    directions: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray]:
    """C+OpenMP ray-segment intersection -- same interface as :func:`cast_ray_segments`.

    Falls back to the NumPy kernel when the C library is unavailable.

    Args:
        origin: Shared ray origin ``(2,)``.
        directions: Unit ray directions ``(N, 2)``.
        seg_start: Segment start points ``(M, 2)``.
        seg_end: Segment end points ``(M, 2)``.
        max_range: Maximum ray length; misses return this.

    Returns:
        ``(ranges, hit_index)`` -- same semantics as
        :func:`~irsim_devices.core.ray_casting_2d.cast_ray_segments`.
    """
    if not is_omp_available():
        from irsim_devices.core.ray_casting_2d import cast_ray_segments

        return cast_ray_segments(origin, directions, seg_start, seg_end, max_range)

    if len(seg_start) == 0:
        return (
            np.full(len(directions), max_range, dtype=np.float64),
            np.full(len(directions), -1, dtype=np.int64),
        )

    N = len(directions)
    M = len(seg_start)

    origin_c = np.ascontiguousarray(origin, dtype=np.float64)
    directions_c = np.ascontiguousarray(directions, dtype=np.float64)
    seg_start_c = np.ascontiguousarray(seg_start, dtype=np.float64)
    seg_end_c = np.ascontiguousarray(seg_end, dtype=np.float64)

    out_ranges = np.empty(N, dtype=np.float64)
    out_hit = np.empty(N, dtype=np.int64)

    _lib.cast_ray_segments_omp(
        _np_ptr(origin_c),
        _np_ptr(directions_c),
        _np_ptr(seg_start_c),
        _np_ptr(seg_end_c),
        ctypes.c_int(N),
        ctypes.c_int(M),
        ctypes.c_double(float(max_range)),
        _np_ptr(out_ranges),
        _i64_ptr(out_hit),
    )
    return out_ranges, out_hit.astype(int)


def cast_ray_segments_avx2(
    origin: np.ndarray,
    directions: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray]:
    """C+OpenMP+AVX2 SoA ray-segment intersection.

    Processes 4 beams simultaneously using 256-bit double-precision registers.
    Converts AoS ``directions`` / ``seg_start`` / ``seg_end`` to SoA layout
    internally before calling the C kernel.

    Falls back to :func:`cast_ray_segments_omp` when the AVX2 kernel is
    unavailable.

    Args:
        origin: Shared ray origin ``(2,)``.
        directions: Unit ray directions ``(N, 2)``.
        seg_start: Segment start points ``(M, 2)``.
        seg_end: Segment end points ``(M, 2)``.
        max_range: Maximum ray length; misses return this.

    Returns:
        ``(ranges, hit_index)`` with the same semantics as
        :func:`~irsim_devices.core.ray_casting_2d.cast_ray_segments`.
    """
    if not _AVX2_AVAILABLE or _lib is None:
        return cast_ray_segments_omp(origin, directions, seg_start, seg_end, max_range)

    if len(seg_start) == 0:
        return (
            np.full(len(directions), max_range, dtype=np.float64),
            np.full(len(directions), -1, dtype=np.int64),
        )

    N = len(directions)
    M = len(seg_start)

    origin_c = np.ascontiguousarray(origin, dtype=np.float64)
    directions_c = np.ascontiguousarray(directions, dtype=np.float64)
    seg_start_c = np.ascontiguousarray(seg_start, dtype=np.float64)
    seg_end_c = np.ascontiguousarray(seg_end, dtype=np.float64)

    # AoS → SoA conversion
    dir_dx = np.ascontiguousarray(directions_c[:, 0])
    dir_dy = np.ascontiguousarray(directions_c[:, 1])
    seg_sx = np.ascontiguousarray(seg_start_c[:, 0])
    seg_sy = np.ascontiguousarray(seg_start_c[:, 1])
    seg_ex = np.ascontiguousarray(seg_end_c[:, 0])
    seg_ey = np.ascontiguousarray(seg_end_c[:, 1])

    out_ranges = np.empty(N, dtype=np.float64)
    out_hit = np.empty(N, dtype=np.int64)

    _lib.cast_ray_segments_avx2_soa(
        _np_ptr(origin_c),
        _np_ptr(dir_dx),
        _np_ptr(dir_dy),
        _np_ptr(seg_sx),
        _np_ptr(seg_sy),
        _np_ptr(seg_ex),
        _np_ptr(seg_ey),
        ctypes.c_int(N),
        ctypes.c_int(M),
        ctypes.c_double(float(max_range)),
        _np_ptr(out_ranges),
        _i64_ptr(out_hit),
    )
    return out_ranges, out_hit.astype(int)


def set_omp_threads(n: int) -> None:
    """Set the number of OpenMP threads used by all subsequent kernel calls.

    Call with ``n=2`` (or the number of cores you can spare) before starting a
    real-time 30 Hz loop so the raycaster leaves enough cores free for the
    robot stack running in the same process.

    Args:
        n: Number of OpenMP threads to use (≥ 1).
    """
    if _OMP_AVAILABLE is None:
        ensure_built()
    if _lib is not None and hasattr(_lib, "set_omp_num_threads"):
        _lib.set_omp_num_threads(ctypes.c_int(max(1, int(n))))


def cast_ray_segments_avx2_f32_inplace(
    origin_f: np.ndarray,
    dir_dx_f: np.ndarray,
    dir_dy_f: np.ndarray,
    seg_sx_f: np.ndarray,
    seg_sy_f: np.ndarray,
    seg_ex_f: np.ndarray,
    seg_ey_f: np.ndarray,
    max_range_f: float,
    out_ranges_f: np.ndarray,
    out_hit_i: np.ndarray,
) -> None:
    """Zero-allocation AVX2 float32 kernel call; writes results in-place.

    All arrays must already be contiguous float32 (``dir_*``, ``seg_*``,
    ``out_ranges_f``, ``origin_f``) or int32 (``out_hit_i``).  No copies are
    made, so the caller is responsible for pre-allocating and reusing them.

    Args:
        origin_f: Ray origin ``(2,)`` float32.
        dir_dx_f: Pre-allocated beam x-directions ``(N,)`` float32 SoA.
        dir_dy_f: Pre-allocated beam y-directions ``(N,)`` float32 SoA.
        seg_sx_f: Segment start x ``(M,)`` float32 SoA.
        seg_sy_f: Segment start y ``(M,)`` float32 SoA.
        seg_ex_f: Segment end x ``(M,)`` float32 SoA.
        seg_ey_f: Segment end y ``(M,)`` float32 SoA.
        max_range_f: Miss distance.
        out_ranges_f: Pre-allocated output ``(N,)`` float32 — filled in place.
        out_hit_i: Pre-allocated output ``(N,)`` int32 — filled in place.

    Raises:
        RuntimeError: When the AVX2 float32 kernel is not compiled in.
    """
    if not _F32_AVAILABLE or _lib is None:
        raise RuntimeError(
            "AVX2 f32 kernel unavailable; call is_avx2_f32_available() to check."
        )
    _lib.cast_ray_segments_avx2_f32_soa(
        _f32_ptr(origin_f),
        _f32_ptr(dir_dx_f),
        _f32_ptr(dir_dy_f),
        _f32_ptr(seg_sx_f),
        _f32_ptr(seg_sy_f),
        _f32_ptr(seg_ex_f),
        _f32_ptr(seg_ey_f),
        ctypes.c_int(len(dir_dx_f)),
        ctypes.c_int(len(seg_sx_f)),
        ctypes.c_float(float(max_range_f)),
        _f32_ptr(out_ranges_f),
        _i32_ptr(out_hit_i),
    )


def cast_ray_segments_avx2_f32(
    origin: np.ndarray,
    directions: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray]:
    """C+OpenMP+AVX2 float32 8-wide SoA ray-segment intersection.

    Processes 8 beams simultaneously using 256-bit single-precision registers,
    giving 2× SIMD throughput over the float64 4-wide kernel.  Absolute range
    error is ≤5 µm at 40 m (float32 mantissa precision), well below LiDAR noise.

    Falls back to :func:`cast_ray_segments_avx2` when the f32 kernel is
    unavailable.

    Args:
        origin: Shared ray origin ``(2,)``.
        directions: Unit ray directions ``(N, 2)``.
        seg_start: Segment start points ``(M, 2)``.
        seg_end: Segment end points ``(M, 2)``.
        max_range: Maximum ray length; misses return this.

    Returns:
        ``(ranges, hit_index)`` — float64 ranges and int64 hit indices with the
        same semantics as
        :func:`~irsim_devices.core.ray_casting_2d.cast_ray_segments`.
    """
    if not _F32_AVAILABLE or _lib is None:
        return cast_ray_segments_avx2(origin, directions, seg_start, seg_end, max_range)

    if len(seg_start) == 0:
        return (
            np.full(len(directions), max_range, dtype=np.float64),
            np.full(len(directions), -1, dtype=np.int64),
        )

    N = len(directions)
    M = len(seg_start)

    origin_f = np.ascontiguousarray(origin, dtype=np.float32)
    directions_f = np.ascontiguousarray(directions, dtype=np.float32)
    seg_start_f = np.ascontiguousarray(seg_start, dtype=np.float32)
    seg_end_f = np.ascontiguousarray(seg_end, dtype=np.float32)

    # AoS → SoA (float32)
    dir_dx = np.ascontiguousarray(directions_f[:, 0])
    dir_dy = np.ascontiguousarray(directions_f[:, 1])
    seg_sx = np.ascontiguousarray(seg_start_f[:, 0])
    seg_sy = np.ascontiguousarray(seg_start_f[:, 1])
    seg_ex = np.ascontiguousarray(seg_end_f[:, 0])
    seg_ey = np.ascontiguousarray(seg_end_f[:, 1])

    out_ranges = np.empty(N, dtype=np.float32)
    out_hit = np.empty(N, dtype=np.int32)

    _lib.cast_ray_segments_avx2_f32_soa(
        _f32_ptr(origin_f),
        _f32_ptr(dir_dx),
        _f32_ptr(dir_dy),
        _f32_ptr(seg_sx),
        _f32_ptr(seg_sy),
        _f32_ptr(seg_ex),
        _f32_ptr(seg_ey),
        ctypes.c_int(N),
        ctypes.c_int(M),
        ctypes.c_float(float(max_range)),
        _f32_ptr(out_ranges),
        _i32_ptr(out_hit),
    )
    return out_ranges.astype(np.float64), out_hit.astype(np.int64)
