"""Thin ctypes wrapper around the libc POSIX shm/fd calls."""

import ctypes
import sys

if sys.platform == "win32":
    # POSIX shm_open/ftruncate/mlock are not available on Windows.
    # Provide sentinels so bridge.py can be imported for struct-layout
    # purposes without crashing; actual SHM operations will fail at runtime.
    _libc = None  # type: ignore[assignment]
    _libshm = None  # type: ignore[assignment]

    import time as _time

    def _monotonic_ns() -> int:
        return _time.monotonic_ns()

else:
    _libc = ctypes.CDLL(None, use_errno=True)

    # shm_open/shm_unlink live in librt on glibc < 2.17; on glibc >= 2.17
    # they moved to libc, but manylinux CPython builds may not have librt in
    # their link map, so CDLL(None) won't expose them.  Probe and fall back.
    try:
        _libc.shm_open  # raises AttributeError when symbol is absent
        _libshm = _libc
    except AttributeError:
        import ctypes.util as _cu
        _rt = _cu.find_library("rt") or "librt.so.1"
        _libshm = ctypes.CDLL(_rt, use_errno=True)

    _libshm.shm_open.restype = ctypes.c_int
    _libshm.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]

    _libshm.shm_unlink.restype = ctypes.c_int
    _libshm.shm_unlink.argtypes = [ctypes.c_char_p]

    _libc.ftruncate.restype = ctypes.c_int
    _libc.ftruncate.argtypes = [ctypes.c_int, ctypes.c_long]

    # mlock / munlock - pin pages in RAM to eliminate page-fault latency on first access
    _libc.mlock.restype = ctypes.c_int
    _libc.mlock.argtypes = [ctypes.c_void_p, ctypes.c_size_t]

    _libc.munlock.restype = ctypes.c_int
    _libc.munlock.argtypes = [ctypes.c_void_p, ctypes.c_size_t]

    # clock_gettime is needed for liveness timestamps (monotonic ns)
    if sys.platform.startswith("linux"):
        CLOCK_MONOTONIC = 1

        class _Timespec(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

        _libc.clock_gettime.restype = ctypes.c_int
        _libc.clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(_Timespec)]

        def _monotonic_ns() -> int:
            ts = _Timespec()
            _libc.clock_gettime(CLOCK_MONOTONIC, ctypes.byref(ts))
            return ts.tv_sec * 1_000_000_000 + ts.tv_nsec

    else:
        import time as _time

        def _monotonic_ns() -> int:
            return int(_time.monotonic_ns())
