"""Platform-specific constants for POSIX shm_open."""

import sys

if sys.platform == "win32":
    # POSIX shm_open is not available on Windows; provide sentinel values.
    O_RDWR = 0x0002
    O_CREAT = 0x0040
    O_EXCL = 0x0080
elif sys.platform == "darwin":
    O_RDWR = 0x0002
    O_CREAT = 0x0200
    O_EXCL = 0x0800
elif sys.platform.startswith("linux"):
    O_RDWR = 0o2
    O_CREAT = 0o100
    O_EXCL = 0o200
else:
    # FreeBSD / other POSIX — use fcntl values at runtime
    import fcntl

    O_RDWR = getattr(fcntl, "O_RDWR", 0o2)
    O_CREAT = getattr(fcntl, "O_CREAT", 0o100)
    O_EXCL = getattr(fcntl, "O_EXCL", 0o200)
