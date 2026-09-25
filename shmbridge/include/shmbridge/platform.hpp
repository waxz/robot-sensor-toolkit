/*
 * shmbridge/platform.hpp — thin cross-platform shm/time/notify abstraction.
 *
 * Supported: Linux, macOS, Windows 10+ (WaitOnAddress / WakeByAddressAll).
 *
 * API surface:
 *   platform::shm_create(name, size)                  → void* (throws on error)
 *   platform::shm_attach(name, size)                  → void* (throws on error)
 *   platform::shm_unmap(ptr, size)                    → void
 *   platform::shm_destroy(name)                       → void (no-op on Windows)
 *   platform::mem_lock(ptr, size)                     → void (best-effort)
 *   platform::now_ns()                                → uint64_t monotonic ns
 *   platform::sleep_ns(ns)                            → void
 *   platform::notify_wake_all(word)                   → void
 *   platform::notify_wait(word, expected, timeout_ms) → void
 *   platform::shm_os_name(name)                       → std::string
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <synchapi.h>   /* WaitOnAddress, WakeByAddressAll */
#  include <mutex>
#  include <unordered_map>
#else
#  include <climits>   /* LONG_MAX */
#  include <fcntl.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <time.h>
#  include <unistd.h>
#  if defined(__linux__)
#    include <climits>
#    include <linux/futex.h>
#    include <syscall.h>
#  endif
#endif

namespace shmbridge {
namespace platform {

/* ── OS name conversion ──────────────────────────────────────────────────── */

/*
 * POSIX requires names like "/mybridge".
 * Windows CreateFileMappingA uses a plain name with no leading '/'.
 */
inline std::string shm_os_name(const std::string& name) {
#if defined(_WIN32)
    if (!name.empty() && name[0] == '/') return name.substr(1);
    return name;
#else
    return name;
#endif
}

/* ── Windows handle registry ─────────────────────────────────────────────── */
/*
 * MapViewOfFile returns a void* but the HANDLE for the file mapping must also
 * stay alive. Track it keyed by view pointer so shm_unmap can close it with
 * the same void* interface that POSIX munmap uses.
 */
#if defined(_WIN32)
namespace detail {
struct HandleRegistry {
    std::mutex                        mtx;
    std::unordered_map<void*, HANDLE> handles;

    static HandleRegistry& instance() {
        static HandleRegistry inst;
        return inst;
    }
    void insert(void* ptr, HANDLE h) {
        std::lock_guard<std::mutex> lk(mtx);
        handles[ptr] = h;
    }
    HANDLE remove(void* ptr) {
        std::lock_guard<std::mutex> lk(mtx);
        auto it = handles.find(ptr);
        if (it == handles.end()) return INVALID_HANDLE_VALUE;
        HANDLE h = it->second;
        handles.erase(it);
        return h;
    }
};
} /* namespace detail */
#endif /* _WIN32 */

/* ── shm_create ──────────────────────────────────────────────────────────── */

/*
 * Create a named shared-memory segment, zero it, return a read-write pointer.
 * On POSIX, an existing stale segment with the same name is unlinked first.
 * Throws std::system_error on failure.
 */
inline void* shm_create(const std::string& name, std::size_t size) {
    std::string os = shm_os_name(name);
#if defined(_WIN32)
    HANDLE h = CreateFileMappingA(
        INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
        static_cast<DWORD>(size >> 32),
        static_cast<DWORD>(size & 0xFFFFFFFFu),
        os.c_str());
    if (!h)
        throw std::system_error(static_cast<int>(GetLastError()),
                                std::system_category(),
                                "CreateFileMappingA " + name);
    void* ptr = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (!ptr) {
        DWORD e = GetLastError(); CloseHandle(h);
        throw std::system_error(static_cast<int>(e), std::system_category(),
                                "MapViewOfFile " + name);
    }
    std::memset(ptr, 0, size);
    detail::HandleRegistry::instance().insert(ptr, h);
    return ptr;
#else
    ::shm_unlink(os.c_str());
    int fd = ::shm_open(os.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(),
                                "shm_open " + name);
    if (::ftruncate(fd, static_cast<off_t>(size)) != 0) {
        int e = errno; ::close(fd);
        throw std::system_error(e, std::generic_category(), "ftruncate");
    }
    void* ptr = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (ptr == MAP_FAILED)
        throw std::system_error(errno, std::generic_category(), "mmap");
    std::memset(ptr, 0, size);
    return ptr;
#endif
}

/* ── shm_attach ──────────────────────────────────────────────────────────── */

/*
 * Attach to an existing named segment.  `size` may be smaller than the real
 * segment size (used for the 2-step probe: attach 4096, read header, unmap,
 * attach full size).  Each call is independent and returns its own view.
 * Throws std::system_error if the segment does not exist.
 */
inline void* shm_attach(const std::string& name, std::size_t size) {
    std::string os = shm_os_name(name);
#if defined(_WIN32)
    HANDLE h = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, os.c_str());
    if (!h)
        throw std::system_error(static_cast<int>(GetLastError()),
                                std::system_category(),
                                "OpenFileMappingA " + name);
    void* ptr = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (!ptr) {
        DWORD e = GetLastError(); CloseHandle(h);
        throw std::system_error(static_cast<int>(e), std::system_category(),
                                "MapViewOfFile " + name);
    }
    detail::HandleRegistry::instance().insert(ptr, h);
    return ptr;
#else
    int fd = ::shm_open(os.c_str(), O_RDWR, 0666);
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(),
                                "shm_open " + name);
    void* ptr = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (ptr == MAP_FAILED)
        throw std::system_error(errno, std::generic_category(), "mmap");
    return ptr;
#endif
}

/* ── shm_unmap ───────────────────────────────────────────────────────────── */

inline void shm_unmap(void* ptr, std::size_t size) noexcept {
    if (!ptr) return;
#if defined(_WIN32)
    UnmapViewOfFile(ptr);
    HANDLE h = detail::HandleRegistry::instance().remove(ptr);
    if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
#else
    ::munmap(ptr, size);
#endif
}

/* ── shm_destroy ─────────────────────────────────────────────────────────── */

/*
 * On POSIX, unlinks the named shm object so it is removed once all mappings
 * are closed.  On Windows this is a no-op: the kernel auto-deletes the file
 * mapping object when its last handle is closed via shm_unmap.
 */
inline void shm_destroy(const std::string& name) noexcept {
#if !defined(_WIN32)
    ::shm_unlink(shm_os_name(name).c_str());
#else
    (void)name;
#endif
}

/* ── mem_lock ────────────────────────────────────────────────────────────── */

/* Pin pages in RAM to avoid page-fault latency on first access. Best-effort. */
inline void mem_lock(void* ptr, std::size_t size) noexcept {
    if (!ptr) return;
#if defined(__linux__)
    ::mlock(ptr, size);
#elif defined(_WIN32)
    VirtualLock(ptr, size);
#else
    (void)ptr; (void)size;
#endif
}

/* ── now_ns ──────────────────────────────────────────────────────────────── */

inline uint64_t now_ns() noexcept {
#if defined(_WIN32)
    LARGE_INTEGER cnt, freq;
    QueryPerformanceCounter(&cnt);
    QueryPerformanceFrequency(&freq);
    uint64_t c = static_cast<uint64_t>(cnt.QuadPart);
    uint64_t f = static_cast<uint64_t>(freq.QuadPart);
    return (c / f) * 1'000'000'000ULL + (c % f) * 1'000'000'000ULL / f;
#else
    struct timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec)  * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
#endif
}

/* ── sleep_ns ────────────────────────────────────────────────────────────── */

inline void sleep_ns(int64_t ns) noexcept {
    if (ns <= 0) return;
#if defined(_WIN32)
    /* Windows Sleep has ~1 ms granularity; round up to at least 1 ms. */
    DWORD ms = static_cast<DWORD>((ns + 999'999LL) / 1'000'000LL);
    if (ms == 0) ms = 1;
    Sleep(ms);
#else
    struct timespec req{
        static_cast<time_t>(ns / 1'000'000'000LL),
        static_cast<long>(ns % 1'000'000'000LL)
    };
    ::nanosleep(&req, nullptr);
#endif
}

/* ── notify_wake_all ─────────────────────────────────────────────────────── */

/*
 * Atomically increment `*word`, then broadcast to all waiters.
 * `word` must point into a shared-memory region visible to all processes and
 * must be naturally aligned (guaranteed by all structs in shmbridge).
 */
inline void notify_wake_all(volatile uint32_t* word) noexcept {
    if (!word) return;
    auto* a = reinterpret_cast<std::atomic<uint32_t>*>(
        const_cast<uint32_t*>(word));
    a->fetch_add(1u, std::memory_order_release);
#if defined(_WIN32)
    WakeByAddressAll(const_cast<uint32_t*>(word));
#elif defined(__linux__)
    ::syscall(SYS_futex, const_cast<uint32_t*>(word),
              FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
#endif
    /* macOS: the atomic increment alone is visible to pollers. */
}

/* ── notify_wait ─────────────────────────────────────────────────────────── */

/*
 * Sleep until `*word != expected` or `timeout_ms` elapses.
 * `timeout_ms < 0` means wait indefinitely.
 */
inline void notify_wait(volatile uint32_t* word, uint32_t expected,
                        int timeout_ms) noexcept {
    if (!word) return;
#if defined(_WIN32)
    DWORD dw = (timeout_ms < 0) ? INFINITE : static_cast<DWORD>(timeout_ms);
    WaitOnAddress(const_cast<uint32_t*>(word), &expected, sizeof(uint32_t), dw);
#elif defined(__linux__)
    if (timeout_ms < 0) {
        ::syscall(SYS_futex, const_cast<uint32_t*>(word),
                  FUTEX_WAIT, expected, nullptr, nullptr, 0);
    } else {
        struct timespec ts{
            static_cast<time_t>(timeout_ms / 1000),
            static_cast<long>(timeout_ms % 1000) * 1'000'000L
        };
        ::syscall(SYS_futex, const_cast<uint32_t*>(word),
                  FUTEX_WAIT, expected, &ts, nullptr, 0);
    }
#else
    /* macOS / generic: poll with 1 ms sleep */
    long waited = 0;
    long limit  = (timeout_ms < 0) ? LONG_MAX
                                   : static_cast<long>(timeout_ms) * 1'000'000L;
    const auto* a = reinterpret_cast<const std::atomic<uint32_t>*>(
        const_cast<const uint32_t*>(word));
    while (a->load(std::memory_order_acquire) == expected && waited < limit) {
        struct timespec sl{0, 1'000'000L};
        ::nanosleep(&sl, nullptr);
        waited += 1'000'000L;
    }
#endif
}

} /* namespace platform */
} /* namespace shmbridge */
