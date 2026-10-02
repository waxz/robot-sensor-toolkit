/*
 * shmbridge/platform.hpp — thin cross-platform shm/time/notify abstraction.
 *
 * Supported: Linux, macOS, Windows 10+ (WaitOnAddress / WakeByAddressAll).
 *
 * API surface:
 *   platform::shm_create(name, size)                  → void* (throws on error)
 *   platform::shm_attach(name, size)                  → void* (throws on error)
 *   platform::shm_open_or_create(name, size)          → void* (throws on error)
 *   platform::shm_unmap(ptr, size)                    → void
 *   platform::shm_destroy(name)                       → void (no-op on Windows)
 *   platform::mem_lock(ptr, size)                     → void (best-effort)
 *   platform::now_ns()                                → uint64_t monotonic ns
 *   platform::sleep_ns(ns)                            → void
 *   platform::notify_bind(name, word)                 → void (Windows only; no-op elsewhere)
 *   platform::notify_unbind(word)                     → void (Windows only; no-op elsewhere)
 *   platform::notify_wake_all(word)                   → void
 *   platform::notify_wait(word, expected, timeout_ms) → void
 *   platform::shm_os_name(name)                       → std::string
 *   platform::current_pid()                           → uint32_t
 *   platform::thread_cpu_time_ns()                    → uint64_t (CPU time
 *                                                        consumed by the
 *                                                        calling thread;
 *                                                        0 if unavailable)
 *   platform::process_alive(pid)                      → bool (best-effort;
 *                                                        defaults to true on
 *                                                        an indeterminate
 *                                                        result -- see below)
 *   platform::spawn_and_reap_process()                → uint32_t (a dead
 *                                                        PID; 0 on failure
 *                                                        to spawn at all)
 *   platform::run_and_capture(cmd)                    → {exit_code, stdout}
 *                                                        (throws on failure
 *                                                        to launch at all)
 *
 * notify_bind/notify_wake_all/notify_wait cross-process contract:
 *   On Linux, notify_wake_all/notify_wait use a futex on `word` directly —
 *   the kernel resolves a futex on MAP_SHARED memory by its physical
 *   backing, so this works correctly even though each process's mapping of
 *   `word` sits at a different virtual address. Windows' WaitOnAddress/
 *   WakeByAddressAll have no equivalent: they match purely by virtual
 *   address within the calling process, confirmed empirically — two
 *   MapViewOfFile calls on the very same segment (even within one process)
 *   get two different addresses, and waking one never wakes a waiter
 *   blocked on the other, even though the underlying memory is genuinely
 *   shared and visibly updates. So on Windows, notify_wake_all/notify_wait
 *   instead use a named kernel Event, one per shared segment, looked up by
 *   `word`'s address in this process — which is why callers must call
 *   notify_bind(name, word) once after shm_create/shm_attach (and
 *   notify_unbind(word) before unmapping): without a bound event, Windows
 *   silently falls back to polling rather than failing, so a caller that
 *   forgets to bind gets correctness (eventually, via the poll) but not the
 *   low-CPU blocking wait this API exists to provide.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <utility>
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
#  include <cerrno>
#  include <climits>   /* LONG_MAX */
#  include <csignal>   /* kill() -- process_alive() */
#  include <fcntl.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <sys/types.h> /* pid_t */
#  include <sys/wait.h>  /* waitpid() -- spawn_and_reap_process() */
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

/* ── Windows named-event registry (cross-process notify) ─────────────────── */
/*
 * Maps a notify word's address (as seen by this process) to the named
 * kernel Event standing in for it — see the API-surface comment above for
 * why WaitOnAddress/WakeByAddressAll can't be used here instead.
 */
#if defined(_WIN32)
namespace detail {
struct EventRegistry {
    std::mutex                              mtx;
    std::unordered_map<const void*, HANDLE> handles;

    static EventRegistry& instance() {
        static EventRegistry inst;
        return inst;
    }
    void insert(const void* word, HANDLE h) {
        std::lock_guard<std::mutex> lk(mtx);
        handles[word] = h;
    }
    HANDLE lookup(const void* word) {
        std::lock_guard<std::mutex> lk(mtx);
        auto it = handles.find(word);
        return it == handles.end() ? nullptr : it->second;
    }
    HANDLE remove(const void* word) {
        std::lock_guard<std::mutex> lk(mtx);
        auto it = handles.find(word);
        if (it == handles.end()) return nullptr;
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

/* ── shm_open_or_create ──────────────────────────────────────────────────── */

/*
 * Idempotent, race-safe open-or-create: if a segment under `name` already
 * exists, attaches to it (preserving its contents) exactly once, atomically
 * with respect to any other process racing to do the same; otherwise
 * creates it, zero-initialized. Unlike shm_create(), this never destroys an
 * existing segment -- shm_create()'s unconditional unlink-then-recreate is
 * right for a resource with a single owning producer that may need to force
 * a fresh start (a ring.hpp topic recovering from a crashed producer), but
 * wrong for a table multiple independent processes share and write into
 * concurrently (registry.hpp's node-discovery table): every process calling
 * shm_create() on it would race to wipe out whatever the others had already
 * written. Throws std::system_error only if the underlying OS call itself
 * fails (not on "already exists", which is the expected, common case).
 */
inline void* shm_open_or_create(const std::string& name, std::size_t size) {
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
    /* Must be read immediately: CreateFileMappingA sets this even on
     * success when it returned a handle to an object that already existed,
     * and any intervening API call could overwrite it. */
    const bool already_existed = (GetLastError() == ERROR_ALREADY_EXISTS);
    void* ptr = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (!ptr) {
        DWORD e = GetLastError(); CloseHandle(h);
        throw std::system_error(static_cast<int>(e), std::system_category(),
                                "MapViewOfFile " + name);
    }
    /* A brand-new mapping's pages are already OS-zero-filled; only an
     * explicit memset is needed is to avoid -- never zero an existing
     * mapping, which would wipe every other process's data in it. */
    if (!already_existed) std::memset(ptr, 0, size);
    detail::HandleRegistry::instance().insert(ptr, h);
    return ptr;
#else
    /* O_CREAT without O_EXCL and no preceding shm_unlink: if the object
     * already exists, this opens that same object (POSIX guarantees this
     * is atomic across racing processes); otherwise it creates a new one.
     * A freshly created or freshly extended (ftruncate) POSIX shm object's
     * pages are OS-zero-filled, so no explicit memset is needed either way
     * -- and memset-ing unconditionally here would, for the "already
     * existed" case, destroy another process's data. */
    int fd = ::shm_open(os.c_str(), O_CREAT | O_RDWR, 0666);
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

/* ── notify_bind / notify_unbind ─────────────────────────────────────────── */

/*
 * Bind `word` to the named cross-process Event standing in for it on
 * Windows (see the API-surface comment above). Both the creator and every
 * attacher call this once, right after shm_create/shm_attach, with the same
 * `name` used for the segment itself (a derived, distinct OS name is used
 * for the event so it doesn't collide with the file-mapping object's name).
 * No-op on Linux/macOS, where the futex/poll path needs no such binding.
 */
inline void notify_bind(const std::string& name, volatile uint32_t* word) noexcept {
#if defined(_WIN32)
    if (!word) return;
    std::string ev_name = shm_os_name(name) + "_evt";
    HANDLE h = CreateEventA(nullptr, /*bManualReset=*/TRUE,
                             /*bInitialState=*/FALSE, ev_name.c_str());
    if (h) detail::EventRegistry::instance().insert(
        const_cast<const uint32_t*>(word), h);
#else
    (void)name; (void)word;
#endif
}

/* Release the Event bound by notify_bind(). Call before unmapping `word`. */
inline void notify_unbind(volatile uint32_t* word) noexcept {
#if defined(_WIN32)
    if (!word) return;
    HANDLE h = detail::EventRegistry::instance().remove(
        const_cast<const uint32_t*>(word));
    if (h) CloseHandle(h);
#else
    (void)word;
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
    HANDLE h = detail::EventRegistry::instance().lookup(
        const_cast<const uint32_t*>(word));
    if (h) {
        /* Manual-reset pulse: Set wakes every thread/process currently
         * blocked in WaitForSingleObject on this handle; Reset immediately
         * after keeps a waiter that arrives later from seeing a stale
         * "already signaled" state and skipping its own wait entirely. */
        SetEvent(h);
        ResetEvent(h);
    }
    /* No bound event (notify_bind() was never called for this word): the
     * atomic increment above is still visible to notify_wait's polling
     * fallback, just without the low-CPU blocking wait. */
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
    const auto* a = reinterpret_cast<const std::atomic<uint32_t>*>(
        const_cast<const uint32_t*>(word));
    HANDLE h = detail::EventRegistry::instance().lookup(
        const_cast<const uint32_t*>(word));
    if (!h) {
        /* Not bound to an event (notify_bind() was never called for this
         * word) -- poll, same as the macOS/generic fallback below. */
        long waited = 0;
        long limit  = (timeout_ms < 0) ? LONG_MAX : static_cast<long>(timeout_ms);
        while (a->load(std::memory_order_acquire) == expected && waited < limit) {
            Sleep(1);
            waited += 1;
        }
        return;
    }
    /* SetEvent()+ResetEvent() in notify_wake_all isn't atomic with this
     * function's value check below, so a wake landing in that narrow gap
     * (value already changed, but this function hasn't yet called
     * WaitForSingleObject) would otherwise be missed until the full
     * timeout elapses. Waiting in short slices and rechecking the value
     * between them bounds that miss to one slice instead -- a configurable
     * engineering margin, not a silent gap, the same trade-off this
     * design's other retry/cadence bounds already make (design_ring_zero_
     * copy.md §8). */
    constexpr long kSliceMs = 15;
    long remaining = (timeout_ms < 0) ? -1 : timeout_ms;
    for (;;) {
        if (a->load(std::memory_order_acquire) != expected) return;
        DWORD slice = (remaining < 0 || remaining > kSliceMs)
                          ? static_cast<DWORD>(kSliceMs)
                          : static_cast<DWORD>(remaining);
        WaitForSingleObject(h, slice);
        if (a->load(std::memory_order_acquire) != expected) return;
        if (remaining >= 0) {
            remaining -= static_cast<long>(slice);
            if (remaining <= 0) return;
        }
    }
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

/* ── current_pid ─────────────────────────────────────────────────────────── */

inline uint32_t current_pid() noexcept {
#if defined(_WIN32)
    return static_cast<uint32_t>(GetCurrentProcessId());
#else
    return static_cast<uint32_t>(::getpid());
#endif
}

/* ── thread_cpu_time_ns ───────────────────────────────────────────────────── */

/*
 * CPU time (kernel + user) consumed by the calling thread so far, for
 * verifying a claim like NFR-1 ("a blocked pop_wait() uses ~0% CPU, versus
 * ~100% for a busy-spin poll loop") with a real measured number instead of
 * eyeballing a system monitor: call this before and after running a
 * workload for a known wall-clock duration, and the delta divided by that
 * duration is the thread's CPU utilization over that window. Returns 0 if
 * the underlying OS query fails (treated as "unavailable," not a usable
 * measurement of zero).
 */
inline uint64_t thread_cpu_time_ns() noexcept {
#if defined(_WIN32)
    FILETIME creation{}, exit{}, kernel{}, user{};
    if (!GetThreadTimes(GetCurrentThread(), &creation, &exit, &kernel, &user)) return 0;
    auto to_ns = [](const FILETIME& ft) -> uint64_t {
        ULARGE_INTEGER t;
        t.LowPart  = ft.dwLowDateTime;
        t.HighPart = ft.dwHighDateTime;
        return t.QuadPart * 100ULL; /* FILETIME is in 100ns units */
    };
    return to_ns(kernel) + to_ns(user);
#else
    struct timespec ts{};
    if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return 0;
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
#endif
}

/* ── process_alive ───────────────────────────────────────────────────────── */

/*
 * Best-effort liveness check for a PID this host has seen before (e.g. a
 * recorded producer_pid). An indeterminate result (permission denied
 * against a live process owned by a different user, or any other query
 * failure) defaults to "alive" -- the safe direction for a caller using
 * this to decide whether to take over a resource the PID might still own:
 * wrongly concluding "dead" risks exactly the double-ownership corruption
 * this check exists to prevent, while wrongly concluding "alive" only
 * costs a spurious conflict/refusal (see design_ring_zero_copy.md §5.10).
 */
inline bool process_alive(uint32_t pid) noexcept {
    if (pid == 0) return false;
#if defined(_WIN32)
    HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (!h) {
        /* ERROR_ACCESS_DENIED means a process with this PID exists but we
         * can't query it -- that's evidence it's alive, not indeterminate. */
        return GetLastError() == ERROR_ACCESS_DENIED;
    }
    DWORD exit_code = 0;
    bool got_exit_code = GetExitCodeProcess(h, &exit_code) != 0;
    CloseHandle(h);
    if (!got_exit_code) return true; /* indeterminate -- default to alive */
    return exit_code == STILL_ACTIVE;
#else
    if (::kill(static_cast<pid_t>(pid), 0) == 0) return true;
    if (errno == ESRCH) return false;       /* definitely no such process */
    return true;                            /* EPERM or anything else: indeterminate -- default to alive */
#endif
}

/* ── spawn_and_reap_process ──────────────────────────────────────────────── */

/*
 * Spawns a trivial, short-lived child process and waits for it to exit,
 * returning its PID -- now guaranteed dead, but a real PID the OS actually
 * assigned to a real process. A std::thread's PID is always its parent
 * process's, always alive, so simulating "a producer crashed" for
 * process_alive()/claim_producer_slot() (ring.hpp §5.10) genuinely needs a
 * separate OS process, not a thread -- this is that process, isolated here
 * (not in a test file) because spawning one is inherently OS-specific.
 * Returns 0 on failure to spawn.
 */
inline uint32_t spawn_and_reap_process() noexcept {
#if defined(_WIN32)
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    char cmd[] = "cmd.exe /c exit 0";
    BOOL ok = CreateProcessA(nullptr, cmd, nullptr, nullptr, FALSE,
                              CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    if (!ok) return 0;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD pid = pi.dwProcessId;
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return static_cast<uint32_t>(pid);
#else
    pid_t child = fork();
    if (child < 0) return 0;
    if (child == 0) {
        _exit(0);
    }
    int status = 0;
    waitpid(child, &status, 0);
    return static_cast<uint32_t>(child);
#endif
}

/* ── run_and_capture ──────────────────────────────────────────────────────── */

/*
 * Runs `cmd` as a child process, captures everything it writes to stdout,
 * waits for it to exit, and returns {exit_code, stdout}. Built on the C
 * runtime's popen()/_popen() (both run the command through the platform's
 * shell, so quote arguments as that shell expects) rather than hand-rolled
 * fork()+exec() or CreateProcess()+pipe plumbing -- popen/_popen already
 * exist, with the same signature, on every platform this project targets,
 * so there is no OS-conditional process/pipe code to maintain here beyond
 * the exit-status translation POSIX's pclose() needs (it returns a raw
 * wait() status, not a plain exit code; Windows' _pclose() already returns
 * the plain exit code). This is the cross-platform replacement for
 * fork()-based multi-process tests (e.g. test_topic.cpp's
 * MultipleSubscribers test): instead of forking a copy of the test binary,
 * launch a small standalone helper executable and read its result back off
 * stdout -- the same genuine separate-OS-process guarantee fork() gave,
 * without a primitive Windows has no equivalent of.
 * Throws std::runtime_error only if the process could not be launched at
 * all (not on a nonzero exit code, which is a normal, reportable outcome
 * returned in exit_code).
 */
inline std::pair<int, std::string> run_and_capture(const std::string& cmd) {
#if defined(_WIN32)
    /* _popen() on Windows runs the command via `cmd.exe /c <cmd>`, which
     * has a documented quirk (see `cmd /?`): unless the whole string
     * consists of *exactly* two quote characters wrapping a single
     * executable name, cmd.exe falls back to stripping only the very
     * first and very last quote character of the string and passing
     * everything between verbatim -- which mangles any command with more
     * than one quoted token (e.g. a quoted executable path followed by a
     * quoted argument: the embedded `" "` between them gets swallowed
     * into one literal token instead of splitting there). Wrapping the
     * whole command in one more throwaway pair of quotes sacrifices
     * exactly those two outer characters to that stripping and leaves
     * every inner quote untouched -- confirmed necessary and sufficient
     * by a real failure during verification (a quoted helper path
     * followed by a quoted argument was being run together as one
     * nonexistent filename until this was added). Not needed on POSIX,
     * where popen() uses /bin/sh -c and ordinary shell quoting applies. */
    FILE* p = _popen(("\"" + cmd + "\"").c_str(), "r");
#else
    FILE* p = ::popen(cmd.c_str(), "r");
#endif
    if (!p)
        throw std::runtime_error("run_and_capture: failed to launch: " + cmd);

    std::string out;
    char buf[4096];
    std::size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), p)) > 0) out.append(buf, n);

#if defined(_WIN32)
    int rc = _pclose(p);
#else
    int status = ::pclose(p);
    int rc = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
#endif
    return {rc, out};
}

} /* namespace platform */
} /* namespace shmbridge */
