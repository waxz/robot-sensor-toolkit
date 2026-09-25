/*
 * shmbridge/notify.hpp — pluggable notification strategies for shmbridge v3.
 *
 * INotifier: abstract base (virtual, for runtime polymorphism).
 * SemNotifier: POSIX semaphore — wakes ONE waiter per post (unicast).
 * ShmFutexNotifier: futex on shared memory — one FUTEX_WAKE wakes ALL waiters
 *   simultaneously (broadcast), O(1) publisher cost regardless of subscriber count.
 * NullNotifier: no-op — pure spin / poll, no side channel.
 *
 * Only ShmFutexNotifier is cross-process safe without extra setup.
 * SemNotifier is most portable (macOS included); ShmFutexNotifier is Linux-only.
 */

#pragma once

#include "platform.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>    /* snprintf */

#if defined(__APPLE__) || defined(__linux__)
#  include <cerrno>
#  include <cstring>
#  include <fcntl.h>
#  include <semaphore.h>
#endif

namespace shmbridge {

/* ── abstract interface ──────────────────────────────────────────────────── */

struct INotifier {
    virtual ~INotifier()              = default;
    virtual void notify()             noexcept = 0;
    virtual void wait(int timeout_ms) noexcept = 0;
};

/* ── no-op notifier (pure-spin subscribers) ──────────────────────────────── */

struct NullNotifier final : INotifier {
    void notify()             noexcept override {}
    void wait(int /*timeout*/) noexcept override {}
};

/* ── POSIX named semaphore (unicast, portable) ───────────────────────────── */

#if defined(__APPLE__) || defined(__linux__)

class SemNotifier final : public INotifier {
public:
    /* Publisher: call open_create; subscriber: call open_attach */
    bool open_create(const char* topic_name) noexcept {
        build_name(topic_name);
        ::sem_unlink(name_);
        sem_ = ::sem_open(name_, O_CREAT | O_EXCL, 0600, 0);
        return sem_ != SEM_FAILED;
    }
    bool open_attach(const char* topic_name, int timeout_ms = 5000) noexcept {
        build_name(topic_name);
        const long deadline_ns = static_cast<long>(timeout_ms) * 1'000'000L;
        long waited = 0;
        while (waited < deadline_ns) {
            sem_ = ::sem_open(name_, 0);
            if (sem_ != SEM_FAILED) return true;
            platform::sleep_ns(5'000'000LL);
            waited += 5'000'000L;
        }
        return false;
    }
    void close() noexcept {
        if (sem_ && sem_ != SEM_FAILED) { ::sem_close(sem_); sem_ = nullptr; }
    }
    void unlink() noexcept { ::sem_unlink(name_); }

    void notify() noexcept override {
        if (sem_ && sem_ != SEM_FAILED) ::sem_post(sem_);
    }
    void wait(int timeout_ms) noexcept override {
        if (!sem_ || sem_ == SEM_FAILED) return;
        if (timeout_ms < 0) {
            ::sem_wait(sem_);
        } else {
#if defined(__linux__)
            struct timespec abs{};
            ::clock_gettime(CLOCK_REALTIME, &abs);
            abs.tv_sec  += timeout_ms / 1000;
            abs.tv_nsec += static_cast<long>(timeout_ms % 1000) * 1'000'000L;
            if (abs.tv_nsec >= 1'000'000'000L) { abs.tv_sec++; abs.tv_nsec -= 1'000'000'000L; }
            ::sem_timedwait(sem_, &abs);
#else
            /* macOS lacks sem_timedwait — busy-poll */
            long waited = 0, deadline = static_cast<long>(timeout_ms) * 1'000'000L;
            while (waited < deadline) {
                if (::sem_trywait(sem_) == 0) return;
                platform::sleep_ns(500'000LL);
                waited += 500'000L;
            }
#endif
        }
    }

private:
    void build_name(const char* t) noexcept {
        ::snprintf(name_, sizeof(name_), "/sbsem_%.56s", t);
    }
    sem_t* sem_  = nullptr;
    char   name_[64]{};
};

#endif /* POSIX */

/* ── broadcast notifier backed by platform::notify_* (cross-platform) ───── */

/*
 * Uses a `volatile uint32_t` word in shared memory as the notification word.
 * On Linux: Linux futex (FUTEX_WAKE/FUTEX_WAIT).
 * On Windows 10+: WakeByAddressAll / WaitOnAddress.
 * On macOS/other: atomic increment + 1ms-poll fallback.
 *
 * Publisher increments the word and wakes every subscriber in one call.
 *
 * Usage:
 *   ShmFutexNotifier pub_n, sub_n;
 *   pub_n.bind(&header->notify_seq);
 *   sub_n.bind(&header->notify_seq);
 *   pub_n.notify();   // increments + broadcasts → wakes all
 *   sub_n.wait(100);  // blocks until changed or timeout
 */
class ShmFutexNotifier final : public INotifier {
public:
    void bind(volatile uint32_t* word) noexcept { word_ = word; }

    void notify() noexcept override {
        platform::notify_wake_all(word_);
    }

    void wait(int timeout_ms = -1) noexcept override {
        if (!word_) return;
        uint32_t val = reinterpret_cast<const std::atomic<uint32_t>*>(
            const_cast<const uint32_t*>(word_))->load(std::memory_order_acquire);
        platform::notify_wait(word_, val, timeout_ms);
    }

private:
    volatile uint32_t* word_ = nullptr;
};

} /* namespace shmbridge */
