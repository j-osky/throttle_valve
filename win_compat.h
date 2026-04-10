/*
 * win_compat.h — POSIX compatibility shims for Windows (MinGW-w64)
 * =============================================================================
 * Provides:
 *   - struct timespec + clock_gettime(CLOCK_MONOTONIC)  [if not already defined]
 *   - clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)
 *   - usleep()
 *   - ssize_t typedef
 *   - Winsock2 init/cleanup helpers
 *   - socket_set_nonblocking / socket_set_blocking
 *
 * Include this file FIRST, before any other system headers.
 * Link with: -lws2_32 -lpthread
 *
 * Requires MinGW-w64.
 * Requires Windows 10 version 1803+ for high-resolution waitable timers.
 * Falls back to standard waitable timer on older Windows (~1ms precision).
 * =============================================================================
 */

#pragma once

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0602   /* Windows 8+ minimum */
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdint.h>
#include <stdio.h>

/* ── ssize_t ─────────────────────────────────────────────────────────────── */
#ifndef ssize_t
typedef int ssize_t;
#endif

/* ── struct timespec + clock_gettime ────────────────────────────────────── */
/* MinGW-w64 >= 8 defines timespec and clock_gettime natively.
 * Earlier versions do not.  We guard by checking CLOCK_MONOTONIC which
 * is defined alongside timespec in MinGW >= 8.                              */
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1

struct timespec {
    long tv_sec;
    long tv_nsec;
};

static inline int clock_gettime(int clk_id, struct timespec *ts)
{
    (void)clk_id;
    static LARGE_INTEGER g_freq;
    static int g_freq_init = 0;
    if (!g_freq_init) {
        QueryPerformanceFrequency(&g_freq);
        g_freq_init = 1;
    }
    LARGE_INTEGER cnt;
    QueryPerformanceCounter(&cnt);
    ts->tv_sec  = (long)(cnt.QuadPart / g_freq.QuadPart);
    ts->tv_nsec = (long)((cnt.QuadPart % g_freq.QuadPart)
                         * 1000000000LL / g_freq.QuadPart);
    return 0;
}
#endif /* CLOCK_MONOTONIC */

/* ── TIMER_ABSTIME ───────────────────────────────────────────────────────── */
#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

/* High-resolution timer flag (Windows 10 1803+) */
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002UL
#endif
#ifndef CREATE_WAITABLE_TIMER_MANUAL_RESET
#define CREATE_WAITABLE_TIMER_MANUAL_RESET    0x00000001UL
#endif

/* ── clock_nanosleep ─────────────────────────────────────────────────────── */
/*
 * Emulates clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, target, NULL).
 * Uses a Windows waitable timer. Win10 1803+ gives ~100us precision.
 * Older Windows falls back to ~1ms Sleep().
 */
static inline int clock_nanosleep(int clk_id, int flags,
                                   const struct timespec *request,
                                   struct timespec *remain)
{
    (void)clk_id; (void)flags; (void)remain;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long long delta_ns =
        (long long)(request->tv_sec  - now.tv_sec)  * 1000000000LL +
        (long long)(request->tv_nsec - now.tv_nsec);

    if (delta_ns <= 0) return 0;

    /* Negative value = relative duration in 100ns units */
    LARGE_INTEGER due;
    due.QuadPart = -(delta_ns / 100LL);

    /* Try high-resolution waitable timer first (Win10 1803+) */
    HANDLE hTimer = CreateWaitableTimerExW(
        NULL, NULL,
        CREATE_WAITABLE_TIMER_HIGH_RESOLUTION | CREATE_WAITABLE_TIMER_MANUAL_RESET,
        TIMER_ALL_ACCESS);

    if (!hTimer) {
        /* Fallback: standard waitable timer */
        hTimer = CreateWaitableTimerW(NULL, TRUE, NULL);
    }

    if (hTimer) {
        SetWaitableTimer(hTimer, &due, 0, NULL, NULL, FALSE);
        WaitForSingleObject(hTimer, INFINITE);
        CloseHandle(hTimer);
    } else {
        /* Last resort: Sleep() — 1ms granularity */
        DWORD ms = (DWORD)((delta_ns + 999999LL) / 1000000LL);
        if (ms > 0) Sleep(ms);
    }
    return 0;
}

/* ── usleep ──────────────────────────────────────────────────────────────── */
static inline void usleep(unsigned long us)
{
    if (us == 0) return;
    struct timespec now, target;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long long total_ns = (long long)now.tv_nsec + (long long)us * 1000LL;
    target.tv_sec  = now.tv_sec + (long)(total_ns / 1000000000LL);
    target.tv_nsec = (long)(total_ns % 1000000000LL);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, NULL);
}

/* ── Winsock2 helpers ────────────────────────────────────────────────────── */
static inline void winsock_init(void)
{
    WSADATA wsa;
    int rc = WSAStartup(MAKEWORD(2, 2), &wsa);
    if (rc != 0) fprintf(stderr, "[NET] WSAStartup failed: %d\n", rc);
}

static inline void winsock_cleanup(void)
{
    WSACleanup();
}

/* ── Socket blocking mode ────────────────────────────────────────────────── */
/* Use ioctlsocket() instead of fcntl() for Winsock sockets on Windows.    */
static inline void socket_set_nonblocking(SOCKET s)
{
    u_long mode = 1;
    ioctlsocket(s, FIONBIO, &mode);
}

static inline void socket_set_blocking(SOCKET s)
{
    u_long mode = 0;
    ioctlsocket(s, FIONBIO, &mode);
}
