/*
 * win_compat.h — POSIX compatibility shims for Windows (MinGW-w64)
 * =============================================================================
 * Provides clock_gettime, clock_nanosleep, usleep using Windows native APIs
 * (QueryPerformanceCounter + WaitableTimer) bypassing pthreads-win32 timing
 * which has epoch/precision issues at 170 Hz.
 *
 * Include BEFORE all other headers.
 * Link with: -lws2_32 -lpthread
 * =============================================================================
 */

#pragma once

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0602
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdint.h>
#include <stdio.h>

/* ── ssize_t ─────────────────────────────────────────────────────────────── */
#ifndef ssize_t
typedef long long ssize_t;
#endif

/* ── TIMER_ABSTIME ───────────────────────────────────────────────────────── */
#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

/* ── CLOCK_MONOTONIC ─────────────────────────────────────────────────────── */
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

/* ── Pull in pthread for mutexes — but we override its timing functions ──── */
/* Include pthread BEFORE our clock_gettime/clock_nanosleep macros so our
 * #define macros take precedence over the pthreads function declarations.  */
#include <pthread.h>

/* Now define our QPC-based overrides — these #defines replace any
 * clock_gettime / clock_nanosleep calls with our Windows-native versions. */
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002UL
#endif
#ifndef CREATE_WAITABLE_TIMER_MANUAL_RESET
#define CREATE_WAITABLE_TIMER_MANUAL_RESET    0x00000001UL
#endif

/* ── struct timespec ─────────────────────────────────────────────────────── */
/* Define our own to avoid conflicts with MinGW/pthreads versions.
 * We use win_timespec internally and alias it.                              */
/* timespec is provided by MinGW pthreads — we reuse it directly.
 * Only define if pthreads has not already defined it.               */
#if !defined(_TIMESPEC_DEFINED) && !defined(_STRUCT_TIMESPEC)
#define _TIMESPEC_DEFINED
struct timespec {
    long tv_sec;
    long tv_nsec;
};
#endif

/* ── QPC-based clock_gettime ─────────────────────────────────────────────── */
/* Uses QueryPerformanceCounter directly — immune to pthreads epoch issues.
 * Epoch is arbitrary (system boot) but consistent within one process run.  */
static inline int win_clock_gettime(struct timespec *ts)
{
    static LARGE_INTEGER g_freq = {{0}};
    static long long     g_origin = 0;
    if (g_freq.QuadPart == 0) {
        QueryPerformanceFrequency(&g_freq);
        LARGE_INTEGER t; QueryPerformanceCounter(&t);
        g_origin = t.QuadPart;
    }
    LARGE_INTEGER cnt;
    QueryPerformanceCounter(&cnt);
    long long delta = cnt.QuadPart - g_origin;
    ts->tv_sec  = (long)(delta / g_freq.QuadPart);
    ts->tv_nsec = (long)((delta % g_freq.QuadPart) * 1000000000LL
                          / g_freq.QuadPart);
    return 0;
}
/* Override clock_gettime for our use — define as macro so it takes priority */
#define clock_gettime(clk, ts)  win_clock_gettime(ts)

/* ── clock_nanosleep (TIMER_ABSTIME) ─────────────────────────────────────── */
/* Sleeps until absolute time expressed in our QPC epoch.
 * Uses high-resolution waitable timer on Win10 1803+, falls back to
 * standard timer (~1ms) on older Windows.                                   */
static inline int win_clock_nanosleep(const struct timespec *request)
{
    struct timespec now;
    win_clock_gettime(&now);

    long long delta_ns =
        (long long)(request->tv_sec  - now.tv_sec)  * 1000000000LL +
        (long long)(request->tv_nsec - now.tv_nsec);

    if (delta_ns <= 0) return 0;

    /* Windows timer units are 100ns, negative = relative */
    LARGE_INTEGER due;
    due.QuadPart = -(delta_ns / 100LL);

    /* Try high-resolution timer (Win10 1803+) */
    HANDLE hTimer = CreateWaitableTimerExW(
        NULL, NULL,
        CREATE_WAITABLE_TIMER_HIGH_RESOLUTION | CREATE_WAITABLE_TIMER_MANUAL_RESET,
        TIMER_ALL_ACCESS);

    if (!hTimer) {
        /* Fallback to standard waitable timer */
        hTimer = CreateWaitableTimerW(NULL, TRUE, NULL);
    }

    if (hTimer) {
        SetWaitableTimer(hTimer, &due, 0, NULL, NULL, FALSE);
        WaitForSingleObject(hTimer, INFINITE);
        CloseHandle(hTimer);
    } else {
        /* Last resort: Sleep in ms */
        DWORD ms = (DWORD)((delta_ns + 999999LL) / 1000000LL);
        if (ms > 0) Sleep(ms);
    }
    return 0;
}
#define clock_nanosleep(clk, flags, req, rem)  win_clock_nanosleep(req)

/* ── usleep ──────────────────────────────────────────────────────────────── */
static inline void usleep(unsigned long us)
{
    if (us == 0) return;
    struct timespec now, target;
    win_clock_gettime(&now);
    long long total_ns = (long long)now.tv_nsec + (long long)us * 1000LL;
    target.tv_sec  = now.tv_sec  + (long)(total_ns / 1000000000LL);
    target.tv_nsec = (long)(total_ns % 1000000000LL);
    win_clock_nanosleep(&target);
}

/* ── Winsock2 init/cleanup ───────────────────────────────────────────────── */
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