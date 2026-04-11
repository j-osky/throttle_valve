/*
 * win_compat.h — POSIX compatibility shims for Windows (MinGW-w64)
 * =============================================================================
 * MSYS2 MinGW-w64 already provides struct timespec, clock_gettime,
 * clock_nanosleep, and ssize_t natively via pthreads-win32.
 * This header only provides:
 *   - usleep()                  (not in MinGW by default)
 *   - Winsock2 init/cleanup
 *   - socket_set_nonblocking / socket_set_blocking
 *   - High-resolution waitable timer constants
 *
 * MUST be included before <time.h> and <pthread.h> to avoid conflicts.
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
#include <time.h>       /* pulls in struct timespec, clock_gettime from MinGW */
#include <pthread.h>    /* pulls in clock_nanosleep from pthreads-win32       */

/* ── High-resolution waitable timer flag (Win10 1803+) ───────────────────── */
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002UL
#endif
#ifndef CREATE_WAITABLE_TIMER_MANUAL_RESET
#define CREATE_WAITABLE_TIMER_MANUAL_RESET    0x00000001UL
#endif

/* ── TIMER_ABSTIME (defined in pthread_time.h, guard anyway) ─────────────── */
#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

/* ── usleep ──────────────────────────────────────────────────────────────── *
 * MinGW-w64 does not provide usleep().  Implement via clock_nanosleep which  *
 * is already provided by pthreads-win32 and declared in pthread_time.h.     */
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

/* ── Socket blocking mode (ioctlsocket replaces fcntl on Windows) ────────── */
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