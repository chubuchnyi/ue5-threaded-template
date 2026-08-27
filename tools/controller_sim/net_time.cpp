// net_time.cpp — see net_time.h.
#include "net_time.h"

#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <timeapi.h>

namespace nt {

static uint64_t g_qpc_freq = 0;

void clock_init() {
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    g_qpc_freq = (uint64_t)f.QuadPart;
}

uint64_t now_ns() {
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    // (counts * 1e9) / freq, done in 128-bit-ish steps to avoid overflow:
    // split to whole seconds + remainder so counts*1e9 never wraps 64 bits.
    uint64_t counts = (uint64_t)c.QuadPart;
    uint64_t secs = counts / g_qpc_freq;
    uint64_t rem  = counts % g_qpc_freq;
    return secs * 1000000000ull + (rem * 1000000000ull) / g_qpc_freq;
}

void scheduler_hi_res(bool enable) {
    if (enable) timeBeginPeriod(1);
    else        timeEndPeriod(1);
}

void sleep_until_ns(uint64_t deadline_ns) {
    for (;;) {
        uint64_t now = now_ns();
        if (now >= deadline_ns) return;
        uint64_t remaining = deadline_ns - now;
        // Sleep in coarse chunks until within ~1.5 ms, then spin. Sleep(1) with
        // a 1 ms scheduler period overshoots by up to ~1 ms, so we stop sleeping
        // early and burn the tail on the CPU to keep the period tight.
        if (remaining > 1500000ull) {
            Sleep(1);
        } else {
            YieldProcessor();
        }
    }
}

// --- sockets -------------------------------------------------------------

bool net_init() {
    WSADATA wsa;
    return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
}

void net_shutdown() {
    WSACleanup();
}

bool udp_open(UdpSocket* s, uint16_t bind_port, bool non_blocking) {
    SOCKET fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd == INVALID_SOCKET) return false;

    if (bind_port != 0) {
        sockaddr_in addr;
        ZeroMemory(&addr, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(bind_port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(fd, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            closesocket(fd);
            return false;
        }
    }

    if (non_blocking) {
        u_long nb = 1;
        ioctlsocket(fd, FIONBIO, &nb);
    }

    s->fd = (uintptr_t)fd;
    return true;
}

void udp_close(UdpSocket* s) {
    if (s->valid()) {
        closesocket((SOCKET)s->fd);
        s->fd = ~(uintptr_t)0;
    }
}

int udp_recv(UdpSocket* s, void* buf, size_t buf_len,
             uint32_t* out_from_ip, uint16_t* out_from_port) {
    sockaddr_in from;
    int fromlen = sizeof(from);
    int n = recvfrom((SOCKET)s->fd, (char*)buf, (int)buf_len, 0,
                     (sockaddr*)&from, &fromlen);
    if (n == SOCKET_ERROR) {
        int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK) return 0; // nothing queued
        return -1;
    }
    if (out_from_ip)   *out_from_ip = ntohl(from.sin_addr.s_addr);
    if (out_from_port) *out_from_port = ntohs(from.sin_port);
    return n;
}

int udp_send_local(UdpSocket* s, uint16_t port, const void* buf, size_t len) {
    sockaddr_in addr;
    ZeroMemory(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    int n = sendto((SOCKET)s->fd, (const char*)buf, (int)len, 0,
                   (sockaddr*)&addr, sizeof(addr));
    return (n == SOCKET_ERROR) ? -1 : n;
}

} // namespace nt
