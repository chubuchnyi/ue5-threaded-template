// net_time.h — small Windows timing + UDP helpers shared by controller_sim and
// motion_gen. This is host-side sim glue, NOT the UE hot path, so ordinary
// blocking sockets and OS calls are fine here.
#ifndef MOTIONLINK_NET_TIME_H
#define MOTIONLINK_NET_TIME_H

#include <stdint.h>
#include <stddef.h>

namespace nt {

// Process-wide QPC clock, nanoseconds. Monotonic; epoch is arbitrary and
// differs between processes, so only differences are meaningful across the
// process boundary. Call clock_init() once at startup.
void     clock_init();
uint64_t now_ns();

// Request a 1 ms system scheduler period (timeBeginPeriod) for the process
// lifetime, and undo it. Improves Sleep() granularity used by sleep_until_ns.
void scheduler_hi_res(bool enable);

// Busy/sleep hybrid: sleeps until `deadline_ns` (nt::now_ns timeline). Sleeps
// coarsely while far from the deadline, spins for the last ~1.5 ms to hit
// sub-millisecond periods that Sleep() alone cannot.
void sleep_until_ns(uint64_t deadline_ns);

// One UDP socket, either bound (receiver) or unbound (sender).
struct UdpSocket {
    uintptr_t fd = ~(uintptr_t)0; // INVALID_SOCKET

    bool valid() const { return fd != ~(uintptr_t)0; }
};

// Startup/teardown of Winsock. init returns false on failure.
bool net_init();
void net_shutdown();

// Create a UDP socket. If bind_port != 0, bind to 127.0.0.1:bind_port.
// non_blocking makes recv return immediately when empty.
bool udp_open(UdpSocket* s, uint16_t bind_port, bool non_blocking);
void udp_close(UdpSocket* s);

// Receive one datagram. Returns bytes read (>0), 0 if none available
// (non-blocking, empty), or -1 on error. On success, out_from_ip/out_from_port
// carry the sender address (host byte order) when the pointers are non-null.
int udp_recv(UdpSocket* s, void* buf, size_t buf_len,
             uint32_t* out_from_ip, uint16_t* out_from_port);

// Send one datagram to 127.0.0.1:port. Returns bytes sent or -1.
int udp_send_local(UdpSocket* s, uint16_t port, const void* buf, size_t len);

} // namespace nt

#endif // MOTIONLINK_NET_TIME_H
