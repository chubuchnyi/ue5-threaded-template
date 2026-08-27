// motion_gen — throwaway stand-in for the UE MotionWorker (Stage 1 testing).
//
// Sends SetpointFrames at --hz (default 1000) carrying a sine on the surge
// axis, and listens for FeedbackFrame echoes to report round-trip latency.
// Quit it (Ctrl+C or --seconds) to exercise controller_sim's watchdog.
//
// PROTOTYPE: this is test scaffolding, replaced by the real UE worker in
// Stage 2. It uses the heap and blocking-ish sockets freely.

#include "motion_protocol.h"
#include "net_time.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <algorithm>

static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

int main(int argc, char** argv) {
    uint16_t tx_port = 5005;   // where controller listens
    uint16_t rx_port = 5006;   // where we listen for echoes
    int      hz      = 1000;
    double   amp     = 0.2;    // metres
    double   freq_hz = 0.5;    // sine frequency
    double   run_secs = 0.0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
        if      (a == "--hz")       { const char* v = nx(); if (v) hz = atoi(v); }
        else if (a == "--amp")      { const char* v = nx(); if (v) amp = atof(v); }
        else if (a == "--freq")     { const char* v = nx(); if (v) freq_hz = atof(v); }
        else if (a == "--seconds")  { const char* v = nx(); if (v) run_secs = atof(v); }
        else if (a == "--tx-port")  { const char* v = nx(); if (v) tx_port = (uint16_t)atoi(v); }
        else if (a == "--rx-port")  { const char* v = nx(); if (v) rx_port = (uint16_t)atoi(v); }
        else { std::fprintf(stderr, "motion_gen: unknown arg %s\n", a.c_str()); return 2; }
    }

    std::signal(SIGINT, on_sigint);
    nt::clock_init();
    nt::scheduler_hi_res(true);
    if (!nt::net_init()) { std::fprintf(stderr, "WSAStartup failed\n"); return 1; }

    nt::UdpSocket tx, rx;
    if (!nt::udp_open(&tx, 0, false))            { std::fprintf(stderr, "tx open failed\n"); return 1; }
    if (!nt::udp_open(&rx, rx_port, true))       { std::fprintf(stderr, "bind :%u failed\n", rx_port); return 1; }

    const uint64_t dt_ns = (uint64_t)(1e9 / hz + 0.5);
    const uint64_t t0 = nt::now_ns();
    uint64_t next = t0;
    uint64_t next_stat = t0 + 1000000000ull;
    uint32_t seq = 0;
    uint64_t sent = 0, echoed = 0;
    std::vector<double> rtt_us;

    std::printf("motion_gen: %d Hz sine amp=%.3g freq=%.3gHz -> :%u, echo on :%u\n",
                hz, amp, freq_hz, tx_port, rx_port);

    while (!g_stop) {
        uint64_t now = nt::now_ns();
        if (now >= next) {
            double t = (now - t0) / 1e9;
            double s = amp * std::sin(2.0 * 3.14159265358979323846 * freq_hz * t);

            SetpointFrame sp;
            std::memset(&sp, 0, sizeof(sp));
            sp.seq = seq++;
            sp.t_tx_ns = now;
            sp.flags = MOTION_FLAG_NONE;
            sp.pose[0] = (float)s;                       // surge
            sp.vel[0]  = (float)(amp * 2.0 * 3.14159265 * freq_hz *
                                 std::cos(2.0 * 3.14159265 * freq_hz * t));
            motion_setpoint_finalize(&sp);
            nt::udp_send_local(&tx, tx_port, &sp, sizeof(sp));
            ++sent;

            next += dt_ns;
            if (now > next + 5 * dt_ns) next = now + dt_ns; // don't spiral after a stall
        }

        // drain echoes for RTT
        for (;;) {
            FeedbackFrame fb;
            int n = nt::udp_recv(&rx, &fb, sizeof(fb), nullptr, nullptr);
            if (n <= 0) break;
            if (n != (int)sizeof(FeedbackFrame) || !motion_feedback_valid(&fb)) continue;
            uint64_t rttn = nt::now_ns() - fb.t_tx_ns;   // same-process clock => valid RTT
            rtt_us.push_back(rttn / 1000.0);
            ++echoed;
        }

        now = nt::now_ns();
        if (now >= next_stat) {
            std::sort(rtt_us.begin(), rtt_us.end());
            auto q = [&](double p) {
                if (rtt_us.empty()) return 0.0;
                size_t i = (size_t)(p * (rtt_us.size() - 1) + 0.5);
                return rtt_us[i < rtt_us.size() ? i : rtt_us.size() - 1];
            };
            std::printf("[%5.0fs] sent=%llu echoed=%llu  rtt p50=%.0f p99=%.0f max=%.0f us\n",
                (now - t0) / 1e9, (unsigned long long)sent, (unsigned long long)echoed,
                q(0.50), q(0.99), rtt_us.empty() ? 0.0 : rtt_us.back());
            std::fflush(stdout);
            rtt_us.clear();
            sent = echoed = 0;
            next_stat += 1000000000ull;
        }

        if (run_secs > 0.0 && (now - t0) / 1e9 >= run_secs) break;
        nt::sleep_until_ns(next);
    }

    nt::udp_close(&tx);
    nt::udp_close(&rx);
    nt::net_shutdown();
    nt::scheduler_hi_res(false);
    std::printf("motion_gen: stopped after %llu frames.\n", (unsigned long long)seq);
    return 0;
}
