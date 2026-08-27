// controller_sim — software stand-in for the motion platform controller.
//
// Pipeline (see CLAUDE.md Stage 1):
//   recv SetpointFrame on :5005  -> validate (size/magic/ver/CRC)
//                                -> seq bookkeeping (loss / reorder)
//                                -> buffer sample keyed by t_tx_ns
//                                -> echo FeedbackFrame on :5006
//   2 kHz control tick           -> resample input on the t_tx_ns timeline
//                                -> first-order actuator model (tau)
//                                -> watchdog: 5 missed periods => ramp to
//                                   neutral over 500 ms (LIMITED -> PARK)
//                                -> CSV row per tick
//   1 Hz                         -> console stats: rx rate, loss, jitter, fill
//
// This is host-side glue, not the UE hot path, so blocking I/O, the heap, and
// std:: containers are all fair game here.

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

// ---------------------------------------------------------------------------
// Config (CLI)
// ---------------------------------------------------------------------------
struct Config {
    double   tau       = 0.02;  // actuator time constant, s
    uint16_t rx_port   = 5005;
    uint16_t tx_port   = 5006;
    int      tick_hz   = 2000;  // controller control rate
    int      input_hz  = 1000;  // expected setpoint rate (for watchdog period)
    double   run_secs  = 0.0;   // 0 => run until Ctrl+C
    std::string csv_path;
};

static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

static bool parse_args(int argc, char** argv, Config* c) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* what) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", what); return nullptr; }
            return argv[++i];
        };
        if (a == "--tau")           { const char* v = next("--tau");       if (!v) return false; c->tau = atof(v); }
        else if (a == "--csv")      { const char* v = next("--csv");       if (!v) return false; c->csv_path = v; }
        else if (a == "--rx-port")  { const char* v = next("--rx-port");   if (!v) return false; c->rx_port = (uint16_t)atoi(v); }
        else if (a == "--tx-port")  { const char* v = next("--tx-port");   if (!v) return false; c->tx_port = (uint16_t)atoi(v); }
        else if (a == "--hz")       { const char* v = next("--hz");        if (!v) return false; c->tick_hz = atoi(v); }
        else if (a == "--input-hz") { const char* v = next("--input-hz");  if (!v) return false; c->input_hz = atoi(v); }
        else if (a == "--seconds")  { const char* v = next("--seconds");   if (!v) return false; c->run_secs = atof(v); }
        else if (a == "--help" || a == "-h") { return false; }
        else { std::fprintf(stderr, "unknown arg: %s\n", a.c_str()); return false; }
    }
    if (c->tau <= 0.0)   { std::fprintf(stderr, "--tau must be > 0\n"); return false; }
    if (c->tick_hz <= 0) { std::fprintf(stderr, "--hz must be > 0\n"); return false; }
    return true;
}

static void usage() {
    std::printf(
        "controller_sim [options]\n"
        "  --tau <s>        actuator time constant (default 0.02)\n"
        "  --csv <path>     write per-tick CSV log\n"
        "  --rx-port <n>    setpoint receive port (default 5005)\n"
        "  --tx-port <n>    feedback send port (default 5006)\n"
        "  --hz <n>         control tick rate (default 2000)\n"
        "  --input-hz <n>   expected setpoint rate for watchdog (default 1000)\n"
        "  --seconds <s>    auto-stop after N seconds (default: until Ctrl+C)\n");
}

// ---------------------------------------------------------------------------
// Sample buffer for t_tx_ns resampling
// ---------------------------------------------------------------------------
struct Sample {
    uint64_t t_tx_ns = 0;
    float    pose[MOTION_DOF] = {0};
};

// Small ring of recent valid samples, ordered by arrival. We interpolate the
// input on the sender's t_tx_ns clock, decoupling the smooth output from
// jittery arrival. Reordered/duplicate frames are dropped from the buffer.
struct SampleRing {
    static const int CAP = 128;
    Sample buf[CAP];
    int    count = 0; // number of valid entries, newest at (head-1)
    int    head  = 0; // next write index

    void push(const Sample& s) {
        buf[head] = s;
        head = (head + 1) % CAP;
        if (count < CAP) ++count;
    }
    // Oldest..newest logical index [0, count).
    const Sample& at(int logical) const {
        int idx = (head - count + logical + CAP * 2) % CAP;
        return buf[idx];
    }
    bool empty() const { return count == 0; }
    uint64_t newest_t() const { return count ? at(count - 1).t_tx_ns : 0; }

    // Linear interpolation of pose at t on the t_tx_ns timeline. Holds the
    // endpoint when t is outside the buffered range. Also returns how many
    // buffered samples lie ahead of t (lookahead "fill").
    void sample_at(uint64_t t, float out[MOTION_DOF], int* fill) const {
        int ahead = 0;
        for (int i = 0; i < count; ++i) if (at(i).t_tx_ns > t) ++ahead;
        if (fill) *fill = ahead;

        if (count == 1 || t <= at(0).t_tx_ns) {
            std::memcpy(out, at(0).pose, sizeof(float) * MOTION_DOF);
            return;
        }
        if (t >= at(count - 1).t_tx_ns) {
            std::memcpy(out, at(count - 1).pose, sizeof(float) * MOTION_DOF);
            return;
        }
        for (int i = 0; i + 1 < count; ++i) {
            const Sample& a = at(i);
            const Sample& b = at(i + 1);
            if (t >= a.t_tx_ns && t <= b.t_tx_ns) {
                double span = double(b.t_tx_ns - a.t_tx_ns);
                double f = span > 0 ? double(t - a.t_tx_ns) / span : 0.0;
                for (int d = 0; d < MOTION_DOF; ++d)
                    out[d] = float(a.pose[d] + (b.pose[d] - a.pose[d]) * f);
                return;
            }
        }
        std::memcpy(out, at(count - 1).pose, sizeof(float) * MOTION_DOF); // fallback
    }
};

// ---------------------------------------------------------------------------
// Rolling stats over a 1 s window
// ---------------------------------------------------------------------------
struct Stats {
    uint64_t received  = 0;   // valid frames counted toward loss
    uint64_t lost      = 0;   // seq jumps forward
    uint64_t reordered = 0;   // seq < expected (late/dup)
    uint64_t crc_fail  = 0;
    std::vector<double> intervals_us; // inter-arrival gaps this window
    int      fill_sum  = 0;
    int      fill_max  = 0;
    int      fill_n    = 0;

    void reset() {
        received = lost = reordered = crc_fail = 0;
        intervals_us.clear();
        fill_sum = fill_max = fill_n = 0;
    }
    void note_fill(int f) {
        fill_sum += f; ++fill_n;
        if (f > fill_max) fill_max = f;
    }
};

static double pct(std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    size_t idx = (size_t)(p * (v.size() - 1) + 0.5);
    if (idx >= v.size()) idx = v.size() - 1;
    return v[idx];
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    Config cfg;
    if (!parse_args(argc, argv, &cfg)) { usage(); return 2; }

    std::signal(SIGINT, on_sigint);
    nt::clock_init();
    nt::scheduler_hi_res(true);
    if (!nt::net_init()) { std::fprintf(stderr, "WSAStartup failed\n"); return 1; }

    nt::UdpSocket rx, tx;
    if (!nt::udp_open(&rx, cfg.rx_port, /*non_blocking*/ true)) {
        std::fprintf(stderr, "bind :%u failed (port in use?)\n", cfg.rx_port);
        return 1;
    }
    if (!nt::udp_open(&tx, 0, /*non_blocking*/ false)) {
        std::fprintf(stderr, "tx socket failed\n");
        return 1;
    }

    FILE* csv = nullptr;
    if (!cfg.csv_path.empty()) {
        csv = std::fopen(cfg.csv_path.c_str(), "wb");
        if (!csv) { std::fprintf(stderr, "cannot open csv %s\n", cfg.csv_path.c_str()); return 1; }
        std::fprintf(csv, "tick_ns,sim_input_ns,seq,t_tx_ns,state,limiter,fill,"
                          "in0,in1,in2,in3,in4,in5,out0,out1,out2,out3,out4,out5\n");
    }

    const double   dt      = 1.0 / cfg.tick_hz;
    const uint64_t dt_ns   = (uint64_t)(1e9 / cfg.tick_hz + 0.5);
    const double   alpha   = 1.0 - std::exp(-dt / cfg.tau); // first-order lag step
    const uint64_t input_period_ns = (uint64_t)(1e9 / (cfg.input_hz > 0 ? cfg.input_hz : 1000));
    const uint64_t watchdog_ns     = 5 * input_period_ns;   // 5 missed periods
    const double   ramp_secs       = 0.5;                    // to neutral

    // --- runtime state ---
    SampleRing ring;
    Stats stats;
    uint32_t expected_seq = 0;
    bool     have_seq = false;
    uint64_t last_rx_ns = 0;          // arrival time of most recent valid frame
    uint64_t prev_rx_ns = 0;
    uint32_t last_seq = 0;
    uint64_t last_t_tx = 0;
    uint16_t last_flags = 0;

    double   y[MOTION_DOF] = {0};     // plant output
    uint64_t sim_input_ns = 0;        // resampling read cursor on t_tx timeline
    bool     started = false;

    int      state = MOTION_STATE_INIT;
    uint64_t limited_since_ns = 0;
    double   ramp_from[MOTION_DOF] = {0}; // output captured at watchdog trip

    const uint64_t t0 = nt::now_ns();
    uint64_t next_tick = t0;
    uint64_t next_stat = t0 + 1000000000ull;

    std::printf("controller_sim: tau=%.4gs tick=%dHz watchdog=%.1fms rx:%u tx:%u%s\n",
                cfg.tau, cfg.tick_hz, watchdog_ns / 1e6, cfg.rx_port, cfg.tx_port,
                csv ? " (csv on)" : "");

    while (!g_stop) {
        // ---- 1. drain all pending datagrams -------------------------------
        for (;;) {
            SetpointFrame sp;
            uint32_t fip; uint16_t fport;
            int n = nt::udp_recv(&rx, &sp, sizeof(sp), &fip, &fport);
            if (n <= 0) break;                 // 0 = empty, -1 = error: stop draining
            (void)fip; (void)fport;            // echo goes to localhost:tx_port
            if (n != (int)sizeof(SetpointFrame)) continue; // wrong size
            if (!motion_setpoint_valid(&sp)) { ++stats.crc_fail; continue; }

            uint64_t rx_ns = nt::now_ns();

            // seq bookkeeping
            if (!have_seq) {
                have_seq = true;
                expected_seq = sp.seq + 1;
                ++stats.received;
            } else if (sp.seq == expected_seq) {
                ++stats.received; ++expected_seq;
            } else if (sp.seq > expected_seq) {
                stats.lost += (sp.seq - expected_seq); // frames skipped
                ++stats.received;
                expected_seq = sp.seq + 1;
            } else {
                ++stats.reordered;                     // late/duplicate
                // still echo it, but don't feed a backwards sample to the ring
            }

            // inter-arrival jitter
            if (prev_rx_ns != 0) stats.intervals_us.push_back((rx_ns - prev_rx_ns) / 1000.0);
            prev_rx_ns = rx_ns;

            // buffer for resampling (skip out-of-order to keep t_tx monotonic)
            if (sp.seq >= last_seq || !started) {
                Sample s; s.t_tx_ns = sp.t_tx_ns;
                std::memcpy(s.pose, sp.pose, sizeof(float) * MOTION_DOF);
                ring.push(s);
            }

            last_rx_ns = rx_ns;
            last_seq   = sp.seq;
            last_t_tx  = sp.t_tx_ns;
            last_flags = sp.flags;

            if (!started) { started = true; sim_input_ns = sp.t_tx_ns; }

            // recover from a watchdog trip once fresh data flows again
            if (state == MOTION_STATE_LIMITED || state == MOTION_STATE_PARK) {
                state = MOTION_STATE_ACTIVE;
                sim_input_ns = ring.newest_t();
            }

            // ---- echo feedback immediately (1:1 with received setpoint) ----
            FeedbackFrame fb;
            std::memset(&fb, 0, sizeof(fb));
            fb.seq = sp.seq;
            fb.t_rx_ns = rx_ns;
            fb.t_tx_ns = sp.t_tx_ns;
            for (int d = 0; d < MOTION_DOF; ++d) fb.measured_pose[d] = (float)y[d];
            fb.state = (uint8_t)state;
            fb.limiter_active = (sp.flags & MOTION_FLAG_LIMITER_ACTIVE) ? 1 : 0;
            fb.fault_mask = 0;
            motion_feedback_finalize(&fb);
            nt::udp_send_local(&tx, cfg.tx_port, &fb, sizeof(fb));
        }

        // ---- 2. control tick ---------------------------------------------
        uint64_t now = nt::now_ns();
        if (now >= next_tick) {
            if (started && state == MOTION_STATE_INIT) state = MOTION_STATE_ACTIVE;

            // watchdog: too long since a valid frame -> begin ramp to neutral
            if (started && state == MOTION_STATE_ACTIVE &&
                (now - last_rx_ns) > watchdog_ns) {
                state = MOTION_STATE_LIMITED;
                limited_since_ns = now;
                for (int d = 0; d < MOTION_DOF; ++d) ramp_from[d] = y[d];
            }

            int fill = 0;
            if (state == MOTION_STATE_ACTIVE) {
                float u[MOTION_DOF];
                ring.sample_at(sim_input_ns, u, &fill);
                for (int d = 0; d < MOTION_DOF; ++d)
                    y[d] += alpha * ((double)u[d] - y[d]);   // first-order lag
                sim_input_ns += dt_ns;                       // advance read cursor
            } else if (state == MOTION_STATE_LIMITED) {
                double t = (now - limited_since_ns) / 1e9;
                double k = 1.0 - t / ramp_secs;              // 1 -> 0 over 500 ms
                if (k <= 0.0) { k = 0.0; state = MOTION_STATE_PARK; }
                for (int d = 0; d < MOTION_DOF; ++d) y[d] = ramp_from[d] * k;
            } else if (state == MOTION_STATE_PARK) {
                for (int d = 0; d < MOTION_DOF; ++d) y[d] = 0.0;
            }

            stats.note_fill(fill);

            if (csv) {
                std::fprintf(csv, "%llu,%llu,%u,%llu,%d,%d,%d",
                    (unsigned long long)now, (unsigned long long)sim_input_ns,
                    last_seq, (unsigned long long)last_t_tx, state,
                    (last_flags & MOTION_FLAG_LIMITER_ACTIVE) ? 1 : 0, fill);
                float u_dbg[MOTION_DOF] = {0};
                if (!ring.empty()) ring.sample_at(sim_input_ns > dt_ns ? sim_input_ns - dt_ns : sim_input_ns, u_dbg, nullptr);
                for (int d = 0; d < MOTION_DOF; ++d) std::fprintf(csv, ",%.6f", u_dbg[d]);
                for (int d = 0; d < MOTION_DOF; ++d) std::fprintf(csv, ",%.6f", y[d]);
                std::fprintf(csv, "\n");
            }

            next_tick += dt_ns;
            // if we fell behind (e.g. debugger paused us), don't spiral
            if (now > next_tick + 5 * dt_ns) next_tick = now + dt_ns;
        }

        // ---- 3. once-per-second console stats ----------------------------
        now = nt::now_ns();
        if (now >= next_stat) {
            std::sort(stats.intervals_us.begin(), stats.intervals_us.end());
            double p50 = pct(stats.intervals_us, 0.50);
            double p99 = pct(stats.intervals_us, 0.99);
            double mx  = stats.intervals_us.empty() ? 0.0 : stats.intervals_us.back();
            uint64_t denom = stats.received + stats.lost;
            double loss_pct = denom ? 100.0 * stats.lost / denom : 0.0;
            double avg_fill = stats.fill_n ? (double)stats.fill_sum / stats.fill_n : 0.0;

            const char* sname[] = {"INIT","ACTIVE","LIMITED","PARK","FAULT"};
            std::printf("[%5.0fs] rx=%4llu/s loss=%.3f%% (crc_fail=%llu reorder=%llu) "
                        "iat p50=%.0f p99=%.0f max=%.0f us  fill avg=%.2f max=%d  state=%s\n",
                (now - t0) / 1e9,
                (unsigned long long)stats.received, loss_pct,
                (unsigned long long)stats.crc_fail, (unsigned long long)stats.reordered,
                p50, p99, mx, avg_fill, stats.fill_max,
                sname[state <= MOTION_STATE_FAULT ? state : 0]);
            std::fflush(stdout);

            stats.reset();
            next_stat += 1000000000ull;
        }

        // stop after the requested duration (for scripted tests)
        if (cfg.run_secs > 0.0 && (now - t0) / 1e9 >= cfg.run_secs) break;

        // pace the loop: wake near the next tick without busy-spinning the CPU
        nt::sleep_until_ns(next_tick < now ? now : next_tick);
    }

    if (csv) std::fclose(csv);
    nt::udp_close(&rx);
    nt::udp_close(&tx);
    nt::net_shutdown();
    nt::scheduler_hi_res(false);
    std::printf("controller_sim: stopped.\n");
    return 0;
}
