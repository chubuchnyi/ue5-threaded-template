# Measurements

Only measured numbers go here. Estimates are marked explicitly as `(estimate)`.

## Stage 0 — protocol

Nothing to measure yet; this stage produces a header only.

- `shared/motion_protocol.h` compiles clean under g++ 13 (MinGW), `-std=c++11
  -Wall -Wextra -Werror`.
- `static_assert` on both frame sizes passes: `SetpointFrame` = 72 B,
  `FeedbackFrame` = 60 B.
- CRC32 known-answer test: `crc32("123456789") == 0xCBF43926` — passes.
- MSVC compile: pending (verified when the UE plugin first builds, Stage 2).

## Stage 1 — controller_sim

Setup: `controller_sim --tau 0.02 --hz 2000` fed by `motion_gen --hz 1000`
(1 kHz sine), both on loopback, MinGW g++ 13 Release build. Generator stopped
at 4 s to exercise the watchdog.

Steady-state (generator running):

- Receive rate: **1000 frames/s**, sustained.
- Loss: **0.000%** (well under the 0.1% target), crc_fail 0, reorder 0.
- Inter-arrival jitter at the controller: **p50 = 1000 µs, p99 = 1004–1009 µs,
  max = 1008–2307 µs**. The occasional ~2 ms max is a Windows scheduler
  hiccup on this non-tuned desktop; core-parking/C-states not yet disabled
  (that tuning is a Stage 5 concern).
- Buffer fill (t_tx lookahead): avg ≈ 0.0, max 1–3 samples — resampling stays
  right at the live edge.

Watchdog (generator stopped):

- Trip to `LIMITED` within 5 input periods (5 ms) of the last valid frame.
- Ramp to neutral: **exactly 1001 control ticks ≈ 500.5 ms** at 2 kHz, then
  `PARK`. Output falls linearly from −0.00975 m to ≈0 with no discontinuity.

Plant tracking: `out0` follows the input sine with the expected first-order
lag (e.g. in0 = 0.1985 → out0 = 0.1993 near a peak).

RTT (from `motion_gen`): p50 ≈ 1010 µs. _(estimate — dominated by the stub's
1 ms echo-polling interval, not true pipeline latency. Real RTT is measured
UE-side in later stages.)_
