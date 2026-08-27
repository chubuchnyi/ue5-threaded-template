# Measurements

Only measured numbers go here. Estimates are marked explicitly as `(estimate)`.

## Stage 0 — protocol

Nothing to measure yet; this stage produces a header only.

- `shared/motion_protocol.h` compiles clean under g++ 13 (MinGW), `-std=c++11
  -Wall -Wextra -Werror`.
- `static_assert` on both frame sizes passes: `SetpointFrame` = 72 B,
  `FeedbackFrame` = 60 B.
- CRC32 known-answer test: `crc32("123456789") == 0xCBF43926` — passes.
- MSVC compile: verified with cl.exe 19.44 (VS 2022 Build Tools),
  `/std:c++20 /permissive- /W4 /WX` — clean, sizes 72/60, CRC + validate pass.
  Required consulting `_MSVC_LANG` (MSVC reports `__cplusplus == 199711L`
  without `/Zc:__cplusplus`).

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

## Stage 2 — UE plugin skeleton

Build: `MotionProtoEditor Win64 Development` via UBT, MSVC 14.44. Both modules
compile and link clean (`UnrealEditor-MotionProto.dll`,
`UnrealEditor-MotionLink.dll`). Run: `UnrealEditor-Cmd MotionProto.uproject
-game -nullrhi -unattended`, worker feeding `controller_sim` on loopback.

Result — **stable 1 kHz from UE**:

- Receive rate at the controller: **999–1001 frames/s**, sustained over 10 s.
- Loss: **0.000%**, crc_fail 0, reorder 0.
- Send-interval jitter (measured at the controller): **p50 = 1000 µs,
  p99 ≈ 1505 µs, max 2000–4074 µs**. The p99/max tail is untuned-desktop
  scheduler noise (core parking / C-states still on — Stage 5 tuning).
- UE's synthetic sine arrives intact: surge amplitude 0.200 m, period 2.0 s
  (0.5 Hz) — matches the `motion.Sine.*` CVar defaults exactly.

### Bug found by measurement: timer quantized to ~2 ms

First run clocked **~530 frames/s** (p50 ≈ 1500–2000 µs), not 1 kHz. Cause: a
periodic `SetWaitableTimer` (lPeriod = 1) still rides the process timer
resolution, and the worker never raised it. Fix: `timeBeginPeriod(1)` plus
re-arming the high-resolution waitable timer as a one-shot against an advancing
absolute deadline each iteration (self-corrects drift). After the fix: solid
1000 Hz as above. This is exactly the "measure, don't assume" point of the
prototype.
