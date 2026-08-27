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

## Stage 3 — SPSC ring + async physics + observer

Setup: async physics at a fixed 120 Hz substep; `UTelemetrySourceComponent`
samples the Chaos body on the physics thread into the hand-written SPSC ring;
the worker runs a const-accel observer at 1 kHz and streams to `controller_sim`.
An auto-spawned `AMotionTestActor` (servo-driven cube) provides the motion.

Full frame rate:

- Observer output tracks the cube's Lissajous path: X ∈ [−0.253, 0.253] m,
  Y ∈ [−0.286, 0.286] m, Z ∈ [0.955, 1.045] m (Z ≈ 1.0 = spawn height).
- **1000 rx/s, 0.000% loss**, iat p50 = 1000 µs.
- Ring: drop 0, depth ≈ 0 (consumer keeps up); samples flow at 120 Hz.

Artificial FPS drop (`t.MaxFPS 20`) — the key criterion:

- Output stays **bounded and smooth**: X ∈ [−0.222, 0.222] m, max step
  0.00051 m per 1 kHz tick — no gaps or discontinuities. 999 rx/s, 0.000% loss.
- The observer bridges the now-bursty 120 Hz telemetry (6 substeps land within
  ~1 ms each 50 ms game frame) to a continuous 1 kHz output.

### Two bugs found by the FPS-drop test

1. **Observer blow-up (~1e9).** Samples were timestamped with wall-clock time.
   Under the FPS drop the async substeps run in a burst, so wall-clock deltas
   between samples collapse toward 0 and the velocity-slope acceleration
   estimate exploded. Fix: timestamp with the solver's `SimTime` (true fixed dt
   per substep) plus an acceleration sanity clamp.
2. **Test-rig instability.** The stimulus cube's position servo (K = 500) went
   unstable when its force was refreshed only at 20 Hz (force held stale for a
   50 ms frame → overshoot → divergence). This was the *stimulus*, not the
   pipeline. Fix: soften the servo (K = 60, critically damped) so it stays
   stable across frame rates.

## Stage 4 — cueing skeleton + limiter

The worker inserts, between the observer and the UDP send: washout (first-order
HPF on translations, LPF on rotations) → workspace limiter (single vector
scale) → velocity + jerk limit. All coefficients are live `motion.Cue.*` /
`motion.Limit.*` CVars.

Washout (default `motion.Cue.TransHighpassHz` = 0.2 Hz):

- The cube's sustained ~1.0 m heave offset is high-passed away: output Z goes
  from raw [0.955, 1.045] m to **[−0.045, 0.046] m** (centred on neutral),
  while X/Y still oscillate ±0.25 m (0.5 Hz passes the 0.2 Hz HPF). This is the
  washout returning the platform toward neutral.

Limiter (single whole-vector scale, no per-component clamp):

- Default limits (0.5 m, 2 m/s, 100 /s²): limiter active ~26% of ticks (the
  velocity limit trimming the observer's per-tick correction spikes).
- Tighten to `motion.Limit.Trans 0.1` **live from the console**: limiter engages
  on **97%** of ticks and the output is capped — X ∈ [−0.103, 0.100],
  Y ∈ [−0.101, 0.101] m (down from ±0.25). Both axes scale together, confirming
  the deviation *vector* is scaled by one factor rather than clamped per axis.
- `limiter_active` is carried in the setpoint flags and shows in the
  `controller_sim` CSV `limiter` column and the on-screen overlay (`lim=ON`).

Meets the done criterion: coefficients change from the console on the fly and
the limiter's action is visible in the log.
