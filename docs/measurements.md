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
