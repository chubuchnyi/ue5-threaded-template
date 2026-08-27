# Shortcuts

Every `// PROTOTYPE:` shortcut in the code gets one line here: what was cut and
why it is acceptable for a training prototype.

| Location | Shortcut | Rationale |
|----------|----------|-----------|
| `shared/motion_protocol.h` | Little-endian only, no byte-swap on serialize | Both hosts are x86-64 (LE); big-endian is out of scope |
| `tools/controller_sim/main.cpp` | Single-threaded drain-then-tick, feedback echoed 1:1 from the recv loop | Sim is not the hot path; avoids a rx thread + mutex. 2 kHz drain interval is fine for 1 kHz input |
| `tools/controller_sim/main.cpp` | Resampling clock seeded from the first sample's `t_tx_ns`, advanced by dt | No cross-process clock sync yet; only relative `t_tx_ns` is used. Absolute phys-tick→controller latency is Stage 5 |
| `tools/controller_sim/motion_gen.cpp` | Whole file is throwaway test scaffolding | Stands in for the UE worker until Stage 2; RTT it reports is bounded by its own 1 ms poll |
| `Plugins/MotionLink/.../MotionWorker.cpp` | Raw Winsock UDP in the worker instead of UE's socket subsystem | Keeps the hot path POD-only and allocation-free; worker is Win64-only anyway |
| `Plugins/MotionLink/.../MotionWorker.cpp` | Worker sends UDP synchronously (non-blocking socket) from the hot path | Arch puts UDP tx/rx in the worker; loopback sendto never blocks. Non-blocking guarantees no stall |
| `Plugins/MotionLink/.../MotionWorker.cpp` | Synthetic sine, no real physics yet | Stage 2 proves the thread/timer/UDP path; real telemetry is Stage 3 |
