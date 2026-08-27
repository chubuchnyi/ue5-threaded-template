# CVars

All runtime-tunable values live behind a CVar prefixed `motion.`. No magic
constants in code. This table is the source of truth; keep it in sync.

| CVar | Default | Scope | Description |
|------|---------|-------|-------------|
| `motion.Enabled` | `1` | live | Worker emits setpoints when 1; idle (still ticking) when 0 |
| `motion.Controller.Ip` | `127.0.0.1` | startup | Destination IP for setpoint UDP |
| `motion.Tx.Port` | `5005` | startup | UDP port setpoints are sent to |
| `motion.Rx.Port` | `5006` | startup | UDP port feedback is received on |
| `motion.CoreAffinity` | `-1` | startup | Pin worker to this CPU core index; -1 = no affinity |
| `motion.Sine.Freq` | `0.5` | live | Synthetic sine frequency, Hz (sine source) |
| `motion.Sine.Amp` | `0.2` | live | Synthetic sine amplitude, metres (sine source) |
| `motion.Overlay` | `1` | live | Draw the on-screen debug overlay |
| `motion.Source` | `1` | live | Worker output: 0 = synthetic sine, 1 = telemetry observer |
| `motion.Obs.CorrectMs` | `8` | live | Observer residual-correction smear time, ms |
| `motion.Test.Spawn` | `1` | startup | Auto-spawn the physics test actor on world begin play |
| `motion.Test.Freq` | `0.5` | live | Test body drive frequency, Hz |
| `motion.Test.Amp` | `0.3` | live | Test body drive amplitude, metres |
| `motion.Test.Stiffness` | `60` | live | Servo spring stiffness driving the body to its sine target |
| `motion.Test.Damping` | `16` | live | Servo damping |

**Scope:** `startup` values are read once when the subsystem initializes and
handed to the worker; changing them at runtime has no effect until restart.
`live` values are pushed to the worker every game-thread frame.
