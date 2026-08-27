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
| `motion.Sine.Freq` | `0.5` | live | Synthetic sine frequency, Hz |
| `motion.Sine.Amp` | `0.2` | live | Synthetic sine amplitude, metres |
| `motion.Overlay` | `1` | live | Draw the on-screen debug overlay |

**Scope:** `startup` values are read once when the subsystem initializes and
handed to the worker; changing them at runtime has no effect until restart.
`live` values are pushed to the worker every game-thread frame.
