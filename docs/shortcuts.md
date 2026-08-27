# Shortcuts

Every `// PROTOTYPE:` shortcut in the code gets one line here: what was cut and
why it is acceptable for a training prototype.

| Location | Shortcut | Rationale |
|----------|----------|-----------|
| `shared/motion_protocol.h` | Little-endian only, no byte-swap on serialize | Both hosts are x86-64 (LE); big-endian is out of scope |
