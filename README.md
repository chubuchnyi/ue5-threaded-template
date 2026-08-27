# MotionLink Prototype

A minimal end-to-end training rig for the pipeline **UE5 physics → cross-thread
hand-off → network → motion-platform controller**. There is no hardware; the
controller is a software stub in a separate process.

The point is **not features — it's practice**. The prototype exists to prove
three things, with numbers rather than assumptions:

1. Data leaves the physics tick to a dedicated thread without blocking the game
   thread and without touching `UObject`s from outside the game thread.
2. A fixed 1 kHz thread holds up on Windows, and its jitter is **measured**.
3. The UE project builds, runs, and can be profiled and debugged.

See [`CLAUDE.md`](CLAUDE.md) for the full brief and the coding rules for the
hot path.

## Architecture

```
UE5 (MotionProto.uproject)                         controller_sim (no UE)
┌───────────────────────────────────────┐          ┌────────────────────────────┐
│ Physics thread (async tick)           │          │ recv :5005, check CRC/seq  │
│   UTelemetrySourceComponent           │  UDP     │ resample on t_tx (2 kHz)   │
│     └─ SPSC ring ─┐                    │  5005 →  │ first-order plant (tau)    │
│ MotionWorker (FRunnable, 1 kHz)       │ ───────► │ watchdog → ramp to neutral │
│   ├─ const-accel observer  ◄──────────┘          │ stats + CSV                │
│   └─ UDP tx / rx           ◄────────── 5006 ◄─── │ echo FeedbackFrame         │
│ Game thread: read-only debug overlay  │          └────────────────────────────┘
└───────────────────────────────────────┘
```

Two processes. `shared/motion_protocol.h` is a single dependency-free header
used verbatim by both, so the wire protocol can be developed and tested before
UE even compiles.

## Repository layout

```
shared/motion_protocol.h      wire protocol (frames, CRC32, static_asserts)
tools/controller_sim/         standalone CMake C++ controller stub + generator
Plugins/MotionLink/           the UE plugin (subsystem, worker, ring, telemetry)
Source/MotionProto*           the UE game module + build targets
Config/                       project config (async physics enabled here)
docs/                         cvars.md, measurements.md, shortcuts.md
MotionProto.uproject
```

`Binaries/`, `Intermediate/`, `Saved/`, the `.sln`/`.vcxproj`, and CMake
`build/` output are git-ignored.

## Requirements

- Windows 10/11
- Unreal Engine 5.7
- Visual Studio 2022 with the MSVC v143 C++ toolchain (14.44 verified) — the
  "Game development with C++" workload
- CMake 3.16+ (for `controller_sim`; MinGW g++ or MSVC both work)

## Step by step: build, debug, test

All commands are PowerShell, run from the repo root. Set `$env:UE` to your
engine root once per shell:

```powershell
$env:UE = "C:\Program Files\Epic Games\UE_5.7"
```

### Step 0 — Check the toolchain

```powershell
& "$env:UE\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" -version   # engine present
cmake --version                                                    # 3.16+
# MSVC: open "x64 Native Tools Command Prompt for VS 2022" and run `cl` — it
# should print the compiler banner (v14.44 verified).
```

### Step 1 — Build `controller_sim` (no UE needed)

The controller stub is a standalone CMake project, so you can develop and test
the whole protocol before UE compiles.

```powershell
cmake -G "MinGW Makefiles" -B tools/controller_sim/build -S tools/controller_sim -DCMAKE_BUILD_TYPE=Release
cmake --build tools/controller_sim/build
```

Prefer MSVC? Drop the `-G` line and add `--config Release` to the build (the
exes then land in `tools/controller_sim/build/Release/`). Output:
`controller_sim.exe` and `motion_gen.exe` (a throwaway 1 kHz sine generator
standing in for UE).

### Step 2 — Smoke-test the protocol (two terminals)

```powershell
# terminal 1 — the controller under test (listens :5005, echoes :5006)
.\tools\controller_sim\build\controller_sim.exe --tau 0.02 --csv run.csv
# terminal 2 — the generator
.\tools\controller_sim\build\motion_gen.exe --hz 1000
```

Expect `rx=1000/s loss=0.000% ... state=ACTIVE`. Now **Ctrl+C the generator**:
the watchdog trips within 5 ms and ramps the output to neutral over 500 ms
(`LIMITED` → `PARK`). One-shot scripted variant (no manual Ctrl+C):

```powershell
Start-Process .\tools\controller_sim\build\controller_sim.exe -ArgumentList '--tau','0.02','--csv','run.csv','--seconds','9' -RedirectStandardOutput ctrl.log
.\tools\controller_sim\build\motion_gen.exe --seconds 4      # stops mid-run -> watchdog fires
Get-Content ctrl.log
```

### Step 3 — Generate the UE Visual Studio project files

Run this after the first checkout and any time you add/remove a module or
source file:

```powershell
& "$env:UE\Engine\Build\BatchFiles\Build.bat" -projectfiles -project="$PWD\MotionProto.uproject" -game
```

This writes `MotionProto.sln`.

### Step 4 — Build the UE editor target

Command line (good for CI / headless):

```powershell
& "$env:UE\Engine\Build\BatchFiles\Build.bat" MotionProtoEditor Win64 Development -project="$PWD\MotionProto.uproject" -waitmutex
```

Or in the IDE: open `MotionProto.sln`, set the configuration to
**Development Editor** + **Win64**, startup project **MotionProto**, then
**Build → Build Solution** (Ctrl+Shift+B). First build compiles the two project
modules against the prebuilt engine — a few minutes.

### Step 5 — Run the full pipeline (windowed)

Start `controller_sim` first (Step 2, terminal 1), then launch the game:

```powershell
& "$env:UE\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" "$PWD\MotionProto.uproject" -game -nosound
```

A `MotionLink  1000 Hz  jitter … RTT …  src=obs ring=… state=ACTIVE` overlay
renders top-left. The auto-spawned test actor drives a physics body; the
worker's observer streams its motion to `controller_sim` at 1 kHz. Toggle live
CVars from the console (`` ` ``): `motion.Source 0` (sine vs observer),
`motion.Sine.Freq 1.0`, `motion.Test.Amp 0.4`, `motion.Overlay 0`.

### Step 6 — Headless verification (no window)

```powershell
& "$env:UE\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" "$PWD\MotionProto.uproject" -game -nullrhi -unattended -nosound
```

With `controller_sim --csv run.csv` running, the CSV records the pose the
controller receives. Confirm motion is present:

```powershell
Import-Csv run.csv | Select-Object -Last 1   # in0..in5 = received pose, out0..out5 = plant
```

### Step 7 — Debug in Visual Studio 2022

1. Open `MotionProto.sln`, configuration **Development Editor / Win64**.
2. **F5** launches the editor under the debugger, or **Debug → Attach to
   Process** and pick a running `UnrealEditor.exe` / `UnrealEditor-Cmd.exe`.
3. Useful breakpoints:
   - `FMotionWorker::Run` (`MotionWorker.cpp`) — the 1 kHz loop and the
     observer. **Note:** pausing here stalls the worker, so the controller's
     watchdog will trip to `PARK` while you're stopped — expected.
   - `UTelemetrySourceComponent::AsyncPhysicsTickComponent` — the physics-thread
     producer writing the ring.
   - `UMotionLinkSubsystem::Tick` — game-thread control push + overlay.
4. Watch the flow live: the `MotionLink` overlay, and `controller_sim`'s
   once-per-second stats line.
5. **Live Coding** (Ctrl+Alt+F11) recompiles function bodies without restarting.
   It does **not** handle header changes, new `UPROPERTY`s, or class-size
   changes — after those, restart the editor rather than debug ghosts.

Enable engine symbols (Epic Games Launcher → Engine → Options → *Editor symbols
for debugging*) or call stacks into the engine are unreadable, and confirm
`UE.natvis` is picked up so `TArray`/`FString` display properly.

### Step 8 — Acceptance checks

- `controller_sim` receives 1 kHz from UE, loss < 0.1%.
- `t.MaxFPS 20` in the UE console (an artificial FPS drop) does **not** cause
  gaps in the controller's output — the const-accel observer keeps it smooth.
- Killing the UE process makes the controller's watchdog ramp to neutral.
- Numbers land in [`docs/measurements.md`](docs/measurements.md).

## Configuration

Everything tunable is a `motion.*` CVar (set from the UE console or an `.ini`).
Full list in [`docs/cvars.md`](docs/cvars.md) — e.g. `motion.Source`
(0 = sine, 1 = telemetry observer), `motion.Sine.Freq`, `motion.CoreAffinity`,
`motion.Test.Amp`, `motion.Obs.CorrectMs`.

## Status

| Stage | What | State |
|------|------|-------|
| 0 | Wire protocol header | ✅ verified (g++ + MSVC) |
| 1 | controller_sim + generator | ✅ verified (1 kHz, 0% loss, watchdog) |
| 2 | UE plugin skeleton: 1 kHz worker → UDP | ✅ verified on device (stable 1000 Hz) |
| 3 | SPSC ring + async physics tick + observer | 🚧 code complete, on-device verify in progress |
| 4 | Cueing skeleton + limiter | ⬜ |
| 5 | Measurements (Insights, jitter, latency) | ⬜ |

Measured results and the bugs each stage surfaced are in
[`docs/measurements.md`](docs/measurements.md). Deliberate prototype
shortcuts are logged in [`docs/shortcuts.md`](docs/shortcuts.md).

## Notes

- Profile only in Standalone or a packaged Development build — PIE measurements
  are not trustworthy (different world, editor holding resources).
- Meaningful jitter numbers need the machine tuned: disable core parking and
  C-states, move the NIC's interrupts off the worker core, pin the worker
  (`motion.CoreAffinity`). Untuned desktop p99 is dominated by scheduler noise.
