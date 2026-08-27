#include "MotionLinkSubsystem.h"
#include "MotionWorker.h"
#include "TelemetryRing.h"
#include "motion_protocol.h"

#include "Engine/Engine.h"
#include "HAL/IConsoleManager.h"

// ---------------------------------------------------------------------------
// CVars. Every tunable is here; no magic constants elsewhere. Keep this list
// mirrored in docs/cvars.md.
// ---------------------------------------------------------------------------
static TAutoConsoleVariable<int32> CVarEnabled(
	TEXT("motion.Enabled"), 1,
	TEXT("1: worker emits setpoints; 0: idle (still ticks)."), ECVF_Default);

static TAutoConsoleVariable<FString> CVarControllerIp(
	TEXT("motion.Controller.Ip"), TEXT("127.0.0.1"),
	TEXT("Destination IP for setpoint UDP. Read once at startup."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarTxPort(
	TEXT("motion.Tx.Port"), 5005,
	TEXT("UDP port setpoints are sent to. Read once at startup."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarRxPort(
	TEXT("motion.Rx.Port"), 5006,
	TEXT("UDP port feedback is received on. Read once at startup."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarCoreAffinity(
	TEXT("motion.CoreAffinity"), -1,
	TEXT("Pin worker to this CPU core index; -1 = no affinity. Read at startup."), ECVF_Default);

static TAutoConsoleVariable<float> CVarSineFreq(
	TEXT("motion.Sine.Freq"), 0.5f,
	TEXT("Synthetic sine frequency, Hz. Live."), ECVF_Default);

static TAutoConsoleVariable<float> CVarSineAmp(
	TEXT("motion.Sine.Amp"), 0.2f,
	TEXT("Synthetic sine amplitude, metres. Live."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarOverlay(
	TEXT("motion.Overlay"), 1,
	TEXT("1: draw the on-screen debug overlay. Live."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarStats(
	TEXT("motion.Stats"), 0,
	TEXT("1: log a measured stats line (Hz/jitter/RTT/latency) once per second. Live."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarSource(
	TEXT("motion.Source"), 1,
	TEXT("Worker output source: 0 = synthetic sine, 1 = telemetry observer. Live."), ECVF_Default);

static TAutoConsoleVariable<int32> CVarCorrectMs(
	TEXT("motion.Obs.CorrectMs"), 8,
	TEXT("Observer residual-correction smear time, ms. Live."), ECVF_Default);

// --- Stage 4: cueing skeleton + limiter ---
static TAutoConsoleVariable<int32> CVarCueEnabled(
	TEXT("motion.Cue.Enabled"), 1,
	TEXT("1: apply washout + limiter; 0: bypass (raw observer output). Live."), ECVF_Default);
static TAutoConsoleVariable<float> CVarTransHpf(
	TEXT("motion.Cue.TransHighpassHz"), 0.2f,
	TEXT("Translation washout high-pass cutoff, Hz. Live."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRotLpf(
	TEXT("motion.Cue.RotLowpassHz"), 2.0f,
	TEXT("Tilt-coordination low-pass cutoff, Hz. Live."), ECVF_Default);
static TAutoConsoleVariable<float> CVarLimitTrans(
	TEXT("motion.Limit.Trans"), 0.5f,
	TEXT("Workspace translation limit, metres. Live."), ECVF_Default);
static TAutoConsoleVariable<float> CVarLimitRot(
	TEXT("motion.Limit.Rot"), 0.35f,
	TEXT("Workspace rotation limit, radians. Live."), ECVF_Default);
static TAutoConsoleVariable<float> CVarLimitVel(
	TEXT("motion.Limit.Vel"), 2.0f,
	TEXT("Velocity limit, m/s or rad/s. Live."), ECVF_Default);
static TAutoConsoleVariable<float> CVarLimitJerk(
	TEXT("motion.Limit.Jerk"), 100.0f,
	TEXT("Jerk limit, (m|rad)/s^2. Live."), ECVF_Default);

// Parse "a.b.c.d" into a host-order uint32. Returns loopback on failure.
static uint32 ParseIpHostOrder(const FString& Ip)
{
	TArray<FString> Parts;
	Ip.ParseIntoArray(Parts, TEXT("."), true);
	if (Parts.Num() != 4) { return 0x7F000001u; } // 127.0.0.1
	uint32 v = 0;
	for (int i = 0; i < 4; ++i)
	{
		const int32 Octet = FCString::Atoi(*Parts[i]);
		if (Octet < 0 || Octet > 255) { return 0x7F000001u; }
		v = (v << 8) | (uint32)Octet;
	}
	return v;
}

// ---------------------------------------------------------------------------
// Defaulted here (not in the header) so TUniquePtr<FTelemetryRing> sees the
// complete type for its deleter.
UMotionLinkSubsystem::UMotionLinkSubsystem() = default;
UMotionLinkSubsystem::~UMotionLinkSubsystem() = default;

void UMotionLinkSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	check(IsInGameThread());

	// Startup-only config, read on the game thread and handed to the worker.
	const uint32 Ip   = ParseIpHostOrder(CVarControllerIp.GetValueOnGameThread());
	const uint16 TxP  = (uint16)CVarTxPort.GetValueOnGameThread();
	const uint16 RxP  = (uint16)CVarRxPort.GetValueOnGameThread();
	const int32  Core = CVarCoreAffinity.GetValueOnGameThread();
	const uint64 Mask = (Core >= 0) ? (1ull << Core) : 0ull;

	// Create the SPSC ring before the worker so the consumer pointer is valid
	// for the thread's whole lifetime.
	Ring = MakeUnique<FTelemetryRing>();

	Worker = new FMotionWorker(&Telemetry, &Controls, Ring.Get(), Ip, TxP, RxP, Mask);
	Worker->StartThread();

	TickHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateUObject(this, &UMotionLinkSubsystem::Tick));

	UE_LOG(LogTemp, Log, TEXT("MotionLink: worker started -> %d.%d.%d.%d:%u (rx :%u), affinity core %d"),
		(Ip >> 24) & 0xFF, (Ip >> 16) & 0xFF, (Ip >> 8) & 0xFF, Ip & 0xFF, TxP, RxP, Core);
}

void UMotionLinkSubsystem::Deinitialize()
{
	check(IsInGameThread());

	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	if (Worker)
	{
		Worker->StopThread(); // joins the thread before we free shared state
		delete Worker;
		Worker = nullptr;
	}
	Ring.Reset(); // safe now: consumer thread has joined

	Super::Deinitialize();
}

bool UMotionLinkSubsystem::Tick(float DeltaSeconds)
{
	check(IsInGameThread());

	// Push live CVars -> worker controls (release; worker reads with acquire).
	Controls.Enabled.store(CVarEnabled.GetValueOnGameThread() != 0 ? 1u : 0u, std::memory_order_release);
	Controls.SineFreqMilliHz.store((uint32)(CVarSineFreq.GetValueOnGameThread() * 1000.0f), std::memory_order_release);
	Controls.SineAmpMicroM.store((uint32)(CVarSineAmp.GetValueOnGameThread() * 1000000.0f), std::memory_order_release);
	Controls.SourceMode.store((uint32)FMath::Max(0, CVarSource.GetValueOnGameThread()), std::memory_order_release);
	Controls.CorrectMs.store((uint32)FMath::Max(1, CVarCorrectMs.GetValueOnGameThread()), std::memory_order_release);

	Controls.CueEnabled.store(CVarCueEnabled.GetValueOnGameThread() != 0 ? 1u : 0u, std::memory_order_release);
	Controls.TransHpfMilliHz.store((uint32)FMath::Max(0.0f, CVarTransHpf.GetValueOnGameThread() * 1000.0f), std::memory_order_release);
	Controls.RotLpfMilliHz.store((uint32)FMath::Max(0.0f, CVarRotLpf.GetValueOnGameThread() * 1000.0f), std::memory_order_release);
	Controls.LimitTransMicroM.store((uint32)FMath::Max(1.0f, CVarLimitTrans.GetValueOnGameThread() * 1e6f), std::memory_order_release);
	Controls.LimitRotMicroRad.store((uint32)FMath::Max(1.0f, CVarLimitRot.GetValueOnGameThread() * 1e6f), std::memory_order_release);
	Controls.LimitVelMicro.store((uint32)FMath::Max(1.0f, CVarLimitVel.GetValueOnGameThread() * 1e6f), std::memory_order_release);
	Controls.LimitJerkMilli.store((uint32)FMath::Max(1.0f, CVarLimitJerk.GetValueOnGameThread() * 1e3f), std::memory_order_release);

	// Once-per-second measured stats line (for on-device numbers without the
	// Insights GUI). Off by default; enable with `motion.Stats 1`.
	if (CVarStats.GetValueOnGameThread() != 0)
	{
		static float StatsAccum = 0.0f;
		StatsAccum += DeltaSeconds;
		if (StatsAccum >= 1.0f)
		{
			StatsAccum = 0.0f;
			UE_LOG(LogTemp, Log,
				TEXT("MotionStats: %u Hz | jitter p50=%u p99=%u max=%u us | RTT=%u us | pipeLatency=%u us | ring=%u age=%u us | lim=%u"),
				Telemetry.TickHz.load(std::memory_order_acquire),
				Telemetry.JitterP50Us.load(std::memory_order_acquire),
				Telemetry.JitterP99Us.load(std::memory_order_acquire),
				Telemetry.JitterMaxUs.load(std::memory_order_acquire),
				Telemetry.RttUs.load(std::memory_order_acquire),
				Telemetry.PipeLatencyUs.load(std::memory_order_acquire),
				Telemetry.RingDepth.load(std::memory_order_acquire),
				Telemetry.SampleAgeUs.load(std::memory_order_acquire),
				Telemetry.LimiterActive.load(std::memory_order_acquire));
		}
	}

	// Read-only overlay.
	if (CVarOverlay.GetValueOnGameThread() != 0 && GEngine)
	{
		const uint32 Seq   = Telemetry.Seq.load(std::memory_order_acquire);
		const uint32 Hz    = Telemetry.TickHz.load(std::memory_order_acquire);
		const uint32 P99   = Telemetry.JitterP99Us.load(std::memory_order_acquire);
		const uint32 MaxUs = Telemetry.JitterMaxUs.load(std::memory_order_acquire);
		const uint32 Rtt   = Telemetry.RttUs.load(std::memory_order_acquire);
		const uint32 St    = Telemetry.State.load(std::memory_order_acquire);
		const uint32 Errs  = Telemetry.SendErrors.load(std::memory_order_acquire);
		const uint32 Depth = Telemetry.RingDepth.load(std::memory_order_acquire);
		const uint32 AgeUs = Telemetry.SampleAgeUs.load(std::memory_order_acquire);
		const uint32 Src   = Controls.SourceMode.load(std::memory_order_acquire);
		const uint32 Lim   = Telemetry.LimiterActive.load(std::memory_order_acquire);

		static const TCHAR* StateNames[] = { TEXT("INIT"), TEXT("ACTIVE"), TEXT("LIMITED"), TEXT("PARK"), TEXT("FAULT") };
		const TCHAR* StateStr = StateNames[St <= MOTION_STATE_FAULT ? St : 0];

		const FColor Color = (Hz >= 990 && Hz <= 1010) ? FColor::Green : FColor::Yellow;
		const FString Line = FString::Printf(
			TEXT("MotionLink  %4u Hz  jitter p99=%u us max=%u us  seq=%u  RTT=%u us  state=%s  src=%s ring=%u age=%u us  lim=%s  tx_err=%u"),
			Hz, P99, MaxUs, Seq, Rtt, StateStr,
			Src == 0 ? TEXT("sine") : TEXT("obs"), Depth, AgeUs, Lim ? TEXT("ON") : TEXT("off"), Errs);

		// Stable key so the line updates in place instead of stacking.
		GEngine->AddOnScreenDebugMessage(/*Key*/ 0x4D4C /*'ML'*/, 1.5f, Color, Line);
	}

	return true; // keep ticking
}
