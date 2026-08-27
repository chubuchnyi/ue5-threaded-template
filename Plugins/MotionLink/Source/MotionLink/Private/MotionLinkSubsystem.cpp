#include "MotionLinkSubsystem.h"
#include "MotionWorker.h"
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

	Worker = new FMotionWorker(&Telemetry, &Controls, Ip, TxP, RxP, Mask);
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

	Super::Deinitialize();
}

bool UMotionLinkSubsystem::Tick(float /*DeltaSeconds*/)
{
	check(IsInGameThread());

	// Push live CVars -> worker controls (release; worker reads with acquire).
	Controls.Enabled.store(CVarEnabled.GetValueOnGameThread() != 0 ? 1u : 0u, std::memory_order_release);
	Controls.SineFreqMilliHz.store((uint32)(CVarSineFreq.GetValueOnGameThread() * 1000.0f), std::memory_order_release);
	Controls.SineAmpMicroM.store((uint32)(CVarSineAmp.GetValueOnGameThread() * 1000000.0f), std::memory_order_release);

	// Read-only overlay.
	if (CVarOverlay.GetValueOnGameThread() != 0 && GEngine)
	{
		const uint32 Seq   = Telemetry.Seq.load(std::memory_order_acquire);
		const uint32 Hz    = Telemetry.TickHz.load(std::memory_order_acquire);
		const uint32 P99   = Telemetry.JitterP99Us.load(std::memory_order_acquire);
		const uint32 MaxUs = Telemetry.JitterMaxUs.load(std::memory_order_acquire);
		const uint32 Rtt   = Telemetry.RttUs.load(std::memory_order_acquire);
		const uint32 St    = Telemetry.State.load(std::memory_order_acquire);
		const uint32 FbSeq = Telemetry.FeedbackSeq.load(std::memory_order_acquire);
		const uint32 Errs  = Telemetry.SendErrors.load(std::memory_order_acquire);

		static const TCHAR* StateNames[] = { TEXT("INIT"), TEXT("ACTIVE"), TEXT("LIMITED"), TEXT("PARK"), TEXT("FAULT") };
		const TCHAR* StateStr = StateNames[St <= MOTION_STATE_FAULT ? St : 0];

		const FColor Color = (Hz >= 990 && Hz <= 1010) ? FColor::Green : FColor::Yellow;
		const FString Line = FString::Printf(
			TEXT("MotionLink  %4u Hz  jitter p99=%u us max=%u us  seq=%u  RTT=%u us  fb_seq=%u  state=%s  tx_err=%u"),
			Hz, P99, MaxUs, Seq, Rtt, FbSeq, StateStr, Errs);

		// Stable key so the line updates in place instead of stacking.
		GEngine->AddOnScreenDebugMessage(/*Key*/ 0x4D4C /*'ML'*/, 1.5f, Color, Line);
	}

	return true; // keep ticking
}
