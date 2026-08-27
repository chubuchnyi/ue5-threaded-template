#include "MotionWorker.h"

#include "HAL/RunnableThread.h"
#include "HAL/PlatformAffinity.h"

#include <cmath>

// The worker is Win64-only (see MotionLink.uplugin WhitelistPlatforms). Wrap
// the raw Windows/Winsock includes in UE's platform-type guards so their
// typedefs don't leak into engine code.
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <timeapi.h> // timeBeginPeriod / timeEndPeriod (winmm)
#include "Windows/HideWindowsPlatformTypes.h"
#endif

// Present since Windows 10 1803; define defensively in case of an older SDK.
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

static const double kTwoPi = 6.28318530717958647692;

// ---------------------------------------------------------------------------
FMotionWorker::FMotionWorker(FMotionLinkTelemetry* InTelemetry,
                             FMotionLinkControls* InControls,
                             uint32 InControllerIp,
                             uint16 InTxPort,
                             uint16 InRxPort,
                             uint64 InAffinityMask)
	: Telemetry(InTelemetry)
	, Controls(InControls)
	, ControllerIp(InControllerIp)
	, TxPort(InTxPort)
	, RxPort(InRxPort)
	, AffinityMask(InAffinityMask)
{
	FMemory::Memzero(&TxFrame, sizeof(TxFrame));
	FMemory::Memzero(&RxFrame, sizeof(RxFrame));
}

FMotionWorker::~FMotionWorker()
{
	// Thread must already be joined via StopThread() before destruction.
	check(Thread == nullptr);
}

uint64 FMotionWorker::NowNs() const
{
#if PLATFORM_WINDOWS
	LARGE_INTEGER c;
	QueryPerformanceCounter(&c);
	const uint64 counts = (uint64)c.QuadPart;
	const uint64 secs = counts / QpcFreq;
	const uint64 rem  = counts % QpcFreq;
	return secs * 1000000000ull + (rem * 1000000000ull) / QpcFreq;
#else
	return 0;
#endif
}

// ---------------------------------------------------------------------------
bool FMotionWorker::Init()
{
#if PLATFORM_WINDOWS
	LARGE_INTEGER f;
	QueryPerformanceFrequency(&f);
	QpcFreq = (uint64)f.QuadPart;

	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		return false;
	}

	// TX socket: unbound, non-blocking. UDP sendto on loopback effectively
	// never blocks, and non-blocking guarantees the hot path can't stall.
	TxSock = (uintptr_t)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if ((SOCKET)TxSock == INVALID_SOCKET) { return false; }

	// RX socket: bound to the feedback port, non-blocking (drained each tick).
	RxSock = (uintptr_t)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if ((SOCKET)RxSock == INVALID_SOCKET) { return false; }

	sockaddr_in rxaddr;
	ZeroMemory(&rxaddr, sizeof(rxaddr));
	rxaddr.sin_family = AF_INET;
	rxaddr.sin_port = htons(RxPort);
	rxaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (bind((SOCKET)RxSock, (sockaddr*)&rxaddr, sizeof(rxaddr)) == SOCKET_ERROR)
	{
		return false;
	}

	u_long nb = 1;
	ioctlsocket((SOCKET)TxSock, FIONBIO, &nb);
	ioctlsocket((SOCKET)RxSock, FIONBIO, &nb);

	// High-resolution periodic 1 ms timer.
	HANDLE timer = CreateWaitableTimerExW(
		nullptr, nullptr,
		CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
		TIMER_ALL_ACCESS);
	if (timer == nullptr)
	{
		// Fall back to a normal waitable timer if the high-res flag is refused.
		timer = CreateWaitableTimerExW(nullptr, nullptr, 0, TIMER_ALL_ACCESS);
		if (timer == nullptr) { return false; }
	}
	TimerHandle = timer;

	// Raise the process timer resolution to 1 ms. Even a high-resolution
	// waitable timer's periodic path can be quantized to the default ~15.6 ms
	// scheduler tick; the timer is re-armed as a one-shot per iteration in
	// Run(), and this keeps the WaitForSingleObject wakeup granularity tight.
	timeBeginPeriod(1);

	return true;
#else
	return false;
#endif
}

uint32 FMotionWorker::Run()
{
#if PLATFORM_WINDOWS
	const uint64 t0 = NowNs();
	uint64 lastTickNs = t0;
	uint64 lastStatNs = t0;
	uint64 nextNs = t0;
	uint32 tickCount = 0;
	uint32 maxUs = 0;

	while (!bStop.load(std::memory_order_acquire))
	{
		// Arm the high-resolution timer for the next 1 ms deadline. Targeting an
		// advancing absolute deadline (nextNs) rather than a fixed relative
		// period makes the loop self-correct drift and hold a true 1 kHz
		// average. The 1 s wait timeout is a safety net so bStop is re-checked
		// even if the timer misbehaves.
		nextNs += 1000000ull; // +1 ms
		const uint64 beforeWait = NowNs();
		if (nextNs > beforeWait)
		{
			LARGE_INTEGER due;
			due.QuadPart = -(LONGLONG)((nextNs - beforeWait) / 100); // 100 ns units, relative
			SetWaitableTimer((HANDLE)TimerHandle, &due, 0 /*one-shot*/, nullptr, nullptr, false);
			WaitForSingleObject((HANDLE)TimerHandle, 1000);
		}
		// else: behind schedule; fall through and run immediately to catch up.
		if (bStop.load(std::memory_order_acquire)) { break; }

		const uint64 now = NowNs();

		// --- tick-interval jitter bookkeeping ---
		uint64 dtNs = now - lastTickNs;
		lastTickNs = now;
		uint32 dtUs = (uint32)(dtNs / 1000);
		Hist[dtUs < (uint32)kHistBuckets ? dtUs : (kHistBuckets - 1)]++;
		if (dtUs > maxUs) { maxUs = dtUs; }
		++tickCount;

		// --- read live controls (acquire) ---
		const bool enabled = Controls->Enabled.load(std::memory_order_acquire) != 0;
		const double freq = Controls->SineFreqMilliHz.load(std::memory_order_acquire) / 1000.0;
		const double amp  = Controls->SineAmpMicroM.load(std::memory_order_acquire) / 1000000.0;

		// --- emit one setpoint ---
		if (enabled)
		{
			const double t = (now - t0) / 1e9;
			const double phase = kTwoPi * freq * t;
			const float s = (float)(amp * std::sin(phase));
			const float v = (float)(amp * kTwoPi * freq * std::cos(phase));

			for (int d = 0; d < MOTION_DOF; ++d) { TxFrame.pose[d] = 0.0f; TxFrame.vel[d] = 0.0f; }
			TxFrame.pose[0] = s; // surge axis carries the synthetic sine
			TxFrame.vel[0]  = v;
			TxFrame.seq = SeqCounter++;
			TxFrame.t_tx_ns = now;
			TxFrame.flags = MOTION_FLAG_NONE;
			motion_setpoint_finalize(&TxFrame);

			sockaddr_in to;
			ZeroMemory(&to, sizeof(to));
			to.sin_family = AF_INET;
			to.sin_port = htons(TxPort);
			to.sin_addr.s_addr = htonl(ControllerIp);
			int sent = sendto((SOCKET)TxSock, (const char*)&TxFrame, sizeof(TxFrame), 0,
			                  (sockaddr*)&to, sizeof(to));
			if (sent == SOCKET_ERROR)
			{
				Telemetry->SendErrors.fetch_add(1, std::memory_order_relaxed);
			}
			else
			{
				Telemetry->Seq.store(TxFrame.seq, std::memory_order_release);
			}
		}

		// --- drain any feedback (non-blocking) ---
		for (;;)
		{
			int n = recvfrom((SOCKET)RxSock, (char*)&RxFrame, sizeof(RxFrame), 0, nullptr, nullptr);
			if (n <= 0) { break; } // WSAEWOULDBLOCK -> -1; nothing more to read
			if (n != (int)sizeof(FeedbackFrame)) { continue; }
			if (!motion_feedback_valid(&RxFrame)) { continue; }

			// RTT is valid here: t_tx_ns was stamped by *this* process, so both
			// ends of the subtraction are on the same QPC timeline.
			const uint64 rtt = NowNs() - RxFrame.t_tx_ns;
			Telemetry->RttUs.store((uint32)(rtt / 1000), std::memory_order_release);
			Telemetry->State.store(RxFrame.state, std::memory_order_release);
			Telemetry->FeedbackSeq.store(RxFrame.seq, std::memory_order_release);
		}

		// --- publish stats once per second ---
		if (now - lastStatNs >= 1000000000ull)
		{
			Telemetry->TickHz.store(tickCount, std::memory_order_release);
			Telemetry->JitterMaxUs.store(maxUs, std::memory_order_release);

			// p99 from the histogram.
			const uint32 threshold = (uint32)(0.99 * tickCount);
			uint32 cum = 0;
			uint32 p99 = 0;
			for (int b = 0; b < kHistBuckets; ++b)
			{
				cum += Hist[b];
				if (p99 == 0 && cum >= threshold) { p99 = (uint32)b; }
				Hist[b] = 0; // reset the window as we scan
			}
			Telemetry->JitterP99Us.store(p99, std::memory_order_release);

			tickCount = 0;
			maxUs = 0;
			lastStatNs += 1000000000ull;
		}
	}
	return 0;
#else
	return 0;
#endif
}

void FMotionWorker::Stop()
{
	bStop.store(true, std::memory_order_release);
}

void FMotionWorker::Exit()
{
#if PLATFORM_WINDOWS
	timeEndPeriod(1); // balance the timeBeginPeriod(1) from Init()
	if (TimerHandle) { CancelWaitableTimer((HANDLE)TimerHandle); CloseHandle((HANDLE)TimerHandle); TimerHandle = nullptr; }
	if ((SOCKET)TxSock != INVALID_SOCKET) { closesocket((SOCKET)TxSock); TxSock = ~(uintptr_t)0; }
	if ((SOCKET)RxSock != INVALID_SOCKET) { closesocket((SOCKET)RxSock); RxSock = ~(uintptr_t)0; }
	WSACleanup();
#endif
}

// ---------------------------------------------------------------------------
void FMotionWorker::StartThread()
{
	check(Thread == nullptr);
	const uint64 mask = (AffinityMask != 0)
		? AffinityMask
		: FPlatformAffinity::GetNoAffinityMask();
	Thread = FRunnableThread::Create(
		this, TEXT("MotionWorker"), 128 * 1024, TPri_TimeCritical, mask);
}

void FMotionWorker::StopThread()
{
	if (Thread)
	{
		Thread->Kill(true); // calls Stop(), waits for Run() to return, then Exit()
		delete Thread;
		Thread = nullptr;
	}
}
