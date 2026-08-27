#include "MotionWorker.h"
#include "TelemetryRing.h"
#include "MotionTime.h"

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
                             FTelemetryRing* InRing,
                             uint32 InControllerIp,
                             uint16 InTxPort,
                             uint16 InRxPort,
                             uint64 InAffinityMask)
	: Telemetry(InTelemetry)
	, Controls(InControls)
	, Ring(InRing)
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
	// Shared process-wide clock so worker timestamps and the physics thread's
	// sample timestamps (also MotionNowNs) share an epoch.
	return MotionNowNs();
}

// ---------------------------------------------------------------------------
bool FMotionWorker::Init()
{
#if PLATFORM_WINDOWS
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
		const uint32 sourceMode = Controls->SourceMode.load(std::memory_order_acquire);
		const double freq = Controls->SineFreqMilliHz.load(std::memory_order_acquire) / 1000.0;
		const double amp  = Controls->SineAmpMicroM.load(std::memory_order_acquire) / 1000000.0;

		// Integration step from the actual tick interval, clamped so a
		// scheduling stall can't inject a huge position jump.
		double dtSec = dtNs / 1e9;
		if (dtSec > 0.010) { dtSec = 0.010; }
		else if (dtSec <= 0.0) { dtSec = 0.001; }

		double outPose[MOTION_DOF] = {0};
		double outVel[MOTION_DOF]  = {0};

		if (sourceMode == 0)
		{
			// --- synthetic sine (Stage 2 behaviour, kept for A/B testing) ---
			const double t = (now - t0) / 1e9;
			const double phase = kTwoPi * freq * t;
			outPose[0] = amp * std::sin(phase);
			outVel[0]  = amp * kTwoPi * freq * std::cos(phase);
		}
		else if (Ring)
		{
			// --- const-accel observer over telemetry samples ---
			// Drain every queued sample; keep the newest as the correction
			// target and update the acceleration estimate from the velocity
			// slope between samples.
			FTelemetrySample smp;
			bool got = false;
			while (Ring->Pop(smp))
			{
				if (HaveMeas)
				{
					// dt is a SIM-time delta (see the source component). Guard
					// against a degenerate step and clamp the estimate so a bad
					// sample can never launch the const-accel extrapolation.
					const double dtm = (smp.t_phys_ns > LastMeasNs)
						? (smp.t_phys_ns - LastMeasNs) / 1e9 : 0.0;
					if (dtm > 1e-4)
					{
						const double kMaxAccel = 1.0e3; // m/s^2, sanity ceiling
						for (int d = 0; d < MOTION_DOF; ++d)
						{
							double a = ((double)smp.vel[d] - LastMeasVel[d]) / dtm;
							if (a >  kMaxAccel) a =  kMaxAccel;
							if (a < -kMaxAccel) a = -kMaxAccel;
							AccelHat[d] = a;
						}
					}
				}
				for (int d = 0; d < MOTION_DOF; ++d) { LastMeasVel[d] = smp.vel[d]; }
				LastMeasNs = smp.t_phys_ns;
				LastConsumeWallNs = now;
				HaveMeas = true;
				got = true;
			}

			if (got)
			{
				// Trust the measured velocity; smear the position residual over
				// CorrectMs ticks so a late sample never snaps the output.
				uint32 correctMs = Controls->CorrectMs.load(std::memory_order_acquire);
				if (correctMs < 1) { correctMs = 1; }
				CorrTicks = (int)correctMs; // ~1 tick per ms at 1 kHz
				for (int d = 0; d < MOTION_DOF; ++d)
				{
					VelHat[d] = LastMeasVel[d];
					const double resid = (double)smp.pose[d] - PoseHat[d];
					CorrPerTick[d] = resid / CorrTicks;
				}
			}

			// Staleness = wall time since a sample was last consumed. Grows
			// during an FPS drop while the observer extrapolates.
			if (LastConsumeWallNs != 0)
			{
				Telemetry->SampleAgeUs.store((uint32)((now - LastConsumeWallNs) / 1000),
					std::memory_order_release);
			}

			// Integrate const-accel every tick (this is what keeps the output
			// smooth when physics samples arrive late during an FPS drop), then
			// apply one slice of the smeared correction.
			for (int d = 0; d < MOTION_DOF; ++d)
			{
				VelHat[d]  += AccelHat[d] * dtSec;
				PoseHat[d] += VelHat[d] * dtSec + 0.5 * AccelHat[d] * dtSec * dtSec;
				if (CorrTicks > 0) { PoseHat[d] += CorrPerTick[d]; }
				outPose[d] = PoseHat[d];
				outVel[d]  = VelHat[d];
			}
			if (CorrTicks > 0) { --CorrTicks; }

			Telemetry->RingDepth.store(Ring->ApproxDepth(), std::memory_order_release);
		}

		// --- Stage 4: cueing skeleton + limiter -------------------------------
		// Order: washout (HPF translations / LPF tilt) -> workspace limiter
		// (single vector scale) -> rate + jerk limit. All coefficients live.
		bool limiterActive = false;
		const bool cueEnabled = Controls->CueEnabled.load(std::memory_order_acquire) != 0;
		if (cueEnabled)
		{
			const double kTwo = kTwoPi;
			const double fcTrans = Controls->TransHpfMilliHz.load(std::memory_order_acquire) / 1000.0;
			const double fcRot   = Controls->RotLpfMilliHz.load(std::memory_order_acquire) / 1000.0;
			// Exact first-order discretizations at the actual tick dt.
			const double aHpf   = std::exp(-kTwo * fcTrans * dtSec);       // pole, HPF
			const double aLpf   = 1.0 - std::exp(-kTwo * fcRot * dtSec);   // gain, LPF

			// Washout: translations (0..2) high-pass to bleed toward neutral;
			// rotations (3..5) low-pass for tilt coordination.
			for (int d = 0; d < 3; ++d)
			{
				const double x = outPose[d];
				const double y = aHpf * (HpfPrevOut[d] + x - HpfPrevIn[d]);
				HpfPrevIn[d] = x;
				HpfPrevOut[d] = y;
				outPose[d] = y;
			}
			for (int d = 3; d < MOTION_DOF; ++d)
			{
				LpfPrev[d] += aLpf * (outPose[d] - LpfPrev[d]);
				outPose[d] = LpfPrev[d];
			}

			// Workspace limiter: scale the WHOLE deviation vector by one factor
			// (no per-component clamp) when any channel exceeds its limit.
			const double limTrans = Controls->LimitTransMicroM.load(std::memory_order_acquire) / 1e6;
			const double limRot   = Controls->LimitRotMicroRad.load(std::memory_order_acquire) / 1e6;
			double ratio = 0.0;
			for (int d = 0; d < 3; ++d) { const double r = std::fabs(outPose[d]) / limTrans; if (r > ratio) ratio = r; }
			for (int d = 3; d < MOTION_DOF; ++d) { const double r = std::fabs(outPose[d]) / limRot; if (r > ratio) ratio = r; }
			if (ratio > 1.0)
			{
				const double s = 1.0 / ratio;
				for (int d = 0; d < MOTION_DOF; ++d) outPose[d] *= s;
				limiterActive = true;
			}

			// Rate (velocity) + jerk limit per channel.
			const double vMax = Controls->LimitVelMicro.load(std::memory_order_acquire) / 1e6;
			const double jMax = Controls->LimitJerkMilli.load(std::memory_order_acquire) / 1e3;
			const double dMax  = vMax * dtSec;      // max |Δpose| this tick
			const double dvMax = jMax * dtSec;      // max |Δvelocity| this tick
			for (int d = 0; d < MOTION_DOF; ++d)
			{
				double delta = outPose[d] - LimPrevPose[d];
				if (std::fabs(delta) > dMax) { delta = (delta < 0 ? -dMax : dMax); limiterActive = true; }
				double appliedVel = delta / dtSec;
				const double dv = appliedVel - LimPrevVel[d];
				if (std::fabs(dv) > dvMax) { appliedVel = LimPrevVel[d] + (dv < 0 ? -dvMax : dvMax); delta = appliedVel * dtSec; limiterActive = true; }
				const double newPose = LimPrevPose[d] + delta;
				LimPrevPose[d] = newPose;
				LimPrevVel[d]  = appliedVel;
				outPose[d] = newPose;
				outVel[d]  = appliedVel;
			}
		}
		else
		{
			// Bypass: keep filter/limiter state synced to the raw output so
			// re-enabling cueing doesn't produce a jump.
			for (int d = 0; d < MOTION_DOF; ++d)
			{
				HpfPrevIn[d] = outPose[d]; HpfPrevOut[d] = 0.0;
				LpfPrev[d] = outPose[d];
				LimPrevPose[d] = outPose[d]; LimPrevVel[d] = outVel[d];
			}
		}
		Telemetry->LimiterActive.store(limiterActive ? 1u : 0u, std::memory_order_release);

		// --- emit one setpoint ---
		if (enabled)
		{
			for (int d = 0; d < MOTION_DOF; ++d)
			{
				TxFrame.pose[d] = (float)outPose[d];
				TxFrame.vel[d]  = (float)outVel[d];
			}
			TxFrame.seq = SeqCounter++;
			TxFrame.t_tx_ns = now;
			TxFrame.flags = limiterActive ? MOTION_FLAG_LIMITER_ACTIVE : MOTION_FLAG_NONE;
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
