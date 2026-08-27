#pragma once

#include "HAL/Runnable.h"
#include "MotionLinkShared.h"
#include "motion_protocol.h"

#include <atomic>
#include <cstdint>

class FRunnableThread;

// The 1 kHz motion loop, on its own TPri_TimeCritical thread pinned (optionally)
// to a dedicated core.
//
// HOT PATH (Run() and everything it calls) obeys the five prohibitions from
// CLAUDE.md: no allocation after Init(), no locks, no blocking calls
// (files/UE_LOG/blocking sockets), no exceptions, no UObject/FString/
// TSharedPtr. Only atomics, POD, and non-blocking UDP.
//
// The worker never touches the shared state's producer side except its own:
// it writes Telemetry (release) and reads Controls (acquire).
class FMotionWorker : public FRunnable
{
public:
	// Ip/ports/affinity are captured on the game thread and are immutable for
	// the worker's lifetime. ControllerIp is host byte order.
	FMotionWorker(FMotionLinkTelemetry* InTelemetry,
	              FMotionLinkControls* InControls,
	              uint32 InControllerIp,
	              uint16 InTxPort,
	              uint16 InRxPort,
	              uint64 InAffinityMask);
	virtual ~FMotionWorker();

	// FRunnable
	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Stop() override;
	virtual void Exit() override;

	// Game-thread lifecycle helpers.
	void StartThread();
	void StopThread(); // signals Stop() and blocks until the thread joins

private:
	// --- immutable config ---
	FMotionLinkTelemetry* Telemetry = nullptr;
	FMotionLinkControls*  Controls = nullptr;
	uint32 ControllerIp = 0;
	uint16 TxPort = 0;
	uint16 RxPort = 0;
	uint64 AffinityMask = 0;

	// --- thread + stop flag ---
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bStop{false};

	// --- hot-path resources, all set up in Init(), released in Exit() ---
	// Opaque handles kept as integers so this header stays free of <windows.h>.
	uintptr_t TxSock = ~(uintptr_t)0; // INVALID_SOCKET
	uintptr_t RxSock = ~(uintptr_t)0;
	void*     TimerHandle = nullptr;  // HANDLE for the waitable timer
	uint64    QpcFreq = 0;

	// Preallocated wire buffers (no per-tick allocation).
	SetpointFrame TxFrame;
	FeedbackFrame RxFrame;

	uint32 SeqCounter = 0;

	// Tick-interval histogram in microseconds, one bucket per us. Cleared each
	// 1 s stats window. Preallocated; no allocation on the hot path.
	static const int kHistBuckets = 4096; // clamps intervals to < ~4.1 ms
	uint32 Hist[kHistBuckets] = {0};

	// small internal helpers (defined in the .cpp, hot-path safe)
	uint64 NowNs() const;
};
