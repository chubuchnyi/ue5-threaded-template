#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Containers/Ticker.h"
#include "Templates/UniquePtr.h"
#include "MotionLinkShared.h"
#include "TelemetryRing.h" // complete type: TUniquePtr member deleter needs it
#include "MotionLinkSubsystem.generated.h"

class FMotionWorker;

// Owns the FMotionWorker thread for the lifetime of the game instance, feeds it
// live CVar values, and draws the read-only debug overlay on the game thread.
//
// This is the only object that touches both threads' state: it is created and
// torn down on the game thread, and it only ever *reads* worker telemetry and
// *writes* worker controls through the atomic blocks in MotionLinkShared.h.
UCLASS()
class MOTIONLINK_API UMotionLinkSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:
	UMotionLinkSubsystem();
	// Out-of-line dtor: TUniquePtr<FTelemetryRing> needs the complete type,
	// which only the .cpp sees (pimpl pattern).
	virtual ~UMotionLinkSubsystem() override;

	// USubsystem
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	// Producer (UTelemetrySourceComponent) resolves the shared ring here.
	FTelemetryRing* GetTelemetryRing() const { return Ring.Get(); }

private:
	// Per-frame game-thread callback: pushes CVars -> controls and draws the
	// overlay. Returns true to stay registered.
	bool Tick(float DeltaSeconds);

	// Shared state blocks. Owned here; the worker holds raw pointers to them
	// and never outlives this subsystem (StopThread joins in Deinitialize).
	FMotionLinkTelemetry Telemetry;
	FMotionLinkControls  Controls;

	// SPSC ring: physics thread (producer) -> worker (consumer). Owned here;
	// created before the worker, freed after the worker is joined.
	TUniquePtr<FTelemetryRing> Ring;

	FMotionWorker* Worker = nullptr;
	FTSTicker::FDelegateHandle TickHandle;
};
