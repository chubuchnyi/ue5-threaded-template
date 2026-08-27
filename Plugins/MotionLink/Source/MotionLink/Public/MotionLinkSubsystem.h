#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Containers/Ticker.h"
#include "MotionLinkShared.h"
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
	// USubsystem
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

private:
	// Per-frame game-thread callback: pushes CVars -> controls and draws the
	// overlay. Returns true to stay registered.
	bool Tick(float DeltaSeconds);

	// Shared state blocks. Owned here; the worker holds raw pointers to them
	// and never outlives this subsystem (StopThread joins in Deinitialize).
	FMotionLinkTelemetry Telemetry;
	FMotionLinkControls  Controls;

	FMotionWorker* Worker = nullptr;
	FTSTicker::FDelegateHandle TickHandle;
};
