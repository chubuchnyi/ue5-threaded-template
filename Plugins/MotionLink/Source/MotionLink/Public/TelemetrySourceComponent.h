#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Physics/PhysicsInterfaceDeclares.h" // FPhysicsActorHandle
#include "TelemetrySourceComponent.generated.h"

class FTelemetryRing;

// Snapshots the owning actor's physics body on the async physics tick and
// pushes a POD sample into the SPSC ring for the MotionWorker to consume.
//
// The producer end of the ring. Runs on the physics thread; it touches no
// UObject state there — only the cached Chaos particle handle and plain math.
UCLASS(ClassGroup=(MotionLink), meta=(BlueprintSpawnableComponent))
class MOTIONLINK_API UTelemetrySourceComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UTelemetrySourceComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Physics thread. Reads the cached particle handle and publishes a sample.
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;

private:
	// Ring is owned by the game-instance subsystem and outlives this component
	// (the subsystem joins the worker before freeing it). Raw pointer, resolved
	// on BeginPlay.
	FTelemetryRing* Ring = nullptr;

	// Chaos physics proxy for the owner's simulated body, captured on the game
	// thread in BeginPlay so the physics-thread tick never dereferences the
	// UObject-owned FBodyInstance.
	FPhysicsActorHandle PhysHandle = nullptr;
};
