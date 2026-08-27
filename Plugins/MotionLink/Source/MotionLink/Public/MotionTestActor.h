#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MotionTestActor.generated.h"

class UStaticMeshComponent;
class UTelemetrySourceComponent;

// A self-contained physics test body so Stage 3 needs no hand-authored map
// assets. A simulated cube (gravity off) is servo-driven toward a moving sine
// target, producing bounded, continuously non-zero accelerations. Carries a
// UTelemetrySourceComponent that publishes its physics state to the ring.
//
// PROTOTYPE: stands in for a real controllable rig / Chaos Vehicle. Drag it
// into any level, or let UMotionLinkSpawnSubsystem auto-spawn it.
UCLASS()
class MOTIONLINK_API AMotionTestActor : public AActor
{
	GENERATED_BODY()

public:
	AMotionTestActor();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* Mesh = nullptr;

	UPROPERTY(VisibleAnywhere)
	UTelemetrySourceComponent* Telemetry = nullptr;

	double Elapsed = 0.0;
	FVector Home = FVector::ZeroVector;
};
