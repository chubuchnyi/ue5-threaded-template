#include "TelemetrySourceComponent.h"

#include "MotionLinkSubsystem.h"
#include "TelemetryRing.h"
#include "MotionTime.h"

#include "ProfilingDebugging/CpuProfilerTrace.h"

#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Chaos/ParticleHandle.h"
#include "Engine/World.h"
#include "Engine/GameInstance.h"

UTelemetrySourceComponent::UTelemetrySourceComponent()
{
	PrimaryComponentTick.bCanEverTick = false; // we use the async physics tick
	// The Chaos async-tick dispatch asserts the registered component IsActive();
	// auto-activate so it is active by BeginPlay.
	bAutoActivate = true;
}

void UTelemetrySourceComponent::BeginPlay()
{
	Super::BeginPlay();
	check(IsInGameThread());

	// Resolve the shared ring from the game-instance subsystem.
	if (const UWorld* World = GetWorld())
	{
		if (UGameInstance* GI = World->GetGameInstance())
		{
			if (UMotionLinkSubsystem* Sub = GI->GetSubsystem<UMotionLinkSubsystem>())
			{
				Ring = Sub->GetTelemetryRing();
			}
		}
	}

	// Cache the physics proxy of the owner's simulated root body.
	if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(GetOwner() ? GetOwner()->GetRootComponent() : nullptr))
	{
		if (FBodyInstance* Body = Prim->GetBodyInstance())
		{
			PhysHandle = Body->ActorHandle;
		}
	}

	if (PhysHandle && Ring)
	{
		// Must be active before enabling the async tick (Chaos asserts on it).
		if (!IsActive())
		{
			Activate(true);
		}
		// Start receiving AsyncPhysicsTickComponent callbacks on the physics
		// thread. Requires bTickPhysicsAsync in project settings.
		SetAsyncPhysicsTickEnabled(true);

		UE_LOG(LogTemp, Log, TEXT("TelemetrySource: async physics sampling enabled (bTickPhysicsAsync=%d, dt=%.4fs)"),
			UPhysicsSettings::Get()->bTickPhysicsAsync ? 1 : 0,
			UPhysicsSettings::Get()->AsyncFixedTimeStepSize);
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TelemetrySource: no %s; async tick not enabled"),
			!PhysHandle ? TEXT("simulated body") : TEXT("telemetry ring"));
	}
}

void UTelemetrySourceComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	check(IsInGameThread());
	SetAsyncPhysicsTickEnabled(false);
	PhysHandle = nullptr;
	Ring = nullptr;
	Super::EndPlay(EndPlayReason);
}

void UTelemetrySourceComponent::AsyncPhysicsTickComponent(float /*DeltaTime*/, float SimTime)
{
	// Physics thread. No UObject access here.
	TRACE_CPUPROFILER_EVENT_SCOPE(MotionTelemetry_PhysTick);
	if (!PhysHandle || !Ring)
	{
		return;
	}

	Chaos::FRigidBodyHandle_Internal* Rigid = PhysHandle->GetPhysicsThreadAPI();
	if (!Rigid)
	{
		return;
	}

	// Chaos is in cm / rad; the wire protocol is in m / rad.
	const FVector X = Rigid->X();
	const FVector V = Rigid->V();
	const FQuat   Q = Rigid->R();
	const FVector Wv = Rigid->W();
	const FRotator Rot = Q.Rotator();

	constexpr float CmToM = 0.01f;
	constexpr float DegToRad = 3.14159265358979323846f / 180.0f;

	FTelemetrySample s;
	// Timestamp with the SOLVER's simulation time, not wall-clock. Under an FPS
	// drop the async substeps run in a burst (many within ~1 ms of wall time),
	// so wall-clock deltas collapse to ~0 and the observer's velocity-slope
	// acceleration estimate would explode. SimTime advances a true fixed dt per
	// substep regardless of when the burst executes.
	s.t_phys_ns = (uint64_t)((double)SimTime * 1.0e9);
	s.t_wall_ns = MotionNowNs(); // wall clock, for phys-tick -> send latency
	s.pose[0] = (float)X.X * CmToM;
	s.pose[1] = (float)X.Y * CmToM;
	s.pose[2] = (float)X.Z * CmToM;
	s.pose[3] = (float)Rot.Roll  * DegToRad;
	s.pose[4] = (float)Rot.Pitch * DegToRad;
	s.pose[5] = (float)Rot.Yaw   * DegToRad;
	s.vel[0] = (float)V.X * CmToM;
	s.vel[1] = (float)V.Y * CmToM;
	s.vel[2] = (float)V.Z * CmToM;
	s.vel[3] = (float)Wv.X;
	s.vel[4] = (float)Wv.Y;
	s.vel[5] = (float)Wv.Z;

	Ring->Push(s); // drops on full; the observer tolerates gaps
}
