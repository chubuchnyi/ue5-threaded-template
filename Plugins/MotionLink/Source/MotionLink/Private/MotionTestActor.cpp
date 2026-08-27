#include "MotionTestActor.h"
#include "TelemetrySourceComponent.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"
#include "HAL/IConsoleManager.h"

// Test-rig tuning. Separate motion.Test.* namespace from the pipeline CVars.
static TAutoConsoleVariable<float> CVarTestFreq(
	TEXT("motion.Test.Freq"), 0.5f, TEXT("Test body drive frequency, Hz."), ECVF_Default);
static TAutoConsoleVariable<float> CVarTestAmp(
	TEXT("motion.Test.Amp"), 0.3f, TEXT("Test body drive amplitude, metres."), ECVF_Default);
// Gains kept soft enough that the servo stays stable even when its force is
// only refreshed at a low game-tick rate (the FPS-drop test runs at 20 Hz):
// a stiff spring whose force is held stale for a whole 50 ms frame overshoots
// and diverges. K=60 -> ~1.2 Hz bandwidth, critically damped, K*T^2<<1 at 20 Hz.
static TAutoConsoleVariable<float> CVarTestStiffness(
	TEXT("motion.Test.Stiffness"), 60.0f, TEXT("Servo spring stiffness driving the body to its sine target."), ECVF_Default);
static TAutoConsoleVariable<float> CVarTestDamping(
	TEXT("motion.Test.Damping"), 16.0f, TEXT("Servo damping."), ECVF_Default);

AMotionTestActor::AMotionTestActor()
{
	PrimaryActorTick.bCanEverTick = true;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	SetRootComponent(Mesh);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Mesh->SetStaticMesh(CubeMesh.Object);
	}
	// Physics enabling is deferred to BeginPlay: doing it here runs during CDO
	// construction (GEngine not yet up) and logs a physical-material error.

	// Producer component: snaps this body's state on the physics thread.
	Telemetry = CreateDefaultSubobject<UTelemetrySourceComponent>(TEXT("Telemetry"));
}

void AMotionTestActor::BeginPlay()
{
	// Enable simulation BEFORE Super::BeginPlay(): Super dispatches the child
	// UTelemetrySourceComponent's BeginPlay, which caches the physics handle,
	// so the body must already be a simulated dynamic particle by then.
	Mesh->SetEnableGravity(false);
	Mesh->SetMassOverrideInKg(NAME_None, 1.0f, true);
	Mesh->SetSimulatePhysics(true);

	Home = GetActorLocation();
	Elapsed = 0.0;

	Super::BeginPlay();
}

void AMotionTestActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	Elapsed += DeltaSeconds;

	const float FreqHz = CVarTestFreq.GetValueOnGameThread();
	const float AmpCm  = CVarTestAmp.GetValueOnGameThread() * 100.0f; // m -> cm
	const float K      = CVarTestStiffness.GetValueOnGameThread();
	const float C      = CVarTestDamping.GetValueOnGameThread();

	const double W = 2.0 * PI * FreqHz;
	// Lissajous-ish target: full amplitude on X, half-rate on Y, small on Z, so
	// several DOF see motion.
	const FVector Target = Home + FVector(
		AmpCm * FMath::Sin(W * Elapsed),
		AmpCm * FMath::Sin(W * Elapsed * 0.5),
		0.25f * AmpCm * FMath::Sin(W * Elapsed * 2.0));

	const FVector X = Mesh->GetComponentLocation();
	const FVector V = Mesh->GetPhysicsLinearVelocity();

	// Critically-damped servo toward the moving target; mass is 1 kg so force
	// equals acceleration in cm/s^2.
	const FVector Force = K * (Target - X) - C * V;
	Mesh->AddForce(Force);
}
