#include "MotionLinkSpawnSubsystem.h"
#include "MotionTestActor.h"

#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<int32> CVarTestSpawn(
	TEXT("motion.Test.Spawn"), 1,
	TEXT("Auto-spawn a physics test actor on world begin play (1) or not (0)."), ECVF_Default);

bool UMotionLinkSpawnSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UMotionLinkSpawnSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	if (CVarTestSpawn.GetValueOnGameThread() == 0)
	{
		return;
	}

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AMotionTestActor* Actor = InWorld.SpawnActor<AMotionTestActor>(
		AMotionTestActor::StaticClass(), FTransform(FVector(0.0, 0.0, 100.0)), Params);

	UE_LOG(LogTemp, Log, TEXT("MotionLink: test actor %s"),
		Actor ? TEXT("spawned") : TEXT("FAILED to spawn"));
}
