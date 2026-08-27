#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "MotionLinkSpawnSubsystem.generated.h"

// Auto-spawns the physics test actor when a game world begins play, so the
// pipeline has a moving body to sample without any hand-authored map content.
// Gated by motion.Test.Spawn (default on). Game/PIE worlds only.
UCLASS()
class MOTIONLINK_API UMotionLinkSpawnSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
};
