#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"

#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

#include "AircraftMovementIntentProvider.generated.h"

UINTERFACE(meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTRUNTIMEINTERFACE_API UAircraftMovementIntentProvider : public UInterface
{
	GENERATED_BODY()
};

/** UObject/Actor 引用必须在实现方的游戏线程边界解析后再返回。 */
class AIRCRAFTRUNTIMEINTERFACE_API IAircraftMovementIntentProvider
{
	GENERATED_BODY()

public:
	virtual bool GetAircraftMovementIntent(
		FAircraftMovementIntent& OutIntent,
		FAircraftMovementIntentHandle& OutHandle,
		uint64& OutRevision) const = 0;
	virtual bool IsAircraftMovementIntentActive() const = 0;
};
