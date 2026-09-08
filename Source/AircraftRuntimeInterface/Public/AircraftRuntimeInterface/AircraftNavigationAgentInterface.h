#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftRuntimeInterface/AircraftNavigationGuidance.h"

#include "AircraftNavigationAgentInterface.generated.h"

UINTERFACE(meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTRUNTIMEINTERFACE_API UAircraftNavigationAgentInterface : public UInterface
{
	GENERATED_BODY()
};

/** Neutral Aircraft-side seam used by a navigation or avoidance module without reverse dependency. */
class AIRCRAFTRUNTIMEINTERFACE_API IAircraftNavigationAgentInterface
{
	GENERATED_BODY()

public:
	virtual bool GetAircraftNavigationAgentSnapshot(
		FAircraftNavigationAgentSnapshot& OutSnapshot) const = 0;
	virtual void SetAircraftNavigationGuidanceProvider(UObject* Provider) = 0;
	virtual FAircraftNavigationGuidanceStatus GetAircraftNavigationGuidanceStatus() const = 0;
};
