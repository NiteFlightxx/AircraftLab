#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftRuntimeInterface/AircraftNavigationGuidance.h"

#include "AircraftNavigationGuidanceProvider.generated.h"

UINTERFACE(meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTRUNTIMEINTERFACE_API UAircraftNavigationGuidanceProvider : public UInterface
{
	GENERATED_BODY()
};

/** External navigation owns planning and neighboring-agent queries; Aircraft only consumes guidance. */
class AIRCRAFTRUNTIMEINTERFACE_API IAircraftNavigationGuidanceProvider
{
	GENERATED_BODY()

public:
	virtual TSharedPtr<const FAircraftNavigationGuidance, ESPMode::ThreadSafe>
		GetAircraftNavigationGuidance(uint64& OutRevision) const = 0;
	virtual bool IsAircraftNavigationGuidanceActive() const = 0;
};
