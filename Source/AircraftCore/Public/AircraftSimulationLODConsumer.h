#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODConsumer.generated.h"

/** Small optional contract; the world subsystem never depends on concrete flight features. */
UINTERFACE(BlueprintType)
class AIRCRAFTCORE_API UAircraftSimulationLODConsumer : public UInterface
{
	GENERATED_BODY()
};

class AIRCRAFTCORE_API IAircraftSimulationLODConsumer
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	void ApplyAircraftSimulationBudget(const FAircraftSimulationBudget& Budget);
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) {}

	/** Return false when this feature does not publish a kinematic movement target. */
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	bool GetAircraftKinematicTarget(FAircraftKinematicTarget& OutTarget) const;
	virtual bool GetAircraftKinematicTarget_Implementation(FAircraftKinematicTarget& OutTarget) const
	{
		OutTarget = FAircraftKinematicTarget();
		return false;
	}
};
