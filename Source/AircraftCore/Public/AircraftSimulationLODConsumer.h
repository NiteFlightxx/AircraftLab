#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODConsumer.generated.h"

/** Small optional contract; the world subsystem never depends on concrete flight features. */
UINTERFACE(BlueprintType, Blueprintable)
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

	/** Return false when this feature does not publish a shared movement target. */
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	bool GetAircraftMotionTarget(FAircraftMotionTarget& OutTarget) const;
	virtual bool GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const
	{
		OutTarget = FAircraftMotionTarget();
		return false;
	}

	/** Active motion sources can request an exact temporary backend. */
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	FAircraftSimulationDriveOverride GetAircraftSimulationDriveOverride() const;
	virtual FAircraftSimulationDriveOverride GetAircraftSimulationDriveOverride_Implementation() const
	{
		return FAircraftSimulationDriveOverride();
	}
};

/** Lets a motion source synchronously refresh its drive request without depending on AircraftLab. */
UINTERFACE(BlueprintType, Blueprintable)
class AIRCRAFTCORE_API UAircraftSimulationLODController : public UInterface
{
	GENERATED_BODY()
};

class AIRCRAFTCORE_API IAircraftSimulationLODController
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	void RefreshAircraftSimulationDrive();
	virtual void RefreshAircraftSimulationDrive_Implementation() {}
};
