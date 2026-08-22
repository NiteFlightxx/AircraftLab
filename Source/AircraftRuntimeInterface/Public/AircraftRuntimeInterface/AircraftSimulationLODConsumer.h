// LOD 世界子系统与飞控组件之间的小契约，子系统永不依赖具体飞行特性。

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODConsumer.generated.h"

/** 小型可选契约；世界子系统从不依赖具体的飞行特性。由 UAircraftComponent 实现。 */
UINTERFACE(BlueprintType, Blueprintable)
class AIRCRAFTRUNTIMEINTERFACE_API UAircraftSimulationLODConsumer : public UInterface
{
	GENERATED_BODY()
};

class AIRCRAFTRUNTIMEINTERFACE_API IAircraftSimulationLODConsumer
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	void ApplyAircraftSimulationBudget(const FAircraftSimulationBudget& Budget);
	virtual void ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget) {}
};
