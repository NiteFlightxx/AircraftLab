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

	/** 当此特性不发布共享运动目标时返回 false。 */
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	bool GetAircraftMotionTarget(FAircraftMotionTarget& OutTarget) const;
	virtual bool GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const
	{
		OutTarget = FAircraftMotionTarget();
		return false;
	}

	/** 活跃运动源可请求一个精确的临时驱动后端。 */
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	FAircraftSimulationDriveOverride GetAircraftSimulationDriveOverride() const;
	virtual FAircraftSimulationDriveOverride GetAircraftSimulationDriveOverride_Implementation() const
	{
		return FAircraftSimulationDriveOverride();
	}
};

/** 让运动源可以同步刷新其驱动请求，而不依赖具体飞控模块。 */
UINTERFACE(BlueprintType, Blueprintable)
class AIRCRAFTRUNTIMEINTERFACE_API UAircraftSimulationLODController : public UInterface
{
	GENERATED_BODY()
};

class AIRCRAFTRUNTIMEINTERFACE_API IAircraftSimulationLODController
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Aircraft|Simulation")
	void RefreshAircraftSimulationDrive();
	virtual void RefreshAircraftSimulationDrive_Implementation() {}
};
