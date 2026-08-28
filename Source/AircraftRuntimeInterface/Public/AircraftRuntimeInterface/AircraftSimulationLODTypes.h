// 模拟 LOD 的共享执行契约（驱动/碰撞模式与应用预算）。
//
// 注意：EAircraftSimulationDriveMode / EAircraftSimulationCollisionMode 原本定义在
// AircraftAssetEngine 的 AircraftSimulationModel.h，现上移到本契约层，
// 以便 AircraftRuntimeCommon 的选择组件与 AircraftAssetEngine 的执行组件共同消费而不产生依赖环。

#pragma once

#include "CoreMinimal.h"

#include "AircraftSimulationLODTypes.generated.h"

/** 当前 LOD 使用的运动驱动。 */
UENUM(BlueprintType)
enum class EAircraftSimulationDriveMode : uint8
{
	FlightController UMETA(DisplayName = "Flight Controller"),
	PhysicsConstraint UMETA(DisplayName = "Physics Constraint"),
	Kinematic UMETA(DisplayName = "Kinematic")
};

UENUM(BlueprintType)
enum class EAircraftSimulationCollisionMode : uint8
{
	Disabled UMETA(DisplayName = "Disabled"),
	QueryOnly UMETA(DisplayName = "Query Only"),
	QueryAndPhysics UMETA(DisplayName = "Query And Physics")
};

/** 可选飞机特性消费的通用预算。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationBudget
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	int32 LODIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bEnablePhysics = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bIsNetworkProxy = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::Disabled;

};

