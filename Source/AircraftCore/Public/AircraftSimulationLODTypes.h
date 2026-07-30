#pragma once

#include "CoreMinimal.h"

#include "AircraftSimulationLODTypes.generated.h"

/** The backend that realizes a shared aircraft motion target. */
UENUM(BlueprintType)
enum class EAircraftSimulationDriveMode : uint8
{
	None UMETA(DisplayName = "无驱动"),
	FlightController UMETA(DisplayName = "飞控驱动"),
	PhysicsConstraint UMETA(DisplayName = "物理约束驱动"),
	Kinematic UMETA(DisplayName = "运动学驱动")
};

UENUM(BlueprintType)
enum class EAircraftSimulationCollisionMode : uint8
{
	Disabled UMETA(DisplayName = "禁用碰撞"),
	QueryOnly UMETA(DisplayName = "仅查询"),
	QueryAndPhysics UMETA(DisplayName = "查询与物理碰撞")
};

/** One data-driven LOD entry. Array order is nearest/highest priority to farthest. */
USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAircraftSimulationLODSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "名称"))
	FName Name = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "驱动模式"))
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::None;

	/** Nominal upper distance from the nearest player. The final array entry ignores this value. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (
		DisplayName = "最大生效距离",
		ToolTip = "该LOD距离最近玩家的最大生效距离。数组最后一个LOD是无限距离兜底，不使用此参数。",
		ClampMin = "0.0", Units = "cm"))
	float MaxDistanceCm = 6000.0f;

	/** Slow guidance/gameplay cadence. Zero means every game frame. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "运行慢速逻辑"))
	bool bRunSlowLogic = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "慢速逻辑更新间隔", ClampMin = "0.0", Units = "s"))
	float SlowLogicIntervalSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "碰撞模式"))
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;

	/** Advisory actor replication cadence on the authority. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "建议网络更新频率", ClampMin = "1.0", Units = "Hz"))
	float SuggestedNetUpdateFrequency = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "允许调试绘制"))
	bool bAllowDebugDraw = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "启用网络休眠"))
	bool bEnableNetworkDormancy = false;
};

/** An exact temporary drive request published by an active motion source. */
USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAircraftSimulationDriveOverride
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	int32 Priority = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bValid = false;
};

/** Gameplay importance is intentionally independent from flight-controller and AI types. */
USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAircraftSimulationImportance
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bPlayerControlled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bInCombat = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bFiring = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bRecentlyDamaged = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bRecoveringFromDamage = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	/** External gameplay constraint (payload, cable, world joint). Internal body/rotor rig constraints do not set this. */
	bool bHasExternalPhysicsConstraint = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bMissionCritical = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bMustRemainPhysical = false;

	bool RequiresHighestPriorityLOD() const
	{
		return bPlayerControlled || bInCombat || bFiring || bRecentlyDamaged
			|| bRecoveringFromDamage || bHasExternalPhysicsConstraint || bMissionCritical || bMustRemainPhysical;
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAircraftSimulationSnapshot
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	float NearestPlayerDistanceCm = TNumericLimits<float>::Max();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance Importance;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FAircraftSimulationDriveOverride DriveOverride;
};

/** Generic budget consumed by optional aircraft features. */
USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAircraftSimulationBudget
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	int32 LODIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::None;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bRunSlowLogic = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bEnablePhysics = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bIsNetworkProxy = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	float SlowLogicIntervalSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	float SuggestedNetUpdateFrequency = 30.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::Disabled;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bAllowDebugDraw = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bEnableNetworkDormancy = false;
};

/** Shared target published by guidance, Root Motion, animation, or gameplay. */
USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAircraftMotionTarget
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	FRotator RotationDegrees = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	FVector AngularVelocityWorldDegPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	int32 Priority = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bValid = false;
};
