#pragma once

#include "CoreMinimal.h"

#include "AircraftSimulationLODTypes.generated.h"

/** Aircraft simulation fidelity, ordered from most to least expensive. */
UENUM(BlueprintType)
enum class EAircraftSimulationTier : uint8
{
	FullPhysics UMETA(DisplayName = "完整物理"),
	ReducedPhysics UMETA(DisplayName = "简化物理"),
	Kinematic UMETA(DisplayName = "运动学"),
	Dormant UMETA(DisplayName = "休眠")
};

UENUM(BlueprintType)
enum class EAircraftSimulationCollisionMode : uint8
{
	Disabled UMETA(DisplayName = "禁用碰撞"),
	QueryOnly UMETA(DisplayName = "仅查询"),
	QueryAndPhysics UMETA(DisplayName = "查询与物理碰撞")
};

/** Per-tier policy authored in UAircraftSimulationLODProfileAsset. */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftSimulationTierSettings
{
	GENERATED_BODY()

	/** Nominal upper distance from the nearest player. Dormant ignores this value. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (
		DisplayName = "最大生效距离",
		ToolTip = "该模拟层级距离最近玩家的最大生效距离。休眠层级不使用此参数。",
		ClampMin = "0.0", Units = "cm"))
	float MaxDistanceCm = 6000.0f;

	/** Slow guidance/gameplay cadence. Zero means every game frame. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "慢速逻辑更新间隔", ClampMin = "0.0", Units = "s"))
	float SlowLogicIntervalSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "启用物理模拟"))
	bool bEnablePhysics = true;

	/** Enables manager-driven transform integration from a generic kinematic target provider. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "启用运动学移动"))
	bool bEnableKinematicMovement = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "碰撞模式"))
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;

	/** Advisory actor replication cadence on the authority. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "建议网络更新频率", ClampMin = "1.0", Units = "Hz"))
	float SuggestedNetUpdateFrequency = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "允许调试绘制"))
	bool bAllowDebugDraw = false;
};

/** Gameplay importance is intentionally independent from flight-controller and AI types. */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftSimulationImportance
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

	bool RequiresFullPhysics() const
	{
		return bPlayerControlled || bInCombat || bFiring || bRecentlyDamaged
			|| bRecoveringFromDamage || bHasExternalPhysicsConstraint || bMissionCritical || bMustRemainPhysical;
	}
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftSimulationSnapshot
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	float NearestPlayerDistanceCm = TNumericLimits<float>::Max();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance Importance;
};

/** Generic budget consumed by optional aircraft features. */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftSimulationBudget
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationTier Tier = EAircraftSimulationTier::FullPhysics;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bRunFlightController = true;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bRunSlowLogic = true;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bEnablePhysics = true;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bEnableKinematicMovement = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bIsNetworkProxy = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	float SlowLogicIntervalSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	float SuggestedNetUpdateFrequency = 30.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bAllowDebugDraw = false;
};

/** Optional target published by guidance and integrated by the LOD component in kinematic mode. */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAircraftKinematicTarget
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	FRotator RotationDegrees = FRotator::ZeroRotator;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation")
	bool bValid = false;
};
