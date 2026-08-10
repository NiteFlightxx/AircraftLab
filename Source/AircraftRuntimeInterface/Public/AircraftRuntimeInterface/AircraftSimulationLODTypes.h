// 对齐 ClothingSystemRuntimeInterface 的契约层职责。
// 对应 NxGame AircraftCore/Public/AircraftSimulationLODTypes.h：
// 模拟 LOD 的共享数据类型（驱动/碰撞模式、LOD 条目、预算、运动目标）。
//
// 注意：EAircraftSimulationDriveMode / EAircraftSimulationCollisionMode 原本定义在
// AircraftAssetEngine 的 AircraftSimulationModel.h，现上移到本契约层，
// 以便 AircraftRuntimeCommon 的 LOD 子系统与 AircraftAssetEngine 的组件共同消费而不产生依赖环。

#pragma once

#include "CoreMinimal.h"

#include "AircraftSimulationLODTypes.generated.h"

/** 当前 LOD 使用的运动驱动。LOD 只负责选择驱动，不实现具体 Gameplay 策略。 */
UENUM(BlueprintType)
enum class EAircraftSimulationDriveMode : uint8
{
	None UMETA(DisplayName = "None"),
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

/** 一条数据驱动的 LOD 条目。数组顺序为最近/最高优先级到最远。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationLODSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "名称"))
	FName Name = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "驱动模式"))
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::None;

	/** 距最近玩家的名义上限距离。数组最后一个 LOD 是无限距离兜底，不使用此参数。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (
		DisplayName = "最大生效距离",
		ToolTip = "该LOD距离最近玩家的最大生效距离。数组最后一个LOD是无限距离兜底，不使用此参数。",
		ClampMin = "0.0", Units = "cm"))
	float MaxDistanceCm = 6000.0f;

	/** 慢速制导/Gameplay 节拍。0 表示每个游戏帧都跑。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "运行慢速逻辑"))
	bool bRunSlowLogic = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "慢速逻辑更新间隔", ClampMin = "0.0", Units = "s"))
	float SlowLogicIntervalSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "碰撞模式"))
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;

	/** 权威端建议的 Actor 复制频率。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "建议网络更新频率", ClampMin = "1.0", Units = "Hz"))
	float SuggestedNetUpdateFrequency = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation", meta = (DisplayName = "启用网络休眠"))
	bool bEnableNetworkDormancy = false;
};

/** 活跃运动源发布的精确临时驱动请求。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationDriveOverride
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	int32 Priority = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
	bool bValid = false;
};

/** Gameplay 重要性刻意与飞控/AI 类型解耦。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationImportance
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

	/** 外部 Gameplay 约束（吊挂、缆绳、世界关节）。机体/旋翼内部约束不设置此项。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation")
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
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationSnapshot
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

/** 可选飞机特性消费的通用预算。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSimulationBudget
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
	bool bEnableNetworkDormancy = false;
};

/** 由制导、Root Motion、动画或 Gameplay 发布的共享运动目标。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMotionTarget
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
