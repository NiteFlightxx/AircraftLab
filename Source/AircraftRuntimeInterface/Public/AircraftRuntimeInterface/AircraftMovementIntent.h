// 玩家、Autopilot 和 GameplayPolicy 共用的唯一移动意图契约。

#pragma once

#include "CoreMinimal.h"
#include "AircraftMovementIntent.generated.h"

class AActor;

UENUM(BlueprintType)
enum class EAutopilotMovementIntentType : uint8
{
	Hold UMETA(DisplayName = "悬停"),
	MoveToPosition UMETA(DisplayName = "移动到位置"),
	MoveWithVelocity UMETA(DisplayName = "速度移动"),
	FollowPath UMETA(DisplayName = "跟随路径"),
	Orbit UMETA(DisplayName = "环绕飞行"),
	/** 围绕 TargetPositionCm/TargetActor 的水平有限圆弧。 */
	CircleArc UMETA(DisplayName = "圆弧飞行"),
	/** 运动轨迹取自动画 Montage 的 Root Motion。 */
	RootMotion UMETA(DisplayName = "Root Motion")
};

UENUM(BlueprintType)
enum class EAutopilotHeadingMode : uint8
{
	KeepCurrent UMETA(DisplayName = "保持当前航向"),
	FixedYaw UMETA(DisplayName = "固定航向"),
	FaceVelocity UMETA(DisplayName = "朝向速度方向"),
	FaceTarget UMETA(DisplayName = "朝向目标")
};

UENUM(BlueprintType)
enum class EAutopilotArrivalMode : uint8
{
	StopAndComplete UMETA(DisplayName = "停止并完成"),
	PassThrough UMETA(DisplayName = "穿过不停")
};

UENUM(BlueprintType)
enum class EAutopilotPathTrajectoryMode : uint8
{
	PiecewiseLinear UMETA(DisplayName = "分段线性"),
	MinimumSnap UMETA(DisplayName = "最小snap"),
	/** 将 PathPointsCm 视为贝塞尔控制点。 */
	Bezier UMETA(DisplayName = "贝塞尔")
};

/** MovementIntent 独占拥有的运动整形约束；Profile 不再重复声明这些字段。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FTrajectoryMotionConstraints
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "巡航速度（厘米/秒）"))
	float CruiseSpeedCmPerSec = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大加速度（厘米/秒²）"))
	float MaxAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大减速度（厘米/秒²）"))
	float MaxDecelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大加加速度（厘米/秒³）"))
	float MaxJerkCmPerSecCubed = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大爬升率（厘米/秒）"))
	float MaxClimbRateCmPerSec = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大下降率（厘米/秒）"))
	float MaxDescentRateCmPerSec = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大垂直加速度（厘米/秒²）"))
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大垂直加加速度（厘米/秒³）"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大偏航角速度（度/秒）"))
	float MaxYawRateDegPerSec = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大偏航角加速度（度/秒²）"))
	float MaxYawAccelerationDegPerSecSq = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大偏航加加速度（度/秒³）"))
	float MaxYawJerkDegPerSecCubed = 600.0f;
};

/**
 * 速度移动和持续环绕使用的约束。
 * 这两类命令没有"终点制动"，其巡航速度分别由期望速度和半径×角速度唯一决定，
 * 因此不暴露有限轨迹中的巡航/减速/终点速度，避免同一件事由多个参数控制。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FContinuousMotionConstraints
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大水平加速度（厘米/秒²）"))
	float MaxAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大水平加加速度（厘米/秒³）"))
	float MaxJerkCmPerSecCubed = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大爬升率（厘米/秒）"))
	float MaxClimbRateCmPerSec = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大下降率（厘米/秒）"))
	float MaxDescentRateCmPerSec = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大垂直加速度（厘米/秒²）"))
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大垂直加加速度（厘米/秒³）"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大偏航角速度（度/秒）"))
	float MaxYawRateDegPerSec = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大偏航角加速度（度/秒²）"))
	float MaxYawAccelerationDegPerSecSq = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0", DisplayName = "最大偏航加加速度（度/秒³）"))
	float MaxYawJerkDegPerSecCubed = 600.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAutopilotArrivalCriteria
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0", DisplayName = "水平容差（厘米）"))
	float HorizontalToleranceCm = 50.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0", DisplayName = "垂直容差（厘米）"))
	float VerticalToleranceCm = 100.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0", DisplayName = "速度容差（厘米/秒）"))
	float SpeedToleranceCmPerSec = 50.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0", ClampMax = "180.0", DisplayName = "航向容差（度）"))
	float YawToleranceDegrees = 5.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0", DisplayName = "稳定时间（秒）"))
	float StableTimeSeconds = 0.2f;
};

/** 玩家、Autopilot 和 GameplayPolicy 共用的唯一移动意图契约。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAutopilotMovementIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "意图类型"))
	EAutopilotMovementIntentType Type = EAutopilotMovementIntentType::Hold;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "目标位置（厘米）"))
	FVector TargetPositionCm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "目标 Actor"))
	TObjectPtr<AActor> TargetActor = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "期望速度（厘米/秒）"))
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "期望姿态角（度）"))
	FRotator DesiredAttitudeDegrees = FRotator::ZeroRotator;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "期望机体角速度（度/秒）"))
	FVector DesiredBodyRatesDegPerSec = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "路径点（厘米）"))
	TArray<FVector> PathPointsCm;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "路径轨迹模式"))
	EAutopilotPathTrajectoryMode PathTrajectoryMode = EAutopilotPathTrajectoryMode::PiecewiseLinear;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "运动约束"))
	FTrajectoryMotionConstraints MotionConstraints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "航向模式"))
	EAutopilotHeadingMode HeadingMode = EAutopilotHeadingMode::FaceVelocity;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "固定航向角（度）"))
	float FixedYawDegrees = 0.0f;
	/** 使用独立于运动终点的航向目标。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "使用独立航向目标"))
	bool bUseIndependentHeadingTarget = false;
	/** 世界坐标；设置了 HeadingTargetActor 时则为 Actor 相对偏移。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "航向目标位置（厘米）"))
	FVector HeadingTargetPositionCm = FVector::ZeroVector;
	/** 可选：朝向某个 Actor 而不改变运动终点。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "航向目标 Actor"))
	TObjectPtr<AActor> HeadingTargetActor = nullptr;
	/** FixedYaw 时表示转向目标航向所使用的正向速度上限。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent",
		meta = (ClampMin = "0.0", EditCondition = "HeadingMode == EAutopilotHeadingMode::FixedYaw", EditConditionHides,
			DisplayName = "航向转动速度（度/秒）"))
	float DesiredYawRateDegPerSec = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "到达模式"))
	EAutopilotArrivalMode ArrivalMode = EAutopilotArrivalMode::StopAndComplete;
	/** 仅 PassThrough 使用；表示穿过有限轨迹终点时保留的速度。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent",
		meta = (ClampMin = "0.0", EditCondition = "ArrivalMode == EAutopilotArrivalMode::PassThrough", EditConditionHides,
			DisplayName = "穿越终点速度（厘米/秒）"))
	float PassThroughSpeedCmPerSec = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "到达判据"))
	FAutopilotArrivalCriteria ArrivalCriteria;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (ClampMin = "0.0", DisplayName = "环绕半径（厘米）"))
	float OrbitRadiusCm = 500.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "环绕角速度（度/秒）"))
	float OrbitAngularRateDegPerSec = 45.0f;
	/** CircleArc 绕 +Z 的起始角；0 度沿世界 +X。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "圆弧起始角（度）"))
	float ArcStartAngleDegrees = 0.0f;
	/** CircleArc 终止角；更大的值逆时针扫掠，更小的值顺时针扫掠。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (DisplayName = "圆弧终止角（度）"))
	float ArcEndAngleDegrees = 90.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};
