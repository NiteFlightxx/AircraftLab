// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AircraftMovementIntent.h"
#include "AutopilotMovementTypes.generated.h"

class UAnimMontage;
class UPrimitiveComponent;
class USkeletalMeshComponent;

/** Root Motion 的内部驱动路径；蓝图通过三个独立提交接口选择。 */
enum class EAutopilotRootMotionDriveMode : uint8
{
	Kinematic,
	FlightController,
	PhysicsConstraint
};

UENUM(BlueprintType)
enum class EAutopilotIntentStatus : uint8
{
	Invalid UMETA(DisplayName = "无效"),
	Accepted UMETA(DisplayName = "已接受"),
	Executing UMETA(DisplayName = "执行中"),
	Succeeded UMETA(DisplayName = "已成功"),
	Failed UMETA(DisplayName = "已失败"),
	Cancelled UMETA(DisplayName = "已取消"),
	Interrupted UMETA(DisplayName = "已中断"),
	Rejected UMETA(DisplayName = "已拒绝")
};

UENUM(BlueprintType)
enum class EAutopilotIntentFailureReason : uint8
{
	None UMETA(DisplayName = "无"),
	InvalidIntent UMETA(DisplayName = "无效意图"),
	AutopilotInactive UMETA(DisplayName = "自动驾驶未激活"),
	FlightControllerUnavailable UMETA(DisplayName = "飞控不可用"),
	TrajectoryGenerationFailed UMETA(DisplayName = "轨迹生成失败"),
	Timeout UMETA(DisplayName = "超时"),
	Replaced UMETA(DisplayName = "被替换"),
	CancelledByCaller UMETA(DisplayName = "调用方取消"),
	AnimationInterrupted UMETA(DisplayName = "动画被中断"),
	PhysicsConstraintBroken UMETA(DisplayName = "物理约束已断开")
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotIntentHandle
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent", meta = (DisplayName = "意图 ID"))
	int64 Id = 0;

	bool IsValid() const { return Id > 0; }
	bool operator==(const FAutopilotIntentHandle& Other) const { return Id == Other.Id; }
	bool operator!=(const FAutopilotIntentHandle& Other) const { return !(*this == Other); }
};

FORCEINLINE uint32 GetTypeHash(const FAutopilotIntentHandle& Handle)
{
	return GetTypeHash(Handle.Id);
}

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotIntentResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent", meta = (DisplayName = "意图句柄"))
	FAutopilotIntentHandle Handle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent", meta = (DisplayName = "状态"))
	EAutopilotIntentStatus Status = EAutopilotIntentStatus::Invalid;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent", meta = (DisplayName = "失败原因"))
	EAutopilotIntentFailureReason FailureReason = EAutopilotIntentFailureReason::None;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent", meta = (DisplayName = "进度"))
	float Progress = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent", meta = (DisplayName = "已用时间（秒）"))
	float ElapsedSeconds = 0.0f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAutopilotIntentChanged, const FAutopilotIntentResult&, Result);

/** Heading is independently composable with every movement command. */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotHeadingOptions
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Heading", meta = (DisplayName = "航向模式"))
	EAutopilotHeadingMode Mode = EAutopilotHeadingMode::FaceVelocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Heading",
		meta = (EditCondition = "Mode == EAutopilotHeadingMode::FixedYaw", EditConditionHides, DisplayName = "固定航向角（度）"))
	float FixedYawDegrees = 0.0f;

	/** FixedYaw: positive turn-speed limit used while rotating toward FixedYawDegrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Heading",
		meta = (ClampMin = "0.0", EditCondition = "Mode == EAutopilotHeadingMode::FixedYaw",
			EditConditionHides, DisplayName = "航向转动速度（度/秒）"))
	float YawRateDegreesPerSec = 90.0f;

	/** Enable a look-at target distinct from the destination/path. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Heading",
		meta = (EditCondition = "Mode == EAutopilotHeadingMode::FaceTarget", EditConditionHides, DisplayName = "使用独立注视目标"))
	bool bUseLookAtTarget = false;

	/** World position, or actor-relative offset when LookAtActor is set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Heading",
		meta = (EditCondition = "Mode == EAutopilotHeadingMode::FaceTarget && bUseLookAtTarget", EditConditionHides, DisplayName = "注视位置（厘米）"))
	FVector LookAtPositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Heading",
		meta = (EditCondition = "Mode == EAutopilotHeadingMode::FaceTarget && bUseLookAtTarget", EditConditionHides, DisplayName = "注视目标 Actor"))
	TObjectPtr<AActor> LookAtActor = nullptr;
};

/** Options shared by finite movement commands. */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotFiniteCommandOptions
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Command", meta = (DisplayName = "运动约束"))
	FTrajectoryMotionConstraints MotionConstraints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Command", meta = (DisplayName = "航向选项"))
	FAutopilotHeadingOptions Heading;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Command", meta = (DisplayName = "到达模式"))
	EAutopilotArrivalMode ArrivalMode = EAutopilotArrivalMode::StopAndComplete;

	/** PassThrough 时在终点保留的速度；其他到达模式固定减速到 0。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Command",
		meta = (ClampMin = "0.0", EditCondition = "ArrivalMode == EAutopilotArrivalMode::PassThrough", EditConditionHides,
			DisplayName = "穿越终点速度（厘米/秒）"))
	float PassThroughSpeedCmPerSec = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Command", meta = (DisplayName = "到达判据"))
	FAutopilotArrivalCriteria ArrivalCriteria;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Command", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotMoveToCommand
{
	GENERATED_BODY()

	/** World destination, or actor-relative offset when TargetActor is set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MoveTo", meta = (DisplayName = "目标位置（厘米）"))
	FVector TargetPositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MoveTo", meta = (DisplayName = "目标 Actor"))
	TObjectPtr<AActor> TargetActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|MoveTo", meta = (DisplayName = "命令选项"))
	FAutopilotFiniteCommandOptions Options;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotFollowPathCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Path", meta = (DisplayName = "路径点（厘米）"))
	TArray<FVector> PathPointsCm;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Path", meta = (DisplayName = "轨迹模式"))
	EAutopilotPathTrajectoryMode TrajectoryMode = EAutopilotPathTrajectoryMode::PiecewiseLinear;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Path", meta = (DisplayName = "命令选项"))
	FAutopilotFiniteCommandOptions Options;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotOrbitCommand
{
	GENERATED_BODY()

	/** World center, or actor-relative offset when CenterActor is set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (DisplayName = "环绕中心位置（厘米）"))
	FVector CenterPositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (DisplayName = "环绕中心 Actor"))
	TObjectPtr<AActor> CenterActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (ClampMin = "0.0", DisplayName = "环绕半径（厘米）"))
	float RadiusCm = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (DisplayName = "角速度（度/秒）"))
	float AngularRateDegPerSec = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (DisplayName = "持续运动约束"))
	FContinuousMotionConstraints MotionConstraints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (DisplayName = "航向选项"))
	FAutopilotHeadingOptions Heading;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Orbit", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotCircleArcCommand
{
	GENERATED_BODY()

	/** World center, or actor-relative offset when CenterActor is set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|CircleArc", meta = (DisplayName = "圆心位置（厘米）"))
	FVector CenterPositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|CircleArc", meta = (DisplayName = "圆心 Actor"))
	TObjectPtr<AActor> CenterActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|CircleArc", meta = (ClampMin = "0.0", DisplayName = "半径（厘米）"))
	float RadiusCm = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|CircleArc", meta = (DisplayName = "起始角（度）"))
	float StartAngleDegrees = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|CircleArc", meta = (DisplayName = "终止角（度）"))
	float EndAngleDegrees = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|CircleArc", meta = (DisplayName = "命令选项"))
	FAutopilotFiniteCommandOptions Options;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotVelocityCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Velocity", meta = (DisplayName = "期望速度（厘米/秒）"))
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Velocity", meta = (DisplayName = "持续运动约束"))
	FContinuousMotionConstraints MotionConstraints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Velocity", meta = (DisplayName = "航向选项"))
	FAutopilotHeadingOptions Heading;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Velocity", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};

/** Chaos 约束驱动参数。Acceleration Mode 下强度基本不随刚体质量和惯量变化。 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotRootMotionConstraintDrive
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (ClampMin = "0.0", DisplayName = "线性位置强度"))
	float LinearPositionStrength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (ClampMin = "0.0", DisplayName = "线性速度阻尼"))
	float LinearVelocityStrength = 20.0f;

	/** 0 表示不限制最大约束力。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (ClampMin = "0.0", DisplayName = "最大线性力"))
	float LinearForceLimit = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (ClampMin = "0.0", DisplayName = "角度位置强度"))
	float AngularPositionStrength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (ClampMin = "0.0", DisplayName = "角速度阻尼"))
	float AngularVelocityStrength = 20.0f;

	/** 0 表示不限制最大约束力矩。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (ClampMin = "0.0", DisplayName = "最大角度力矩"))
	float AngularTorqueLimit = 0.0f;

	/** 开启后线性/角度驱动强度按加速度解释，减少质量和惯量变化带来的调参差异。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Constraint", meta = (DisplayName = "使用加速度驱动"))
	bool bAccelerationMode = true;
};

/** 三种 Root Motion 命令共享的动画播放参数。 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotRootMotionPlayback
{
	GENERATED_BODY()

	/** 必须是 Autopilot Owner 的根组件。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Playback", meta = (DisplayName = "骨骼网格体"))
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Playback", meta = (DisplayName = "动画蒙太奇"))
	TObjectPtr<UAnimMontage> Montage = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Playback", meta = (ClampMin = "0.01", DisplayName = "播放速率"))
	float PlayRate = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Playback", meta = (ClampMin = "0.0", DisplayName = "起始时间（秒）"))
	float StartPositionSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Playback", meta = (DisplayName = "停止其他 Montage"))
	bool bStopAllMontages = true;
};

/** 直接把 Root Motion 增量应用到 Actor 根组件。 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotKinematicRootMotionCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion", meta = (DisplayName = "播放参数"))
	FAutopilotRootMotionPlayback Playback;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Kinematic", meta = (DisplayName = "使用 Root Motion 旋转"))
	bool bApplyRootMotionRotation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Kinematic", meta = (DisplayName = "移动时检测碰撞"))
	bool bSweep = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|Kinematic", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};

/** Root Motion 生成参考轨迹，由现有飞控闭环跟踪。 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotFlightControllerRootMotionCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion", meta = (DisplayName = "播放参数"))
	FAutopilotRootMotionPlayback Playback;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|FlightController", meta = (DisplayName = "使用 Root Motion 偏航"))
	bool bApplyRootMotionRotation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|FlightController", meta = (EditCondition = "!bApplyRootMotionRotation", EditConditionHides, DisplayName = "航向选项"))
	FAutopilotHeadingOptions Heading;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|FlightController", meta = (DisplayName = "运动约束"))
	FTrajectoryMotionConstraints MotionConstraints;

	/** Montage 结束后，飞控达到最终 Root Motion 目标才会完成 Intent。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|FlightController", meta = (DisplayName = "到达判据"))
	FAutopilotArrivalCriteria ArrivalCriteria;

	/** 0 表示不超时；计时包含 Montage 播放和结束后的飞控收敛阶段。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|FlightController", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};

/** Root Motion 更新一个连接到世界的 Chaos 六自由度软约束目标。 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotPhysicsConstraintRootMotionCommand
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion", meta = (DisplayName = "播放参数"))
	FAutopilotRootMotionPlayback Playback;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|PhysicsConstraint", meta = (DisplayName = "使用 Root Motion 旋转"))
	bool bApplyRootMotionRotation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|PhysicsConstraint", meta = (EditCondition = "!bApplyRootMotionRotation", EditConditionHides, DisplayName = "航向选项"))
	FAutopilotHeadingOptions Heading;

	/** 根 SkeletalMesh 上的可选物理骨骼；驱动根刚体时保持 None。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|PhysicsConstraint", meta = (DisplayName = "物理骨骼"))
	FName PhysicsBoneName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|PhysicsConstraint", meta = (DisplayName = "约束驱动参数"))
	FAutopilotRootMotionConstraintDrive ConstraintDrive;

	/** Montage 结束后，刚体达到最终 Root Motion 目标才会完成 Intent。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|PhysicsConstraint", meta = (DisplayName = "到达判据"))
	FAutopilotArrivalCriteria ArrivalCriteria;

	/** 0 表示不超时；计时包含 Montage 播放和结束后的物理收敛阶段。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|RootMotion|PhysicsConstraint", meta = (ClampMin = "0.0", DisplayName = "超时时间（秒）"))
	float TimeoutSeconds = 0.0f;
};
