// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AircraftMovementIntent.h"
#include "AutopilotMovementTypes.generated.h"

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
	CancelledByCaller UMETA(DisplayName = "调用方取消")
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
