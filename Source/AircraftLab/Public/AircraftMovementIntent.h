#pragma once

#include "CoreMinimal.h"
#include "AircraftMovementIntent.generated.h"

class AActor;

UENUM(BlueprintType)
enum class EAutopilotMovementIntentType : uint8
{
	Hold,
	MoveToPosition,
	MoveWithVelocity,
	FollowPath,
	Orbit
};

UENUM(BlueprintType)
enum class EAutopilotHeadingMode : uint8
{
	KeepCurrent,
	FixedYaw,
	FaceVelocity,
	FaceTarget
};

UENUM(BlueprintType)
enum class EAutopilotArrivalMode : uint8
{
	StopAndComplete,
	PassThrough,
	HoldAtTarget
};

UENUM(BlueprintType)
enum class EAutopilotPathTrajectoryMode : uint8
{
	PiecewiseLinear,
	MinimumSnap
};

/** MovementIntent 独占拥有的运动整形约束；Profile 不再重复声明这些字段。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FTrajectoryMotionConstraints
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxDecelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float TargetSpeedCmPerSec = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxJerkCmPerSecCubed = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxYawRateDegPerSec = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxYawAccelerationDegPerSecSq = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxYawJerkDegPerSecCubed = 600.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxRollRateDegPerSec = 120.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Constraints", meta = (ClampMin = "0.0"))
	float MaxPitchRateDegPerSec = 120.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAutopilotArrivalCriteria
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0"))
	float HorizontalToleranceCm = 50.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0"))
	float VerticalToleranceCm = 100.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0"))
	float SpeedToleranceCmPerSec = 50.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0", ClampMax = "180.0"))
	float YawToleranceDegrees = 5.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Arrival", meta = (ClampMin = "0.0"))
	float StableTimeSeconds = 0.2f;
};

/** 玩家、Autopilot 和 GameplayPolicy 共用的唯一移动意图契约。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FAutopilotMovementIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	EAutopilotMovementIntentType Type = EAutopilotMovementIntentType::Hold;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FVector TargetPositionCm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	TObjectPtr<AActor> TargetActor = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FVector DesiredAccelerationCmPerSecSq = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FRotator DesiredAttitudeDegrees = FRotator::ZeroRotator;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FVector DesiredBodyRatesDegPerSec = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	TArray<FVector> PathPointsCm;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	EAutopilotPathTrajectoryMode PathTrajectoryMode = EAutopilotPathTrajectoryMode::PiecewiseLinear;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FTrajectoryMotionConstraints MotionConstraints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	EAutopilotHeadingMode HeadingMode = EAutopilotHeadingMode::FaceVelocity;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	float FixedYawDegrees = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	float DesiredYawRateDegPerSec = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	float ThrustFeedForward = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	EAutopilotArrivalMode ArrivalMode = EAutopilotArrivalMode::StopAndComplete;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	FAutopilotArrivalCriteria ArrivalCriteria;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (ClampMin = "0.0"))
	float OrbitRadiusCm = 500.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent")
	float OrbitAngularRateDegPerSec = 45.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement|Intent", meta = (ClampMin = "0.0"))
	float TimeoutSeconds = 0.0f;
};
