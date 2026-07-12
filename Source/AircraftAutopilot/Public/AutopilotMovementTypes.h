// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/AutopilotTrajectoryTypes.h"
#include "AutopilotMovementTypes.generated.h"

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

UENUM(BlueprintType)
enum class EAutopilotIntentStatus : uint8
{
	Invalid,
	Accepted,
	Executing,
	Succeeded,
	Failed,
	Cancelled,
	Interrupted,
	Rejected
};

UENUM(BlueprintType)
enum class EAutopilotIntentFailureReason : uint8
{
	None,
	InvalidIntent,
	AutopilotInactive,
	FlightControllerUnavailable,
	TrajectoryGenerationFailed,
	Timeout,
	Replaced,
	CancelledByCaller
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotIntentHandle
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
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
struct AIRCRAFTAUTOPILOT_API FAutopilotArrivalCriteria
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0"))
	float HorizontalToleranceCm = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0"))
	float VerticalToleranceCm = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0"))
	float SpeedToleranceCmPerSec = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0", ClampMax = "180.0"))
	float YawToleranceDegrees = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0"))
	float StableTimeSeconds = 0.2f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotMovementIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	EAutopilotMovementIntentType Type = EAutopilotMovementIntentType::Hold;

	/** Fixed world target, or an offset from TargetActor when one is supplied. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	FVector TargetPositionCm = FVector::ZeroVector;

	/** Optional moving reference for MoveToPosition and Orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	TObjectPtr<AActor> TargetActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;

	/** Collision-free world-space path supplied by the navigation system. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	TArray<FVector> PathPointsCm;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	EAutopilotPathTrajectoryMode PathTrajectoryMode = EAutopilotPathTrajectoryMode::PiecewiseLinear;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	FTrajectoryMotionConstraints MotionConstraints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	EAutopilotHeadingMode HeadingMode = EAutopilotHeadingMode::FaceVelocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	float FixedYawDegrees = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	EAutopilotArrivalMode ArrivalMode = EAutopilotArrivalMode::StopAndComplete;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	FAutopilotArrivalCriteria ArrivalCriteria;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0"))
	float OrbitRadiusCm = 500.0f;

	/** Positive is counter-clockwise in Unreal's XY plane. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent")
	float OrbitAngularRateDegPerSec = 45.0f;

	/** Zero disables the timeout. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Intent", meta = (ClampMin = "0.0"))
	float TimeoutSeconds = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FAutopilotIntentResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	FAutopilotIntentHandle Handle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	EAutopilotIntentStatus Status = EAutopilotIntentStatus::Invalid;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	EAutopilotIntentFailureReason FailureReason = EAutopilotIntentFailureReason::None;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	float Progress = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Intent")
	float ElapsedSeconds = 0.0f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAutopilotIntentChanged, const FAutopilotIntentResult&, Result);
