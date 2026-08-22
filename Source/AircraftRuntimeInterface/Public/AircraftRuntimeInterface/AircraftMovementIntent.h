#pragma once

#include "CoreMinimal.h"

#include "AircraftMovementIntent.generated.h"

class AActor;

UENUM(BlueprintType)
enum class EAircraftMovementIntentType : uint8
{
	Hold,
	Velocity,
	Route,
	Orbit,
	TimedTrajectory
};

UENUM(BlueprintType)
enum class EAircraftVelocityFrame : uint8
{
	World,
	ControlHeading
};

UENUM(BlueprintType)
enum class EAircraftHeadingMode : uint8
{
	KeepCurrent,
	FixedYaw,
	FaceVelocity,
	FaceTarget
};

UENUM(BlueprintType)
enum class EAircraftArrivalMode : uint8
{
	Stop,
	PassThrough
};

UENUM(BlueprintType)
enum class EAircraftMovementIntentStatus : uint8
{
	Invalid,
	Accepted,
	Planning,
	Executing,
	Succeeded,
	Failed,
	Cancelled,
	Interrupted
};

UENUM(BlueprintType)
enum class EAircraftMovementFailureReason : uint8
{
	None,
	InvalidIntent,
	AutopilotInactive,
	FlightControllerUnavailable,
	PlanningFailed,
	SolverFailed,
	Timeout,
	Replaced,
	CancelledByCaller
};

/** 本次运动请求的软限制。飞控硬限制和实时控制权限始终具有更高优先级。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftRequestedMotionLimits
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "cm/s"))
	float CruiseSpeedCmPerSec = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "cm/s^2"))
	float MaxAccelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "cm/s^2"))
	float MaxDecelerationCmPerSecSq = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0"))
	float MaxJerkCmPerSecCubed = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "cm/s"))
	float MaxClimbRateCmPerSec = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "cm/s"))
	float MaxDescentRateCmPerSec = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "cm/s^2"))
	float MaxVerticalAccelerationCmPerSecSq = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0"))
	float MaxVerticalJerkCmPerSecCubed = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "deg/s"))
	float MaxYawRateDegPerSec = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0"))
	float MaxYawAccelerationDegPerSecSq = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0"))
	float MaxYawJerkDegPerSecCubed = 600.0f;

	bool IsValid() const;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftHeadingObjective
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Heading")
	EAircraftHeadingMode Mode = EAircraftHeadingMode::FaceVelocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Heading", meta = (Units = "deg"))
	float FixedYawDegrees = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Heading", meta = (Units = "cm"))
	FVector TargetPositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Heading")
	TObjectPtr<AActor> TargetActor = nullptr;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftCompletionPolicy
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion")
	EAircraftArrivalMode ArrivalMode = EAircraftArrivalMode::Stop;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion", meta = (ClampMin = "0.0", Units = "cm/s"))
	float TerminalSpeedCmPerSec = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion", meta = (ClampMin = "0.0", Units = "cm"))
	float HorizontalToleranceCm = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion", meta = (ClampMin = "0.0", Units = "cm"))
	float VerticalToleranceCm = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion", meta = (ClampMin = "0.0", Units = "cm/s"))
	float SpeedToleranceCmPerSec = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion", meta = (ClampMin = "0.0", ClampMax = "180.0", Units = "deg"))
	float YawToleranceDegrees = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Completion", meta = (ClampMin = "0.0", Units = "s"))
	float StableTimeSeconds = 0.2f;
};

/** 所有自动驾驶命令共享的运动约束、朝向目标和超时。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMovementIntentSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	FAircraftRequestedMotionLimits Limits;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	FAircraftHeadingObjective Heading;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "s"))
	float TimeoutSeconds = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftHoldIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "cm"))
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	TObjectPtr<AActor> TargetActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	bool bCaptureCurrentPosition = true;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftVelocityIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "cm/s"))
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	EAircraftVelocityFrame Frame = EAircraftVelocityFrame::World;
};

/** 一段凸安全走廊。Plane 的法向朝向可行区域外侧，约束为 PlaneDot(Position) <= 0。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftSafeCorridorSegment
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (ClampMin = "0.0", Units = "cm"))
	float StartDistanceCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation", meta = (ClampMin = "0.0", Units = "cm"))
	float EndDistanceCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation")
	TArray<FPlane> BoundaryPlanes;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftRouteIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation")
	TArray<FVector> PointsCm;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation")
	TArray<FAircraftSafeCorridorSegment> Corridor;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Navigation")
	bool bClosed = false;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftOrbitIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "cm"))
	FVector CenterCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	TObjectPtr<AActor> CenterActor = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "1.0", Units = "cm"))
	float RadiusCm = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "deg/s"))
	float AngularRateDegPerSec = 45.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftTimedTrajectorySample
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (ClampMin = "0.0", Units = "s"))
	float TimeSeconds = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "cm"))
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "cm/s"))
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "cm/s^2"))
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "deg"))
	float YawDegrees = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement", meta = (Units = "deg/s"))
	float YawRateDegPerSec = 0.0f;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftTimedTrajectoryIntent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Movement")
	TArray<FAircraftTimedTrajectorySample> Samples;
};

/** 飞控内部传输格式；蓝图使用 UAutopilotComponent 的强类型提交接口。 */
USTRUCT()
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMovementIntent
{
	GENERATED_BODY()

	UPROPERTY()
	EAircraftMovementIntentType Type = EAircraftMovementIntentType::Hold;

	UPROPERTY()
	FAircraftHoldIntent Hold;

	UPROPERTY()
	FAircraftVelocityIntent Velocity;

	UPROPERTY()
	FAircraftRouteIntent Route;

	UPROPERTY()
	FAircraftOrbitIntent Orbit;

	UPROPERTY()
	FAircraftTimedTrajectoryIntent TimedTrajectory;

	UPROPERTY()
	FAircraftRequestedMotionLimits Limits;

	UPROPERTY()
	FAircraftHeadingObjective Heading;

	UPROPERTY()
	FAircraftCompletionPolicy Completion;

	UPROPERTY()
	float TimeoutSeconds = 0.0f;

	bool IsValid() const;
};

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMovementIntentHandle
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Movement")
	int64 Id = 0;

	bool IsValid() const { return Id > 0; }
	bool operator==(const FAircraftMovementIntentHandle& Other) const { return Id == Other.Id; }
};

FORCEINLINE uint32 GetTypeHash(const FAircraftMovementIntentHandle& Handle)
{
	return GetTypeHash(Handle.Id);
}

USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMovementIntentResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Movement")
	FAircraftMovementIntentHandle Handle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Movement")
	EAircraftMovementIntentStatus Status = EAircraftMovementIntentStatus::Invalid;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Movement")
	EAircraftMovementFailureReason FailureReason = EAircraftMovementFailureReason::None;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Movement")
	float Progress = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Movement", meta = (Units = "s"))
	float ElapsedSeconds = 0.0f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
	FOnAircraftMovementIntentChanged, const FAircraftMovementIntentResult&, Result);
