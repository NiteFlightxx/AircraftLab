#pragma once

#include "CoreMinimal.h"

#include "AircraftAutopilotTypes.generated.h"

/** 同一物理子步采集的完整运动状态。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftVehicleStateSnapshot
{
	double TimeSeconds = 0.0;
	uint64 Sequence = 0;
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
	FQuat BodyRotation = FQuat::Identity;
	/** Yaw-only aircraft control frame; may differ from body axes configured in Dataflow. */
	FQuat ControlRotation = FQuat::Identity;
	FVector AngularVelocityBodyRadPerSec = FVector::ZeroVector;
};

/** 由飞控、分配器和当前旋翼健康共同产生的实时硬能力。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftDynamicCapabilitySnapshot
{
	double TimeSeconds = 0.0;
	uint64 Revision = 0;
	float MassKg = 0.0f;
	FVector InertiaKgM2 = FVector::ZeroVector;
	FVector CenterOfMassBodyCm = FVector::ZeroVector;
	float GravityCmPerSecSq = 980.0f;
	float MaxHorizontalSpeedCmPerSec = 0.0f;
	float MaxHorizontalAccelerationCmPerSecSq = 0.0f;
	float MaxVerticalAccelerationCmPerSecSq = 0.0f;
	float MaxClimbRateCmPerSec = 0.0f;
	float MaxDescentRateCmPerSec = 0.0f;
	float MaxTiltRadians = 0.0f;
	FVector MaxBodyRateRadPerSec = FVector::ZeroVector;
	float CollectiveAuthorityN = 0.0f;
	FVector PositiveTorqueAuthorityNm = FVector::ZeroVector;
	FVector NegativeTorqueAuthorityNm = FVector::ZeroVector;
	FVector LinearDampingPerSecond = FVector::ZeroVector;
	FVector AngularDampingPerSecond = FVector::ZeroVector;
	bool bHasExplicitAerodynamics = false;
	float AirDensityKgPerM3 = 0.0f;
	FVector LinearDragBodyNsPerM = FVector::ZeroVector;
	FVector DragAreaCoefficientBodyM2 = FVector::ZeroVector;
	float MinimumRotorTimeConstantSeconds = 0.0f;
	bool bValid = false;
};

/** 预测控制器向全部驱动后端发布的唯一运动参考。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftTrajectoryReference
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	int64 IntentId = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	int64 IntentRevision = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	int64 PlanRevision = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	int64 StateSequence = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "s"))
	double GeneratedAtSeconds = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "s"))
	double ValidUntilSeconds = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "cm"))
	FVector PositionCm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "cm/s"))
	FVector VelocityCmPerSec = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "cm/s^2"))
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "deg"))
	float YawDegrees = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "deg/s"))
	float YawRateDegPerSec = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float YawAccelerationDegPerSecSq = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "deg/s"))
	float YawRateLimitDegPerSec = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float PathProgress = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	bool bValid = false;

	bool IsFresh(double CurrentTimeSeconds) const
	{
		return bValid && CurrentTimeSeconds <= ValidUntilSeconds;
	}
};

struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftAutopilotDiagnostics
{
	int64 ActiveIntentId = 0;
	uint64 IntentRevision = 0;
	uint64 PlanRevision = 0;
	int32 SolverIterations = 0;
	int32 ConsecutiveFailures = 0;
	double LastSolveMilliseconds = 0.0;
	double MaximumSolveMilliseconds = 0.0;
	float ContourErrorCm = 0.0f;
	float LagErrorCm = 0.0f;
	bool bPlanValid = false;
	bool bReferenceFresh = false;
};
