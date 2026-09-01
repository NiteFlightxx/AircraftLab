#pragma once

#include "CoreMinimal.h"

#include "AircraftAutopilotTypes.generated.h"

UENUM(BlueprintType)
enum class EAircraftPathTrackingState : uint8
{
	NotTracking,
	Nominal,
	ContourLimited,
	CorridorConstrained,
	CorridorRecovery
};

/** 自动驾驶规划器生成的、已经完成运动约束解算的世界空间轨迹样本。 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftMotionPlanSample
{
	float TimeSeconds = 0.0f;
	float DistanceCm = 0.0f;
	float RouteDistanceCm = 0.0f;
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
	float YawDegrees = 0.0f;
	float YawRateDegPerSec = 0.0f;
};

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
	float MaxHorizontalDecelerationCmPerSecSq = 0.0f;
	float MaxHorizontalJerkCmPerSecCubed = 0.0f;
	float MaxVerticalAccelerationCmPerSecSq = 0.0f;
	float MaxVerticalJerkCmPerSecCubed = 0.0f;
	float MaxClimbRateCmPerSec = 0.0f;
	float MaxDescentRateCmPerSec = 0.0f;
	float MaxTiltRadians = 0.0f;
	bool bHasTiltLimit = false;
	FVector MaxBodyRateRadPerSec = FVector::ZeroVector;
	FVector MaxBodyAngularAccelerationRadPerSecSq = FVector::ZeroVector;
	FVector MaxBodyAngularJerkRadPerSecCubed = FVector::ZeroVector;
	bool bCanControlRoll = false;
	bool bCanControlPitch = false;
	bool bCanControlYaw = false;
	float CollectiveAuthorityN = 0.0f;
	FVector PositiveTorqueAuthorityNm = FVector::ZeroVector;
	FVector NegativeTorqueAuthorityNm = FVector::ZeroVector;
	FVector LinearDampingPerSecond = FVector::ZeroVector;
	FVector AngularDampingPerSecond = FVector::ZeroVector;
	bool bHasExplicitAerodynamics = false;
	float AirDensityKgPerM3 = 0.0f;
	FVector LinearDragBodyNsPerM = FVector::ZeroVector;
	FVector DragAreaCoefficientBodyM2 = FVector::ZeroVector;
	float MaxRelativeAirspeedCmPerSec = 0.0f;
	/** 当前有效旋翼中最慢的增推力响应时间常数。 */
	float ThrustRiseResponseTimeSeconds = 0.0f;
	/** 当前有效旋翼中最慢的减推力响应时间常数。 */
	float ThrustFallResponseTimeSeconds = 0.0f;
	bool bValid = false;

	bool HasAngularControlAuthority(int32 Axis) const
	{
		switch (Axis)
		{
		case 0: return bCanControlRoll;
		case 1: return bCanControlPitch;
		case 2: return bCanControlYaw;
		default: return false;
		}
	}
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

	/** 预测控制器求得的控制加速度；轨迹加速度保持描述参考轨迹本身。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "cm/s^2"))
	FVector ControlAccelerationCmPerSecSq = FVector::ZeroVector;

	/** 维持参考速度所需的阻尼/空气动力学补偿；与轨迹运动学加速度分开，避免飞控重复前馈。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot", meta = (Units = "cm/s^2"))
	FVector DynamicsFeedForwardAccelerationCmPerSecSq = FVector::ZeroVector;

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

	/** 原始 Route 折线参数进度；安全走廊选择和调试绘制使用该语义。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float RouteProgress = 0.0f;

	/** Velocity 意图只跟踪速度；路径、环绕与 Hold 才启用位置外环。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	bool bPositionTrackingEnabled = false;

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
	float CorridorViolationCm = 0.0f;
	float PredictedCorridorViolationCm = 0.0f;
	float ProgressScale = 1.0f;
	bool bPlanValid = false;
	bool bReferenceFresh = false;
	/** 求解器连续失败达到配置阈值且已无可复用参考。 */
	bool bSolverFailed = false;
	bool bCorridorViolated = false;
	EAircraftPathTrackingState PathTrackingState = EAircraftPathTrackingState::NotTracking;
};
