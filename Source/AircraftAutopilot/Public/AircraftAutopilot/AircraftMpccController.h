#pragma once

#include "CoreMinimal.h"
#include "AircraftAutopilot/AircraftMotionPlan.h"

/** Fixed-horizon constrained contouring controller used exclusively by the flight-control backend. */
class AIRCRAFTAUTOPILOT_API FAircraftMpccController
{
public:
	bool SetIntent(const FAircraftMovementIntent& Intent, int64 IntentId, uint64 IntentRevision,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability);

	bool Update(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	/** Rebase absolute clocks after a simulation pause without changing plan progress. */
	void RebaseTime(double TimeSeconds);
	bool RefreshPlan(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		return RefreshPlanForCapability(State, Capability);
	}

	void Reset();
	const FAircraftAutopilotDiagnostics& GetDiagnostics() const { return Diagnostics; }
	const FAircraftMotionPlan& GetPlan() const { return Plan; }

private:
	FAircraftMotionPlan Plan;
	FAircraftMovementIntent RequestedIntent;
	FAircraftDynamicCapabilitySnapshot PlanningCapability;
	FAircraftAutopilotRuntimeConfig RuntimeConfig;
	FAircraftAutopilotDiagnostics Diagnostics;
	FAircraftTrajectoryReference LastReference;
	FVector LastVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector CandidateVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	TArray<FVector> ControlCorrectionHorizon;
	/** 发布前安全覆盖的临时副本；绝不能回写优化器的跨帧初值。 */
	TArray<FVector> SafetyCorrectionScratch;
	TArray<FAircraftMotionPlanSample> ReferenceScratch;
	TArray<FVector> CommandAccelerationScratch;
	TArray<FVector> PositionScratch;
	TArray<FVector> VelocityScratch;
	TArray<FVector> RealizedAccelerationScratch;
	TArray<float> ResponseAlphaScratch;
	TArray<FVector> GradientScratch;
	TArray<FVector> PreviousCorrectionScratch;
	/** 实测加速度的一阶低通值：作视界第一步 jerk 限制的种子（原始差分值噪声大）。 */
	FVector FilteredAccelerationCmPerSecSq = FVector::ZeroVector;
	double LastFilterUpdateTimeSeconds = 0.0;
	bool bFilterInitialized = false;
	/** Stop Route 已进入终点位置收敛；锁存到意图/几何真正变化。 */
	bool bTerminalConvergenceActive = false;
	uint64 IntentRevision = 0;
	int64 ActiveIntentId = 0;
	uint64 PlanRevision = 0;
	uint64 PlanGeometryHash = 0;
	uint64 IntentMetadataHash = 0;
	uint64 PlanConfigHash = 0;
	float EstimatedPlanTimeSeconds = 0.0f;
	float EstimatedDistanceCm = 0.0f;
	float PathReferenceScale = 1.0f;
	double PlanStartTimeSeconds = 0.0;
	double LastPlanSolveTimeSeconds = 0.0;
	double NextSolveTimeSeconds = -DBL_MAX;
	FVector LastProjectionStatePositionCm = FVector::ZeroVector;
	double LastProjectionStateTimeSeconds = 0.0;
	bool bProjectionStateInitialized = false;

	bool RefreshPlanForCapability(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability);

	bool SolveVelocityIntent(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool SolvePlan(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference,
		double SolveDeadlineSeconds);
	/** 视界前向滚动：指令投影/jerk 限幅/推力响应滞后/位置速度积分（迭代内与发布前共用）。 */
	void RolloutHorizon(
		const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		const FAircraftRequestedMotionLimits& Limits,
		const TArray<FAircraftMotionPlanSample>& References,
		TConstArrayView<FVector> ControlCorrections,
		float Dt, int32 Steps,
		TArray<FVector>& CommandAccelerationHorizon,
		TArray<FVector>& Positions, TArray<FVector>& Velocities,
		TArray<FVector>& RealizedAccelerations, TArray<float>& ResponseAlphas);
	static FVector ProjectAcceleration(const FVector& Acceleration,
		const FAircraftRequestedMotionLimits& Limits,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		const FVector& VelocityCmPerSec);
	static FVector ProjectControlAcceleration(const FVector& Acceleration,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	static FVector ApplyJerkLimit(const FVector& PreviousAcceleration,
		const FVector& DesiredAcceleration, float DeltaTime,
		const FAircraftRequestedMotionLimits& Limits);
	static FVector ComputeDragCompensation(const FVector& DesiredVelocityWorldCmPerSec,
		const FQuat& BodyRotation, const FAircraftDynamicCapabilitySnapshot& Capability);
};
