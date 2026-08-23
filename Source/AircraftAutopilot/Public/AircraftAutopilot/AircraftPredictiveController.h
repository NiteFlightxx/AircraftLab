#pragma once

#include "CoreMinimal.h"
#include "AircraftAutopilot/AircraftMotionPlan.h"

/** Fixed-horizon constrained contouring controller for the common motion plan. */
class AIRCRAFTAUTOPILOT_API FAircraftPredictiveController
{
public:
	bool SetIntent(const FAircraftMovementIntent& Intent, int64 IntentId, uint64 IntentRevision,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability);

	bool Update(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);

	void Reset();
	const FAircraftAutopilotDiagnostics& GetDiagnostics() const { return Diagnostics; }
	const FAircraftMotionPlan& GetPlan() const { return Plan; }

private:
	FAircraftMotionPlan Plan;
	FAircraftAutopilotRuntimeConfig RuntimeConfig;
	FAircraftAutopilotDiagnostics Diagnostics;
	FAircraftTrajectoryReference LastReference;
	FVector LastVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector CandidateVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	TArray<FVector> AccelerationHorizon;
	uint64 IntentRevision = 0;
	int64 ActiveIntentId = 0;
	uint64 PlanRevision = 0;
	float EstimatedPlanTimeSeconds = 0.0f;
	float EstimatedDistanceCm = 0.0f;
	double PlanStartTimeSeconds = 0.0;
	double NextSolveTimeSeconds = -DBL_MAX;

	bool SolveVelocityIntent(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool SolvePlan(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	static FVector ProjectAcceleration(const FVector& Acceleration,
		const FAircraftRequestedMotionLimits& Limits,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		const FVector& VelocityCmPerSec);
	static FVector ProjectControlAcceleration(const FVector& Acceleration,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	static FVector ApplyJerkLimit(const FVector& PreviousAcceleration,
		const FVector& DesiredAcceleration, float DeltaTime,
		const FAircraftRequestedMotionLimits& Limits);
	void ApplyYawConstraints(const FAircraftVehicleStateSnapshot& State,
		const FAircraftRequestedMotionLimits& Limits, float DeltaTime,
		float DesiredYawDegrees,
		FAircraftTrajectoryReference& InOutReference) const;
	static FVector ComputeDragCompensation(const FVector& DesiredVelocityWorldCmPerSec,
		const FQuat& BodyRotation, const FAircraftDynamicCapabilitySnapshot& Capability);
};
