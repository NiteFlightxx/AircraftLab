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
	uint64 IntentRevision = 0;
	int64 ActiveIntentId = 0;
	uint64 PlanRevision = 0;
	float EstimatedPlanTimeSeconds = 0.0f;
	float EstimatedDistanceCm = 0.0f;
	float PathReferenceScale = 1.0f;
	double PlanStartTimeSeconds = 0.0;
	double LastPlanSolveTimeSeconds = 0.0;
	double NextSolveTimeSeconds = -DBL_MAX;

	bool RefreshPlanForCapability(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability);

	bool SolveVelocityIntent(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool SolvePlan(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference,
		double SolveDeadlineSeconds);
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
