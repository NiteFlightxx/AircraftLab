#pragma once

#include "CoreMinimal.h"
#include "AircraftAutopilot/AircraftMpccController.h"

/**
 * Single owner of intent, immutable motion plan, cursor and published reference.
 * MPCC is invoked only by UpdateFlightController; the other backends sample the plan
 * deterministically and never allocate an optimization horizon.
 */
class AIRCRAFTAUTOPILOT_API FAircraftTrajectoryRuntime
{
public:
	bool SetIntent(const FAircraftMovementIntent& Intent, int64 IntentId, uint64 IntentRevision,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	void Reset();
	/** Rebase absolute clocks after a pause while preserving the active plan and cursor. */
	void RebaseTime(double TimeSeconds);

	bool UpdateFlightController(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool UpdatePhysicsConstraint(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);
	bool UpdateKinematic(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FAircraftTrajectoryReference& OutReference);

	const FAircraftMotionPlan& GetPlan() const { return MpccController.GetPlan(); }
	const FAircraftAutopilotDiagnostics& GetDiagnostics() const { return Diagnostics; }

private:
	FAircraftMpccController MpccController;
	FAircraftAutopilotDiagnostics Diagnostics;
	FAircraftTrajectoryReference LastReference;
	FVector VelocityReferenceCmPerSec = FVector::ZeroVector;
	FVector VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
	FVector PositionReferenceCm = FVector::ZeroVector;
	float PlanTimeSeconds = 0.0f;
	float PlanDistanceCm = 0.0f;
	float ProgressScale = 1.0f;
	float GovernorScaleCm = 100.0f;
	float GovernorResponseRatePerSecond = 5.0f;
	double LastUpdateTimeSeconds = 0.0;

	bool UpdateDeterministic(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		bool bUseProgressGovernor, bool bUseDynamics,
		FAircraftTrajectoryReference& OutReference);
	bool UpdateVelocity(const FAircraftVehicleStateSnapshot& State,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		bool bUseDynamics, float DeltaTime,
		FAircraftTrajectoryReference& OutReference);
	void FinalizeReference(const FAircraftVehicleStateSnapshot& State,
		FAircraftTrajectoryReference& OutReference);
};
