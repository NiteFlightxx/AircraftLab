#pragma once

#include "CoreMinimal.h"

/** A velocity-space half-space. Velocities are valid when Normal dot Velocity >= MinimumProjectionCmPerSec. */
struct AIRCRAFTNAVIGATION_API FAircraftVelocityConstraintPlane
{
	FVector Normal = FVector::ZeroVector;
	double MinimumProjectionCmPerSec = 0.0;

	bool IsValid() const;
	double ComputeViolation(const FVector& VelocityCmPerSec) const;
};

/** Immutable value-only state consumed by the three-dimensional ORCA solver. */
struct AIRCRAFTNAVIGATION_API FAircraftAvoidanceAgentState
{
	uint64 StableId = 0;
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector CommandedVelocityCmPerSec = FVector::ZeroVector;
	float BodyRadiusCm = 0.0f;
	float TrackingReserveCm = 0.0f;
	uint8 Priority = 128;
	bool bAnchored = false;

	bool IsValid() const;
};

/** Physical reachability and collision-prediction limits for one avoidance solve. */
struct AIRCRAFTNAVIGATION_API FAircraftAvoidanceLimits
{
	float DeltaTimeSeconds = 0.1f;
	float TimeHorizonSeconds = 2.0f;
	float SeparationPaddingCm = 20.0f;
	float MaxHorizontalSpeedCmPerSec = 0.0f;
	float MaxHorizontalAccelerationCmPerSecSq = 0.0f;
	float MaxHorizontalDecelerationCmPerSecSq = 0.0f;
	float MaxVerticalAccelerationCmPerSecSq = 0.0f;
	float MaxClimbRateCmPerSec = 0.0f;
	float MaxDescentRateCmPerSec = 0.0f;
	float MaxHorizontalJerkCmPerSecCubed = 0.0f;
	float MaxVerticalJerkCmPerSecCubed = 0.0f;
	float SmoothingWeight = 0.15f;
	int32 HorizontalPlaneCount = 16;

	bool IsValid() const;
};

struct AIRCRAFTNAVIGATION_API FAircraftAvoidanceResult
{
	FVector TargetVelocityCmPerSec = FVector::ZeroVector;
	float MinimumPredictedSeparationCm = TNumericLimits<float>::Max();
	float EarliestConflictTimeSeconds = TNumericLimits<float>::Max();
	float MaximumConstraintViolation = 0.0f;
	int32 ConstraintPlaneCount = 0;
	uint64 MostDangerousAgentId = 0;
	bool bAvoidanceRequired = false;
	bool bFeasible = false;
};

/** Stateless, deterministic, capability-constrained three-dimensional ORCA solver. */
class AIRCRAFTNAVIGATION_API FAircraftOrcaSolver
{
public:
	static FAircraftAvoidanceResult Solve(
		const FAircraftAvoidanceAgentState& Self,
		TConstArrayView<FAircraftAvoidanceAgentState> Neighbors,
		const FVector& PreferredVelocityCmPerSec,
		const FVector& PreviousCommandAccelerationCmPerSecSq,
		const FAircraftAvoidanceLimits& Limits,
		TConstArrayView<FAircraftVelocityConstraintPlane> EnvironmentPlanes);
};
