#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftNavigationGuidance.h"

/** Value-only settings used to build a bounded, jerk-limited guidance trajectory. */
struct AIRCRAFTNAVIGATION_API FAircraftGuidanceTrajectorySettings
{
	int64 SourceIntentId = 0;
	uint64 SourceIntentRevision = 0;
	float SolveDeltaTimeSeconds = 0.1f;
	float HorizonSeconds = 0.5f;
	float SampleIntervalSeconds = 0.1f;
	float ValiditySeconds = 0.25f;
	FVector PreviousCommandAccelerationCmPerSecSq = FVector::ZeroVector;

	bool IsValid() const;
};

class AIRCRAFTNAVIGATION_API FAircraftGuidanceTrajectoryBuilder
{
public:
	/** Builds a trajectory from a one-step velocity already proven reachable by the avoidance solver. */
	static bool BuildVelocityGuidance(
		const FAircraftNavigationAgentSnapshot& AircraftState,
		const FVector& TargetVelocityCmPerSec,
		const FAircraftGuidanceTrajectorySettings& Settings,
		FAircraftNavigationGuidance& OutGuidance,
		FVector& OutCommandAccelerationCmPerSecSq);
};
