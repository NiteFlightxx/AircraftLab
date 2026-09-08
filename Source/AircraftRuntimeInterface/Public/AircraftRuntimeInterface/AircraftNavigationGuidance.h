#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"

#include "AircraftNavigationGuidance.generated.h"

/** Short-lived world-space guidance layered over, but never replacing, a MovementIntent. */
UENUM(BlueprintType)
enum class EAircraftNavigationGuidanceMode : uint8
{
	TimedTrajectory,
	Brake
};

UENUM(BlueprintType)
enum class EAircraftNavigationGuidanceState : uint8
{
	Inactive,
	Applied,
	Braking
};

UENUM(BlueprintType)
enum class EAircraftNavigationGuidanceFailureReason : uint8
{
	None,
	InvalidGuidance,
	Unavailable,
	Expired,
	IntentMismatch
};

/** A sample relative to FAircraftNavigationGuidance::GeneratedAtSeconds. */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftNavigationGuidanceSample
{
	float TimeSeconds = 0.0f;

	FVector PositionCm = FVector::ZeroVector;

	FVector VelocityCmPerSec = FVector::ZeroVector;

	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
};

/**
 * Immutable bounded-horizon navigation output. Providers publish this through a thread-safe
 * shared snapshot; neither Aircraft nor the execution domain mutates it.
 */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftNavigationGuidance
{
	EAircraftNavigationGuidanceMode Mode = EAircraftNavigationGuidanceMode::TimedTrajectory;

	/** MovementIntent identity this guidance was generated from. */
	int64 SourceIntentId = 0;

	uint64 SourceIntentRevision = 0;

	double GeneratedAtSeconds = 0.0;

	double ValidUntilSeconds = 0.0;

	TArray<FAircraftNavigationGuidanceSample> Samples;

	bool IsValid() const;
	bool IsFresh(double CurrentTimeSeconds) const;
	bool Evaluate(double CurrentTimeSeconds, FAircraftNavigationGuidanceSample& OutSample) const;
};

struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftNavigationGuidanceStatus
{
	uint64 Revision = 0;

	EAircraftNavigationGuidanceState State = EAircraftNavigationGuidanceState::Inactive;

	EAircraftNavigationGuidanceFailureReason FailureReason = EAircraftNavigationGuidanceFailureReason::None;

	double GeneratedAtSeconds = 0.0;

	double ValidUntilSeconds = 0.0;
};

/** Read-only state and physical limits exported to an external navigation/avoidance owner. */
struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftNavigationAgentSnapshot
{
	FAircraftVehicleStateSnapshot VehicleState;
	FAircraftDynamicCapabilitySnapshot Capability;
	FAircraftTrajectoryReference NominalReference;
	FAircraftNavigationGuidanceStatus GuidanceStatus;
	bool bValid = false;
};
