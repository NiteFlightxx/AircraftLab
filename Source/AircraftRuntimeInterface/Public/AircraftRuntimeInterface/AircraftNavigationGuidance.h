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

	/**
	 * 求解器最小违约速度的降级档（可行域不可行时求解器算出的 LeastViolation 速度）。
	 * 全分量 < 0 表示本次未产生降级档（哨兵值），消费端走现有路径。
	 * 由发布端可选填充；不填即保持既有行为。
	 */
	FVector DegradedVelocityCmPerSec = FVector(-1.0);

	bool IsValid() const;
	bool IsFresh(double CurrentTimeSeconds) const;
	bool Evaluate(double CurrentTimeSeconds, FAircraftNavigationGuidanceSample& OutSample) const;

	/** 是否携带有效的降级速度档（用于失败路径的中间档消费）。 */
	bool HasDegradedVelocity() const
	{
		return DegradedVelocityCmPerSec.X >= 0.0 || DegradedVelocityCmPerSec.Y >= 0.0
			|| DegradedVelocityCmPerSec.Z >= 0.0;
	}
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
