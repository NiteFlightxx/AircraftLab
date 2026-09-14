#pragma once

#include "CoreMinimal.h"

/** Hard limits shared by manual-rate and automatic-angle yaw reference generation. */
struct AIRCRAFT_API FAircraftYawReferenceLimits
{
	float MaxRateDegPerSec = 0.0f;
	float MaxAccelerationDegPerSecSq = 0.0f;
	float MaxJerkDegPerSecCubed = 0.0f;
	float ResponseTimeSeconds = 0.04f;

	bool IsValid() const;
};

/** Explicit control phase for rate-command yaw. */
enum class EAircraftYawReferencePhase : uint8
{
	AngleTracking,
	RateTracking,
	RateBraking,
	RateHold
};

/** Complete yaw reference state, including the rate-to-hold transition phase. */
struct AIRCRAFT_API FAircraftYawReferenceState
{
	float YawDegrees = 0.0f;
	float RateDegPerSec = 0.0f;
	float AccelerationDegPerSecSq = 0.0f;
	EAircraftYawReferencePhase Phase = EAircraftYawReferencePhase::RateHold;
	float BrakingDirection = 0.0f;
	bool bInitialized = false;

	void Reset()
	{
		*this = FAircraftYawReferenceState();
	}
};

/**
 * Stateless yaw reference integrator. The caller owns the state in its execution domain.
 * Both command forms publish a coherent tuple where d(Yaw)/dt == Rate and d(Rate)/dt == Acceleration.
 */
class AIRCRAFT_API FAircraftYawReferenceDynamics
{
public:
	/** Manual/Acro command: shape a desired yaw rate, then integrate the eventual hold angle. */
	static bool UpdateRateCommand(
		float TargetRateDegPerSec,
		float MeasuredYawDegrees,
		float MeasuredRateDegPerSec,
		float DeltaSeconds,
		const FAircraftYawReferenceLimits& Limits,
		FAircraftYawReferenceState& InOutState);

	/** Automatic command: converge to a desired yaw without publishing a fixed angle with non-zero rate. */
	static bool UpdateAngleCommand(
		float TargetYawDegrees,
		float MeasuredYawDegrees,
		float MeasuredRateDegPerSec,
		float DeltaSeconds,
		const FAircraftYawReferenceLimits& Limits,
		FAircraftYawReferenceState& InOutState);
};
