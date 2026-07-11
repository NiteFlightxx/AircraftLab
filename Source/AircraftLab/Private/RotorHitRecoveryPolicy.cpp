#include "RotorHitRecoveryPolicyAsset.h"

float RotorHitRecoveryPolicy::ComputeRecoveryAlpha(
	float TimeSinceHitSeconds,
	float UnstableDurationSeconds,
	float RecoveryRampSeconds)
{
	const float RampElapsed = FMath::Max(TimeSinceHitSeconds - UnstableDurationSeconds, 0.0f);
	return RecoveryRampSeconds <= UE_SMALL_NUMBER
		? 1.0f
		: FMath::Clamp(RampElapsed / RecoveryRampSeconds, 0.0f, 1.0f);
}

bool RotorHitRecoveryPolicy::IsKinematicStateStable(
	const FDroneKinematicState& State,
	float MaximumTiltDegrees,
	float MaximumAngularRateDegPerSec,
	float MaximumVerticalSpeedCmPerSec)
{
	const float RollDegrees = FMath::Abs(FRotator::NormalizeAxis(State.AttitudeDegrees.Roll));
	const float PitchDegrees = FMath::Abs(FRotator::NormalizeAxis(State.AttitudeDegrees.Pitch));
	return FMath::Max(RollDegrees, PitchDegrees) <= MaximumTiltDegrees
		&& State.AngularVelocityBodyDegreesPerSec.Size() <= MaximumAngularRateDegPerSec
		&& FMath::Abs(State.VelocityCmPerSec.Z) <= MaximumVerticalSpeedCmPerSec;
}
