#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

namespace
{
	bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}
}

bool FAircraftRequestedMotionLimits::IsValid() const
{
	return FMath::IsFinite(CruiseSpeedCmPerSec) && CruiseSpeedCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxAccelerationCmPerSecSq) && MaxAccelerationCmPerSecSq > 0.0f
		&& FMath::IsFinite(MaxDecelerationCmPerSecSq) && MaxDecelerationCmPerSecSq > 0.0f
		&& FMath::IsFinite(MaxJerkCmPerSecCubed) && MaxJerkCmPerSecCubed > 0.0f
		&& FMath::IsFinite(MaxClimbRateCmPerSec) && MaxClimbRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxDescentRateCmPerSec) && MaxDescentRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxVerticalAccelerationCmPerSecSq) && MaxVerticalAccelerationCmPerSecSq > 0.0f
		&& FMath::IsFinite(MaxVerticalJerkCmPerSecCubed) && MaxVerticalJerkCmPerSecCubed > 0.0f
		&& FMath::IsFinite(MaxYawRateDegPerSec) && MaxYawRateDegPerSec >= 0.0f
		&& FMath::IsFinite(MaxYawAccelerationDegPerSecSq) && MaxYawAccelerationDegPerSecSq > 0.0f
		&& FMath::IsFinite(MaxYawJerkDegPerSecCubed) && MaxYawJerkDegPerSecCubed > 0.0f;
}

bool FAircraftMovementIntent::IsValid() const
{
	if (!Limits.IsValid() || !FMath::IsFinite(TimeoutSeconds) || TimeoutSeconds < 0.0f)
	{
		return false;
	}
	if (!FMath::IsFinite(Heading.FixedYawDegrees)
		|| !IsFiniteVector(Heading.TargetPositionCm)
		|| !FMath::IsFinite(Completion.TerminalSpeedCmPerSec)
		|| !FMath::IsFinite(Completion.HorizontalToleranceCm)
		|| !FMath::IsFinite(Completion.VerticalToleranceCm)
		|| !FMath::IsFinite(Completion.SpeedToleranceCmPerSec)
		|| !FMath::IsFinite(Completion.YawToleranceDegrees)
		|| !FMath::IsFinite(Completion.StableTimeSeconds)
		|| Completion.TerminalSpeedCmPerSec < 0.0f
		|| Completion.HorizontalToleranceCm < 0.0f
		|| Completion.VerticalToleranceCm < 0.0f
		|| Completion.SpeedToleranceCmPerSec < 0.0f
		|| Completion.YawToleranceDegrees < 0.0f
		|| Completion.YawToleranceDegrees > 180.0f
		|| Completion.StableTimeSeconds < 0.0f)
	{
		return false;
	}

	switch (Type)
	{
	case EAircraftMovementIntentType::Hold:
		return Hold.bCaptureCurrentPosition || Hold.TargetActor || IsFiniteVector(Hold.PositionCm);
	case EAircraftMovementIntentType::Velocity:
		return IsFiniteVector(Velocity.VelocityCmPerSec);
	case EAircraftMovementIntentType::Route:
		if (Route.PointsCm.Num() < (Route.bClosed ? 3 : 2))
		{
			return false;
		}
		for (const FVector& Point : Route.PointsCm)
		{
			if (!IsFiniteVector(Point))
			{
				return false;
			}
		}
		for (int32 CorridorIndex = 0; CorridorIndex < Route.Corridor.Num(); ++CorridorIndex)
		{
			const FAircraftSafeCorridorSegment& Segment = Route.Corridor[CorridorIndex];
			if (!FMath::IsFinite(Segment.StartDistanceCm)
				|| !FMath::IsFinite(Segment.EndDistanceCm)
				|| Segment.StartDistanceCm < 0.0f
				|| Segment.EndDistanceCm <= Segment.StartDistanceCm
				|| (CorridorIndex > 0 && Segment.StartDistanceCm
					< Route.Corridor[CorridorIndex - 1].EndDistanceCm))
			{
				return false;
			}
			for (const FPlane& Plane : Segment.BoundaryPlanes)
			{
				const FVector Normal(Plane.X, Plane.Y, Plane.Z);
				if (!IsFiniteVector(Normal) || !FMath::IsFinite(Plane.W)
					|| Normal.IsNearlyZero())
				{
					return false;
				}
			}
		}
		return true;
	case EAircraftMovementIntentType::Orbit:
		return IsFiniteVector(Orbit.CenterCm)
			&& FMath::IsFinite(Orbit.RadiusCm) && Orbit.RadiusCm > UE_SMALL_NUMBER
			&& FMath::IsFinite(Orbit.AngularRateDegPerSec)
			&& FMath::Abs(Orbit.AngularRateDegPerSec) > UE_SMALL_NUMBER;
	case EAircraftMovementIntentType::TimedTrajectory:
		if (TimedTrajectory.Samples.Num() < 2)
		{
			return false;
		}
		for (int32 Index = 0; Index < TimedTrajectory.Samples.Num(); ++Index)
		{
			const FAircraftTimedTrajectorySample& Sample = TimedTrajectory.Samples[Index];
			if (!FMath::IsFinite(Sample.TimeSeconds) || Sample.TimeSeconds < 0.0f
				|| !IsFiniteVector(Sample.PositionCm)
				|| !IsFiniteVector(Sample.VelocityCmPerSec)
				|| !IsFiniteVector(Sample.AccelerationCmPerSecSq)
				|| (Index > 0 && Sample.TimeSeconds <= TimedTrajectory.Samples[Index - 1].TimeSeconds))
			{
				return false;
			}
		}
		return true;
	default:
		return false;
	}
}
