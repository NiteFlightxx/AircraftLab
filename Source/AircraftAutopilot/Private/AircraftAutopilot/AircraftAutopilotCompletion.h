#pragma once

#include "AircraftRuntimeInterface/AircraftFlightControllerInterface.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

namespace UE::AircraftLab::Autopilot::Private
{
	inline bool ResolveCompletionTarget(
		const FAircraftMovementIntent& Intent,
		const bool bHasReference,
		const FAircraftTrajectoryReference& Reference,
		FVector& OutTargetCm)
	{
		switch (Intent.Type)
		{
		case EAircraftMovementIntentType::Hold:
			if (Intent.Hold.bCaptureCurrentPosition)
			{
				if (!bHasReference)
				{
					return false;
				}
				OutTargetCm = Reference.PositionCm;
			}
			else
			{
				OutTargetCm = Intent.Hold.PositionCm;
			}
			return true;
		case EAircraftMovementIntentType::Route:
			if (Intent.Route.PointsCm.IsEmpty())
			{
				return false;
			}
			OutTargetCm = Intent.Route.PointsCm.Last();
			return true;
		case EAircraftMovementIntentType::TimedTrajectory:
			if (Intent.TimedTrajectory.Samples.IsEmpty())
			{
				return false;
			}
			OutTargetCm = Intent.TimedTrajectory.Samples.Last().PositionCm;
			return true;
		default:
			return false;
		}
	}

	inline bool IsPlanComplete(
		const EAircraftMovementIntentType Type,
		const bool bHasReference,
		const float Progress)
	{
		return (Type != EAircraftMovementIntentType::Route
				&& Type != EAircraftMovementIntentType::TimedTrajectory)
			|| (bHasReference && Progress >= 0.999f);
	}

	inline bool HasStableCompletion(
		const bool bArrivalConditionsSatisfied,
		const float StableTimeSeconds,
		const float RequiredStableTimeSeconds)
	{
		return bArrivalConditionsSatisfied
			&& StableTimeSeconds + UE_SMALL_NUMBER
				>= FMath::Max(RequiredStableTimeSeconds, 0.0f);
	}

	inline FAircraftMovementIntent BuildTerminalContinuationIntent(
		const FAircraftMovementIntent& CompletedIntent,
		const FVector& PositionCm,
		const float FixedYawDegrees)
	{
		if (CompletedIntent.Type == EAircraftMovementIntentType::Route
			&& CompletedIntent.Completion.ArrivalMode == EAircraftArrivalMode::Stop)
		{
			return CompletedIntent;
		}

		FAircraftMovementIntent Continuation;
		Continuation.Type = EAircraftMovementIntentType::Hold;
		Continuation.Hold.PositionCm = PositionCm;
		Continuation.Hold.bCaptureCurrentPosition = false;
		Continuation.Limits = CompletedIntent.Limits;
		Continuation.bHasRequestedMotionLimits =
			CompletedIntent.bHasRequestedMotionLimits;
		Continuation.Heading.Mode = EAircraftHeadingMode::FixedYaw;
		Continuation.Heading.FixedYawDegrees = FixedYawDegrees;
		return Continuation;
	}
}
