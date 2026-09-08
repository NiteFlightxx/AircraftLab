#include "AircraftRuntimeInterface/AircraftNavigationGuidance.h"

namespace
{
	bool IsFiniteGuidanceVector(const FVector& Value)
	{
		return !Value.ContainsNaN();
	}

	bool IsFiniteSample(const FAircraftNavigationGuidanceSample& Sample)
	{
		return FMath::IsFinite(Sample.TimeSeconds)
			&& IsFiniteGuidanceVector(Sample.PositionCm)
			&& IsFiniteGuidanceVector(Sample.VelocityCmPerSec)
			&& IsFiniteGuidanceVector(Sample.AccelerationCmPerSecSq);
	}
}

bool FAircraftNavigationGuidance::IsValid() const
{
	if (!FMath::IsFinite(GeneratedAtSeconds) || !FMath::IsFinite(ValidUntilSeconds)
		|| ValidUntilSeconds <= GeneratedAtSeconds || SourceIntentId <= 0)
	{
		return false;
	}
	if (Mode == EAircraftNavigationGuidanceMode::Brake)
	{
		return Samples.IsEmpty();
	}
	if (Samples.Num() < 2 || !FMath::IsNearlyZero(Samples[0].TimeSeconds))
	{
		return false;
	}
	float PreviousTime = -1.0f;
	for (const FAircraftNavigationGuidanceSample& Sample : Samples)
	{
		if (!IsFiniteSample(Sample) || Sample.TimeSeconds <= PreviousTime)
		{
			return false;
		}
		PreviousTime = Sample.TimeSeconds;
	}
	return static_cast<double>(Samples.Last().TimeSeconds) + UE_KINDA_SMALL_NUMBER
		>= ValidUntilSeconds - GeneratedAtSeconds;
}

bool FAircraftNavigationGuidance::IsFresh(const double CurrentTimeSeconds) const
{
	return FMath::IsFinite(CurrentTimeSeconds)
		&& FMath::IsFinite(GeneratedAtSeconds)
		&& FMath::IsFinite(ValidUntilSeconds)
		&& ValidUntilSeconds > GeneratedAtSeconds
		&& CurrentTimeSeconds <= ValidUntilSeconds;
}

bool FAircraftNavigationGuidance::Evaluate(
	const double CurrentTimeSeconds, FAircraftNavigationGuidanceSample& OutSample) const
{
	OutSample = {};
	if (Mode != EAircraftNavigationGuidanceMode::TimedTrajectory
		|| Samples.Num() < 2 || !IsFresh(CurrentTimeSeconds))
	{
		return false;
	}
	const float RelativeTime = static_cast<float>(FMath::Max(
		CurrentTimeSeconds - GeneratedAtSeconds, 0.0));
	if (RelativeTime <= Samples[0].TimeSeconds)
	{
		OutSample = Samples[0];
		return true;
	}
	for (int32 Index = 1; Index < Samples.Num(); ++Index)
	{
		if (RelativeTime <= Samples[Index].TimeSeconds)
		{
			const FAircraftNavigationGuidanceSample& A = Samples[Index - 1];
			const FAircraftNavigationGuidanceSample& B = Samples[Index];
			const float Alpha = FMath::Clamp(
				(RelativeTime - A.TimeSeconds) / (B.TimeSeconds - A.TimeSeconds), 0.0f, 1.0f);
			OutSample.TimeSeconds = RelativeTime;
			OutSample.PositionCm = FMath::Lerp(A.PositionCm, B.PositionCm, Alpha);
			OutSample.VelocityCmPerSec = FMath::Lerp(A.VelocityCmPerSec, B.VelocityCmPerSec, Alpha);
			OutSample.AccelerationCmPerSecSq = FMath::Lerp(
				A.AccelerationCmPerSecSq, B.AccelerationCmPerSecSq, Alpha);
			return true;
		}
	}
	OutSample = Samples.Last();
	return true;
}
