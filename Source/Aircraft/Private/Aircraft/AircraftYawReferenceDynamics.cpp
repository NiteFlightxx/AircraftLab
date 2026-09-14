#include "Aircraft/AircraftYawReferenceDynamics.h"

namespace
{
	constexpr float MaximumSubstepSeconds = 1.0f / 120.0f;
	constexpr float MaximumAcceptedDeltaSeconds = 0.25f;
	constexpr float MinimumStateMagnitude = 1.e-3f;
	constexpr float RateCommandDeadbandDegPerSec = 1.e-3f;
	constexpr float ReferenceStopRateDegPerSec = 0.1f;
	constexpr float ReferenceStopAccelerationDegPerSecSq = 0.5f;
	constexpr float MeasuredStopRateDegPerSec = 1.0f;

	bool IsFiniteState(const FAircraftYawReferenceState& State)
	{
		return FMath::IsFinite(State.YawDegrees)
			&& FMath::IsFinite(State.RateDegPerSec)
			&& FMath::IsFinite(State.AccelerationDegPerSecSq);
	}

	void InitializeState(
		const float MeasuredYawDegrees,
		const float MeasuredRateDegPerSec,
		const FAircraftYawReferenceLimits& Limits,
		FAircraftYawReferenceState& State)
	{
		State.YawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees);
		State.RateDegPerSec = FMath::Clamp(
			MeasuredRateDegPerSec, -Limits.MaxRateDegPerSec, Limits.MaxRateDegPerSec);
		State.AccelerationDegPerSecSq = 0.0f;
		State.Phase = EAircraftYawReferencePhase::RateHold;
		State.BrakingDirection = 0.0f;
		State.bInitialized = true;
	}

	float ResolveRateNaturalFrequency(
		const float RateError,
		const FAircraftYawReferenceLimits& Limits)
	{
		const float ErrorMagnitude = FMath::Max(FMath::Abs(RateError), MinimumStateMagnitude);
		const float ResponseOmega = 1.0f / Limits.ResponseTimeSeconds;
		const float JerkOmega = FMath::Sqrt(Limits.MaxJerkDegPerSecCubed / ErrorMagnitude);
		const float AccelerationOmega = 4.0f
			* Limits.MaxAccelerationDegPerSecSq / ErrorMagnitude;
		return FMath::Max(FMath::Min3(ResponseOmega, JerkOmega, AccelerationOmega), UE_SMALL_NUMBER);
	}

	float ResolveAngleNaturalFrequency(
		const float YawError,
		const FAircraftYawReferenceLimits& Limits)
	{
		const float ErrorMagnitude = FMath::Max(FMath::Abs(YawError), MinimumStateMagnitude);
		const float ResponseOmega = 1.0f / Limits.ResponseTimeSeconds;
		const float RateOmega = 3.0f * Limits.MaxRateDegPerSec / ErrorMagnitude;
		const float AccelerationOmega = FMath::Sqrt(
			3.0f * Limits.MaxAccelerationDegPerSecSq / ErrorMagnitude);
		const float JerkOmega = FMath::Pow(
			6.0f * Limits.MaxJerkDegPerSecCubed / ErrorMagnitude, 1.0f / 3.0f);
		return FMath::Max(
			FMath::Min(FMath::Min(ResponseOmega, RateOmega),
				FMath::Min(AccelerationOmega, JerkOmega)),
			UE_SMALL_NUMBER);
	}

	void IntegrateStep(
		const float JerkDegPerSecCubed,
		const float StepSeconds,
		const FAircraftYawReferenceLimits& Limits,
		FAircraftYawReferenceState& State)
	{
		const float PreviousRate = State.RateDegPerSec;
		State.AccelerationDegPerSecSq = FMath::Clamp(
			State.AccelerationDegPerSecSq + JerkDegPerSecCubed * StepSeconds,
			-Limits.MaxAccelerationDegPerSecSq,
			Limits.MaxAccelerationDegPerSecSq);
		State.RateDegPerSec = FMath::Clamp(
			State.RateDegPerSec + State.AccelerationDegPerSecSq * StepSeconds,
			-Limits.MaxRateDegPerSec,
			Limits.MaxRateDegPerSec);
		State.YawDegrees = FRotator::NormalizeAxis(
			State.YawDegrees + 0.5f * (PreviousRate + State.RateDegPerSec) * StepSeconds);
	}

	void IntegrateRateBrakingStep(
		const float StepSeconds,
		const FAircraftYawReferenceLimits& Limits,
		FAircraftYawReferenceState& State)
	{
		const float Direction = State.BrakingDirection;
		if (FMath::IsNearlyZero(Direction))
		{
			State.RateDegPerSec = 0.0f;
			State.AccelerationDegPerSecSq = 0.0f;
			return;
		}

		const float DirectedRate = FMath::Max(State.RateDegPerSec * Direction, 0.0f);
		const float DirectedAcceleration = State.AccelerationDegPerSecSq * Direction;
		if (DirectedRate <= ReferenceStopRateDegPerSec)
		{
			const float MaximumAccelerationStep =
				Limits.MaxJerkDegPerSecCubed * StepSeconds;
			State.RateDegPerSec = 0.0f;
			State.AccelerationDegPerSecSq += FMath::Clamp(
				-State.AccelerationDegPerSecSq,
				-MaximumAccelerationStep, MaximumAccelerationStep);
			return;
		}
		const float AccelerationReleaseRate = DirectedAcceleration < 0.0f
			? FMath::Square(DirectedAcceleration)
				/ (2.0f * Limits.MaxJerkDegPerSecCubed)
			: 0.0f;
		const float DirectedJerk = DirectedAcceleration < 0.0f
			&& DirectedRate <= AccelerationReleaseRate
			? Limits.MaxJerkDegPerSecCubed
			: (DirectedAcceleration > -Limits.MaxAccelerationDegPerSecSq
				? -Limits.MaxJerkDegPerSecCubed
				: 0.0f);

		State.AccelerationDegPerSecSq = FMath::Clamp(
			State.AccelerationDegPerSecSq + Direction * DirectedJerk * StepSeconds,
			-Limits.MaxAccelerationDegPerSecSq,
			Limits.MaxAccelerationDegPerSecSq);
		const float NextDirectedRate = FMath::Max(
			0.0f, DirectedRate + State.AccelerationDegPerSecSq * Direction * StepSeconds);
		State.RateDegPerSec = Direction * NextDirectedRate;
		if (NextDirectedRate <= ReferenceStopRateDegPerSec)
		{
			State.RateDegPerSec = 0.0f;
		}
	}

	template <typename StepFunction>
	bool UpdateReference(
		const float MeasuredYawDegrees,
		const float MeasuredRateDegPerSec,
		const float DeltaSeconds,
		const FAircraftYawReferenceLimits& Limits,
		FAircraftYawReferenceState& State,
		StepFunction&& Step)
	{
		if (!FMath::IsFinite(MeasuredYawDegrees)
			|| !FMath::IsFinite(MeasuredRateDegPerSec)
			|| !FMath::IsFinite(DeltaSeconds)
			|| DeltaSeconds <= 0.0f
			|| DeltaSeconds > MaximumAcceptedDeltaSeconds
			|| !Limits.IsValid())
		{
			return false;
		}
		if (!State.bInitialized || !IsFiniteState(State))
		{
			InitializeState(MeasuredYawDegrees, MeasuredRateDegPerSec, Limits, State);
		}
		if (Limits.MaxRateDegPerSec <= UE_SMALL_NUMBER
			|| Limits.MaxAccelerationDegPerSecSq <= UE_SMALL_NUMBER
			|| Limits.MaxJerkDegPerSecCubed <= UE_SMALL_NUMBER)
		{
			State.YawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees);
			State.RateDegPerSec = 0.0f;
			State.AccelerationDegPerSecSq = 0.0f;
			return true;
		}

		const int32 SubstepCount = FMath::Max(
			1, FMath::CeilToInt(DeltaSeconds / MaximumSubstepSeconds));
		const float StepSeconds = DeltaSeconds / static_cast<float>(SubstepCount);
		for (int32 Substep = 0; Substep < SubstepCount; ++Substep)
		{
			Step(StepSeconds);
		}
		return IsFiniteState(State);
	}
}

bool FAircraftYawReferenceLimits::IsValid() const
{
	return FMath::IsFinite(MaxRateDegPerSec) && MaxRateDegPerSec >= 0.0f
		&& FMath::IsFinite(MaxAccelerationDegPerSecSq) && MaxAccelerationDegPerSecSq >= 0.0f
		&& FMath::IsFinite(MaxJerkDegPerSecCubed) && MaxJerkDegPerSecCubed >= 0.0f
		&& FMath::IsFinite(ResponseTimeSeconds) && ResponseTimeSeconds > 0.0f;
}

bool FAircraftYawReferenceDynamics::UpdateRateCommand(
	const float TargetRateDegPerSec,
	const float MeasuredYawDegrees,
	const float MeasuredRateDegPerSec,
	const float DeltaSeconds,
	const FAircraftYawReferenceLimits& Limits,
	FAircraftYawReferenceState& InOutState)
{
	if (!FMath::IsFinite(TargetRateDegPerSec))
	{
		return false;
	}
	const float LimitedTargetRate = FMath::Clamp(
		TargetRateDegPerSec, -Limits.MaxRateDegPerSec, Limits.MaxRateDegPerSec);
	const bool bHasRateCommand = FMath::Abs(LimitedTargetRate)
		> RateCommandDeadbandDegPerSec;
	if (bHasRateCommand)
	{
		InOutState.Phase = EAircraftYawReferencePhase::RateTracking;
		InOutState.BrakingDirection = 0.0f;
	}
	else if (InOutState.Phase == EAircraftYawReferencePhase::RateTracking)
	{
		InOutState.Phase = EAircraftYawReferencePhase::RateBraking;
		InOutState.BrakingDirection = FMath::Sign(
			FMath::Abs(InOutState.RateDegPerSec) > ReferenceStopRateDegPerSec
				? InOutState.RateDegPerSec
				: MeasuredRateDegPerSec);
	}

	const bool bUpdated = UpdateReference(
		MeasuredYawDegrees, MeasuredRateDegPerSec, DeltaSeconds, Limits, InOutState,
		[&InOutState, LimitedTargetRate, bHasRateCommand, &Limits](const float StepSeconds)
		{
			if (!bHasRateCommand)
			{
				if (InOutState.Phase == EAircraftYawReferencePhase::RateBraking)
				{
					IntegrateRateBrakingStep(StepSeconds, Limits, InOutState);
				}
				return;
			}
			InOutState.Phase = EAircraftYawReferencePhase::RateTracking;
			const float RateError = LimitedTargetRate - InOutState.RateDegPerSec;
			const float Omega = ResolveRateNaturalFrequency(RateError, Limits);
			const float RequestedJerk = Omega * Omega * RateError
				- 2.0f * Omega * InOutState.AccelerationDegPerSecSq;
			IntegrateStep(
				FMath::Clamp(RequestedJerk,
					-Limits.MaxJerkDegPerSecCubed, Limits.MaxJerkDegPerSecCubed),
				StepSeconds, Limits, InOutState);
		});
	if (!bUpdated)
	{
		return false;
	}

	if (InOutState.Phase == EAircraftYawReferencePhase::RateTracking
		|| InOutState.Phase == EAircraftYawReferencePhase::RateBraking)
	{
		// Rate mode must not leave a simultaneous stale angle target. Keeping the
		// angle target on the measured heading makes the rate loop authoritative;
		// otherwise rigid-body/motor lag turns the integrated reference into a
		// reverse angle command as soon as the stick is released.
		InOutState.YawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees);
	}
	if (InOutState.Phase == EAircraftYawReferencePhase::RateBraking
		&& FMath::Abs(InOutState.RateDegPerSec) <= ReferenceStopRateDegPerSec
		&& FMath::Abs(InOutState.AccelerationDegPerSecSq)
			<= ReferenceStopAccelerationDegPerSecSq
		&& FMath::Abs(MeasuredRateDegPerSec) <= MeasuredStopRateDegPerSec)
	{
		InOutState.YawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees);
		InOutState.RateDegPerSec = 0.0f;
		InOutState.AccelerationDegPerSecSq = 0.0f;
		InOutState.Phase = EAircraftYawReferencePhase::RateHold;
		InOutState.BrakingDirection = 0.0f;
	}
	return true;
}

bool FAircraftYawReferenceDynamics::UpdateAngleCommand(
	const float TargetYawDegrees,
	const float MeasuredYawDegrees,
	const float MeasuredRateDegPerSec,
	const float DeltaSeconds,
	const FAircraftYawReferenceLimits& Limits,
	FAircraftYawReferenceState& InOutState)
{
	if (!FMath::IsFinite(TargetYawDegrees))
	{
		return false;
	}
	const float NormalizedTargetYaw = FRotator::NormalizeAxis(TargetYawDegrees);
	const bool bUpdated = UpdateReference(
		MeasuredYawDegrees, MeasuredRateDegPerSec, DeltaSeconds, Limits, InOutState,
		[&InOutState, NormalizedTargetYaw, &Limits](const float StepSeconds)
		{
			const float YawError = FMath::FindDeltaAngleDegrees(
				InOutState.YawDegrees, NormalizedTargetYaw);
			const float Omega = ResolveAngleNaturalFrequency(YawError, Limits);
			const float RequestedJerk = Omega * Omega * Omega * YawError
				- 3.0f * Omega * Omega * InOutState.RateDegPerSec
				- 3.0f * Omega * InOutState.AccelerationDegPerSecSq;
			IntegrateStep(
				FMath::Clamp(RequestedJerk,
					-Limits.MaxJerkDegPerSecCubed, Limits.MaxJerkDegPerSecCubed),
				StepSeconds, Limits, InOutState);
		});
	if (bUpdated)
	{
		InOutState.Phase = EAircraftYawReferencePhase::AngleTracking;
		InOutState.BrakingDirection = 0.0f;
	}
	if (bUpdated
		&& FMath::Abs(FMath::FindDeltaAngleDegrees(
			InOutState.YawDegrees, NormalizedTargetYaw)) < 1.e-3f
		&& FMath::Abs(InOutState.RateDegPerSec) < 1.e-2f
		&& FMath::Abs(InOutState.AccelerationDegPerSecSq) < 1.e-1f)
	{
		InOutState.YawDegrees = NormalizedTargetYaw;
		InOutState.RateDegPerSec = 0.0f;
		InOutState.AccelerationDegPerSecSq = 0.0f;
	}
	return bUpdated;
}
