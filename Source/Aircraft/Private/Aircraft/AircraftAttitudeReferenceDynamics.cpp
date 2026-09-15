#include "Aircraft/AircraftAttitudeReferenceDynamics.h"

#include "Aircraft/AircraftAttitudeReference.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	constexpr float MaximumIntegrationStepSeconds = 1.0f / 120.0f;
	constexpr float MaximumCatchUpSeconds = 0.25f;

	bool IsFiniteVector(const FVector& Value)
	{
		return !Value.ContainsNaN()
			&& FMath::IsFinite(Value.X)
			&& FMath::IsFinite(Value.Y)
			&& FMath::IsFinite(Value.Z);
	}

	FVector ClampAxes(const FVector& Value, const FVector& PositiveLimits, bool& bOutLimited)
	{
		FVector Result;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const float Limit = FMath::Max(PositiveLimits[Axis], 0.0);
			Result[Axis] = FMath::Clamp(Value[Axis], -Limit, Limit);
			bOutLimited |= !FMath::IsNearlyEqual(Result[Axis], Value[Axis]);
		}
		return Result;
	}

	FQuat IntegrateLocalAngularVelocity(const FQuat& Rotation, const FVector& LocalRate, float DeltaSeconds)
	{
		const FVector RotationVector = LocalRate * DeltaSeconds;
		const float Angle = RotationVector.Size();
		if (Angle <= UE_SMALL_NUMBER)
		{
			return Rotation;
		}
		FQuat Result = Rotation * FQuat(RotationVector / Angle, Angle);
		Result.Normalize();
		return Result;
	}

	bool BuildRawReference(
		const FVector& ControlAccelerationWorldCmPerSecSq,
		const FVector& DynamicsFeedForwardAccelerationWorldCmPerSecSq,
		const float YawDegrees,
		const float GravityMagnitudeCmPerSecSq,
		const FAircraftFlightControllerRuntimeConfig& FrameConfig,
		const float MaxTiltAngleDegrees,
		const float DynamicsFeedForwardScale,
		FAircraftAttitudeReference& OutRawReference)
	{
		if (!FrameConfig.FrameBinding.IsValid()
			|| !FMath::IsFinite(YawDegrees)
			|| !FMath::IsFinite(GravityMagnitudeCmPerSecSq) || GravityMagnitudeCmPerSecSq < 0.0f
			|| !FMath::IsFinite(MaxTiltAngleDegrees)
			|| MaxTiltAngleDegrees < 0.0f || MaxTiltAngleDegrees >= 90.0f
			|| !FMath::IsFinite(DynamicsFeedForwardScale)
			|| DynamicsFeedForwardScale < 0.0f
			|| !IsFiniteVector(ControlAccelerationWorldCmPerSecSq)
			|| !IsFiniteVector(DynamicsFeedForwardAccelerationWorldCmPerSecSq))
		{
			return false;
		}

		const FVector AttitudeAcceleration = ControlAccelerationWorldCmPerSecSq
			+ DynamicsFeedForwardAccelerationWorldCmPerSecSq
				* DynamicsFeedForwardScale;
		OutRawReference = AircraftAttitudeReference::Build(
			AttitudeAcceleration, YawDegrees, GravityMagnitudeCmPerSecSq,
			MaxTiltAngleDegrees, FrameConfig);
		return !OutRawReference.ControlWorldRotation.ContainsNaN()
			&& !OutRawReference.BodyWorldRotation.ContainsNaN();
	}

	void InitializeFromActual(
		const FQuat& ActualBodyWorldRotation,
		const FVector& ActualAngularVelocityBodyRadPerSec,
		const FAircraftFlightControllerRuntimeConfig& FrameConfig,
		FAircraftAttitudeMotionState& State)
	{
		State.ControlWorldRotation = FrameConfig.GetControlWorldRotation(
			ActualBodyWorldRotation);
		State.ControlWorldRotation.Normalize();
		State.AngularVelocityControlRadPerSec = FrameConfig.BodyToControlVector(
			ActualAngularVelocityBodyRadPerSec);
		State.AngularAccelerationControlRadPerSecSq = FVector::ZeroVector;
		State.bInitialized = true;
	}

	bool UpdateInternal(
		const FQuat& RawControlWorldRotation,
		const float YawRateDegPerSec,
		const float DeltaSeconds,
		const FQuat& ActualBodyWorldRotation,
		const FVector& ActualAngularVelocityBodyRadPerSec,
		const FAircraftFlightControllerRuntimeConfig& FrameConfig,
		const FAircraftAttitudeMotionConfig& MotionConfig,
		FAircraftAttitudeMotionState& InOutState,
		FAircraftAttitudeMotionOutput& OutReference)
	{
		OutReference = {};
		if (!FrameConfig.FrameBinding.IsValid() || !MotionConfig.IsValid()
			|| DeltaSeconds <= 0.0f || !FMath::IsFinite(DeltaSeconds)
			|| !FMath::IsFinite(YawRateDegPerSec)
			|| RawControlWorldRotation.ContainsNaN()
			|| RawControlWorldRotation.SizeSquared() <= UE_SMALL_NUMBER
			|| ActualBodyWorldRotation.ContainsNaN()
			|| ActualBodyWorldRotation.SizeSquared() <= UE_SMALL_NUMBER
			|| !IsFiniteVector(ActualAngularVelocityBodyRadPerSec))
		{
			InOutState = {};
			return false;
		}
		const FQuat NormalizedActualBody = ActualBodyWorldRotation.GetNormalized();
		const FQuat NormalizedRawControl = RawControlWorldRotation.GetNormalized();
		OutReference.RawControlWorldRotation = NormalizedRawControl;

		if (!InOutState.bInitialized || DeltaSeconds > MaximumCatchUpSeconds)
		{
			InitializeFromActual(NormalizedActualBody, ActualAngularVelocityBodyRadPerSec,
				FrameConfig, InOutState);
		}
		else
		{
			const int32 SubstepCount = FMath::Max(
				1, FMath::CeilToInt(DeltaSeconds / MaximumIntegrationStepSeconds));
			const float SubstepSeconds = DeltaSeconds / static_cast<float>(SubstepCount);
			const FVector MaxRateRad = FMath::DegreesToRadians(
				MotionConfig.MaxAngularRateDegPerSec);
			const FVector MaxAccelerationRad = FMath::DegreesToRadians(
				MotionConfig.MaxAngularAccelerationDegPerSecSq);
			const FVector MaxJerkRad = FMath::DegreesToRadians(
				MotionConfig.MaxAngularJerkDegPerSecCubed);
			const float NaturalAngularFrequency =
				MotionConfig.NaturalFrequencyHz * UE_TWO_PI;

			for (int32 Substep = 0; Substep < SubstepCount; ++Substep)
			{
				const FVector RotationErrorControl =
					AircraftAttitudeReference::GetShortestRotationVector(
						InOutState.ControlWorldRotation, NormalizedRawControl);
				const FVector YawFeedForwardControl =
					InOutState.ControlWorldRotation.UnrotateVector(
						FVector::UpVector * FMath::DegreesToRadians(YawRateDegPerSec));
				FVector RequestedAcceleration = RotationErrorControl
					* FMath::Square(NaturalAngularFrequency)
					+ (YawFeedForwardControl
						- InOutState.AngularVelocityControlRadPerSec)
						* (2.0f * MotionConfig.DampingRatio
							* NaturalAngularFrequency);

				RequestedAcceleration = ClampAxes(
					RequestedAcceleration, MaxAccelerationRad,
					OutReference.bAccelerationLimited);
				FVector AccelerationDelta = RequestedAcceleration
					- InOutState.AngularAccelerationControlRadPerSecSq;
				AccelerationDelta = ClampAxes(
					AccelerationDelta, MaxJerkRad * SubstepSeconds,
					OutReference.bJerkLimited);
				InOutState.AngularAccelerationControlRadPerSecSq += AccelerationDelta;
				InOutState.AngularVelocityControlRadPerSec +=
					InOutState.AngularAccelerationControlRadPerSecSq * SubstepSeconds;
				InOutState.AngularVelocityControlRadPerSec = ClampAxes(
					InOutState.AngularVelocityControlRadPerSec,
					MaxRateRad, OutReference.bRateLimited);
				InOutState.ControlWorldRotation = IntegrateLocalAngularVelocity(
					InOutState.ControlWorldRotation,
					InOutState.AngularVelocityControlRadPerSec, SubstepSeconds);
			}
		}

		OutReference.ControlWorldRotation = InOutState.ControlWorldRotation;
		OutReference.BodyWorldRotation = FrameConfig.GetBodyWorldRotation(
			InOutState.ControlWorldRotation);
		OutReference.AngularVelocityBodyRadPerSec = FrameConfig.ControlToBodyVector(
			InOutState.AngularVelocityControlRadPerSec);
		OutReference.AngularAccelerationBodyRadPerSecSq = FrameConfig.ControlToBodyVector(
			InOutState.AngularAccelerationControlRadPerSecSq);
		OutReference.bValid = !OutReference.ControlWorldRotation.ContainsNaN()
			&& !OutReference.BodyWorldRotation.ContainsNaN()
			&& IsFiniteVector(OutReference.AngularVelocityBodyRadPerSec)
			&& IsFiniteVector(OutReference.AngularAccelerationBodyRadPerSecSq);
		if (!OutReference.bValid)
		{
			InOutState = {};
		}
		return OutReference.bValid;
	}
}

bool FAircraftAttitudeMotionConfig::IsValid() const
{
	return FMath::IsFinite(MaxTiltAngleDegrees)
		&& MaxTiltAngleDegrees >= 0.0f && MaxTiltAngleDegrees < 90.0f
		&& FMath::IsFinite(NaturalFrequencyHz) && NaturalFrequencyHz >= 0.0f
		&& FMath::IsFinite(DampingRatio) && DampingRatio >= 0.0f
		&& IsFiniteVector(MaxAngularRateDegPerSec) && MaxAngularRateDegPerSec.GetMin() >= 0.0
		&& IsFiniteVector(MaxAngularAccelerationDegPerSecSq)
		&& MaxAngularAccelerationDegPerSecSq.GetMin() >= 0.0
		&& IsFiniteVector(MaxAngularJerkDegPerSecCubed)
		&& MaxAngularJerkDegPerSecCubed.GetMin() >= 0.0
		&& FMath::IsFinite(DynamicsFeedForwardScale) && DynamicsFeedForwardScale >= 0.0f;
}

bool FAircraftAttitudeReferenceDynamics::Update(
	const FVector& ControlAccelerationWorldCmPerSecSq,
	const FVector& DynamicsFeedForwardAccelerationWorldCmPerSecSq,
	const float YawDegrees,
	const float YawRateDegPerSec,
	const float GravityMagnitudeCmPerSecSq,
	const float DeltaSeconds,
	const FQuat& ActualBodyWorldRotation,
	const FVector& ActualAngularVelocityBodyRadPerSec,
	const FAircraftFlightControllerRuntimeConfig& FrameConfig,
	const FAircraftAttitudeMotionConfig& MotionConfig,
	FAircraftAttitudeMotionState& InOutState,
	FAircraftAttitudeMotionOutput& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_AlternativeAttitudeReference_Update);
	FAircraftAttitudeReference RawReference;
	if (!BuildRawReference(
		ControlAccelerationWorldCmPerSecSq,
		DynamicsFeedForwardAccelerationWorldCmPerSecSq,
		YawDegrees, GravityMagnitudeCmPerSecSq,
		FrameConfig, MotionConfig.MaxTiltAngleDegrees,
		MotionConfig.DynamicsFeedForwardScale, RawReference))
	{
		InOutState = {};
		OutReference = {};
		return false;
	}
	return UpdateInternal(
		RawReference.ControlWorldRotation, YawRateDegPerSec, DeltaSeconds,
		ActualBodyWorldRotation, ActualAngularVelocityBodyRadPerSec,
		FrameConfig, MotionConfig,
		InOutState, OutReference);
}

bool FAircraftAttitudeReferenceDynamics::UpdateTiltTarget(
	const FQuat& RawTiltControlWorldRotation,
	const float DeltaSeconds,
	const FQuat& ActualBodyWorldRotation,
	const FVector& ActualAngularVelocityBodyRadPerSec,
	const FAircraftFlightControllerRuntimeConfig& FrameConfig,
	const FAircraftAttitudeMotionConfig& MotionConfig,
	FAircraftAttitudeMotionState& InOutState,
	FAircraftAttitudeMotionOutput& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_AttitudeReference_UpdateTiltTarget);
	return UpdateInternal(
		RawTiltControlWorldRotation, 0.0f, DeltaSeconds,
		ActualBodyWorldRotation, ActualAngularVelocityBodyRadPerSec,
		FrameConfig, MotionConfig,
		InOutState, OutReference);
}

bool FAircraftAttitudeReferenceDynamics::BuildDriveTarget(
	const FVector& ControlAccelerationWorldCmPerSecSq,
	const FVector& DynamicsFeedForwardAccelerationWorldCmPerSecSq,
	const float YawDegrees,
	const float YawRateDegPerSec,
	const float GravityMagnitudeCmPerSecSq,
	const FAircraftFlightControllerRuntimeConfig& FrameConfig,
	const float MaxTiltAngleDegrees,
	const float DynamicsFeedForwardScale,
	FAircraftAttitudeMotionOutput& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Constraint_AttitudeTarget_Build);
	OutReference = {};
	FAircraftAttitudeReference RawReference;
	if (!BuildRawReference(
		ControlAccelerationWorldCmPerSecSq,
		DynamicsFeedForwardAccelerationWorldCmPerSecSq,
		YawDegrees, GravityMagnitudeCmPerSecSq,
		FrameConfig, MaxTiltAngleDegrees,
		DynamicsFeedForwardScale, RawReference))
	{
		return false;
	}
	if (!FMath::IsFinite(YawRateDegPerSec))
	{
		return false;
	}

	OutReference.RawControlWorldRotation = RawReference.ControlWorldRotation;
	OutReference.ControlWorldRotation = RawReference.ControlWorldRotation;
	OutReference.BodyWorldRotation = RawReference.BodyWorldRotation;
	const FVector TargetAngularVelocityWorldRadPerSec =
		FVector::UpVector * FMath::DegreesToRadians(YawRateDegPerSec);
	OutReference.AngularVelocityBodyRadPerSec =
		RawReference.BodyWorldRotation.UnrotateVector(
			TargetAngularVelocityWorldRadPerSec);
	OutReference.AngularAccelerationBodyRadPerSecSq = FVector::ZeroVector;
	OutReference.bValid = IsFiniteVector(OutReference.AngularVelocityBodyRadPerSec);
	return OutReference.bValid;
}

void FAircraftAttitudeReferenceDynamics::Reset(FAircraftAttitudeMotionState& State)
{
	State = {};
}
