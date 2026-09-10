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

	FVector QuaternionLogShortest(FQuat Error)
	{
		Error.Normalize();
		if (Error.W < 0.0f)
		{
			Error = FQuat(-Error.X, -Error.Y, -Error.Z, -Error.W);
		}
		FVector Axis = FVector::ZeroVector;
		float Angle = 0.0f;
		Error.ToAxisAndAngle(Axis, Angle);
		return Axis.IsNormalized() && FMath::IsFinite(Angle)
			? Axis * Angle
			: FVector::ZeroVector;
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
	OutReference = {};
	if (!FrameConfig.FrameBinding.IsValid() || !MotionConfig.IsValid()
		|| DeltaSeconds <= 0.0f || !FMath::IsFinite(DeltaSeconds)
		|| !FMath::IsFinite(YawDegrees) || !FMath::IsFinite(YawRateDegPerSec)
		|| !FMath::IsFinite(GravityMagnitudeCmPerSecSq) || GravityMagnitudeCmPerSecSq < 0.0f
		|| !IsFiniteVector(ControlAccelerationWorldCmPerSecSq)
		|| !IsFiniteVector(DynamicsFeedForwardAccelerationWorldCmPerSecSq)
		|| ActualBodyWorldRotation.ContainsNaN()
		|| ActualBodyWorldRotation.SizeSquared() <= UE_SMALL_NUMBER
		|| !IsFiniteVector(ActualAngularVelocityBodyRadPerSec))
	{
		Reset(InOutState);
		return false;
	}

	const FVector AttitudeAcceleration = ControlAccelerationWorldCmPerSecSq
		+ DynamicsFeedForwardAccelerationWorldCmPerSecSq
			* MotionConfig.DynamicsFeedForwardScale;
	const FAircraftAttitudeReference RawReference = AircraftAttitudeReference::Build(
		AttitudeAcceleration, YawDegrees, GravityMagnitudeCmPerSecSq,
		MotionConfig.MaxTiltAngleDegrees, FrameConfig);
	OutReference.RawControlWorldRotation = RawReference.ControlWorldRotation;

	FQuat NormalizedActualBody = ActualBodyWorldRotation;
	NormalizedActualBody.Normalize();
	if (!InOutState.bInitialized || DeltaSeconds > MaximumCatchUpSeconds)
	{
		InOutState.ControlWorldRotation = FrameConfig.GetControlWorldRotation(
			NormalizedActualBody);
		InOutState.ControlWorldRotation.Normalize();
		InOutState.AngularVelocityControlRadPerSec = FrameConfig.BodyToControlVector(
			ActualAngularVelocityBodyRadPerSec);
		InOutState.AngularAccelerationControlRadPerSecSq = FVector::ZeroVector;
		InOutState.bInitialized = true;
	}
	else
	{
		const int32 SubstepCount = FMath::Max(
			1, FMath::CeilToInt(DeltaSeconds / MaximumIntegrationStepSeconds));
		const float SubstepSeconds = DeltaSeconds / static_cast<float>(SubstepCount);
		const float NaturalAngularFrequency = MotionConfig.NaturalFrequencyHz * UE_TWO_PI;
		const FVector MaxRateRad = FMath::DegreesToRadians(
			MotionConfig.MaxAngularRateDegPerSec);
		const FVector MaxAccelerationRad = FMath::DegreesToRadians(
			MotionConfig.MaxAngularAccelerationDegPerSecSq);
		const FVector MaxJerkRad = FMath::DegreesToRadians(
			MotionConfig.MaxAngularJerkDegPerSecCubed);

		for (int32 Substep = 0; Substep < SubstepCount; ++Substep)
		{
			const FQuat Error = InOutState.ControlWorldRotation.Inverse()
				* RawReference.ControlWorldRotation;
			const FVector RotationErrorControl = QuaternionLogShortest(Error);
			const FVector YawFeedForwardWorld = FVector::UpVector
				* FMath::DegreesToRadians(YawRateDegPerSec);
			const FVector FeedForwardControl = InOutState.ControlWorldRotation.UnrotateVector(
				YawFeedForwardWorld);
			FVector RequestedAcceleration = RotationErrorControl
				* FMath::Square(NaturalAngularFrequency)
				+ (FeedForwardControl - InOutState.AngularVelocityControlRadPerSec)
					* (2.0f * MotionConfig.DampingRatio * NaturalAngularFrequency);
			RequestedAcceleration = ClampAxes(
				RequestedAcceleration, MaxAccelerationRad, OutReference.bAccelerationLimited);

			const FVector MaximumAccelerationDelta = MaxJerkRad * SubstepSeconds;
			FVector AccelerationDelta = RequestedAcceleration
				- InOutState.AngularAccelerationControlRadPerSecSq;
			AccelerationDelta = ClampAxes(
				AccelerationDelta, MaximumAccelerationDelta, OutReference.bJerkLimited);
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
		Reset(InOutState);
	}
	return OutReference.bValid;
}

void FAircraftAttitudeReferenceDynamics::Reset(FAircraftAttitudeMotionState& State)
{
	State = {};
}

bool FAircraftAttitudeReferenceDynamics::ComputeServoTorqueBody(
	const FQuat& ActualBodyWorldRotation,
	const FVector& ActualAngularVelocityBodyRadPerSec,
	const FVector& InertiaPrincipalKgM2,
	const FQuat& PrincipalToBodyRotation,
	const FAircraftAttitudeMotionOutput& Reference,
	const float NaturalFrequencyHz,
	const float DampingRatio,
	const float ExtraDampingPerSecond,
	const float TorqueLimitNm,
	FVector& OutTorqueBodyNm,
	bool& bOutTorqueLimited)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_AlternativeAttitudeServo_ComputeTorque);
	OutTorqueBodyNm = FVector::ZeroVector;
	bOutTorqueLimited = false;
	if (!Reference.bValid || ActualBodyWorldRotation.ContainsNaN()
		|| ActualBodyWorldRotation.SizeSquared() <= UE_SMALL_NUMBER
		|| !IsFiniteVector(ActualAngularVelocityBodyRadPerSec)
		|| !IsFiniteVector(InertiaPrincipalKgM2)
		|| InertiaPrincipalKgM2.GetMin() <= 0.0
		|| PrincipalToBodyRotation.ContainsNaN()
		|| PrincipalToBodyRotation.SizeSquared() <= UE_SMALL_NUMBER
		|| !FMath::IsFinite(NaturalFrequencyHz) || NaturalFrequencyHz < 0.0f
		|| !FMath::IsFinite(DampingRatio) || DampingRatio < 0.0f
		|| !FMath::IsFinite(ExtraDampingPerSecond) || ExtraDampingPerSecond < 0.0f
		|| !FMath::IsFinite(TorqueLimitNm) || TorqueLimitNm < 0.0f)
	{
		return false;
	}

	FQuat ActualRotation = ActualBodyWorldRotation;
	ActualRotation.Normalize();
	const FVector RotationErrorBody = QuaternionLogShortest(
		ActualRotation.Inverse() * Reference.BodyWorldRotation);
	const float NaturalAngularFrequency = NaturalFrequencyHz * UE_TWO_PI;
	const float Damping = ExtraDampingPerSecond
		+ 2.0f * DampingRatio * NaturalAngularFrequency;
	const FVector RequestedAngularAccelerationBody =
		Reference.AngularAccelerationBodyRadPerSecSq
		+ RotationErrorBody * FMath::Square(NaturalAngularFrequency)
		+ (Reference.AngularVelocityBodyRadPerSec
			- ActualAngularVelocityBodyRadPerSec) * Damping;
	FQuat NormalizedPrincipalToBody = PrincipalToBodyRotation;
	NormalizedPrincipalToBody.Normalize();
	const FVector AngularVelocityPrincipalRadPerSec =
		NormalizedPrincipalToBody.UnrotateVector(ActualAngularVelocityBodyRadPerSec);
	const FVector RequestedAngularAccelerationPrincipalRadPerSecSq =
		NormalizedPrincipalToBody.UnrotateVector(RequestedAngularAccelerationBody);
	const FVector AngularMomentumPrincipal =
		InertiaPrincipalKgM2 * AngularVelocityPrincipalRadPerSec;
	const FVector UnclampedTorquePrincipalNm =
		InertiaPrincipalKgM2 * RequestedAngularAccelerationPrincipalRadPerSecSq
		+ FVector::CrossProduct(
			AngularVelocityPrincipalRadPerSec, AngularMomentumPrincipal);
	const FVector UnclampedTorqueBodyNm =
		NormalizedPrincipalToBody.RotateVector(UnclampedTorquePrincipalNm);
	OutTorqueBodyNm = UnclampedTorqueBodyNm;
	if (TorqueLimitNm > 0.0f)
	{
		OutTorqueBodyNm.X = FMath::Clamp(OutTorqueBodyNm.X, -TorqueLimitNm, TorqueLimitNm);
		OutTorqueBodyNm.Y = FMath::Clamp(OutTorqueBodyNm.Y, -TorqueLimitNm, TorqueLimitNm);
		OutTorqueBodyNm.Z = FMath::Clamp(OutTorqueBodyNm.Z, -TorqueLimitNm, TorqueLimitNm);
		bOutTorqueLimited = !OutTorqueBodyNm.Equals(
			UnclampedTorqueBodyNm, UE_KINDA_SMALL_NUMBER);
	}
	return IsFiniteVector(OutTorqueBodyNm);
}
