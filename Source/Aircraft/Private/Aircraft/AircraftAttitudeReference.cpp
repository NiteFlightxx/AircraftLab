#include "Aircraft/AircraftAttitudeReference.h"

#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	bool TryGetProjectedHeadingRateRadians(
		const FVector& DirectionWorld,
		const FVector& DirectionRateWorld,
		float& OutRateRadiansPerSecond)
	{
		const double HorizontalMagnitudeSquared =
			FMath::Square(DirectionWorld.X) + FMath::Square(DirectionWorld.Y);
		if (HorizontalMagnitudeSquared <= UE_SMALL_NUMBER)
		{
			return false;
		}
		OutRateRadiansPerSecond = static_cast<float>(
			(DirectionWorld.X * DirectionRateWorld.Y
				- DirectionWorld.Y * DirectionRateWorld.X)
			/ HorizontalMagnitudeSquared);
		return FMath::IsFinite(OutRateRadiansPerSecond);
	}
}

FVector AircraftAttitudeReference::GetShortestRotationVector(
	const FQuat& From,
	const FQuat& To)
{
	if (From.ContainsNaN() || To.ContainsNaN()
		|| From.SizeSquared() <= UE_SMALL_NUMBER
		|| To.SizeSquared() <= UE_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	FQuat Error = From.GetNormalized().Inverse() * To.GetNormalized();
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

float AircraftAttitudeReference::GetPlanarHeadingDegrees(
	const FQuat& BodyWorldRotation,
	const FAircraftFlightControllerRuntimeConfig& Config)
{
	const FQuat NormalizedBodyRotation = BodyWorldRotation.GetNormalized();
	FVector Forward = NormalizedBodyRotation.RotateVector(Config.GetForwardAxisBody());
	Forward.Z = 0.0f;
	if (Forward.Normalize())
	{
		return FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
	}

	FVector Right = NormalizedBodyRotation.RotateVector(Config.GetRightAxisBody());
	Right.Z = 0.0f;
	if (Right.Normalize())
	{
		return FMath::RadiansToDegrees(FMath::Atan2(-Right.X, Right.Y));
	}
	return 0.0f;
}

float AircraftAttitudeReference::GetPlanarHeadingRateDegreesPerSecond(
	const FQuat& BodyWorldRotation,
	const FVector& AngularVelocityWorldRadPerSec,
	const FVector& ForwardAxisBody,
	const FVector& RightAxisBody)
{
	const FQuat NormalizedBodyRotation = BodyWorldRotation.GetNormalized();
	const FVector ForwardWorld = NormalizedBodyRotation.RotateVector(ForwardAxisBody);
	const FVector ForwardRateWorld = FVector::CrossProduct(
		AngularVelocityWorldRadPerSec, ForwardWorld);
	float HeadingRateRadiansPerSecond = 0.0f;
	if (TryGetProjectedHeadingRateRadians(
		ForwardWorld, ForwardRateWorld, HeadingRateRadiansPerSecond))
	{
		return FMath::RadiansToDegrees(HeadingRateRadiansPerSecond);
	}

	const FVector RightWorld = NormalizedBodyRotation.RotateVector(RightAxisBody);
	const FVector RightRateWorld = FVector::CrossProduct(
		AngularVelocityWorldRadPerSec, RightWorld);
	const FVector ReconstructedForwardWorld(RightWorld.Y, -RightWorld.X, 0.0f);
	const FVector ReconstructedForwardRateWorld(RightRateWorld.Y, -RightRateWorld.X, 0.0f);
	if (TryGetProjectedHeadingRateRadians(
		ReconstructedForwardWorld, ReconstructedForwardRateWorld,
		HeadingRateRadiansPerSecond))
	{
		return FMath::RadiansToDegrees(HeadingRateRadiansPerSecond);
	}
	return 0.0f;
}

FVector AircraftAttitudeReference::GetControllerRatesForPlanarYawDegreesPerSecond(
	const FQuat& BodyWorldRotation,
	const float PlanarYawRateDegPerSec,
	const FAircraftFlightControllerRuntimeConfig& Config)
{
	const FVector AngularVelocityWorldRadPerSec = FVector::UpVector
		* FMath::DegreesToRadians(PlanarYawRateDegPerSec);
	const FVector AngularVelocityBodyRadPerSec = BodyWorldRotation.GetNormalized().UnrotateVector(
		AngularVelocityWorldRadPerSec);
	return FMath::RadiansToDegrees(
		Config.BodyAngularToController(AngularVelocityBodyRadPerSec));
}

FAircraftAttitudeReference AircraftAttitudeReference::Build(
	const FVector& AccelerationWorldCmPerSecSq,
	float YawDegrees,
	float GravityMagnitudeCmPerSecSq,
	float MaxTiltDegrees,
	const FAircraftFlightControllerRuntimeConfig& Config)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_AttitudeReference_Build);

	const FRotator HeadingRotation(0.0f, YawDegrees, 0.0f);
	const FVector Forward = HeadingRotation.RotateVector(FVector::ForwardVector);
	const FVector Right = HeadingRotation.RotateVector(FVector::RightVector);
	const float SpecificForceUp = FMath::Max(
		GravityMagnitudeCmPerSecSq + AccelerationWorldCmPerSecSq.Z,
		UE_SMALL_NUMBER);
	const float ForwardAcceleration = FVector::DotProduct(
		AccelerationWorldCmPerSecSq, Forward);
	const float RightAcceleration = FVector::DotProduct(
		AccelerationWorldCmPerSecSq, Right);

	const float UnclampedPitch = -FMath::RadiansToDegrees(
		FMath::Atan2(ForwardAcceleration, SpecificForceUp));
	const float UnclampedRoll = FMath::RadiansToDegrees(
		FMath::Atan2(RightAcceleration, SpecificForceUp));
	const float TiltLimit = FMath::Max(MaxTiltDegrees, 0.0f);
	const FVector2D Tilt(UnclampedRoll, UnclampedPitch);
	const FVector2D LimitedTilt = Tilt.GetSafeNormal()
		* FMath::Min(Tilt.Size(), TiltLimit);

	FAircraftAttitudeReference Result;
	Result.ControlWorldRotation = FRotator(
		LimitedTilt.Y, YawDegrees, LimitedTilt.X).Quaternion();
	Result.BodyWorldRotation = Config.GetBodyWorldRotation(Result.ControlWorldRotation);
	return Result;
}
