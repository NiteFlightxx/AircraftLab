#include "Aircraft/AircraftAttitudeReference.h"

#include "Aircraft/FlightControllerRuntimeConfig.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

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
