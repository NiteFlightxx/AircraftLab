#include "AircraftNavigation/AircraftGuidanceTrajectoryBuilder.h"

namespace UE::AircraftLab::Navigation::Private
{
	static bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}
}

bool FAircraftGuidanceTrajectorySettings::IsValid() const
{
	return SourceIntentId != 0
		&& FMath::IsFinite(SolveDeltaTimeSeconds) && SolveDeltaTimeSeconds > 0.0f
		&& FMath::IsFinite(HorizonSeconds) && HorizonSeconds > 0.0f
		&& FMath::IsFinite(SampleIntervalSeconds) && SampleIntervalSeconds > 0.0f
		&& SampleIntervalSeconds <= HorizonSeconds
		&& FMath::IsFinite(ValiditySeconds) && ValiditySeconds > 0.0f
		&& UE::AircraftLab::Navigation::Private::IsFiniteVector(
			PreviousCommandAccelerationCmPerSecSq);
}

bool FAircraftGuidanceTrajectoryBuilder::BuildVelocityGuidance(
	const FAircraftNavigationAgentSnapshot& AircraftState,
	const FVector& TargetVelocityCmPerSec,
	const FAircraftGuidanceTrajectorySettings& Settings,
	FAircraftNavigationGuidance& OutGuidance,
	FVector& OutCommandAccelerationCmPerSecSq)
{
	using namespace UE::AircraftLab::Navigation::Private;
	OutGuidance = {};
	OutCommandAccelerationCmPerSecSq = FVector::ZeroVector;
	if (!AircraftState.bValid || !AircraftState.Capability.bValid || !Settings.IsValid()
		|| !IsFiniteVector(AircraftState.VehicleState.PositionCm)
		|| !IsFiniteVector(AircraftState.VehicleState.VelocityCmPerSec)
		|| !IsFiniteVector(TargetVelocityCmPerSec))
	{
		return false;
	}

	const FAircraftDynamicCapabilitySnapshot& Capability = AircraftState.Capability;
	FVector DesiredAcceleration = (TargetVelocityCmPerSec
		- AircraftState.VehicleState.VelocityCmPerSec) / Settings.SolveDeltaTimeSeconds;
	const FVector CurrentHorizontalVelocity(
		AircraftState.VehicleState.VelocityCmPerSec.X,
		AircraftState.VehicleState.VelocityCmPerSec.Y,
		0.0);
	FVector HorizontalAcceleration(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0);
	const bool bDecelerating = FVector::DotProduct(
		HorizontalAcceleration, CurrentHorizontalVelocity) < 0.0;
	const double HorizontalAccelerationLimit = bDecelerating
		? Capability.MaxHorizontalDecelerationCmPerSecSq
		: Capability.MaxHorizontalAccelerationCmPerSecSq;
	HorizontalAcceleration = HorizontalAcceleration.GetClampedToMaxSize(
		FMath::Max(HorizontalAccelerationLimit, 0.0));
	DesiredAcceleration.X = HorizontalAcceleration.X;
	DesiredAcceleration.Y = HorizontalAcceleration.Y;
	DesiredAcceleration.Z = FMath::Clamp(
		DesiredAcceleration.Z,
		-static_cast<double>(FMath::Max(Capability.MaxVerticalAccelerationCmPerSecSq, 0.0f)),
		static_cast<double>(FMath::Max(Capability.MaxVerticalAccelerationCmPerSecSq, 0.0f)));

	FVector AccelerationDelta = DesiredAcceleration
		- Settings.PreviousCommandAccelerationCmPerSecSq;
	FVector HorizontalAccelerationDelta(AccelerationDelta.X, AccelerationDelta.Y, 0.0);
	HorizontalAccelerationDelta = HorizontalAccelerationDelta.GetClampedToMaxSize(
		FMath::Max(Capability.MaxHorizontalJerkCmPerSecCubed, 0.0f)
		* Settings.SolveDeltaTimeSeconds);
	AccelerationDelta.X = HorizontalAccelerationDelta.X;
	AccelerationDelta.Y = HorizontalAccelerationDelta.Y;
	const double VerticalJerkStep = FMath::Max(
		Capability.MaxVerticalJerkCmPerSecCubed, 0.0f) * Settings.SolveDeltaTimeSeconds;
	AccelerationDelta.Z = FMath::Clamp(
		AccelerationDelta.Z, -VerticalJerkStep, VerticalJerkStep);
	OutCommandAccelerationCmPerSecSq = Settings.PreviousCommandAccelerationCmPerSecSq
		+ AccelerationDelta;

	OutGuidance.Mode = EAircraftNavigationGuidanceMode::TimedTrajectory;
	OutGuidance.SourceIntentId = Settings.SourceIntentId;
	OutGuidance.SourceIntentRevision = Settings.SourceIntentRevision;
	OutGuidance.GeneratedAtSeconds = AircraftState.VehicleState.TimeSeconds;
	OutGuidance.ValidUntilSeconds = AircraftState.VehicleState.TimeSeconds
		+ Settings.ValiditySeconds;

	const int32 SampleCount = FMath::Max(
		FMath::CeilToInt(Settings.HorizonSeconds / Settings.SampleIntervalSeconds), 1) + 1;
	OutGuidance.Samples.Reserve(SampleCount);
	for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
	{
		const double TimeSeconds = FMath::Min(
			static_cast<double>(SampleIndex) * Settings.SampleIntervalSeconds,
			static_cast<double>(Settings.HorizonSeconds));
		FAircraftNavigationGuidanceSample& Sample = OutGuidance.Samples.AddDefaulted_GetRef();
		Sample.TimeSeconds = static_cast<float>(TimeSeconds);
		Sample.AccelerationCmPerSecSq = OutCommandAccelerationCmPerSecSq;
		Sample.VelocityCmPerSec = AircraftState.VehicleState.VelocityCmPerSec
			+ OutCommandAccelerationCmPerSecSq * TimeSeconds;
		FVector HorizontalVelocity(Sample.VelocityCmPerSec.X, Sample.VelocityCmPerSec.Y, 0.0);
		HorizontalVelocity = HorizontalVelocity.GetClampedToMaxSize(
			FMath::Max(Capability.MaxHorizontalSpeedCmPerSec, 0.0f));
		Sample.VelocityCmPerSec.X = HorizontalVelocity.X;
		Sample.VelocityCmPerSec.Y = HorizontalVelocity.Y;
		Sample.VelocityCmPerSec.Z = FMath::Clamp(
			Sample.VelocityCmPerSec.Z,
			-static_cast<double>(FMath::Max(Capability.MaxDescentRateCmPerSec, 0.0f)),
			static_cast<double>(FMath::Max(Capability.MaxClimbRateCmPerSec, 0.0f)));
		Sample.PositionCm = AircraftState.VehicleState.PositionCm
			+ AircraftState.VehicleState.VelocityCmPerSec * TimeSeconds
			+ 0.5 * OutCommandAccelerationCmPerSecSq * TimeSeconds * TimeSeconds;
	}
	return OutGuidance.IsValid();
}
