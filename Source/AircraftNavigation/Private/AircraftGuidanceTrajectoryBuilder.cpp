#include "AircraftNavigation/AircraftGuidanceTrajectoryBuilder.h"

namespace UE::AircraftLab::Navigation::Private
{
	static bool IsFiniteGuidanceVector(const FVector& Value)
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
		&& UE::AircraftLab::Navigation::Private::IsFiniteGuidanceVector(
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
		|| !IsFiniteGuidanceVector(AircraftState.VehicleState.PositionCm)
		|| !IsFiniteGuidanceVector(AircraftState.VehicleState.VelocityCmPerSec)
		|| !IsFiniteGuidanceVector(TargetVelocityCmPerSec))
	{
		return false;
	}

	const FAircraftDynamicCapabilitySnapshot& Capability = AircraftState.Capability;
	const FVector DesiredAcceleration = (TargetVelocityCmPerSec
		- AircraftState.VehicleState.VelocityCmPerSec) / Settings.SolveDeltaTimeSeconds;
	const FVector CurrentHorizontalVelocity(
		AircraftState.VehicleState.VelocityCmPerSec.X,
		AircraftState.VehicleState.VelocityCmPerSec.Y,
		0.0);
	const FVector HorizontalAcceleration(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0);
	const bool bDecelerating = FVector::DotProduct(
		HorizontalAcceleration, CurrentHorizontalVelocity) < 0.0;
	const double HorizontalAccelerationLimit = bDecelerating
		? Capability.MaxHorizontalDecelerationCmPerSecSq
		: Capability.MaxHorizontalAccelerationCmPerSecSq;
	const FVector TargetHorizontalVelocity(TargetVelocityCmPerSec.X, TargetVelocityCmPerSec.Y, 0.0);
	if (TargetHorizontalVelocity.Size()
			> Capability.MaxHorizontalSpeedCmPerSec + UE_KINDA_SMALL_NUMBER
		|| TargetVelocityCmPerSec.Z
			> Capability.MaxClimbRateCmPerSec + UE_KINDA_SMALL_NUMBER
		|| TargetVelocityCmPerSec.Z
			< -Capability.MaxDescentRateCmPerSec - UE_KINDA_SMALL_NUMBER
		|| HorizontalAcceleration.Size()
			> HorizontalAccelerationLimit + UE_KINDA_SMALL_NUMBER
		|| FMath::Abs(DesiredAcceleration.Z)
			> Capability.MaxVerticalAccelerationCmPerSecSq + UE_KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FVector AccelerationDelta = DesiredAcceleration
		- Settings.PreviousCommandAccelerationCmPerSecSq;
	const FVector HorizontalAccelerationDelta(AccelerationDelta.X, AccelerationDelta.Y, 0.0);
	const double HorizontalJerkStep = FMath::Max(
		Capability.MaxHorizontalJerkCmPerSecCubed, 0.0f) * Settings.SolveDeltaTimeSeconds;
	const double VerticalJerkStep = FMath::Max(
		Capability.MaxVerticalJerkCmPerSecCubed, 0.0f) * Settings.SolveDeltaTimeSeconds;
	if (HorizontalAccelerationDelta.Size() > HorizontalJerkStep + UE_KINDA_SMALL_NUMBER
		|| FMath::Abs(AccelerationDelta.Z) > VerticalJerkStep + UE_KINDA_SMALL_NUMBER)
	{
		return false;
	}
	OutCommandAccelerationCmPerSecSq = DesiredAcceleration;

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
		const double AccelerationTimeSeconds = FMath::Min(
			TimeSeconds, static_cast<double>(Settings.SolveDeltaTimeSeconds));
		const double CruiseTimeSeconds = TimeSeconds - AccelerationTimeSeconds;
		FAircraftNavigationGuidanceSample& Sample = OutGuidance.Samples.AddDefaulted_GetRef();
		Sample.TimeSeconds = static_cast<float>(TimeSeconds);
		Sample.AccelerationCmPerSecSq = TimeSeconds < Settings.SolveDeltaTimeSeconds
			? OutCommandAccelerationCmPerSecSq
			: FVector::ZeroVector;
		Sample.VelocityCmPerSec = AircraftState.VehicleState.VelocityCmPerSec
			+ OutCommandAccelerationCmPerSecSq * AccelerationTimeSeconds;
		Sample.PositionCm = AircraftState.VehicleState.PositionCm
			+ AircraftState.VehicleState.VelocityCmPerSec * AccelerationTimeSeconds
			+ 0.5 * OutCommandAccelerationCmPerSecSq
				* AccelerationTimeSeconds * AccelerationTimeSeconds
			+ TargetVelocityCmPerSec * CruiseTimeSeconds;
	}
	return OutGuidance.IsValid();
}
