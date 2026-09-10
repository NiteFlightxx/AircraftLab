#include "Aircraft/AircraftAlternativeDriveConfig.h"

namespace
{
	bool IsFiniteNonNegative(const float Value)
	{
		return FMath::IsFinite(Value) && Value >= 0.0f;
	}
}

FAircraftConstraintSimulationRuntimeConfig::FAircraftConstraintSimulationRuntimeConfig() = default;

bool FAircraftConstraintSimulationRuntimeConfig::IsValid() const
{
	return IsFiniteNonNegative(LinearNaturalFrequencyHz)
		&& IsFiniteNonNegative(LinearDampingRatio)
		&& IsFiniteNonNegative(LinearExtraDampingPerSecond)
		&& IsFiniteNonNegative(LinearForceLimitN)
		&& IsFiniteNonNegative(GravityFeedForwardScale)
		&& IsFiniteNonNegative(DynamicsFeedForwardScale)
		&& Attitude.IsValid()
		&& IsFiniteNonNegative(AttitudeExtraDampingPerSecond)
		&& IsFiniteNonNegative(AttitudeTorqueLimitNm);
}

FAircraftKinematicSimulationRuntimeConfig::FAircraftKinematicSimulationRuntimeConfig()
{
	AttitudeReference.NaturalFrequencyHz = 1.5f;
	AttitudeReference.MaxAngularRateDegPerSec = FVector(120.0, 120.0, 90.0);
	AttitudeReference.MaxAngularAccelerationDegPerSecSq = FVector(480.0, 480.0, 240.0);
	AttitudeReference.MaxAngularJerkDegPerSecCubed = FVector(1920.0, 1920.0, 960.0);
	AttitudeReference.DynamicsFeedForwardScale = 0.0f;
}

bool FAircraftKinematicSimulationRuntimeConfig::IsValid() const
{
	return AttitudeReference.IsValid();
}
