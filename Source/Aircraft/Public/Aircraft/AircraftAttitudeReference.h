#pragma once

#include "CoreMinimal.h"

struct FAircraftFlightControllerRuntimeConfig;

/** Full three-axis attitude reference derived from a world-space acceleration request. */
struct AIRCRAFT_API FAircraftAttitudeReference
{
	FQuat ControlWorldRotation = FQuat::Identity;
	FQuat BodyWorldRotation = FQuat::Identity;
};

namespace AircraftAttitudeReference
{
	/**
	 * Converts translational acceleration and heading into one authoritative attitude target.
	 * Acceleration is expressed in cm/s^2. GravityMagnitudeCmPerSecSq is positive.
	 */
	AIRCRAFT_API FAircraftAttitudeReference Build(
		const FVector& AccelerationWorldCmPerSecSq,
		float YawDegrees,
		float GravityMagnitudeCmPerSecSq,
		float MaxTiltDegrees,
		const FAircraftFlightControllerRuntimeConfig& Config);
}
