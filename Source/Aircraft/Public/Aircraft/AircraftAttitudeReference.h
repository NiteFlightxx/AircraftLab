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
	/** Returns the shortest local rotation vector that rotates From into To. */
	AIRCRAFT_API FVector GetShortestRotationVector(
		const FQuat& From,
		const FQuat& To);

	/** Returns the aircraft forward heading projected onto the world horizontal plane. */
	AIRCRAFT_API float GetPlanarHeadingDegrees(
		const FQuat& BodyWorldRotation,
		const FAircraftFlightControllerRuntimeConfig& Config);

	/**
	 * Returns the time derivative of the projected aircraft heading.
	 * AngularVelocityWorldRadPerSec is the physical Chaos angular velocity in world space.
	 */
	AIRCRAFT_API float GetPlanarHeadingRateDegreesPerSecond(
		const FQuat& BodyWorldRotation,
		const FVector& AngularVelocityWorldRadPerSec,
		const FVector& ForwardAxisBody,
		const FVector& RightAxisBody);

	/**
	 * Converts a requested rotation around world up into the complete controller-axis rate.
	 * A tilted aircraft requires Roll and Pitch components as well as controller Yaw.
	 */
	AIRCRAFT_API FVector GetControllerRatesForPlanarYawDegreesPerSecond(
		const FQuat& BodyWorldRotation,
		float PlanarYawRateDegPerSec,
		const FAircraftFlightControllerRuntimeConfig& Config);

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
