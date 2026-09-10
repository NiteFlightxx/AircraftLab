#pragma once

#include "CoreMinimal.h"
#include "Aircraft/AircraftAttitudeReferenceDynamics.h"

/** Runtime-only PhysicsConstraint drive configuration compiled from Dataflow. */
struct AIRCRAFT_API FAircraftConstraintSimulationRuntimeConfig
{
	FAircraftConstraintSimulationRuntimeConfig();

	float LinearNaturalFrequencyHz = 1.59154943f;
	float LinearDampingRatio = 1.0f;
	float LinearExtraDampingPerSecond = 0.0f;
	float LinearForceLimitN = 0.0f;
	float GravityFeedForwardScale = 1.0f;
	float DynamicsFeedForwardScale = 1.0f;
	bool bLinearAccelerationMode = true;

	FAircraftAttitudeMotionConfig AttitudeReference;
	float AttitudeServoNaturalFrequencyHz = 1.59154943f;
	float AttitudeServoDampingRatio = 1.0f;
	float AttitudeServoExtraDampingPerSecond = 0.0f;
	float AttitudeTorqueLimitNm = 0.0f;

	bool IsValid() const;
};

/** Runtime-only kinematic drive configuration compiled from Dataflow. */
struct AIRCRAFT_API FAircraftKinematicSimulationRuntimeConfig
{
	FAircraftKinematicSimulationRuntimeConfig();

	FAircraftAttitudeMotionConfig AttitudeReference;
	bool bSweepMovement = true;

	bool IsValid() const;
};
