#pragma once

#include "CoreMinimal.h"
#include "AircraftAutopilot/AircraftSpatialPath.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"

struct AIRCRAFTAUTOPILOT_API FAircraftMotionPlanSample
{
	float TimeSeconds = 0.0f;
	float DistanceCm = 0.0f;
	FVector PositionCm = FVector::ZeroVector;
	FVector VelocityCmPerSec = FVector::ZeroVector;
	FVector AccelerationCmPerSecSq = FVector::ZeroVector;
	float YawDegrees = 0.0f;
	float YawRateDegPerSec = 0.0f;
};

/** Spatial path plus dynamically feasible time parameterization. */
class AIRCRAFTAUTOPILOT_API FAircraftMotionPlan
{
public:
	bool Build(const FAircraftMovementIntent& Intent,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& InitialState,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	void Reset();

	bool Evaluate(float TimeSeconds, FAircraftMotionPlanSample& OutSample) const;
	bool Project(const FVector& PositionCm, float InitialDistanceCm,
		FAircraftMotionPlanSample& OutSample) const;
	float TimeAtDistance(float DistanceCm) const;

	bool IsValid() const { return bValid; }
	bool IsContinuous() const { return bContinuous; }
	float GetDurationSeconds() const { return DurationSeconds; }
	float GetLengthCm() const { return SpatialPath.GetLengthCm(); }
	const TArray<FAircraftMotionPlanSample>& GetSamples() const { return Samples; }
	const FAircraftMovementIntent& GetIntent() const { return SourceIntent; }
	static float ResolveYaw(const FAircraftHeadingObjective& Heading,
		const FVector& PositionCm, const FVector& VelocityCmPerSec, float PreviousYawDegrees);

private:
	FAircraftMovementIntent SourceIntent;
	FAircraftSpatialPath SpatialPath;
	TArray<FAircraftMotionPlanSample> Samples;
	float DurationSeconds = 0.0f;
	bool bContinuous = false;
	bool bValid = false;

	bool BuildSpatialPlan(const FAircraftMovementIntent& Intent,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& InitialState,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	bool BuildTimedPlan(const FAircraftMovementIntent& Intent);
	bool BuildHoldPlan(const FAircraftMovementIntent& Intent,
		const FAircraftVehicleStateSnapshot& InitialState);
};
