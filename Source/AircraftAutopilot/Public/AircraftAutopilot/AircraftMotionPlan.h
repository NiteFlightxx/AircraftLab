#pragma once

#include "CoreMinimal.h"
#include "AircraftAutopilot/AircraftSpatialPath.h"
#include "AircraftRuntimeInterface/AircraftAutopilotTypes.h"

/** Spatial path plus dynamically feasible time parameterization. */
class AIRCRAFTAUTOPILOT_API FAircraftMotionPlan
{
public:
	bool Build(const FAircraftMovementIntent& Intent,
		const FAircraftAutopilotRuntimeConfig& Config,
		const FAircraftVehicleStateSnapshot& InitialState,
		const FAircraftDynamicCapabilitySnapshot& Capability);
	void Reset();
	/** Update fields that do not change the geometric/time plan without rebuilding samples. */
	void UpdateMetadata(const FAircraftMovementIntent& Intent);

	bool Evaluate(float TimeSeconds, FAircraftMotionPlanSample& OutSample) const;
	bool Project(const FVector& PositionCm, float InitialDistanceCm, bool bGlobalSearch,
		FAircraftMotionPlanSample& OutSample) const;
	float ComputeCorridorViolationCm(const FVector& PositionCm, float DistanceCm) const
	{
		return SpatialPath.ComputeCorridorViolationCm(PositionCm, DistanceCm);
	}
	FVector ComputeCorridorCorrectionCm(const FVector& PositionCm, float DistanceCm) const
	{
		return SpatialPath.ComputeCorridorCorrectionCm(PositionCm, DistanceCm);
	}
	bool BuildContinuousCorridorCandidates(const FVector& ActualPositionCm,
		const FVector& PredictedPositionCm, int32& InOutActiveSegmentIndex,
		TArray<int32>& OutCandidateIndices) const
	{
		return SpatialPath.BuildContinuousCorridorCandidates(ActualPositionCm,
			PredictedPositionCm, InOutActiveSegmentIndex, OutCandidateIndices);
	}
	float ComputeCorridorUnionViolationCm(const FVector& PositionCm,
		TConstArrayView<int32> CandidateIndices) const
	{
		return SpatialPath.ComputeCorridorUnionViolationCm(PositionCm, CandidateIndices);
	}
	bool IsCorridorLineContinuouslyCovered(const FVector& StartCm,
		const FVector& EndCm, TConstArrayView<int32> CandidateIndices,
		float AdditionalSafetyMarginCm) const
	{
		return SpatialPath.IsCorridorLineContinuouslyCovered(StartCm, EndCm,
			CandidateIndices, AdditionalSafetyMarginCm);
	}
	float TimeAtDistance(float DistanceCm) const;

	bool IsValid() const { return bValid; }
	bool IsContinuous() const { return bContinuous; }
	float GetDurationSeconds() const { return DurationSeconds; }
	float GetLengthCm() const { return SpatialPath.GetLengthCm(); }
	float GetRouteLengthCm() const { return SpatialPath.GetRouteLengthCm(); }
	bool HasCorridor() const { return SpatialPath.HasCorridor(); }
	float GetRouteDistanceCm(float DistanceCm) const
	{
		return SpatialPath.GetRouteDistanceCm(DistanceCm);
	}
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
