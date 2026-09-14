#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftAutopilotConfig.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

struct AIRCRAFTAUTOPILOT_API FAircraftSpatialPathState
{
	FVector PositionCm = FVector::ZeroVector;
	FVector Tangent = FVector::ForwardVector;
	FVector CurvaturePerCm = FVector::ZeroVector;
	float DistanceCm = 0.0f;
	bool bValid = false;
};

/** C2 quintic spatial spline parameterized by arc length with an explicit source-route parameter. */
class AIRCRAFTAUTOPILOT_API FAircraftSpatialPath
{
public:
	bool Build(const FAircraftRouteIntent& Route, const FAircraftPathOptimizationRuntimeConfig& Config);
	void Reset();
	bool Evaluate(float DistanceCm, FAircraftSpatialPathState& OutState) const;
	bool Project(const FVector& PositionCm, float InitialDistanceCm,
		bool bGlobalSearch, FAircraftSpatialPathState& OutState) const;
	float ComputeCorridorViolationCm(const FVector& PositionCm, float DistanceCm) const;
	FVector ComputeCorridorCorrectionCm(const FVector& PositionCm, float DistanceCm) const;
	bool BuildContinuousCorridorCandidates(const FVector& ActualPositionCm,
		const FVector& PredictedPositionCm, int32& InOutActiveSegmentIndex,
		TArray<int32>& OutCandidateIndices) const;
	float ComputeCorridorUnionViolationCm(const FVector& PositionCm,
		TConstArrayView<int32> CandidateIndices) const;
	bool IsCorridorLineContinuouslyCovered(const FVector& StartCm,
		const FVector& EndCm, TConstArrayView<int32> CandidateIndices,
		float AdditionalSafetyMarginCm) const;
	float GetRouteDistanceCm(float DistanceCm) const;

	float GetLengthCm() const { return TotalLengthCm; }
	float GetRouteLengthCm() const { return RouteLengthCm; }
	bool IsClosed() const { return bClosed; }
	bool IsValid() const { return !Segments.IsEmpty() && TotalLengthCm > UE_SMALL_NUMBER; }
	bool HasCorridor() const { return !Corridor.IsEmpty(); }

private:
	struct FSegment
	{
		FVector Coefficients[6]{};
		TArray<float> ArcLengthsCm;
		float StartDistanceCm = 0.0f;
		float LengthCm = 0.0f;
		float RouteStartDistanceCm = 0.0f;
		float RouteEndDistanceCm = 0.0f;

		FVector Evaluate(float U) const;
		FVector FirstDerivative(float U) const;
		FVector SecondDerivative(float U) const;
		float ParameterAtArcLength(float ArcLengthCm) const;
	};

	TArray<FSegment> Segments;
	TArray<float> SegmentEndDistancesCm;
	mutable int32 CachedSegmentIndex = INDEX_NONE;
	float TotalLengthCm = 0.0f;
	float RouteLengthCm = 0.0f;
	bool bClosed = false;
	TArray<FAircraftSafeCorridorSegment> Corridor;
	float CorridorSafetyMarginCm = 0.0f;
	float ProjectionBacktrackToleranceCm = 0.0f;
	float ProjectionSearchDistanceCm = 0.0f;
	float ProjectionSampleSpacingCm = 100.0f;

	bool ResolveSegmentParameter(float DistanceCm, int32& OutSegmentIndex,
		float& OutParameter) const;
	static void OptimizeKnots(TArray<FVector>& Points, bool bInClosed,
		const TArray<FAircraftSafeCorridorSegment>& Corridor,
		TConstArrayView<int32> SegmentCorridorIndices,
		const FAircraftPathOptimizationRuntimeConfig& Config);
	static void BuildDerivatives(const TArray<FVector>& Points, bool bInClosed,
		TArray<FVector>& OutFirst, TArray<FVector>& OutSecond);
};
