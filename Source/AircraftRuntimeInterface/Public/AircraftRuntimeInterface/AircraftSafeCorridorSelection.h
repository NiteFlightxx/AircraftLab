#pragma once

#include "CoreMinimal.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

struct AIRCRAFTRUNTIMEINTERFACE_API FAircraftCorridorParameterInterval
{
	float Start = 0.0f;
	float End = 0.0f;
	int32 CorridorSegmentIndex = INDEX_NONE;
};

/** Shared, value-only selection for one topologically continuous part of a capsule corridor. */
class AIRCRAFTRUNTIMEINTERFACE_API FAircraftSafeCorridorSelection
{
public:
	static bool BuildContinuousCandidates(
		TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
		const FVector& ActualPositionCm,
		const FVector& PredictedPositionCm,
		float SegmentHysteresisCm,
		int32& InOutActiveSegmentIndex,
		TArray<int32>& OutCandidateIndices);

	static float ComputePointViolationCm(
		TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
		TConstArrayView<int32> CandidateIndices,
		const FVector& PositionCm,
		float SafetyMarginCm);

	/** Computes and merges the exact parameter intervals where P(t)=Start+t(End-Start) is inside the capsule union. */
	static bool BuildLineCoverageIntervals(
		TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
		TConstArrayView<int32> CandidateIndices,
		const FVector& StartCm,
		const FVector& EndCm,
		float SafetyMarginCm,
		TArray<FAircraftCorridorParameterInterval>& OutMergedIntervals);

	static bool IsLineContinuouslyCovered(
		TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
		TConstArrayView<int32> CandidateIndices,
		const FVector& StartCm,
		const FVector& EndCm,
		float SafetyMarginCm);
};
