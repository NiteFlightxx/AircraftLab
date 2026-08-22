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
	int32 SegmentIndex = INDEX_NONE;
	bool bValid = false;
};

/** C2 quintic spatial spline parameterized by arc length. */
class AIRCRAFTAUTOPILOT_API FAircraftSpatialPath
{
public:
	bool Build(const FAircraftRouteIntent& Route, const FAircraftPathOptimizationRuntimeConfig& Config);
	void Reset();
	bool Evaluate(float DistanceCm, FAircraftSpatialPathState& OutState) const;
	bool Project(const FVector& PositionCm, float InitialDistanceCm, FAircraftSpatialPathState& OutState) const;

	float GetLengthCm() const { return TotalLengthCm; }
	bool IsClosed() const { return bClosed; }
	bool IsValid() const { return !Segments.IsEmpty() && TotalLengthCm > UE_SMALL_NUMBER; }

private:
	struct FSegment
	{
		FVector Coefficients[6]{};
		TArray<float> ArcLengthsCm;
		float StartDistanceCm = 0.0f;
		float LengthCm = 0.0f;

		FVector Evaluate(float U) const;
		FVector FirstDerivative(float U) const;
		FVector SecondDerivative(float U) const;
		float ParameterAtArcLength(float ArcLengthCm) const;
	};

	TArray<FSegment> Segments;
	float TotalLengthCm = 0.0f;
	bool bClosed = false;

	static void OptimizeKnots(TArray<FVector>& Points, bool bInClosed,
		const TArray<FAircraftSafeCorridorSegment>& Corridor,
		const FAircraftPathOptimizationRuntimeConfig& Config);
	static void BuildDerivatives(const TArray<FVector>& Points, bool bInClosed,
		TArray<FVector>& OutFirst, TArray<FVector>& OutSecond);
};
