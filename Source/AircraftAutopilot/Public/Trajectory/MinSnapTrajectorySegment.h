// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/TrajectorySegment.h"
#include "MinSnapTrajectorySegment.generated.h"

/**
 * Globally optimized, piecewise seventh-order minimum-snap trajectory.
 *
 * Position is fixed at every waypoint. Start/end velocity and acceleration are
 * taken from FTrajectoryRequest; endpoint jerk is zero. Velocity,
 * acceleration and jerk are continuous at every internal waypoint. Segment
 * time is allocated from path length and iteratively scaled to satisfy the
 * requested velocity, acceleration and optional jerk limits.
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API UMinSnapTrajectorySegment : public UTrajectorySegment
{
	GENERATED_BODY()

public:
	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;
	virtual bool UsesNativeTimeParameterization() const override { return true; }
	virtual FTrajectoryPoint SampleAtTime(float TimeSeconds) const override;
	virtual float GetArcLengthAtTime(float TimeSeconds) const override;

	/** Derivative order 0..4 at global trajectory time; intended for diagnostics/tests. */
	FVector EvaluateDerivativeAtTime(float TimeSeconds, int32 DerivativeOrder) const;
	float GetWaypointTimeSeconds(int32 WaypointIndex) const;
	int32 GetWaypointCount() const { return Waypoints.Num(); }

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	TArray<FVector> Waypoints;

private:
	struct FPolynomialSegment
	{
		float StartTimeSeconds = 0.0f;
		float DurationSeconds = 0.0f;
		TArray<FVector> Coefficients;
	};

	TArray<FPolynomialSegment> PolynomialSegments;
	TArray<float> WaypointTimes;
	TArray<float> ArcLookupTimes;
	TArray<float> ArcLookupLengths;

	bool SolvePolynomials(const FTrajectoryRequest& Request, FString& OutError);
	bool SolveAxis(int32 Axis, const FTrajectoryRequest& Request, FString& OutError);
	void AllocateInitialTimes(float CruiseSpeedCmPerSec);
	bool ScaleTimesToLimits(const FTrajectoryRequest& Request, FString& OutError);
	void BuildArcLengthLookup();
	int32 FindSegmentAtTime(float TimeSeconds, float& OutLocalTimeSeconds) const;
	float FindTimeAtArcLength(float S) const;
	FVector EvaluateSegmentDerivative(int32 SegmentIndex, float LocalTimeSeconds, int32 Order) const;
	void MeasureDerivativePeaks(float& OutMaxSpeed, float& OutMaxAcceleration, float& OutMaxJerk) const;
};

