#include "AircraftRuntimeInterface/AircraftSafeCorridorSelection.h"

namespace
{
	bool IsFiniteCorridorVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y)
			&& FMath::IsFinite(Value.Z);
	}

	double DistanceToAxisCm(
		const FAircraftSafeCorridorSegment& Segment,
		const FVector& PositionCm)
	{
		return FVector::Distance(PositionCm, Segment.GetClosestAxisPoint(PositionCm));
	}

	bool SweepIntersectsCapsule(
		const FVector& SweepStartCm,
		const FVector& SweepEndCm,
		const FAircraftSafeCorridorSegment& Segment)
	{
		FVector SweepPoint;
		FVector AxisPoint;
		FMath::SegmentDistToSegmentSafe(
			SweepStartCm, SweepEndCm,
			Segment.AxisStartCm, Segment.AxisEndCm,
			SweepPoint, AxisPoint);
		return FVector::DistSquared(SweepPoint, AxisPoint)
			<= FMath::Square(Segment.RadiusCm);
	}

	double DistanceSquaredAtParameter(const FVector& StartCm, const FVector& DeltaCm,
		const double Parameter, const FAircraftSafeCorridorSegment& Segment)
	{
		const FVector PositionCm = StartCm + Parameter * DeltaCm;
		return FVector::DistSquared(PositionCm, Segment.GetClosestAxisPoint(PositionCm));
	}

	bool BuildCapsuleInterval(const FVector& StartCm, const FVector& EndCm,
		const FAircraftSafeCorridorSegment& Segment, const float SafetyMarginCm,
		FAircraftCorridorParameterInterval& OutInterval)
	{
		const double RadiusCm = Segment.RadiusCm - SafetyMarginCm;
		if (!Segment.IsGeometryValid() || RadiusCm <= UE_DOUBLE_SMALL_NUMBER)
		{
			return false;
		}
		const FVector DeltaCm = EndCm - StartCm;
		if (DeltaCm.IsNearlyZero())
		{
			if (DistanceSquaredAtParameter(StartCm, DeltaCm, 0.0, Segment)
				> FMath::Square(RadiusCm))
			{
				return false;
			}
			OutInterval.Start = 0.0f;
			OutInterval.End = 1.0f;
			return true;
		}
		FVector SweepPoint;
		FVector AxisPoint;
		FMath::SegmentDistToSegmentSafe(StartCm, EndCm,
			Segment.AxisStartCm, Segment.AxisEndCm, SweepPoint, AxisPoint);
		const double ClosestParameter = FMath::Clamp(
			FVector::DotProduct(SweepPoint - StartCm, DeltaCm) / DeltaCm.SizeSquared(), 0.0, 1.0);
		const double RadiusSquaredCm = FMath::Square(RadiusCm);
		if (DistanceSquaredAtParameter(StartCm, DeltaCm, ClosestParameter, Segment)
			> RadiusSquaredCm + UE_KINDA_SMALL_NUMBER)
		{
			return false;
		}
		const auto FindBoundary = [&](double Outside, double Inside)
		{
			for (int32 Iteration = 0; Iteration < 32; ++Iteration)
			{
				const double Middle = 0.5 * (Outside + Inside);
				if (DistanceSquaredAtParameter(StartCm, DeltaCm, Middle, Segment)
					<= RadiusSquaredCm)
				{
					Inside = Middle;
				}
				else
				{
					Outside = Middle;
				}
			}
			return Inside;
		};
		OutInterval.Start = DistanceSquaredAtParameter(StartCm, DeltaCm, 0.0, Segment)
			<= RadiusSquaredCm ? 0.0f : static_cast<float>(FindBoundary(0.0, ClosestParameter));
		OutInterval.End = DistanceSquaredAtParameter(StartCm, DeltaCm, 1.0, Segment)
			<= RadiusSquaredCm ? 1.0f : static_cast<float>(FindBoundary(1.0, ClosestParameter));
		return OutInterval.Start <= OutInterval.End;
	}
}

bool FAircraftSafeCorridorSelection::BuildContinuousCandidates(
	const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
	const FVector& ActualPositionCm,
	const FVector& PredictedPositionCm,
	const float SegmentHysteresisCm,
	int32& InOutActiveSegmentIndex,
	TArray<int32>& OutCandidateIndices)
{
	OutCandidateIndices.Reset();
	if (Corridor.IsEmpty() || !IsFiniteCorridorVector(ActualPositionCm)
		|| !IsFiniteCorridorVector(PredictedPositionCm)
		|| !FMath::IsFinite(SegmentHysteresisCm) || SegmentHysteresisCm < 0.0f)
	{
		InOutActiveSegmentIndex = INDEX_NONE;
		return false;
	}

	int32 ClosestSegmentIndex = INDEX_NONE;
	double ClosestDistanceCm = TNumericLimits<double>::Max();
	for (int32 SegmentIndex = 0; SegmentIndex < Corridor.Num(); ++SegmentIndex)
	{
		const FAircraftSafeCorridorSegment& Segment = Corridor[SegmentIndex];
		if (!Segment.IsGeometryValid())
		{
			continue;
		}
		const double DistanceCm = DistanceToAxisCm(Segment, ActualPositionCm);
		if (DistanceCm < ClosestDistanceCm)
		{
			ClosestDistanceCm = DistanceCm;
			ClosestSegmentIndex = SegmentIndex;
		}
	}
	if (ClosestSegmentIndex == INDEX_NONE)
	{
		InOutActiveSegmentIndex = INDEX_NONE;
		return false;
	}

	if (Corridor.IsValidIndex(InOutActiveSegmentIndex)
		&& Corridor[InOutActiveSegmentIndex].IsGeometryValid()
		&& DistanceToAxisCm(Corridor[InOutActiveSegmentIndex], ActualPositionCm)
			<= ClosestDistanceCm + SegmentHysteresisCm)
	{
		ClosestSegmentIndex = InOutActiveSegmentIndex;
	}
	InOutActiveSegmentIndex = ClosestSegmentIndex;
	OutCandidateIndices.Add(ClosestSegmentIndex);

	for (int32 SegmentIndex = ClosestSegmentIndex + 1; SegmentIndex < Corridor.Num(); ++SegmentIndex)
	{
		const FAircraftSafeCorridorSegment& Segment = Corridor[SegmentIndex];
		if (!Segment.IsGeometryValid()
			|| !SweepIntersectsCapsule(ActualPositionCm, PredictedPositionCm, Segment))
		{
			break;
		}
		OutCandidateIndices.Add(SegmentIndex);
	}
	for (int32 SegmentIndex = ClosestSegmentIndex - 1; SegmentIndex >= 0; --SegmentIndex)
	{
		const FAircraftSafeCorridorSegment& Segment = Corridor[SegmentIndex];
		if (!Segment.IsGeometryValid()
			|| !SweepIntersectsCapsule(ActualPositionCm, PredictedPositionCm, Segment))
		{
			break;
		}
		OutCandidateIndices.Add(SegmentIndex);
	}
	return true;
}

float FAircraftSafeCorridorSelection::ComputePointViolationCm(
	const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
	const TConstArrayView<int32> CandidateIndices,
	const FVector& PositionCm,
	const float SafetyMarginCm)
{
	if (!IsFiniteCorridorVector(PositionCm) || !FMath::IsFinite(SafetyMarginCm)
		|| SafetyMarginCm < 0.0f || CandidateIndices.IsEmpty())
	{
		return TNumericLimits<float>::Max();
	}
	float MinimumViolationCm = TNumericLimits<float>::Max();
	for (const int32 CandidateIndex : CandidateIndices)
	{
		if (!Corridor.IsValidIndex(CandidateIndex)
			|| !Corridor[CandidateIndex].IsGeometryValid())
		{
			continue;
		}
		MinimumViolationCm = FMath::Min(MinimumViolationCm,
			static_cast<float>(Corridor[CandidateIndex].ComputeCorrectionCm(
				PositionCm, SafetyMarginCm).Size()));
	}
	return MinimumViolationCm;
}

bool FAircraftSafeCorridorSelection::BuildLineCoverageIntervals(
	const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
	const TConstArrayView<int32> CandidateIndices, const FVector& StartCm,
	const FVector& EndCm, const float SafetyMarginCm,
	TArray<FAircraftCorridorParameterInterval>& OutMergedIntervals)
{
	OutMergedIntervals.Reset();
	if (!IsFiniteCorridorVector(StartCm) || !IsFiniteCorridorVector(EndCm)
		|| !FMath::IsFinite(SafetyMarginCm) || SafetyMarginCm < 0.0f)
	{
		return false;
	}
	for (const int32 CandidateIndex : CandidateIndices)
	{
		if (!Corridor.IsValidIndex(CandidateIndex))
		{
			continue;
		}
		FAircraftCorridorParameterInterval Interval;
		Interval.CorridorSegmentIndex = CandidateIndex;
		if (BuildCapsuleInterval(StartCm, EndCm, Corridor[CandidateIndex],
			SafetyMarginCm, Interval))
		{
			OutMergedIntervals.Add(Interval);
		}
	}
	OutMergedIntervals.Sort([](const auto& A, const auto& B)
	{
		return A.Start == B.Start ? A.End > B.End : A.Start < B.Start;
	});
	TArray<FAircraftCorridorParameterInterval> Merged;
	for (const FAircraftCorridorParameterInterval& Interval : OutMergedIntervals)
	{
		if (Merged.IsEmpty() || Interval.Start > Merged.Last().End + UE_KINDA_SMALL_NUMBER)
		{
			Merged.Add(Interval);
		}
		else if (Interval.End > Merged.Last().End)
		{
			Merged.Last().End = Interval.End;
		}
	}
	OutMergedIntervals = MoveTemp(Merged);
	return !OutMergedIntervals.IsEmpty();
}

bool FAircraftSafeCorridorSelection::IsLineContinuouslyCovered(
	const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
	const TConstArrayView<int32> CandidateIndices, const FVector& StartCm,
	const FVector& EndCm, const float SafetyMarginCm)
{
	TArray<FAircraftCorridorParameterInterval> Intervals;
	return BuildLineCoverageIntervals(Corridor, CandidateIndices, StartCm, EndCm,
		SafetyMarginCm, Intervals)
		&& Intervals.Num() == 1
		&& Intervals[0].Start <= UE_KINDA_SMALL_NUMBER
		&& Intervals[0].End >= 1.0f - UE_KINDA_SMALL_NUMBER;
}
