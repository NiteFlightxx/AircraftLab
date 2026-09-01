#include "AircraftAutopilot/AircraftSpatialPath.h"

#include "AircraftDiagnostics/AircraftDebug.h"

namespace
{
	constexpr int32 ArcTableSubdivisions = 64;

	float WrapDistance(float Distance, float Length)
	{
		const float Wrapped = FMath::Fmod(Distance, Length);
		return Wrapped < 0.0f ? Wrapped + Length : Wrapped;
	}

	void ResamplePolyline(TArray<FVector>& Points, TArray<float>& RouteDistancesCm,
		bool bClosed, float SpacingCm, float RouteLengthCm,
		const TConstArrayView<FAircraftSafeCorridorSegment> Corridor)
	{
		if (Points.Num() < 2 || Points.Num() != RouteDistancesCm.Num() || SpacingCm <= 0.0f)
		{
			return;
		}
		TArray<FVector> Resampled;
		TArray<float> ResampledRouteDistancesCm;
		Resampled.Reserve(Points.Num());
		ResampledRouteDistancesCm.Reserve(RouteDistancesCm.Num());
		Resampled.Add(Points[0]);
		ResampledRouteDistancesCm.Add(RouteDistancesCm[0]);
		const int32 SegmentCount = bClosed ? Points.Num() : Points.Num() - 1;
		for (int32 Index = 0; Index < SegmentCount; ++Index)
		{
			const FVector& A = Points[Index];
			const int32 NextIndex = (Index + 1) % Points.Num();
			const FVector& B = Points[NextIndex];
			const float RouteStartDistanceCm = RouteDistancesCm[Index];
			const float RouteEndDistanceCm = bClosed && Index == SegmentCount - 1
				? RouteLengthCm : RouteDistancesCm[NextIndex];
			const int32 Subdivisions = FMath::Max(1,
				FMath::CeilToInt(FVector::Distance(A, B) / SpacingCm));
			TArray<float, TInlineAllocator<16>> SampleRouteDistancesCm;
			SampleRouteDistancesCm.Reserve(Subdivisions + Corridor.Num() * 2);
			for (int32 Step = 1; Step <= Subdivisions; ++Step)
			{
				const float Alpha = static_cast<float>(Step) / static_cast<float>(Subdivisions);
				SampleRouteDistancesCm.Add(FMath::Lerp(
					RouteStartDistanceCm, RouteEndDistanceCm, Alpha));
			}
			for (const FAircraftSafeCorridorSegment& Segment : Corridor)
			{
				const float BoundaryDistancesCm[] = {
					Segment.StartDistanceCm, Segment.EndDistanceCm};
				for (const float BoundaryDistanceCm : BoundaryDistancesCm)
				{
					if (BoundaryDistanceCm > RouteStartDistanceCm + UE_KINDA_SMALL_NUMBER
						&& BoundaryDistanceCm < RouteEndDistanceCm - UE_KINDA_SMALL_NUMBER)
					{
						SampleRouteDistancesCm.Add(BoundaryDistanceCm);
					}
				}
			}
			SampleRouteDistancesCm.Sort();
			float PreviousSampleDistanceCm = -TNumericLimits<float>::Max();
			for (const float SampleRouteDistanceCm : SampleRouteDistancesCm)
			{
				if (FMath::IsNearlyEqual(SampleRouteDistanceCm,
					PreviousSampleDistanceCm, UE_KINDA_SMALL_NUMBER))
				{
					continue;
				}
				PreviousSampleDistanceCm = SampleRouteDistanceCm;
				if (bClosed && Index == SegmentCount - 1
					&& FMath::IsNearlyEqual(SampleRouteDistanceCm,
						RouteLengthCm, UE_KINDA_SMALL_NUMBER))
				{
					continue;
				}
				const float Alpha = (SampleRouteDistanceCm - RouteStartDistanceCm)
					/ (RouteEndDistanceCm - RouteStartDistanceCm);
				Resampled.Add(FMath::Lerp(A, B, Alpha));
				ResampledRouteDistancesCm.Add(SampleRouteDistanceCm);
			}
		}
		Points = MoveTemp(Resampled);
		RouteDistancesCm = MoveTemp(ResampledRouteDistancesCm);
	}

	bool IsCorridorBoundaryDistance(
		const float RouteDistanceCm,
		const TConstArrayView<FAircraftSafeCorridorSegment> Corridor)
	{
		for (const FAircraftSafeCorridorSegment& Segment : Corridor)
		{
			if (FMath::IsNearlyEqual(RouteDistanceCm,
				Segment.StartDistanceCm, UE_KINDA_SMALL_NUMBER)
				|| FMath::IsNearlyEqual(RouteDistanceCm,
					Segment.EndDistanceCm, UE_KINDA_SMALL_NUMBER))
			{
				return true;
			}
		}
		return false;
	}

	bool BuildSegmentCorridorIndices(
		const TConstArrayView<float> RouteDistancesCm,
		const bool bClosed, const float RouteLengthCm,
		const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
		TArray<int32>& OutSegmentCorridorIndices, int32& OutFailedSegmentIndex)
	{
		const int32 SegmentCount = bClosed
			? RouteDistancesCm.Num() : RouteDistancesCm.Num() - 1;
		OutSegmentCorridorIndices.Init(INDEX_NONE, SegmentCount);
		OutFailedSegmentIndex = INDEX_NONE;
		if (Corridor.IsEmpty())
		{
			return true;
		}

		for (int32 SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
		{
			const int32 NextIndex = (SegmentIndex + 1) % RouteDistancesCm.Num();
			const float SegmentStartDistanceCm = RouteDistancesCm[SegmentIndex];
			const float SegmentEndDistanceCm = bClosed && SegmentIndex == SegmentCount - 1
				? RouteLengthCm : RouteDistancesCm[NextIndex];
			if (SegmentEndDistanceCm <= SegmentStartDistanceCm + UE_KINDA_SMALL_NUMBER)
			{
				OutFailedSegmentIndex = SegmentIndex;
				return false;
			}
			const int32 CorridorIndex = ResolveAircraftSafeCorridorSegment(
				Corridor, 0.5f * (SegmentStartDistanceCm + SegmentEndDistanceCm), RouteLengthCm);
			if (!Corridor.IsValidIndex(CorridorIndex))
			{
				OutFailedSegmentIndex = SegmentIndex;
				return false;
			}

			const FAircraftSafeCorridorSegment& CorridorSegment = Corridor[CorridorIndex];
			if (SegmentStartDistanceCm < CorridorSegment.StartDistanceCm - UE_KINDA_SMALL_NUMBER
				|| SegmentEndDistanceCm > CorridorSegment.EndDistanceCm + UE_KINDA_SMALL_NUMBER)
			{
				OutFailedSegmentIndex = SegmentIndex;
				return false;
			}
			OutSegmentCorridorIndices[SegmentIndex] = CorridorIndex;
		}
		return true;
	}

	bool AccumulateDerivativeControlScale(
		const FVector& KnotPositionCm, const FVector& ControlDeltaCm,
		const FAircraftSafeCorridorSegment& CorridorSegment,
		const float SafetyMarginCm, float& InOutScale)
	{
		float ExitParameter = 0.0f;
		if (!CorridorSegment.ComputeRayExitParameter(
			KnotPositionCm, ControlDeltaCm, SafetyMarginCm, ExitParameter))
		{
			return false;
		}
		InOutScale = FMath::Min(InOutScale, ExitParameter);
		return true;
	}

	bool ConstrainDerivativesToCorridor(
		const TConstArrayView<FVector> Points,
		const bool bClosed,
		const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
		const TConstArrayView<int32> SegmentCorridorIndices,
		const float SafetyMarginCm,
		TArray<FVector>& InOutFirst, TArray<FVector>& InOutSecond,
		int32& OutFailedKnotIndex)
	{
		OutFailedKnotIndex = INDEX_NONE;
		if (Corridor.IsEmpty())
		{
			return true;
		}

		for (int32 KnotIndex = 0; KnotIndex < Points.Num(); ++KnotIndex)
		{
			float DerivativeScale = 1.0f;
			const bool bHasIncomingSegment = bClosed || KnotIndex > 0;
			const bool bHasOutgoingSegment = bClosed || KnotIndex < Points.Num() - 1;
			if (bHasIncomingSegment)
			{
				const int32 IncomingSegmentIndex =
					(KnotIndex - 1 + SegmentCorridorIndices.Num()) % SegmentCorridorIndices.Num();
				const int32 CorridorIndex = SegmentCorridorIndices[IncomingSegmentIndex];
				if (!Corridor.IsValidIndex(CorridorIndex)
					|| !AccumulateDerivativeControlScale(Points[KnotIndex],
						-InOutFirst[KnotIndex] / 5.0f,
						Corridor[CorridorIndex], SafetyMarginCm, DerivativeScale)
					|| !AccumulateDerivativeControlScale(Points[KnotIndex],
						-2.0f * InOutFirst[KnotIndex] / 5.0f + InOutSecond[KnotIndex] / 20.0f,
						Corridor[CorridorIndex], SafetyMarginCm, DerivativeScale))
				{
					OutFailedKnotIndex = KnotIndex;
					return false;
				}
			}
			if (bHasOutgoingSegment)
			{
				const int32 OutgoingSegmentIndex = KnotIndex % SegmentCorridorIndices.Num();
				const int32 CorridorIndex = SegmentCorridorIndices[OutgoingSegmentIndex];
				if (!Corridor.IsValidIndex(CorridorIndex)
					|| !AccumulateDerivativeControlScale(Points[KnotIndex],
						InOutFirst[KnotIndex] / 5.0f,
						Corridor[CorridorIndex], SafetyMarginCm, DerivativeScale)
					|| !AccumulateDerivativeControlScale(Points[KnotIndex],
						2.0f * InOutFirst[KnotIndex] / 5.0f + InOutSecond[KnotIndex] / 20.0f,
						Corridor[CorridorIndex], SafetyMarginCm, DerivativeScale))
				{
					OutFailedKnotIndex = KnotIndex;
					return false;
				}
			}

			DerivativeScale = FMath::Clamp(DerivativeScale, 0.0f, 1.0f);
			if (DerivativeScale <= UE_SMALL_NUMBER)
			{
				OutFailedKnotIndex = KnotIndex;
				return false;
			}
			InOutFirst[KnotIndex] *= DerivativeScale;
			InOutSecond[KnotIndex] *= DerivativeScale;
		}
		return true;
	}
}

FVector FAircraftSpatialPath::FSegment::Evaluate(float U) const
{
	FVector Result = Coefficients[5];
	for (int32 Index = 4; Index >= 0; --Index)
	{
		Result = Result * U + Coefficients[Index];
	}
	return Result;
}

FVector FAircraftSpatialPath::FSegment::FirstDerivative(float U) const
{
	return Coefficients[1] + U * (2.0 * Coefficients[2] + U *
		(3.0 * Coefficients[3] + U * (4.0 * Coefficients[4] + U * 5.0 * Coefficients[5])));
}

FVector FAircraftSpatialPath::FSegment::SecondDerivative(float U) const
{
	return 2.0 * Coefficients[2] + U *
		(6.0 * Coefficients[3] + U * (12.0 * Coefficients[4] + U * 20.0 * Coefficients[5]));
}

float FAircraftSpatialPath::FSegment::ParameterAtArcLength(float ArcLengthCm) const
{
	const float Clamped = FMath::Clamp(ArcLengthCm, 0.0f, LengthCm);
	const int32 Upper = Algo::LowerBound(ArcLengthsCm, Clamped);
	if (Upper <= 0)
	{
		return 0.0f;
	}
	if (Upper >= ArcLengthsCm.Num())
	{
		return 1.0f;
	}
	const float A = ArcLengthsCm[Upper - 1];
	const float B = ArcLengthsCm[Upper];
	const float Alpha = B > A ? (Clamped - A) / (B - A) : 0.0f;
	return (static_cast<float>(Upper - 1) + Alpha) / static_cast<float>(ArcTableSubdivisions);
}

void FAircraftSpatialPath::Reset()
{
	Segments.Reset();
	TotalLengthCm = 0.0f;
	RouteLengthCm = 0.0f;
	bClosed = false;
	Corridor.Reset();
	CorridorSafetyMarginCm = 0.0f;
	ProjectionBacktrackToleranceCm = 0.0f;
	ProjectionSearchDistanceCm = 0.0f;
	ProjectionSampleSpacingCm = 100.0f;
}

void FAircraftSpatialPath::OptimizeKnots(
	TArray<FVector>& Points, bool bInClosed,
	const TArray<FAircraftSafeCorridorSegment>& Corridor,
	const TConstArrayView<int32> SegmentCorridorIndices,
	const FAircraftPathOptimizationRuntimeConfig& Config)
{
	const int32 ExpectedSegmentCount = bInClosed ? Points.Num() : Points.Num() - 1;
	if (Points.Num() < 3
		|| SegmentCorridorIndices.Num() != ExpectedSegmentCount
		|| Config.MaxIterations <= 0)
	{
		return;
	}

	const TArray<FVector> Centerline = Points;

	const float WeightSum = FMath::Max(
		Config.CenterlineWeight + Config.CurvatureWeight + Config.SnapWeight, UE_SMALL_NUMBER);
	for (int32 Iteration = 0; Iteration < Config.MaxIterations; ++Iteration)
	{
		TArray<FVector> Updated = Points;
		float MaxDisplacement = 0.0f;
		const int32 Begin = bInClosed ? 0 : 1;
		const int32 End = bInClosed ? Points.Num() : Points.Num() - 1;
		for (int32 Index = Begin; Index < End; ++Index)
		{
			const int32 IncomingCorridorIndex = bInClosed || Index > 0
				? SegmentCorridorIndices[(Index - 1 + ExpectedSegmentCount) % ExpectedSegmentCount]
				: INDEX_NONE;
			const int32 OutgoingCorridorIndex = bInClosed || Index < Points.Num() - 1
				? SegmentCorridorIndices[Index % ExpectedSegmentCount]
				: INDEX_NONE;
			if (IncomingCorridorIndex != INDEX_NONE
				&& OutgoingCorridorIndex != INDEX_NONE
				&& IncomingCorridorIndex != OutgoingCorridorIndex)
			{
				continue;
			}

			const int32 Previous = (Index - 1 + Points.Num()) % Points.Num();
			const int32 Next = (Index + 1) % Points.Num();
			const FVector CurvatureTarget = 0.5 * (Points[Previous] + Points[Next]);
			FVector SnapTarget = CurvatureTarget;
			const bool bHasFivePointStencil = bInClosed || (Index >= 2 && Index + 2 < Points.Num());
			if (bHasFivePointStencil)
			{
				const int32 Previous2 = (Index - 2 + Points.Num()) % Points.Num();
				const int32 Next2 = (Index + 2) % Points.Num();
				// Solve the discrete fourth-difference (snap) term for the center knot.
				SnapTarget = (-Points[Previous2] + 4.0 * Points[Previous]
					+ 4.0 * Points[Next] - Points[Next2]) / 6.0;
			}
			FVector Candidate = (Config.CenterlineWeight * Centerline[Index]
				+ Config.CurvatureWeight * CurvatureTarget
				+ Config.SnapWeight * SnapTarget) / WeightSum;

			const int32 CorridorIndex = IncomingCorridorIndex != INDEX_NONE
				? IncomingCorridorIndex : OutgoingCorridorIndex;
			if (Corridor.IsValidIndex(CorridorIndex))
			{
				Candidate += Corridor[CorridorIndex].ComputeCorrectionCm(
					Candidate, Config.CorridorSafetyMarginCm);
			}

			Updated[Index] = Candidate;
			MaxDisplacement = FMath::Max(MaxDisplacement,
				static_cast<float>(FVector::Distance(Points[Index], Candidate)));
		}
		Points = MoveTemp(Updated);
		if (MaxDisplacement <= Config.ConvergenceToleranceCm)
		{
			break;
		}
	}
}

void FAircraftSpatialPath::BuildDerivatives(
	const TArray<FVector>& Points, bool bInClosed,
	TArray<FVector>& OutFirst, TArray<FVector>& OutSecond)
{
	const int32 Count = Points.Num();
	OutFirst.SetNumZeroed(Count);
	OutSecond.SetNumZeroed(Count);
	if (Count == 2)
	{
		OutFirst[0] = Points[1] - Points[0];
		OutFirst[1] = OutFirst[0];
		return;
	}
	for (int32 Index = 0; Index < Count; ++Index)
	{
		if (!bInClosed && Index == 0)
		{
			OutFirst[Index] = Points[1] - Points[0];
			OutSecond[Index] = Points[2] - 2.0 * Points[1] + Points[0];
		}
		else if (!bInClosed && Index == Count - 1)
		{
			OutFirst[Index] = Points[Index] - Points[Index - 1];
			OutSecond[Index] = Points[Index] - 2.0 * Points[Index - 1] + Points[Index - 2];
		}
		else
		{
			const int32 Previous = (Index - 1 + Count) % Count;
			const int32 Next = (Index + 1) % Count;
			OutFirst[Index] = 0.5 * (Points[Next] - Points[Previous]);
			OutSecond[Index] = Points[Next] - 2.0 * Points[Index] + Points[Previous];
		}
	}
}

bool FAircraftSpatialPath::Build(
	const FAircraftRouteIntent& Route, const FAircraftPathOptimizationRuntimeConfig& Config)
{
	Reset();
	for (int32 CorridorIndex = 0; CorridorIndex < Route.Corridor.Num(); ++CorridorIndex)
	{
		const FAircraftSafeCorridorSegment& Segment = Route.Corridor[CorridorIndex];
		if (!Segment.IsGeometryValid()
			|| Segment.RadiusCm - Config.CorridorSafetyMarginCm <= UE_SMALL_NUMBER)
		{
			FAircraftPlanningFailureDiagnostics Diagnostics;
			Diagnostics.Stage = TEXT("SpatialPath.Input");
			Diagnostics.Reason = TEXT("InvalidCorridorCapsule");
			Diagnostics.RoutePointCount = Route.PointsCm.Num();
			Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
			Diagnostics.CorridorSegmentIndex = CorridorIndex;
			Diagnostics.CorridorAxisStartCm = Segment.AxisStartCm;
			Diagnostics.CorridorAxisEndCm = Segment.AxisEndCm;
			Diagnostics.CorridorRadiusCm = Segment.RadiusCm;
			Diagnostics.CorridorEffectiveRadiusCm =
				Segment.RadiusCm - Config.CorridorSafetyMarginCm;
			FAircraftDebug::LogPlanningFailure(Diagnostics);
			return false;
		}
	}
	TArray<FVector> Points;
	TArray<float> RouteDistancesCm;
	Points.Reserve(Route.PointsCm.Num());
	RouteDistancesCm.Reserve(Route.PointsCm.Num());
	TArray<float> InputRouteDistancesCm;
	InputRouteDistancesCm.SetNumZeroed(Route.PointsCm.Num());
	for (int32 PointIndex = 1; PointIndex < Route.PointsCm.Num(); ++PointIndex)
	{
		InputRouteDistancesCm[PointIndex] = InputRouteDistancesCm[PointIndex - 1]
			+ FVector::Distance(Route.PointsCm[PointIndex - 1], Route.PointsCm[PointIndex]);
	}
	RouteLengthCm = InputRouteDistancesCm.IsEmpty() ? 0.0f : InputRouteDistancesCm.Last();
	if (Route.bClosed && Route.PointsCm.Num() > 1)
	{
		RouteLengthCm += FVector::Distance(Route.PointsCm.Last(), Route.PointsCm[0]);
	}
	for (int32 PointIndex = 0; PointIndex < Route.PointsCm.Num(); ++PointIndex)
	{
		const FVector& Point = Route.PointsCm[PointIndex];
		const bool bCorridorBoundary = IsCorridorBoundaryDistance(
			InputRouteDistancesCm[PointIndex], Route.Corridor);
		if (!Point.ContainsNaN() && (Points.IsEmpty()
			|| FVector::Distance(Points.Last(), Point) >= Config.MinimumSegmentLengthCm
			|| bCorridorBoundary))
		{
			Points.Add(Point);
			RouteDistancesCm.Add(InputRouteDistancesCm[PointIndex]);
		}
	}
	if (Route.bClosed && Points.Num() > 2
		&& FVector::Distance(Points[0], Points.Last()) < Config.MinimumSegmentLengthCm)
	{
		Points.Pop();
		RouteDistancesCm.Pop();
	}
	if (Points.Num() < (Route.bClosed ? 3 : 2))
	{
		FAircraftPlanningFailureDiagnostics Diagnostics;
		Diagnostics.Stage = TEXT("SpatialPath.Input");
		Diagnostics.Reason = TEXT("InsufficientPointsAfterFiltering");
		Diagnostics.RoutePointCount = Route.PointsCm.Num();
		Diagnostics.PathPointCount = Points.Num();
		Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
		FAircraftDebug::LogPlanningFailure(Diagnostics);
		return false;
	}

	ResamplePolyline(Points, RouteDistancesCm, Route.bClosed,
		Config.ResampleSpacingCm, RouteLengthCm, Route.Corridor);
	TArray<int32> SegmentCorridorIndices;
	int32 FailedSegmentIndex = INDEX_NONE;
	if (!BuildSegmentCorridorIndices(RouteDistancesCm, Route.bClosed, RouteLengthCm,
			Route.Corridor, SegmentCorridorIndices, FailedSegmentIndex))
	{
		FAircraftPlanningFailureDiagnostics Diagnostics;
		Diagnostics.Stage = TEXT("SpatialPath.CorridorTopology");
		Diagnostics.Reason = TEXT("PathSegmentCrossesCorridorBoundary");
		Diagnostics.RoutePointCount = Route.PointsCm.Num();
		Diagnostics.PathPointCount = Points.Num();
		Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
		Diagnostics.PathSegmentIndex = FailedSegmentIndex;
		Diagnostics.RouteLengthCm = RouteLengthCm;
		FAircraftDebug::LogPlanningFailure(Diagnostics);
		Reset();
		return false;
	}
	OptimizeKnots(Points, Route.bClosed,
		Route.Corridor, SegmentCorridorIndices, Config);
	TArray<FVector> First;
	TArray<FVector> Second;
	BuildDerivatives(Points, Route.bClosed, First, Second);
	int32 FailedKnotIndex = INDEX_NONE;
	if (!ConstrainDerivativesToCorridor(Points, Route.bClosed,
			Route.Corridor, SegmentCorridorIndices, Config.CorridorSafetyMarginCm,
			First, Second, FailedKnotIndex))
	{
		FAircraftPlanningFailureDiagnostics Diagnostics;
		Diagnostics.Stage = TEXT("SpatialPath.CorridorDerivatives");
		Diagnostics.Reason = TEXT("NoFeasibleBezierControlHull");
		Diagnostics.RoutePointCount = Route.PointsCm.Num();
		Diagnostics.PathPointCount = Points.Num();
		Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
		Diagnostics.PathSampleIndex = FailedKnotIndex;
		Diagnostics.RouteLengthCm = RouteLengthCm;
		Diagnostics.PositionCm = Points.IsValidIndex(FailedKnotIndex)
			? Points[FailedKnotIndex] : FVector::ZeroVector;
		FAircraftDebug::LogPlanningFailure(Diagnostics);
		Reset();
		return false;
	}

	bClosed = Route.bClosed;
	const int32 SegmentCount = bClosed ? Points.Num() : Points.Num() - 1;
	Segments.Reserve(SegmentCount);
	for (int32 Index = 0; Index < SegmentCount; ++Index)
	{
		const int32 Next = (Index + 1) % Points.Num();
		FSegment& Segment = Segments.AddDefaulted_GetRef();
		Segment.Coefficients[0] = Points[Index];
		Segment.Coefficients[1] = First[Index];
		Segment.Coefficients[2] = 0.5 * Second[Index];
		const FVector C0 = Points[Next] - Segment.Coefficients[0]
			- Segment.Coefficients[1] - Segment.Coefficients[2];
		const FVector C1 = First[Next] - Segment.Coefficients[1]
			- 2.0 * Segment.Coefficients[2];
		const FVector C2 = Second[Next] - 2.0 * Segment.Coefficients[2];
		Segment.Coefficients[3] = 10.0 * C0 - 4.0 * C1 + 0.5 * C2;
		Segment.Coefficients[4] = -15.0 * C0 + 7.0 * C1 - C2;
		Segment.Coefficients[5] = 6.0 * C0 - 3.0 * C1 + 0.5 * C2;
		Segment.StartDistanceCm = TotalLengthCm;
		Segment.RouteStartDistanceCm = RouteDistancesCm[Index];
		Segment.RouteEndDistanceCm = bClosed && Index == SegmentCount - 1
			? RouteLengthCm : RouteDistancesCm[Next];
		Segment.ArcLengthsCm.SetNumZeroed(ArcTableSubdivisions + 1);
		FVector Previous = Segment.Evaluate(0.0f);
		for (int32 Sample = 1; Sample <= ArcTableSubdivisions; ++Sample)
		{
			const FVector Current = Segment.Evaluate(
				static_cast<float>(Sample) / static_cast<float>(ArcTableSubdivisions));
			Segment.ArcLengthsCm[Sample] = Segment.ArcLengthsCm[Sample - 1]
				+ FVector::Distance(Previous, Current);
			Previous = Current;
		}
		Segment.LengthCm = Segment.ArcLengthsCm.Last();
		TotalLengthCm += Segment.LengthCm;
	}
	Corridor = Route.Corridor;
	CorridorSafetyMarginCm = Config.CorridorSafetyMarginCm;
	ProjectionBacktrackToleranceCm = Config.ProjectionBacktrackToleranceCm;
	ProjectionSearchDistanceCm = Config.ProjectionSearchDistanceCm;
	ProjectionSampleSpacingCm = FMath::Max(Config.ResampleSpacingCm, 10.0f);
	for (int32 PathSegmentIndex = 0; PathSegmentIndex < Segments.Num(); ++PathSegmentIndex)
	{
		const FSegment& Segment = Segments[PathSegmentIndex];
		for (int32 SampleIndex = 0; SampleIndex <= ArcTableSubdivisions; ++SampleIndex)
		{
			const float Parameter = static_cast<float>(SampleIndex)
				/ static_cast<float>(ArcTableSubdivisions);
			const float DistanceCm = Segment.StartDistanceCm
				+ Segment.ArcLengthsCm[SampleIndex];
			const float RouteDistanceCm = FMath::Lerp(
				Segment.RouteStartDistanceCm, Segment.RouteEndDistanceCm, Parameter);
			const int32 CorridorIndex = ResolveAircraftSafeCorridorSegment(
				Route.Corridor, RouteDistanceCm, RouteLengthCm);
			if (!Route.Corridor.IsValidIndex(CorridorIndex))
			{
				if (!Route.Corridor.IsEmpty())
				{
					FAircraftPlanningFailureDiagnostics Diagnostics;
					Diagnostics.Stage = TEXT("SpatialPath.CorridorValidation");
					Diagnostics.Reason = TEXT("NoCorridorAtDistance");
					Diagnostics.RoutePointCount = Route.PointsCm.Num();
					Diagnostics.PathPointCount = Points.Num();
					Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
					Diagnostics.PathSegmentIndex = PathSegmentIndex;
					Diagnostics.PathSampleIndex = SampleIndex;
					Diagnostics.PathDistanceCm = DistanceCm;
					Diagnostics.RouteDistanceCm = RouteDistanceCm;
					Diagnostics.RouteLengthCm = RouteLengthCm;
					Diagnostics.PlannedLengthCm = TotalLengthCm;
					Diagnostics.PositionCm = Segment.Evaluate(Parameter);
					FAircraftDebug::LogPlanningFailure(Diagnostics);
					Reset();
					return false;
				}
				continue;
			}
			const FVector Position = Segment.Evaluate(Parameter);
			const FAircraftSafeCorridorSegment& CorridorSegment = Route.Corridor[CorridorIndex];
			const float ConstraintValueCm = static_cast<float>(
				CorridorSegment.ComputeCorrectionCm(
					Position, Config.CorridorSafetyMarginCm).Size());
			if (ConstraintValueCm > Config.ConvergenceToleranceCm)
			{
				FAircraftPlanningFailureDiagnostics Diagnostics;
				Diagnostics.Stage = TEXT("SpatialPath.CorridorValidation");
				Diagnostics.Reason = TEXT("CorridorCapsuleViolation");
				Diagnostics.RoutePointCount = Route.PointsCm.Num();
				Diagnostics.PathPointCount = Points.Num();
				Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
				Diagnostics.PathSegmentIndex = PathSegmentIndex;
				Diagnostics.PathSampleIndex = SampleIndex;
				Diagnostics.CorridorSegmentIndex = CorridorIndex;
				Diagnostics.PathDistanceCm = DistanceCm;
				Diagnostics.RouteDistanceCm = RouteDistanceCm;
				Diagnostics.RouteLengthCm = RouteLengthCm;
				Diagnostics.PlannedLengthCm = TotalLengthCm;
				Diagnostics.ConstraintValueCm = ConstraintValueCm;
				Diagnostics.ConstraintToleranceCm = Config.ConvergenceToleranceCm;
				Diagnostics.PositionCm = Position;
				Diagnostics.CorridorAxisStartCm = CorridorSegment.AxisStartCm;
				Diagnostics.CorridorAxisEndCm = CorridorSegment.AxisEndCm;
				Diagnostics.CorridorRadiusCm = CorridorSegment.RadiusCm;
				Diagnostics.CorridorEffectiveRadiusCm = CorridorSegment.RadiusCm
					- Config.CorridorSafetyMarginCm;
				FAircraftDebug::LogPlanningFailure(Diagnostics);
				Reset();
				return false;
			}
		}
	}
	if (!IsValid())
	{
		FAircraftPlanningFailureDiagnostics Diagnostics;
		Diagnostics.Stage = TEXT("SpatialPath.Output");
		Diagnostics.Reason = TEXT("InvalidBuiltPath");
		Diagnostics.RoutePointCount = Route.PointsCm.Num();
		Diagnostics.PathPointCount = Points.Num();
		Diagnostics.CorridorSegmentCount = Route.Corridor.Num();
		Diagnostics.RouteLengthCm = RouteLengthCm;
		Diagnostics.PlannedLengthCm = TotalLengthCm;
		FAircraftDebug::LogPlanningFailure(Diagnostics);
		return false;
	}
	return true;
}

bool FAircraftSpatialPath::ResolveSegmentParameter(
	const float DistanceCm, int32& OutSegmentIndex, float& OutParameter) const
{
	OutSegmentIndex = INDEX_NONE;
	OutParameter = 0.0f;
	if (!IsValid())
	{
		return false;
	}
	const float Distance = bClosed
		? WrapDistance(DistanceCm, TotalLengthCm)
		: FMath::Clamp(DistanceCm, 0.0f, TotalLengthCm);
	OutSegmentIndex = Segments.Num() - 1;
	for (int32 Index = 0; Index < Segments.Num(); ++Index)
	{
		if (Distance <= Segments[Index].StartDistanceCm + Segments[Index].LengthCm)
		{
			OutSegmentIndex = Index;
			break;
		}
	}
	const FSegment& Segment = Segments[OutSegmentIndex];
	OutParameter = Segment.ParameterAtArcLength(Distance - Segment.StartDistanceCm);
	return true;
}

bool FAircraftSpatialPath::Evaluate(float DistanceCm, FAircraftSpatialPathState& OutState) const
{
	OutState = {};
	int32 SegmentIndex = INDEX_NONE;
	float U = 0.0f;
	if (!ResolveSegmentParameter(DistanceCm, SegmentIndex, U))
	{
		return false;
	}
	const float Distance = bClosed
		? WrapDistance(DistanceCm, TotalLengthCm)
		: FMath::Clamp(DistanceCm, 0.0f, TotalLengthCm);
	const FSegment& Segment = Segments[SegmentIndex];
	const FVector D1 = Segment.FirstDerivative(U);
	const FVector D2 = Segment.SecondDerivative(U);
	const double D1Squared = D1.SizeSquared();
	OutState.PositionCm = Segment.Evaluate(U);
	OutState.Tangent = D1.GetSafeNormal();
	OutState.CurvaturePerCm = D1Squared > UE_DOUBLE_SMALL_NUMBER
		? (D2 - D1 * (FVector::DotProduct(D1, D2) / D1Squared)) / D1Squared
		: FVector::ZeroVector;
	OutState.DistanceCm = Distance;
	OutState.SegmentIndex = SegmentIndex;
	OutState.bValid = true;
	return true;
}

float FAircraftSpatialPath::GetRouteDistanceCm(const float DistanceCm) const
{
	if (bClosed && DistanceCm > 0.0f && TotalLengthCm > UE_SMALL_NUMBER
		&& FMath::IsNearlyZero(FMath::Fmod(DistanceCm, TotalLengthCm), UE_KINDA_SMALL_NUMBER))
	{
		return RouteLengthCm;
	}
	int32 SegmentIndex = INDEX_NONE;
	float Parameter = 0.0f;
	if (!ResolveSegmentParameter(DistanceCm, SegmentIndex, Parameter))
	{
		return 0.0f;
	}
	const FSegment& Segment = Segments[SegmentIndex];
	return FMath::Lerp(
		Segment.RouteStartDistanceCm, Segment.RouteEndDistanceCm, Parameter);
}

bool FAircraftSpatialPath::Project(
	const FVector& PositionCm, float InitialDistanceCm,
	bool bGlobalSearch, FAircraftSpatialPathState& OutState) const
{
	if (!IsValid())
	{
		OutState = {};
		return false;
	}
	const float SearchCenter = bClosed ? InitialDistanceCm
		: FMath::Clamp(InitialDistanceCm, 0.0f, TotalLengthCm);
	const float SearchStart = bGlobalSearch ? 0.0f : FMath::Max(
		SearchCenter - ProjectionBacktrackToleranceCm, bClosed ? -TotalLengthCm : 0.0f);
	const float SearchEnd = bGlobalSearch ? TotalLengthCm : FMath::Min(
		SearchCenter + ProjectionSearchDistanceCm, bClosed ? SearchCenter + TotalLengthCm : TotalLengthCm);
	float BestDistance = SearchCenter;
	double BestErrorSquared = TNumericLimits<double>::Max();
	const int32 Samples = FMath::Clamp(
		FMath::CeilToInt((SearchEnd - SearchStart) / ProjectionSampleSpacingCm), 16, 512);
	for (int32 Index = 0; Index <= Samples; ++Index)
	{
		const float CandidateDistance = FMath::Lerp(SearchStart, SearchEnd,
			static_cast<float>(Index) / static_cast<float>(Samples));
		FAircraftSpatialPathState Candidate;
		Evaluate(CandidateDistance, Candidate);
		const double ErrorSquared = FVector::DistSquared(PositionCm, Candidate.PositionCm);
		if (ErrorSquared < BestErrorSquared)
		{
			BestErrorSquared = ErrorSquared;
			BestDistance = CandidateDistance;
		}
	}

	for (int32 Iteration = 0; Iteration < 6; ++Iteration)
	{
		FAircraftSpatialPathState Candidate;
		Evaluate(BestDistance, Candidate);
		const float Step = static_cast<float>(FVector::DotProduct(
			PositionCm - Candidate.PositionCm, Candidate.Tangent));
		BestDistance = FMath::Clamp(BestDistance + Step, SearchStart, SearchEnd);
		if (FMath::Abs(Step) < 0.01f)
		{
			break;
		}
	}
	return Evaluate(BestDistance, OutState);
}

float FAircraftSpatialPath::ComputeCorridorViolationCm(
	const FVector& PositionCm, float DistanceCm) const
{
	return static_cast<float>(ComputeCorridorCorrectionCm(PositionCm, DistanceCm).Size());
}

FVector FAircraftSpatialPath::ComputeCorridorCorrectionCm(
	const FVector& PositionCm, float DistanceCm) const
{
	if (Corridor.IsEmpty())
	{
		return FVector::ZeroVector;
	}
	const int32 CorridorIndex = ResolveAircraftSafeCorridorSegment(
		Corridor, GetRouteDistanceCm(DistanceCm), RouteLengthCm);
	if (!Corridor.IsValidIndex(CorridorIndex))
	{
		return FVector::ZeroVector;
	}
	return Corridor[CorridorIndex].ComputeCorrectionCm(PositionCm, CorridorSafetyMarginCm);
}
