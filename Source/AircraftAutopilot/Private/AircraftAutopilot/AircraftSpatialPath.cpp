#include "AircraftAutopilot/AircraftSpatialPath.h"

namespace
{
	constexpr int32 ArcTableSubdivisions = 32;

	float WrapDistance(float Distance, float Length)
	{
		const float Wrapped = FMath::Fmod(Distance, Length);
		return Wrapped < 0.0f ? Wrapped + Length : Wrapped;
	}

	int32 FindCorridorSegment(const TArray<FAircraftSafeCorridorSegment>& Corridor, float DistanceCm)
	{
		return Corridor.IndexOfByPredicate([DistanceCm](const FAircraftSafeCorridorSegment& Segment)
		{
			return DistanceCm >= Segment.StartDistanceCm && DistanceCm <= Segment.EndDistanceCm;
		});
	}

	void ResamplePolyline(TArray<FVector>& Points, bool bClosed, float SpacingCm)
	{
		if (Points.Num() < 2 || SpacingCm <= 0.0f)
		{
			return;
		}
		TArray<FVector> Resampled;
		Resampled.Reserve(Points.Num());
		Resampled.Add(Points[0]);
		const int32 SegmentCount = bClosed ? Points.Num() : Points.Num() - 1;
		for (int32 Index = 0; Index < SegmentCount; ++Index)
		{
			const FVector& A = Points[Index];
			const FVector& B = Points[(Index + 1) % Points.Num()];
			const int32 Subdivisions = FMath::Max(1,
				FMath::CeilToInt(FVector::Distance(A, B) / SpacingCm));
			for (int32 Step = 1; Step <= Subdivisions; ++Step)
			{
				if (bClosed && Index == SegmentCount - 1 && Step == Subdivisions)
				{
					continue;
				}
				Resampled.Add(FMath::Lerp(A, B,
					static_cast<double>(Step) / static_cast<double>(Subdivisions)));
			}
		}
		Points = MoveTemp(Resampled);
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
	bClosed = false;
}

void FAircraftSpatialPath::OptimizeKnots(
	TArray<FVector>& Points, bool bInClosed,
	const TArray<FAircraftSafeCorridorSegment>& Corridor,
	const FAircraftPathOptimizationRuntimeConfig& Config)
{
	if (Points.Num() < 3 || Config.MaxIterations <= 0)
	{
		return;
	}

	const TArray<FVector> Centerline = Points;
	TArray<float> Distances;
	Distances.SetNumZeroed(Points.Num());
	for (int32 Index = 1; Index < Points.Num(); ++Index)
	{
		Distances[Index] = Distances[Index - 1] + FVector::Distance(Points[Index - 1], Points[Index]);
	}

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

			if (const int32 CorridorIndex = FindCorridorSegment(Corridor, Distances[Index]);
				Corridor.IsValidIndex(CorridorIndex))
			{
				for (const FPlane& Plane : Corridor[CorridorIndex].BoundaryPlanes)
				{
					const FVector Normal(Plane.X, Plane.Y, Plane.Z);
					const double NormalSquared = Normal.SizeSquared();
					const double Violation = Plane.PlaneDot(Candidate) + Config.CorridorSafetyMarginCm;
					if (Violation > 0.0 && NormalSquared > UE_DOUBLE_SMALL_NUMBER)
					{
						Candidate -= Normal * (Violation / NormalSquared);
					}
				}
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
	TArray<FVector> Points;
	Points.Reserve(Route.PointsCm.Num());
	for (const FVector& Point : Route.PointsCm)
	{
		if (!Point.ContainsNaN() && (Points.IsEmpty()
			|| FVector::Distance(Points.Last(), Point) >= Config.MinimumSegmentLengthCm))
		{
			Points.Add(Point);
		}
	}
	if (Route.bClosed && Points.Num() > 2
		&& FVector::Distance(Points[0], Points.Last()) < Config.MinimumSegmentLengthCm)
	{
		Points.Pop();
	}
	if (Points.Num() < (Route.bClosed ? 3 : 2))
	{
		return false;
	}

	ResamplePolyline(Points, Route.bClosed, Config.ResampleSpacingCm);
	OptimizeKnots(Points, Route.bClosed, Route.Corridor, Config);
	TArray<FVector> First;
	TArray<FVector> Second;
	BuildDerivatives(Points, Route.bClosed, First, Second);

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
	for (const FSegment& Segment : Segments)
	{
		for (int32 SampleIndex = 0; SampleIndex <= ArcTableSubdivisions; ++SampleIndex)
		{
			const float DistanceCm = Segment.StartDistanceCm
				+ Segment.ArcLengthsCm[SampleIndex];
			const int32 CorridorIndex = FindCorridorSegment(Route.Corridor, DistanceCm);
			if (!Route.Corridor.IsValidIndex(CorridorIndex))
			{
				continue;
			}
			const FVector Position = Segment.Evaluate(
				static_cast<float>(SampleIndex) / static_cast<float>(ArcTableSubdivisions));
			for (const FPlane& Plane : Route.Corridor[CorridorIndex].BoundaryPlanes)
			{
				if (Plane.PlaneDot(Position) + Config.CorridorSafetyMarginCm
					> Config.ConvergenceToleranceCm)
				{
					Reset();
					return false;
				}
			}
		}
	}
	return IsValid();
}

bool FAircraftSpatialPath::Evaluate(float DistanceCm, FAircraftSpatialPathState& OutState) const
{
	OutState = {};
	if (!IsValid())
	{
		return false;
	}
	const float Distance = bClosed
		? WrapDistance(DistanceCm, TotalLengthCm)
		: FMath::Clamp(DistanceCm, 0.0f, TotalLengthCm);
	int32 SegmentIndex = Segments.Num() - 1;
	for (int32 Index = 0; Index < Segments.Num(); ++Index)
	{
		if (Distance <= Segments[Index].StartDistanceCm + Segments[Index].LengthCm)
		{
			SegmentIndex = Index;
			break;
		}
	}
	const FSegment& Segment = Segments[SegmentIndex];
	const float U = Segment.ParameterAtArcLength(Distance - Segment.StartDistanceCm);
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

bool FAircraftSpatialPath::Project(
	const FVector& PositionCm, float InitialDistanceCm, FAircraftSpatialPathState& OutState) const
{
	if (!IsValid())
	{
		OutState = {};
		return false;
	}
	const float SearchRadius = FMath::Max(500.0f, TotalLengthCm * 0.1f);
	float BestDistance = bClosed ? WrapDistance(InitialDistanceCm, TotalLengthCm)
		: FMath::Clamp(InitialDistanceCm, 0.0f, TotalLengthCm);
	double BestErrorSquared = TNumericLimits<double>::Max();
	constexpr int32 Samples = 32;
	for (int32 Index = 0; Index <= Samples; ++Index)
	{
		const float Offset = FMath::Lerp(-SearchRadius, SearchRadius,
			static_cast<float>(Index) / static_cast<float>(Samples));
		FAircraftSpatialPathState Candidate;
		Evaluate(BestDistance + Offset, Candidate);
		const double ErrorSquared = FVector::DistSquared(PositionCm, Candidate.PositionCm);
		if (ErrorSquared < BestErrorSquared)
		{
			BestErrorSquared = ErrorSquared;
			BestDistance = Candidate.DistanceCm;
		}
	}

	for (int32 Iteration = 0; Iteration < 6; ++Iteration)
	{
		FAircraftSpatialPathState Candidate;
		Evaluate(BestDistance, Candidate);
		const float Step = static_cast<float>(FVector::DotProduct(
			PositionCm - Candidate.PositionCm, Candidate.Tangent));
		BestDistance = bClosed ? WrapDistance(BestDistance + Step, TotalLengthCm)
			: FMath::Clamp(BestDistance + Step, 0.0f, TotalLengthCm);
		if (FMath::Abs(Step) < 0.01f)
		{
			break;
		}
	}
	return Evaluate(BestDistance, OutState);
}
