#include "AircraftAutopilot/AircraftSafeCorridorBuilder.h"

namespace
{
	constexpr float GeometryToleranceCm = 0.01f;

	struct FCanonicalPathPoint
	{
		FVector PositionCm = FVector::ZeroVector;
		int32 InputPointIndex = INDEX_NONE;
	};

	struct FCornerGeometry
	{
		FVector CenterCm = FVector::ZeroVector;
		FVector IncomingDirection = FVector::ZeroVector;
		FVector OutgoingDirection = FVector::ZeroVector;
		float EntryExtentCm = 0.0f;
		float ExitExtentCm = 0.0f;
	};

	bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X)
			&& FMath::IsFinite(Value.Y)
			&& FMath::IsFinite(Value.Z);
	}

	FVector EvaluateQuadraticBezier(
		const FVector& Entry, const FVector& Control, const FVector& Exit, const float Alpha)
	{
		const float OneMinusAlpha = 1.0f - Alpha;
		return FMath::Square(OneMinusAlpha) * Entry
			+ 2.0f * OneMinusAlpha * Alpha * Control
			+ FMath::Square(Alpha) * Exit;
	}

	FVector EvaluateQuadraticBezierDerivative(
		const FVector& Entry, const FVector& Control, const FVector& Exit, const float Alpha)
	{
		return 2.0f * ((1.0f - Alpha) * (Control - Entry) + Alpha * (Exit - Control));
	}

	bool AppendRoutePoint(
		const FVector& PointCm, FAircraftRouteIntent& OutRoute, float& InOutRouteLengthCm)
	{
		if (OutRoute.PointsCm.IsEmpty())
		{
			OutRoute.PointsCm.Add(PointCm);
			return true;
		}

		const float SegmentLengthCm = FVector::Distance(OutRoute.PointsCm.Last(), PointCm);
		if (SegmentLengthCm <= GeometryToleranceCm)
		{
			return true;
		}
		if (!FMath::IsFinite(SegmentLengthCm))
		{
			return false;
		}
		InOutRouteLengthCm += SegmentLengthCm;
		OutRoute.PointsCm.Add(PointCm);
		return true;
	}

	bool AppendQuadraticBezierRange(
		const FVector& Entry, const FVector& Control, const FVector& Exit,
		const float StartAlpha, const float EndAlpha, const float ResampleSpacingCm,
		FAircraftRouteIntent& OutRoute, float& InOutRouteLengthCm)
	{
		const FVector RangeStart = EvaluateQuadraticBezier(Entry, Control, Exit, StartAlpha);
		const FVector RangeEnd = EvaluateQuadraticBezier(Entry, Control, Exit, EndAlpha);
		const FVector RangeControl = RangeStart
			+ 0.5f * (EndAlpha - StartAlpha)
			* EvaluateQuadraticBezierDerivative(Entry, Control, Exit, StartAlpha);
		const float ControlPolygonLengthCm = FVector::Distance(RangeStart, RangeControl)
			+ FVector::Distance(RangeControl, RangeEnd);
		if (!FMath::IsFinite(ControlPolygonLengthCm)
			|| ControlPolygonLengthCm <= GeometryToleranceCm)
		{
			return false;
		}

		const int32 SubdivisionCount = FMath::Max(1,
			FMath::CeilToInt(ControlPolygonLengthCm / ResampleSpacingCm));
		for (int32 Step = 1; Step <= SubdivisionCount; ++Step)
		{
			const float RangeAlpha = static_cast<float>(Step)
				/ static_cast<float>(SubdivisionCount);
			const float Alpha = FMath::Lerp(StartAlpha, EndAlpha, RangeAlpha);
			const FVector PointCm = Step == SubdivisionCount
				? RangeEnd : EvaluateQuadraticBezier(Entry, Control, Exit, Alpha);
			if (!AppendRoutePoint(PointCm, OutRoute, InOutRouteLengthCm))
			{
				return false;
			}
		}
		return true;
	}

	bool IsCorridorPartitionValid(const FAircraftRouteIntent& Route, const float RouteLengthCm)
	{
		if (Route.PointsCm.Num() < 2 || Route.Corridor.IsEmpty())
		{
			return false;
		}

		float ExpectedStartDistanceCm = 0.0f;
		for (const FAircraftSafeCorridorSegment& Segment : Route.Corridor)
		{
			if (!Segment.IsGeometryValid()
				|| !FMath::IsNearlyEqual(Segment.StartDistanceCm,
					ExpectedStartDistanceCm, UE_KINDA_SMALL_NUMBER)
				|| Segment.EndDistanceCm <= Segment.StartDistanceCm)
			{
				return false;
			}
			ExpectedStartDistanceCm = Segment.EndDistanceCm;
		}
		return FMath::IsNearlyEqual(ExpectedStartDistanceCm,
			RouteLengthCm, UE_KINDA_SMALL_NUMBER);
	}
}

bool FAircraftSafeCorridorBuildSettings::IsValid() const
{
	return FMath::IsFinite(OuterRadiusCm) && OuterRadiusCm > 0.0f
		&& FMath::IsFinite(MinimumSegmentLengthCm) && MinimumSegmentLengthCm > 0.0f;
}

FAircraftSafeCorridorBuildResult FAircraftSafeCorridorBuilder::BuildOpenPolyline(
	const TConstArrayView<FVector> PathPointsCm,
	const FAircraftSafeCorridorBuildSettings& Settings,
	const FAircraftPathOptimizationRuntimeConfig& PathConfig,
	FAircraftRouteIntent& OutRoute)
{
	OutRoute = {};
	FAircraftSafeCorridorBuildResult Result;
	if (!Settings.IsValid()
		|| !FMath::IsFinite(PathConfig.CorridorSafetyMarginCm)
		|| PathConfig.CorridorSafetyMarginCm < 0.0f
		|| !FMath::IsFinite(PathConfig.ResampleSpacingCm)
		|| PathConfig.ResampleSpacingCm <= 0.0f)
	{
		return Result;
	}

	const float EffectiveRadiusCm = Settings.OuterRadiusCm - PathConfig.CorridorSafetyMarginCm;
	if (EffectiveRadiusCm <= GeometryToleranceCm)
	{
		Result.Status = EAircraftSafeCorridorBuildStatus::InsufficientClearance;
		return Result;
	}

	TArray<FCanonicalPathPoint> FilteredPoints;
	FilteredPoints.Reserve(PathPointsCm.Num());
	for (int32 InputPointIndex = 0; InputPointIndex < PathPointsCm.Num(); ++InputPointIndex)
	{
		const FVector& PointCm = PathPointsCm[InputPointIndex];
		if (!IsFiniteVector(PointCm))
		{
			Result.Status = EAircraftSafeCorridorBuildStatus::InvalidPoint;
			Result.InputPointIndex = InputPointIndex;
			return Result;
		}
		if (FilteredPoints.IsEmpty()
			|| FVector::Distance(FilteredPoints.Last().PositionCm, PointCm)
				>= Settings.MinimumSegmentLengthCm)
		{
			FilteredPoints.Add({PointCm, InputPointIndex});
		}
	}
	if (FilteredPoints.Num() < 2)
	{
		Result.Status = EAircraftSafeCorridorBuildStatus::InsufficientPoints;
		return Result;
	}

	TArray<FCanonicalPathPoint> Points;
	Points.Reserve(FilteredPoints.Num());
	for (const FCanonicalPathPoint& Point : FilteredPoints)
	{
		while (Points.Num() >= 2)
		{
			const FVector IncomingDirection =
				(Points.Last().PositionCm - Points[Points.Num() - 2].PositionCm).GetSafeNormal();
			const FVector OutgoingDirection =
				(Point.PositionCm - Points.Last().PositionCm).GetSafeNormal();
			if (FVector::CrossProduct(IncomingDirection, OutgoingDirection).Size()
					> UE_KINDA_SMALL_NUMBER)
			{
				break;
			}
			if (FVector::DotProduct(IncomingDirection, OutgoingDirection) < 0.0)
			{
				Result.Status = EAircraftSafeCorridorBuildStatus::DegenerateTurn;
				Result.InputPointIndex = Points.Last().InputPointIndex;
				return Result;
			}
			Points.Pop();
		}
		Points.Add(Point);
	}
	if (Points.Num() < 2)
	{
		Result.Status = EAircraftSafeCorridorBuildStatus::InsufficientPoints;
		return Result;
	}

	TArray<FCornerGeometry> Corners;
	Corners.SetNum(Points.Num());
	for (int32 PointIndex = 1; PointIndex + 1 < Points.Num(); ++PointIndex)
	{
		FCornerGeometry& Corner = Corners[PointIndex];
		Corner.CenterCm = Points[PointIndex].PositionCm;
		Corner.IncomingDirection =
			(Points[PointIndex].PositionCm - Points[PointIndex - 1].PositionCm).GetSafeNormal();
		Corner.OutgoingDirection =
			(Points[PointIndex + 1].PositionCm - Points[PointIndex].PositionCm).GetSafeNormal();
		Corner.EntryExtentCm = EffectiveRadiusCm;
		Corner.ExitExtentCm = EffectiveRadiusCm;
	}

	for (int32 PointIndex = 0; PointIndex + 1 < Points.Num(); ++PointIndex)
	{
		const float SegmentLengthCm = FVector::Distance(
			Points[PointIndex].PositionCm, Points[PointIndex + 1].PositionCm);
		float& StartCornerExtentCm = Corners[PointIndex].ExitExtentCm;
		float& EndCornerExtentCm = Corners[PointIndex + 1].EntryExtentCm;
		const float TotalCornerExtentCm = StartCornerExtentCm + EndCornerExtentCm;
		if (TotalCornerExtentCm > SegmentLengthCm)
		{
			const float ExtentScale = SegmentLengthCm / TotalCornerExtentCm;
			StartCornerExtentCm *= ExtentScale;
			EndCornerExtentCm *= ExtentScale;
		}
	}

	OutRoute.PointsCm.Reserve(Points.Num() * 4);
	float RouteLengthCm = 0.0f;
	if (!AppendRoutePoint(Points[0].PositionCm, OutRoute, RouteLengthCm))
	{
		return Result;
	}

	TArray<float> CornerBoundaryDistancesCm;
	CornerBoundaryDistancesCm.Reserve(FMath::Max(Points.Num() - 2, 0));
	for (int32 PointIndex = 1; PointIndex + 1 < Points.Num(); ++PointIndex)
	{
		const FCornerGeometry& Corner = Corners[PointIndex];
		const FVector EntryCm = Corner.CenterCm
			- Corner.IncomingDirection * Corner.EntryExtentCm;
		const FVector ExitCm = Corner.CenterCm
			+ Corner.OutgoingDirection * Corner.ExitExtentCm;
		if (!AppendRoutePoint(EntryCm, OutRoute, RouteLengthCm)
			|| !AppendQuadraticBezierRange(EntryCm, Corner.CenterCm, ExitCm,
				0.0f, 0.5f, PathConfig.ResampleSpacingCm, OutRoute, RouteLengthCm))
		{
			Result.Status = EAircraftSafeCorridorBuildStatus::InsufficientClearance;
			Result.InputPointIndex = Points[PointIndex].InputPointIndex;
			OutRoute = {};
			return Result;
		}
		CornerBoundaryDistancesCm.Add(RouteLengthCm);
		if (!AppendQuadraticBezierRange(EntryCm, Corner.CenterCm, ExitCm,
				0.5f, 1.0f, PathConfig.ResampleSpacingCm, OutRoute, RouteLengthCm))
		{
			Result.Status = EAircraftSafeCorridorBuildStatus::InsufficientClearance;
			Result.InputPointIndex = Points[PointIndex].InputPointIndex;
			OutRoute = {};
			return Result;
		}
	}
	if (!AppendRoutePoint(Points.Last().PositionCm, OutRoute, RouteLengthCm))
	{
		OutRoute = {};
		return Result;
	}

	const int32 LegCount = Points.Num() - 1;
	if (CornerBoundaryDistancesCm.Num() != LegCount - 1)
	{
		OutRoute = {};
		return Result;
	}
	OutRoute.Corridor.Reserve(LegCount);
	for (int32 LegIndex = 0; LegIndex < LegCount; ++LegIndex)
	{
		FAircraftSafeCorridorSegment& Segment = OutRoute.Corridor.AddDefaulted_GetRef();
		Segment.AxisStartCm = Points[LegIndex].PositionCm;
		Segment.AxisEndCm = Points[LegIndex + 1].PositionCm;
		Segment.RadiusCm = Settings.OuterRadiusCm;
		Segment.StartDistanceCm = LegIndex == 0
			? 0.0f : CornerBoundaryDistancesCm[LegIndex - 1];
		Segment.EndDistanceCm = LegIndex + 1 == LegCount
			? RouteLengthCm : CornerBoundaryDistancesCm[LegIndex];
	}

	if (!IsCorridorPartitionValid(OutRoute, RouteLengthCm))
	{
		Result.Status = EAircraftSafeCorridorBuildStatus::InsufficientClearance;
		OutRoute = {};
		return Result;
	}

	Result.Status = EAircraftSafeCorridorBuildStatus::Succeeded;
	Result.RouteLengthCm = RouteLengthCm;
	return Result;
}
