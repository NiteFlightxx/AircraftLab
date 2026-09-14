#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

namespace
{
	bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	void AccumulateQuadraticRoots(
		const double A, const double B, const double C, TArray<double, TInlineAllocator<4>>& OutRoots)
	{
		if (A <= UE_DOUBLE_SMALL_NUMBER)
		{
			return;
		}
		const double Discriminant = B * B - 4.0 * A * C;
		if (Discriminant < 0.0)
		{
			return;
		}
		const double SquareRoot = FMath::Sqrt(FMath::Max(Discriminant, 0.0));
		OutRoots.Add((-B - SquareRoot) / (2.0 * A));
		if (SquareRoot > UE_DOUBLE_SMALL_NUMBER)
		{
			OutRoots.Add((-B + SquareRoot) / (2.0 * A));
		}
	}
}

bool FAircraftSafeCorridorSegment::IsGeometryValid() const
{
	return IsFiniteVector(AxisStartCm)
		&& IsFiniteVector(AxisEndCm)
		&& FMath::IsFinite(RadiusCm)
		&& RadiusCm > UE_SMALL_NUMBER
		&& FVector::DistSquared(AxisStartCm, AxisEndCm) > FMath::Square(UE_SMALL_NUMBER);
}

FVector FAircraftSafeCorridorSegment::GetClosestAxisPoint(const FVector& PositionCm) const
{
	const FVector Axis = AxisEndCm - AxisStartCm;
	const double AxisLengthSquared = Axis.SizeSquared();
	if (AxisLengthSquared <= UE_DOUBLE_SMALL_NUMBER)
	{
		return AxisStartCm;
	}
	const double Parameter = FMath::Clamp(
		FVector::DotProduct(PositionCm - AxisStartCm, Axis) / AxisLengthSquared, 0.0, 1.0);
	return AxisStartCm + Parameter * Axis;
}

FVector FAircraftSafeCorridorSegment::ComputeCorrectionCm(
	const FVector& PositionCm, const float SafetyMarginCm) const
{
	const double EffectiveRadiusCm = static_cast<double>(RadiusCm - SafetyMarginCm);
	if (!IsGeometryValid()
		|| !IsFiniteVector(PositionCm)
		|| !FMath::IsFinite(SafetyMarginCm)
		|| EffectiveRadiusCm <= UE_DOUBLE_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	const FVector ClosestAxisPoint = GetClosestAxisPoint(PositionCm);
	const FVector Offset = PositionCm - ClosestAxisPoint;
	const double DistanceSquared = Offset.SizeSquared();
	if (DistanceSquared <= FMath::Square(EffectiveRadiusCm))
	{
		return FVector::ZeroVector;
	}
	const double DistanceCm = FMath::Sqrt(DistanceSquared);
	return -Offset * ((DistanceCm - EffectiveRadiusCm) / DistanceCm);
}

bool FAircraftSafeCorridorSegment::ComputeRayExitParameter(
	const FVector& StartCm, const FVector& DirectionCm,
	const float SafetyMarginCm, float& OutExitParameter) const
{
	OutExitParameter = 0.0f;
	const double EffectiveRadiusCm = static_cast<double>(RadiusCm - SafetyMarginCm);
	if (!IsGeometryValid()
		|| !IsFiniteVector(StartCm)
		|| !IsFiniteVector(DirectionCm)
		|| !FMath::IsFinite(SafetyMarginCm)
		|| EffectiveRadiusCm <= UE_DOUBLE_SMALL_NUMBER)
	{
		return false;
	}

	const FVector ClosestAxisPoint = GetClosestAxisPoint(StartCm);
	if (FVector::DistSquared(StartCm, ClosestAxisPoint)
		> FMath::Square(EffectiveRadiusCm) + UE_KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const double DirectionLengthSquared = DirectionCm.SizeSquared();
	if (DirectionLengthSquared <= UE_DOUBLE_SMALL_NUMBER)
	{
		OutExitParameter = TNumericLimits<float>::Max();
		return true;
	}

	const FVector AxisDelta = AxisEndCm - AxisStartCm;
	const double AxisLengthCm = AxisDelta.Size();
	const FVector AxisDirection = AxisDelta / AxisLengthCm;
	const FVector RelativeStart = StartCm - AxisStartCm;
	const double StartAxialCm = FVector::DotProduct(RelativeStart, AxisDirection);
	const double DirectionAxialCm = FVector::DotProduct(DirectionCm, AxisDirection);
	const FVector StartRadial = RelativeStart - StartAxialCm * AxisDirection;
	const FVector DirectionRadial = DirectionCm - DirectionAxialCm * AxisDirection;

	TArray<double, TInlineAllocator<4>> Roots;
	AccumulateQuadraticRoots(
		DirectionRadial.SizeSquared(),
		2.0 * FVector::DotProduct(StartRadial, DirectionRadial),
		StartRadial.SizeSquared() - FMath::Square(EffectiveRadiusCm), Roots);

	const FVector StartCapOffset = StartCm - AxisStartCm;
	AccumulateQuadraticRoots(
		DirectionLengthSquared,
		2.0 * FVector::DotProduct(StartCapOffset, DirectionCm),
		StartCapOffset.SizeSquared() - FMath::Square(EffectiveRadiusCm), Roots);
	const FVector EndCapOffset = StartCm - AxisEndCm;
	AccumulateQuadraticRoots(
		DirectionLengthSquared,
		2.0 * FVector::DotProduct(EndCapOffset, DirectionCm),
		EndCapOffset.SizeSquared() - FMath::Square(EffectiveRadiusCm), Roots);

	bool bFoundExit = false;
	double ExitParameter = 0.0;
	for (const double Root : Roots)
	{
		if (!FMath::IsFinite(Root) || Root < -UE_DOUBLE_SMALL_NUMBER)
		{
			continue;
		}
		const double Candidate = FMath::Max(Root, 0.0);
		const FVector Point = StartCm + Candidate * DirectionCm;
		const double AxialCm = FVector::DotProduct(Point - AxisStartCm, AxisDirection);
		const double RadialDistanceSquared = FVector::DistSquared(
			Point, AxisStartCm + FMath::Clamp(AxialCm, 0.0, AxisLengthCm) * AxisDirection);
		if (RadialDistanceSquared > FMath::Square(EffectiveRadiusCm)
			+ UE_KINDA_SMALL_NUMBER)
		{
			continue;
		}
		ExitParameter = FMath::Max(ExitParameter, Candidate);
		bFoundExit = true;
	}

	if (!bFoundExit || ExitParameter > TNumericLimits<float>::Max())
	{
		return false;
	}
	OutExitParameter = static_cast<float>(ExitParameter);
	return true;
}

int32 ResolveAircraftSafeCorridorSegment(
	const TConstArrayView<FAircraftSafeCorridorSegment> Corridor,
	const float RouteDistanceCm,
	const float RouteLengthCm)
{
	if (Corridor.IsEmpty()
		|| !FMath::IsFinite(RouteDistanceCm)
		|| !FMath::IsFinite(RouteLengthCm)
		|| RouteLengthCm <= UE_SMALL_NUMBER
		|| RouteDistanceCm < -UE_KINDA_SMALL_NUMBER
		|| RouteDistanceCm > RouteLengthCm + UE_KINDA_SMALL_NUMBER)
	{
		return INDEX_NONE;
	}

	const float DistanceCm = FMath::Clamp(RouteDistanceCm, 0.0f, RouteLengthCm);
	for (int32 CorridorIndex = 0; CorridorIndex < Corridor.Num(); ++CorridorIndex)
	{
		const FAircraftSafeCorridorSegment& Segment = Corridor[CorridorIndex];
		const bool bLastSegment = CorridorIndex == Corridor.Num() - 1;
		if (DistanceCm >= Segment.StartDistanceCm
			&& (DistanceCm < Segment.EndDistanceCm
				|| (bLastSegment && DistanceCm <= Segment.EndDistanceCm)))
		{
			return CorridorIndex;
		}
	}
	return INDEX_NONE;
}

bool FAircraftRequestedMotionLimits::IsValid() const
{
	return FMath::IsFinite(CruiseSpeedCmPerSec) && CruiseSpeedCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxAccelerationCmPerSecSq) && MaxAccelerationCmPerSecSq >= 0.0f
		&& FMath::IsFinite(MaxDecelerationCmPerSecSq) && MaxDecelerationCmPerSecSq >= 0.0f
		&& FMath::IsFinite(MaxJerkCmPerSecCubed) && MaxJerkCmPerSecCubed >= 0.0f
		&& FMath::IsFinite(MaxClimbRateCmPerSec) && MaxClimbRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxDescentRateCmPerSec) && MaxDescentRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxVerticalAccelerationCmPerSecSq) && MaxVerticalAccelerationCmPerSecSq >= 0.0f
		&& FMath::IsFinite(MaxVerticalJerkCmPerSecCubed) && MaxVerticalJerkCmPerSecCubed >= 0.0f
		&& FMath::IsFinite(MaxYawRateDegPerSec) && MaxYawRateDegPerSec >= 0.0f
		&& FMath::IsFinite(MaxYawAccelerationDegPerSecSq) && MaxYawAccelerationDegPerSecSq >= 0.0f
		&& FMath::IsFinite(MaxYawJerkDegPerSecCubed) && MaxYawJerkDegPerSecCubed >= 0.0f;
}

bool FAircraftMovementIntent::IsValid() const
{
	if ((bHasRequestedMotionLimits && !Limits.IsValid())
		|| !FMath::IsFinite(TimeoutSeconds) || TimeoutSeconds < 0.0f)
	{
		return false;
	}
	if (!FMath::IsFinite(Heading.FixedYawDegrees)
		|| !FMath::IsFinite(Heading.YawRateDegPerSec)
		|| !IsFiniteVector(Heading.TargetPositionCm)
		|| !FMath::IsFinite(Completion.TerminalHorizontalSpeedCmPerSec)
		|| !FMath::IsFinite(Completion.TerminalVerticalSpeedCmPerSec)
		|| !FMath::IsFinite(Completion.HorizontalToleranceCm)
		|| !FMath::IsFinite(Completion.VerticalToleranceCm)
		|| !FMath::IsFinite(Completion.HorizontalSpeedToleranceCmPerSec)
		|| !FMath::IsFinite(Completion.VerticalSpeedToleranceCmPerSec)
		|| !FMath::IsFinite(Completion.YawToleranceDegrees)
		|| !FMath::IsFinite(Completion.StableTimeSeconds)
		|| Completion.TerminalHorizontalSpeedCmPerSec < 0.0f
		|| Completion.TerminalVerticalSpeedCmPerSec < 0.0f
		|| Completion.HorizontalToleranceCm < 0.0f
		|| Completion.VerticalToleranceCm < 0.0f
		|| Completion.HorizontalSpeedToleranceCmPerSec < 0.0f
		|| Completion.VerticalSpeedToleranceCmPerSec < 0.0f
		|| Completion.YawToleranceDegrees < 0.0f
		|| Completion.YawToleranceDegrees > 180.0f
		|| Completion.StableTimeSeconds < 0.0f)
	{
		return false;
	}

	switch (Type)
	{
	case EAircraftMovementIntentType::Hold:
		return Hold.bCaptureCurrentPosition || Hold.TargetActor || IsFiniteVector(Hold.PositionCm);
	case EAircraftMovementIntentType::Velocity:
		return IsFiniteVector(Velocity.VelocityCmPerSec);
	case EAircraftMovementIntentType::Route:
	{
		if (Route.PointsCm.Num() < (Route.bClosed ? 3 : 2))
		{
			return false;
		}
		float RouteLengthCm = 0.0f;
		for (const FVector& Point : Route.PointsCm)
		{
			if (!IsFiniteVector(Point))
			{
				return false;
			}
		}
		for (int32 PointIndex = 1; PointIndex < Route.PointsCm.Num(); ++PointIndex)
		{
			RouteLengthCm += static_cast<float>(FVector::Distance(
				Route.PointsCm[PointIndex - 1], Route.PointsCm[PointIndex]));
		}
		if (Route.bClosed)
		{
			RouteLengthCm += static_cast<float>(FVector::Distance(
				Route.PointsCm.Last(), Route.PointsCm[0]));
		}
		for (int32 CorridorIndex = 0; CorridorIndex < Route.Corridor.Num(); ++CorridorIndex)
		{
			const FAircraftSafeCorridorSegment& Segment = Route.Corridor[CorridorIndex];
			const float ExpectedStartDistanceCm = CorridorIndex == 0
				? 0.0f : Route.Corridor[CorridorIndex - 1].EndDistanceCm;
			if (!FMath::IsFinite(Segment.StartDistanceCm)
				|| !FMath::IsFinite(Segment.EndDistanceCm)
				|| Segment.StartDistanceCm < 0.0f
				|| Segment.EndDistanceCm <= Segment.StartDistanceCm
				|| !Segment.IsGeometryValid()
				|| !FMath::IsNearlyEqual(Segment.StartDistanceCm,
					ExpectedStartDistanceCm, UE_KINDA_SMALL_NUMBER))
			{
				return false;
			}
		}
		if (!Route.Corridor.IsEmpty()
			&& !FMath::IsNearlyEqual(Route.Corridor.Last().EndDistanceCm,
				RouteLengthCm, UE_KINDA_SMALL_NUMBER))
		{
			return false;
		}
		return true;
	}
	case EAircraftMovementIntentType::Orbit:
		return IsFiniteVector(Orbit.CenterCm)
			&& FMath::IsFinite(Orbit.RadiusCm) && Orbit.RadiusCm > UE_SMALL_NUMBER
			&& FMath::IsFinite(Orbit.AngularRateDegPerSec)
			&& FMath::Abs(Orbit.AngularRateDegPerSec) > UE_SMALL_NUMBER;
	case EAircraftMovementIntentType::TimedTrajectory:
		if (TimedTrajectory.Samples.Num() < 2)
		{
			return false;
		}
		for (int32 Index = 0; Index < TimedTrajectory.Samples.Num(); ++Index)
		{
			const FAircraftTimedTrajectorySample& Sample = TimedTrajectory.Samples[Index];
			if (!FMath::IsFinite(Sample.TimeSeconds) || Sample.TimeSeconds < 0.0f
				|| !IsFiniteVector(Sample.PositionCm)
				|| !IsFiniteVector(Sample.VelocityCmPerSec)
				|| !IsFiniteVector(Sample.AccelerationCmPerSecSq)
				|| (Index > 0 && Sample.TimeSeconds <= TimedTrajectory.Samples[Index - 1].TimeSeconds))
			{
				return false;
			}
		}
		return true;
	default:
		return false;
	}
}
