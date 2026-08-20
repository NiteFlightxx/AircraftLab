#include "AircraftRuntimeCommon/Autopilot/AircraftPathCompiler.h"

namespace
{
	void AppendDistinct(TArray<FVector>& Points, const FVector& Point)
	{
		if (Points.IsEmpty() || !Points.Last().Equals(Point, 1.0f))
		{
			Points.Add(Point);
		}
	}

	FVector ComputeTerminalTangent(const FAircraftPathDefinition& Path)
	{
		if (Path.Geometry == EAircraftPathGeometry::Circle)
		{
			const float EndAngle = FMath::DegreesToRadians(
				Path.CircleStartAngleDegrees + Path.CircleSweepAngleDegrees);
			const float Direction = FMath::Sign(Path.CircleSweepAngleDegrees);
			return FVector(-FMath::Sin(EndAngle) * Direction,
				FMath::Cos(EndAngle) * Direction, 0.0f);
		}
		if (Path.PointsCm.Num() >= 2)
		{
			return (Path.PointsCm.Last() - Path.PointsCm[Path.PointsCm.Num() - 2]).GetSafeNormal();
		}
		return FVector::ZeroVector;
	}
}

bool FAircraftPathCompiler::Compile(
	const FAutopilotMovementIntent& Intent,
	const FAircraftPathCompileContext& Context,
	FAircraftTrajectoryPlan& OutPlan,
	FString& OutError)
{
	OutPlan = FAircraftTrajectoryPlan();
	OutPlan.InitialVelocityCmPerSec = Context.VelocityCmPerSec;
	OutPlan.InitialAccelerationCmPerSecSq = Context.AccelerationCmPerSecSq;
	OutPlan.MotionConstraints = Intent.MotionConstraints;
	OutPlan.PhysicalMaxHorizontalSpeedCmPerSec = Context.PhysicalMaxHorizontalSpeedCmPerSec;
	OutPlan.PhysicalMaxHorizontalAccelerationCmPerSecSq = Context.PhysicalMaxHorizontalAccelerationCmPerSecSq;
	OutPlan.AcceptanceRadiusCm = FMath::Max(FMath::Min(
		Intent.ArrivalCriteria.HorizontalToleranceCm,
		Intent.ArrivalCriteria.VerticalToleranceCm), 1.0f);

	switch (Intent.Type)
	{
	case EAutopilotMovementIntentType::MoveToPosition:
		OutPlan.Path.Geometry = EAircraftPathGeometry::Polyline;
		AppendDistinct(OutPlan.Path.PointsCm, Context.PositionCm);
		AppendDistinct(OutPlan.Path.PointsCm, Context.ResolvedTargetCm);
		break;

	case EAutopilotMovementIntentType::FollowPath:
		switch (Intent.PathTrajectoryMode)
		{
		case EAutopilotPathTrajectoryMode::MinimumSnap:
			OutPlan.Path.Geometry = EAircraftPathGeometry::MinimumSnap;
			break;
		case EAutopilotPathTrajectoryMode::Bezier:
			OutPlan.Path.Geometry = EAircraftPathGeometry::Bezier;
			break;
		case EAutopilotPathTrajectoryMode::PiecewiseLinear:
		default:
			OutPlan.Path.Geometry = EAircraftPathGeometry::Polyline;
			break;
		}
		AppendDistinct(OutPlan.Path.PointsCm, Context.PositionCm);
		for (const FVector& Point : Intent.PathPointsCm)
		{
			AppendDistinct(OutPlan.Path.PointsCm, Point);
		}
		break;

	case EAutopilotMovementIntentType::CircleArc:
		OutPlan.Path.Geometry = EAircraftPathGeometry::Circle;
		OutPlan.Path.CircleCenterCm = Context.ResolvedTargetCm;
		OutPlan.Path.CircleRadiusCm = Intent.OrbitRadiusCm;
		OutPlan.Path.CircleStartAngleDegrees = Intent.ArcStartAngleDegrees;
		OutPlan.Path.CircleSweepAngleDegrees = Intent.ArcEndAngleDegrees
			- Intent.ArcStartAngleDegrees;
		break;

	case EAutopilotMovementIntentType::Orbit:
		{
			OutPlan.Path.Geometry = EAircraftPathGeometry::Circle;
			OutPlan.Traversal = EAircraftPathTraversal::Loop;
			OutPlan.Path.CircleCenterCm = Context.ResolvedTargetCm;
			OutPlan.Path.CircleRadiusCm = Intent.OrbitRadiusCm;
			const FVector Radial = Context.PositionCm - Context.ResolvedTargetCm;
			OutPlan.Path.CircleStartAngleDegrees = FMath::RadiansToDegrees(
				FMath::Atan2(Radial.Y, Radial.X));
			OutPlan.Path.CircleSweepAngleDegrees = FMath::Sign(Intent.OrbitAngularRateDegPerSec) * 360.0f;
		}
		break;

	default:
		OutError = TEXT("Movement intent does not describe a path.");
		return false;
	}

	const bool bPointPath = OutPlan.Path.Geometry != EAircraftPathGeometry::Circle;
	if (bPointPath && OutPlan.Path.PointsCm.Num() < 2
		&& Intent.Type != EAutopilotMovementIntentType::MoveToPosition)
	{
		OutError = TEXT("Path requires at least two distinct points.");
		return false;
	}
	if (OutPlan.Path.Geometry == EAircraftPathGeometry::Circle
		&& (OutPlan.Path.CircleRadiusCm <= UE_SMALL_NUMBER
			|| FMath::IsNearlyZero(OutPlan.Path.CircleSweepAngleDegrees)))
	{
		OutError = TEXT("Circle path requires a positive radius and non-zero sweep.");
		return false;
	}

	if (OutPlan.Traversal == EAircraftPathTraversal::Once
		&& Intent.ArrivalMode == EAutopilotArrivalMode::PassThrough)
	{
		OutPlan.TerminalVelocityCmPerSec = ComputeTerminalTangent(OutPlan.Path)
			* FMath::Min(Intent.PassThroughSpeedCmPerSec,
				Intent.MotionConstraints.CruiseSpeedCmPerSec);
	}
	return true;
}
