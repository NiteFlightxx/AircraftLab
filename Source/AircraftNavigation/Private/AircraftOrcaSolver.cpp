#include "AircraftNavigation/AircraftOrcaSolver.h"

namespace UE::AircraftLab::Navigation::Private
{
	constexpr double PlaneToleranceCmPerSec = 0.1;
	constexpr double ParallelTolerance = 1.e-8;

	static bool IsFiniteOrcaVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	double ComputeResponsibility(
		const FAircraftAvoidanceAgentState& Self,
		const FAircraftAvoidanceAgentState& Other,
		const bool bOverlapping)
	{
		if (!bOverlapping)
		{
			if (Self.bAnchored)
			{
				return 0.0;
			}
			if (Other.bAnchored)
			{
				return 1.0;
			}
		}
		const double SelfYieldWeight = static_cast<double>(256 - Self.Priority);
		const double OtherYieldWeight = static_cast<double>(256 - Other.Priority);
		return SelfYieldWeight / FMath::Max(SelfYieldWeight + OtherYieldWeight, 1.0);
	}

	FVector BuildDeterministicNormal(
		const FAircraftAvoidanceAgentState& Self,
		const FAircraftAvoidanceAgentState& Other,
		const FVector& RelativePositionCm)
	{
		const bool bSelfFirst = Self.StableId < Other.StableId;
		FVector Canonical = bSelfFirst ? RelativePositionCm : -RelativePositionCm;
		Canonical = Canonical.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
		const FVector Reference = FMath::Abs(Canonical.Z) < 0.9
			? FVector::UpVector
			: FVector::RightVector;
		const FVector Normal = FVector::CrossProduct(Canonical, Reference)
			.GetSafeNormal(UE_SMALL_NUMBER, FVector::RightVector);
		return bSelfFirst ? Normal : -Normal;
	}

	void AddUpperBoundPlane(
		TArray<FAircraftVelocityConstraintPlane, TInlineAllocator<48>>& Planes,
		const FVector& Direction,
		const double MaximumProjection)
	{
		FAircraftVelocityConstraintPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.Normal = -Direction;
		Plane.MinimumProjectionCmPerSec = -MaximumProjection;
	}

	void AddLowerBoundPlane(
		TArray<FAircraftVelocityConstraintPlane, TInlineAllocator<48>>& Planes,
		const FVector& Direction,
		const double MinimumProjection)
	{
		FAircraftVelocityConstraintPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.Normal = Direction;
		Plane.MinimumProjectionCmPerSec = MinimumProjection;
	}

	void AddCapabilityPlanes(
		const FAircraftAvoidanceAgentState& Self,
		const FVector& PreviousCommandAccelerationCmPerSecSq,
		const FAircraftAvoidanceLimits& Limits,
		TArray<FAircraftVelocityConstraintPlane, TInlineAllocator<48>>& Planes)
	{
		const int32 PlaneCount = FMath::Max(Limits.HorizontalPlaneCount, 8);
		const double PolygonScale = FMath::Cos(UE_DOUBLE_PI / static_cast<double>(PlaneCount));
		const FVector CurrentHorizontal(Self.VelocityCmPerSec.X, Self.VelocityCmPerSec.Y, 0.0);
		const FVector CurrentDirection = CurrentHorizontal.GetSafeNormal();
		const double DeltaTime = Limits.DeltaTimeSeconds;
		for (int32 PlaneIndex = 0; PlaneIndex < PlaneCount; ++PlaneIndex)
		{
			const double Angle = UE_DOUBLE_TWO_PI * static_cast<double>(PlaneIndex)
				/ static_cast<double>(PlaneCount);
			const FVector Direction(FMath::Cos(Angle), FMath::Sin(Angle), 0.0);
			AddUpperBoundPlane(Planes, Direction,
				Limits.MaxHorizontalSpeedCmPerSec * PolygonScale);

			const double DirectionAgainstVelocity = FMath::Max(
				-FVector::DotProduct(Direction, CurrentDirection), 0.0);
			const double DirectionalAcceleration = FMath::Lerp(
				static_cast<double>(Limits.MaxHorizontalAccelerationCmPerSecSq),
				static_cast<double>(Limits.MaxHorizontalDecelerationCmPerSecSq),
				DirectionAgainstVelocity);
			AddUpperBoundPlane(Planes, Direction,
				FVector::DotProduct(Direction, Self.VelocityCmPerSec)
				+ DirectionalAcceleration * DeltaTime * PolygonScale);

			if (Limits.MaxHorizontalJerkCmPerSecCubed > 0.0f)
			{
				const FVector JerkCenterVelocity = Self.VelocityCmPerSec
					+ PreviousCommandAccelerationCmPerSecSq * DeltaTime;
				AddUpperBoundPlane(Planes, Direction,
					FVector::DotProduct(Direction, JerkCenterVelocity)
					+ Limits.MaxHorizontalJerkCmPerSecCubed * DeltaTime * DeltaTime * PolygonScale);
			}
		}

		AddUpperBoundPlane(Planes, FVector::UpVector, Limits.MaxClimbRateCmPerSec);
		AddLowerBoundPlane(Planes, FVector::UpVector, -Limits.MaxDescentRateCmPerSec);
		AddUpperBoundPlane(Planes, FVector::UpVector,
			Self.VelocityCmPerSec.Z + Limits.MaxVerticalAccelerationCmPerSecSq * DeltaTime);
		AddLowerBoundPlane(Planes, FVector::UpVector,
			Self.VelocityCmPerSec.Z - Limits.MaxVerticalAccelerationCmPerSecSq * DeltaTime);
		if (Limits.MaxVerticalJerkCmPerSecCubed > 0.0f)
		{
			const double CenterVelocityZ = Self.VelocityCmPerSec.Z
				+ PreviousCommandAccelerationCmPerSecSq.Z * DeltaTime;
			const double JerkVelocityStep = Limits.MaxVerticalJerkCmPerSecCubed
				* DeltaTime * DeltaTime;
			AddUpperBoundPlane(Planes, FVector::UpVector, CenterVelocityZ + JerkVelocityStep);
			AddLowerBoundPlane(Planes, FVector::UpVector, CenterVelocityZ - JerkVelocityStep);
		}
	}

	bool IsFeasible(
		const FVector& Candidate,
		TConstArrayView<FAircraftVelocityConstraintPlane> Planes,
		const double Tolerance = PlaneToleranceCmPerSec)
	{
		if (!IsFiniteOrcaVector(Candidate))
		{
			return false;
		}
		for (const FAircraftVelocityConstraintPlane& Plane : Planes)
		{
			if (Plane.ComputeViolation(Candidate) > Tolerance)
			{
				return false;
			}
		}
		return true;
	}

	double ComputeMaximumViolation(
		const FVector& Candidate,
		TConstArrayView<FAircraftVelocityConstraintPlane> Planes)
	{
		double MaximumViolation = 0.0;
		for (const FAircraftVelocityConstraintPlane& Plane : Planes)
		{
			MaximumViolation = FMath::Max(MaximumViolation, Plane.ComputeViolation(Candidate));
		}
		return MaximumViolation;
	}

	bool SolveOnPlane(
		const int32 PlaneIndex,
		const FVector& Objective,
		TConstArrayView<FAircraftVelocityConstraintPlane> Planes,
		FVector& OutVelocity)
	{
		const FAircraftVelocityConstraintPlane& ActivePlane = Planes[PlaneIndex];
		OutVelocity = Objective + ActivePlane.Normal * (
			ActivePlane.MinimumProjectionCmPerSec
			- FVector::DotProduct(ActivePlane.Normal, Objective));
		for (int32 ConstraintIndex = 0; ConstraintIndex < PlaneIndex; ++ConstraintIndex)
		{
			const FAircraftVelocityConstraintPlane& Constraint = Planes[ConstraintIndex];
			if (Constraint.ComputeViolation(OutVelocity) <= PlaneToleranceCmPerSec)
			{
				continue;
			}
			const FVector Direction = FVector::CrossProduct(
				ActivePlane.Normal, Constraint.Normal);
			const double DirectionSquared = Direction.SizeSquared();
			if (DirectionSquared <= ParallelTolerance)
			{
				return false;
			}
			const FVector Point = (
				ActivePlane.MinimumProjectionCmPerSec
					* FVector::CrossProduct(Constraint.Normal, Direction)
				+ Constraint.MinimumProjectionCmPerSec
					* FVector::CrossProduct(Direction, ActivePlane.Normal))
				/ DirectionSquared;
			double MinimumT = -TNumericLimits<double>::Max();
			double MaximumT = TNumericLimits<double>::Max();
			for (int32 PreviousIndex = 0; PreviousIndex < ConstraintIndex; ++PreviousIndex)
			{
				const FAircraftVelocityConstraintPlane& Previous = Planes[PreviousIndex];
				const double Coefficient = FVector::DotProduct(Previous.Normal, Direction);
				const double Required = Previous.MinimumProjectionCmPerSec
					- FVector::DotProduct(Previous.Normal, Point);
				if (FMath::Abs(Coefficient) <= ParallelTolerance)
				{
					if (Required > PlaneToleranceCmPerSec)
					{
						return false;
					}
					continue;
				}
				const double BoundaryT = Required / Coefficient;
				if (Coefficient > 0.0)
				{
					MinimumT = FMath::Max(MinimumT, BoundaryT);
				}
				else
				{
					MaximumT = FMath::Min(MaximumT, BoundaryT);
				}
				if (MinimumT > MaximumT + PlaneToleranceCmPerSec)
				{
					return false;
				}
			}
			const double ObjectiveT = FVector::DotProduct(Objective - Point, Direction)
				/ DirectionSquared;
			OutVelocity = Point + Direction * FMath::Clamp(
				ObjectiveT, MinimumT, MaximumT);
		}
		return true;
	}

	bool SolveClosestPoint(
		const FVector& Objective,
		TConstArrayView<FAircraftVelocityConstraintPlane> Planes,
		FVector& OutVelocity)
	{
		OutVelocity = Objective;
		for (int32 PlaneIndex = 0; PlaneIndex < Planes.Num(); ++PlaneIndex)
		{
			if (Planes[PlaneIndex].ComputeViolation(OutVelocity) <= PlaneToleranceCmPerSec)
			{
				continue;
			}
			if (!SolveOnPlane(PlaneIndex, Objective, Planes, OutVelocity))
			{
				return false;
			}
		}
		return IsFeasible(OutVelocity, Planes);
	}

	FVector SolveLeastViolation(
		const FVector& InitialVelocity,
		TConstArrayView<FAircraftVelocityConstraintPlane> Planes)
	{
		FVector Velocity = InitialVelocity;
		for (int32 Iteration = 0; Iteration < 16; ++Iteration)
		{
			for (const FAircraftVelocityConstraintPlane& Plane : Planes)
			{
				const double Violation = Plane.ComputeViolation(Velocity);
				if (Violation > 0.0)
				{
					Velocity += Plane.Normal * Violation;
				}
			}
		}
		return Velocity;
	}
}

bool FAircraftVelocityConstraintPlane::IsValid() const
{
	return UE::AircraftLab::Navigation::Private::IsFiniteOrcaVector(Normal)
		&& Normal.IsNormalized()
		&& FMath::IsFinite(MinimumProjectionCmPerSec);
}

double FAircraftVelocityConstraintPlane::ComputeViolation(const FVector& VelocityCmPerSec) const
{
	return FMath::Max(MinimumProjectionCmPerSec - FVector::DotProduct(Normal, VelocityCmPerSec), 0.0);
}

bool FAircraftAvoidanceAgentState::IsValid() const
{
	using namespace UE::AircraftLab::Navigation::Private;
	return StableId != 0
		&& IsFiniteOrcaVector(PositionCm)
		&& IsFiniteOrcaVector(VelocityCmPerSec)
		&& IsFiniteOrcaVector(CommandedVelocityCmPerSec)
		&& FMath::IsFinite(BodyRadiusCm) && BodyRadiusCm > 0.0f
		&& FMath::IsFinite(TrackingReserveCm) && TrackingReserveCm >= 0.0f;
}

bool FAircraftAvoidanceLimits::IsValid() const
{
	return FMath::IsFinite(DeltaTimeSeconds) && DeltaTimeSeconds > 0.0f
		&& FMath::IsFinite(TimeHorizonSeconds) && TimeHorizonSeconds >= DeltaTimeSeconds
		&& FMath::IsFinite(SeparationPaddingCm) && SeparationPaddingCm >= 0.0f
		&& FMath::IsFinite(MaxHorizontalSpeedCmPerSec) && MaxHorizontalSpeedCmPerSec > 0.0f
		&& FMath::IsFinite(MaxHorizontalAccelerationCmPerSecSq) && MaxHorizontalAccelerationCmPerSecSq > 0.0f
		&& FMath::IsFinite(MaxHorizontalDecelerationCmPerSecSq) && MaxHorizontalDecelerationCmPerSecSq > 0.0f
		&& FMath::IsFinite(MaxVerticalAccelerationCmPerSecSq) && MaxVerticalAccelerationCmPerSecSq > 0.0f
		&& FMath::IsFinite(MaxClimbRateCmPerSec) && MaxClimbRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxDescentRateCmPerSec) && MaxDescentRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxHorizontalJerkCmPerSecCubed) && MaxHorizontalJerkCmPerSecCubed >= 0.0f
		&& FMath::IsFinite(MaxVerticalJerkCmPerSecCubed) && MaxVerticalJerkCmPerSecCubed >= 0.0f
		&& FMath::IsFinite(SmoothingWeight) && SmoothingWeight >= 0.0f
		&& HorizontalPlaneCount >= 8;
}

FAircraftAvoidanceResult FAircraftOrcaSolver::Solve(
	const FAircraftAvoidanceAgentState& Self,
	const TConstArrayView<FAircraftAvoidanceAgentState> Neighbors,
	const FVector& PreferredVelocityCmPerSec,
	const FVector& PreviousCommandAccelerationCmPerSecSq,
	const FAircraftAvoidanceLimits& Limits,
	const TConstArrayView<FAircraftVelocityConstraintPlane> EnvironmentPlanes)
{
	using namespace UE::AircraftLab::Navigation::Private;
	FAircraftAvoidanceResult Result;
	if (!Self.IsValid() || !Limits.IsValid() || !IsFiniteOrcaVector(PreferredVelocityCmPerSec)
		|| !IsFiniteOrcaVector(PreviousCommandAccelerationCmPerSecSq))
	{
		return Result;
	}

	TArray<FAircraftVelocityConstraintPlane, TInlineAllocator<48>> Planes;
	Planes.Reserve(Neighbors.Num() + EnvironmentPlanes.Num() + Limits.HorizontalPlaneCount * 3 + 8);
	for (const FAircraftVelocityConstraintPlane& Plane : EnvironmentPlanes)
	{
		if (Plane.IsValid())
		{
			Planes.Add(Plane);
		}
	}

	for (const FAircraftAvoidanceAgentState& Other : Neighbors)
	{
		if (!Other.IsValid() || Other.StableId == Self.StableId)
		{
			continue;
		}
		const FVector RelativePositionCm = Other.PositionCm - Self.PositionCm;
		const FVector RelativeVelocityCmPerSec = Self.VelocityCmPerSec - Other.VelocityCmPerSec;
		const double DistanceSquaredCm = RelativePositionCm.SizeSquared();
		const double CombinedRadiusCm = Self.BodyRadiusCm + Other.BodyRadiusCm
			+ Limits.SeparationPaddingCm + Self.TrackingReserveCm + Other.TrackingReserveCm;
		const double CombinedRadiusSquaredCm = FMath::Square(CombinedRadiusCm);

		const FVector ClosingVelocityCmPerSec = Other.VelocityCmPerSec - Self.VelocityCmPerSec;
		const double ClosingSpeedSquared = ClosingVelocityCmPerSec.SizeSquared();
		const double ClosestTimeSeconds = ClosingSpeedSquared > UE_DOUBLE_SMALL_NUMBER
			? FMath::Clamp(-FVector::DotProduct(RelativePositionCm, ClosingVelocityCmPerSec)
				/ ClosingSpeedSquared, 0.0, static_cast<double>(Limits.TimeHorizonSeconds))
			: 0.0;
		const double PredictedSeparationCm = (
			RelativePositionCm + ClosingVelocityCmPerSec * ClosestTimeSeconds).Size();
		if (PredictedSeparationCm < Result.MinimumPredictedSeparationCm)
		{
			Result.MinimumPredictedSeparationCm = static_cast<float>(PredictedSeparationCm);
			Result.MostDangerousAgentId = Other.StableId;
		}
		if (PredictedSeparationCm >= CombinedRadiusCm && DistanceSquaredCm > CombinedRadiusSquaredCm)
		{
			continue;
		}

		Result.bAvoidanceRequired = true;
		Result.EarliestConflictTimeSeconds = FMath::Min(
			Result.EarliestConflictTimeSeconds, static_cast<float>(ClosestTimeSeconds));
		const bool bOverlapping = DistanceSquaredCm <= CombinedRadiusSquaredCm;
		const double InverseTime = bOverlapping
			? 1.0 / static_cast<double>(Limits.DeltaTimeSeconds)
			: 1.0 / static_cast<double>(Limits.TimeHorizonSeconds);
		const FVector W = RelativeVelocityCmPerSec - RelativePositionCm * InverseTime;
		const double WLengthSquared = W.SizeSquared();
		const double DotProduct = FVector::DotProduct(W, RelativePositionCm);
		FVector UnitW;
		FVector Correction;

		if (bOverlapping || (DotProduct < 0.0
			&& FMath::Square(DotProduct) > CombinedRadiusSquaredCm * WLengthSquared))
		{
			const double WLength = FMath::Sqrt(FMath::Max(WLengthSquared, 0.0));
			UnitW = WLength > UE_DOUBLE_SMALL_NUMBER
				? W / WLength
				: BuildDeterministicNormal(Self, Other, RelativePositionCm);
			Correction = (CombinedRadiusCm * InverseTime - WLength) * UnitW;
		}
		else
		{
			const double A = DistanceSquaredCm;
			const double B = FVector::DotProduct(RelativePositionCm, RelativeVelocityCmPerSec);
			const FVector Cross = FVector::CrossProduct(RelativePositionCm, RelativeVelocityCmPerSec);
			const double C = RelativeVelocityCmPerSec.SizeSquared()
				- Cross.SizeSquared() / FMath::Max(DistanceSquaredCm - CombinedRadiusSquaredCm, UE_DOUBLE_SMALL_NUMBER);
			const double Discriminant = FMath::Max(B * B - A * C, 0.0);
			const double T = (B + FMath::Sqrt(Discriminant)) / FMath::Max(A, UE_DOUBLE_SMALL_NUMBER);
			const FVector ConeW = RelativeVelocityCmPerSec - T * RelativePositionCm;
			const double ConeWLength = ConeW.Size();
			UnitW = ConeWLength > UE_DOUBLE_SMALL_NUMBER
				? ConeW / ConeWLength
				: BuildDeterministicNormal(Self, Other, RelativePositionCm);
			Correction = (CombinedRadiusCm * T - ConeWLength) * UnitW;
		}

		FAircraftVelocityConstraintPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.Normal = UnitW.GetSafeNormal(UE_SMALL_NUMBER,
			BuildDeterministicNormal(Self, Other, RelativePositionCm));
		const double Responsibility = ComputeResponsibility(Self, Other, bOverlapping);
		const FVector PlanePoint = Self.VelocityCmPerSec + Correction * Responsibility;
		Plane.MinimumProjectionCmPerSec = FVector::DotProduct(Plane.Normal, PlanePoint);
	}

	AddCapabilityPlanes(Self, PreviousCommandAccelerationCmPerSecSq, Limits, Planes);
	const double SmoothingAlpha = Limits.SmoothingWeight / (1.0 + Limits.SmoothingWeight);
	const FVector Objective = FMath::Lerp(PreferredVelocityCmPerSec,
		Self.CommandedVelocityCmPerSec, SmoothingAlpha);
	Result.ConstraintPlaneCount = Planes.Num();
	Result.bFeasible = SolveClosestPoint(Objective, Planes, Result.TargetVelocityCmPerSec);
	if (!Result.bFeasible)
	{
		Result.TargetVelocityCmPerSec = SolveLeastViolation(FVector::ZeroVector, Planes);
	}
	Result.MaximumConstraintViolation = static_cast<float>(
		ComputeMaximumViolation(Result.TargetVelocityCmPerSec, Planes));
	return Result;
}
