#include "AircraftNavigation/AircraftOrcaSolver.h"

namespace UE::AircraftLab::Navigation::Private
{
	constexpr double PlaneToleranceCmPerSec = 0.1;
	constexpr double ParallelTolerance = 1.e-8;
	constexpr int32 ConstraintProjectionIterations = 32;

	struct FVelocityConstraintPlane
	{
		FVector Normal = FVector::ZeroVector;
		double MinimumProjectionCmPerSec = 0.0;

		double ComputeViolation(const FVector& VelocityCmPerSec) const
		{
			return FMath::Max(MinimumProjectionCmPerSec
				- FVector::DotProduct(Normal, VelocityCmPerSec), 0.0);
		}
	};

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
		TArray<FVelocityConstraintPlane, TInlineAllocator<72>>& Planes,
		const FVector& Direction,
		const double MaximumProjection)
	{
		FVelocityConstraintPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.Normal = -Direction;
		Plane.MinimumProjectionCmPerSec = -MaximumProjection;
	}

	void AddLowerBoundPlane(
		TArray<FVelocityConstraintPlane, TInlineAllocator<72>>& Planes,
		const FVector& Direction,
		const double MinimumProjection)
	{
		FVelocityConstraintPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.Normal = Direction;
		Plane.MinimumProjectionCmPerSec = MinimumProjection;
	}

	void AddCapabilityPlanes(
		const FAircraftAvoidanceAgentState& Self,
		const FVector& PreviousCommandAccelerationCmPerSecSq,
		const FAircraftAvoidanceLimits& Limits,
		TArray<FVelocityConstraintPlane, TInlineAllocator<72>>& Planes)
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
		TConstArrayView<FVelocityConstraintPlane> Planes,
		const double Tolerance = PlaneToleranceCmPerSec)
	{
		if (!IsFiniteOrcaVector(Candidate))
		{
			return false;
		}
		for (const FVelocityConstraintPlane& Plane : Planes)
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
		TConstArrayView<FVelocityConstraintPlane> Planes)
	{
		double MaximumViolation = 0.0;
		for (const FVelocityConstraintPlane& Plane : Planes)
		{
			MaximumViolation = FMath::Max(MaximumViolation, Plane.ComputeViolation(Candidate));
		}
		return MaximumViolation;
	}

	bool SolveOnPlane(
		const int32 PlaneIndex,
		const FVector& Objective,
		TConstArrayView<FVelocityConstraintPlane> Planes,
		FVector& OutVelocity)
	{
		const FVelocityConstraintPlane& ActivePlane = Planes[PlaneIndex];
		OutVelocity = Objective + ActivePlane.Normal * (
			ActivePlane.MinimumProjectionCmPerSec
			- FVector::DotProduct(ActivePlane.Normal, Objective));
		for (int32 ConstraintIndex = 0; ConstraintIndex < PlaneIndex; ++ConstraintIndex)
		{
			const FVelocityConstraintPlane& Constraint = Planes[ConstraintIndex];
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
				const FVelocityConstraintPlane& Previous = Planes[PreviousIndex];
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
		TConstArrayView<FVelocityConstraintPlane> Planes,
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
		TConstArrayView<FVelocityConstraintPlane> Planes)
	{
		FVector Velocity = InitialVelocity;
		for (int32 Iteration = 0; Iteration < 16; ++Iteration)
		{
			for (const FVelocityConstraintPlane& Plane : Planes)
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

	FVector ProjectVelocityIntoCapsule(
		const FVector& PositionCm,
		const FVector& VelocityCmPerSec,
		const FAircraftVelocityConstraintCapsule& Capsule)
	{
		const FVector PredictedPositionCm = PositionCm
			+ VelocityCmPerSec * Capsule.PredictionTimeSeconds;
		const FVector Axis = Capsule.AxisEndCm - Capsule.AxisStartCm;
		const double AxisLengthSquared = Axis.SizeSquared();
		const double AxisAlpha = AxisLengthSquared > UE_DOUBLE_SMALL_NUMBER
			? FMath::Clamp(FVector::DotProduct(
				PredictedPositionCm - Capsule.AxisStartCm, Axis) / AxisLengthSquared, 0.0, 1.0)
			: 0.0;
		const FVector AxisPointCm = Capsule.AxisStartCm + Axis * AxisAlpha;
		const FVector OffsetCm = PredictedPositionCm - AxisPointCm;
		const double DistanceCm = OffsetCm.Size();
		if (DistanceCm <= Capsule.RadiusCm)
		{
			return VelocityCmPerSec;
		}
		const FVector BoundaryPositionCm = AxisPointCm
			+ OffsetCm.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector) * Capsule.RadiusCm;
		return (BoundaryPositionCm - PositionCm) / Capsule.PredictionTimeSeconds;
	}

	bool SolveClosestPointWithCapsule(
		const FVector& Objective,
		const FVector& PositionCm,
		TConstArrayView<FVelocityConstraintPlane> Planes,
		const FAircraftVelocityConstraintCapsule* Capsule,
		FVector& OutVelocity)
	{
		if (!SolveClosestPoint(Objective, Planes, OutVelocity))
		{
			return false;
		}
		if (!Capsule)
		{
			return true;
		}
		for (int32 Iteration = 0; Iteration < ConstraintProjectionIterations; ++Iteration)
		{
			const FVector PreviousVelocity = OutVelocity;
			OutVelocity = ProjectVelocityIntoCapsule(PositionCm, OutVelocity, *Capsule);
			if (!IsFeasible(OutVelocity, Planes))
			{
				const FVector PlaneObjective = OutVelocity;
				if (!SolveClosestPoint(PlaneObjective, Planes, OutVelocity))
				{
					return false;
				}
			}
			if (OutVelocity.Equals(PreviousVelocity, PlaneToleranceCmPerSec))
			{
				break;
			}
		}
		if (!IsFeasible(OutVelocity, Planes))
		{
			return false;
		}
		return Capsule->ComputeViolationCmPerSec(PositionCm, OutVelocity)
			<= PlaneToleranceCmPerSec;
	}
}

bool FAircraftVelocityConstraintCapsule::IsValid() const
{
	return UE::AircraftLab::Navigation::Private::IsFiniteOrcaVector(AxisStartCm)
		&& UE::AircraftLab::Navigation::Private::IsFiniteOrcaVector(AxisEndCm)
		&& FMath::IsFinite(RadiusCm) && RadiusCm > 0.0f
		&& FMath::IsFinite(PredictionTimeSeconds) && PredictionTimeSeconds > 0.0f;
}

double FAircraftVelocityConstraintCapsule::ComputeViolationCmPerSec(
	const FVector& PositionCm,
	const FVector& VelocityCmPerSec) const
{
	if (!IsValid()
		|| !UE::AircraftLab::Navigation::Private::IsFiniteOrcaVector(PositionCm)
		|| !UE::AircraftLab::Navigation::Private::IsFiniteOrcaVector(VelocityCmPerSec))
	{
		return TNumericLimits<double>::Max();
	}
	const FVector PredictedPositionCm = PositionCm + VelocityCmPerSec * PredictionTimeSeconds;
	const FVector Axis = AxisEndCm - AxisStartCm;
	const double AxisLengthSquared = Axis.SizeSquared();
	const double AxisAlpha = AxisLengthSquared > UE_DOUBLE_SMALL_NUMBER
		? FMath::Clamp(FVector::DotProduct(PredictedPositionCm - AxisStartCm, Axis)
			/ AxisLengthSquared, 0.0, 1.0)
		: 0.0;
	const double DistanceCm = FVector::Distance(
		PredictedPositionCm, AxisStartCm + Axis * AxisAlpha);
	return FMath::Max(DistanceCm - RadiusCm, 0.0) / PredictionTimeSeconds;
}

bool FAircraftAvoidanceAgentState::IsValid() const
{
	using namespace UE::AircraftLab::Navigation::Private;
	return StableId != 0
		&& IsFiniteOrcaVector(PositionCm)
		&& IsFiniteOrcaVector(VelocityCmPerSec)
		&& IsFiniteOrcaVector(CommandedVelocityCmPerSec)
		&& FMath::IsFinite(SampleTimeSeconds)
		&& FMath::IsFinite(MaxHorizontalSpeedCmPerSec) && MaxHorizontalSpeedCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxClimbRateCmPerSec) && MaxClimbRateCmPerSec >= 0.0f
		&& FMath::IsFinite(MaxDescentRateCmPerSec) && MaxDescentRateCmPerSec >= 0.0f
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
	const TConstArrayView<FAircraftVelocityConstraintCapsule> CorridorAlternatives)
{
	using namespace UE::AircraftLab::Navigation::Private;
	FAircraftAvoidanceResult Result;
	if (!Self.IsValid() || !Limits.IsValid() || !IsFiniteOrcaVector(PreferredVelocityCmPerSec)
		|| !IsFiniteOrcaVector(PreviousCommandAccelerationCmPerSecSq))
	{
		return Result;
	}

	double PreferredCorridorViolationCmPerSec = CorridorAlternatives.IsEmpty()
		? 0.0 : TNumericLimits<double>::Max();
	for (const FAircraftVelocityConstraintCapsule& Capsule : CorridorAlternatives)
	{
		if (!Capsule.IsValid())
		{
			return Result;
		}
		PreferredCorridorViolationCmPerSec = FMath::Min(
			PreferredCorridorViolationCmPerSec,
			Capsule.ComputeViolationCmPerSec(Self.PositionCm, PreferredVelocityCmPerSec));
	}
	if (PreferredCorridorViolationCmPerSec > PlaneToleranceCmPerSec)
	{
		Result.bAvoidanceRequired = true;
	}

	// 邻居按当前重叠、有限视界 CPA 净间距、TCPA、StableId 排序后构造平面：
	// 贪心投影求解器按插入顺序满足约束，最紧迫的邻居必须最先获得满足。
	TArray<const FAircraftAvoidanceAgentState*, TInlineAllocator<32>> SortedNeighbors;
	SortedNeighbors.Reserve(Neighbors.Num());
	for (const FAircraftAvoidanceAgentState& Other : Neighbors)
	{
		if (Other.IsValid() && Other.StableId != Self.StableId)
		{
			SortedNeighbors.Add(&Other);
		}
	}
	SortedNeighbors.Sort([Self, Limits](const FAircraftAvoidanceAgentState& A,
		const FAircraftAvoidanceAgentState& B)
	{
		struct FRisk
		{
			double NetSeparationCm = TNumericLimits<double>::Max();
			double TimeToClosestApproachSeconds = TNumericLimits<double>::Max();
			bool bOverlapping = false;
		};
		const auto ComputeRisk = [Self, Limits](const FAircraftAvoidanceAgentState& Other)
		{
			FRisk Risk;
			const FVector Relative = Other.PositionCm - Self.PositionCm;
			const FVector RelativeVelocity = Other.VelocityCmPerSec - Self.VelocityCmPerSec;
			const double CombinedRadiusCm = Self.BodyRadiusCm + Other.BodyRadiusCm
				+ Limits.SeparationPaddingCm + Self.TrackingReserveCm + Other.TrackingReserveCm;
			Risk.bOverlapping = Relative.SizeSquared() <= FMath::Square(CombinedRadiusCm);
			const double SpeedSquared = RelativeVelocity.SizeSquared();
			const double UnclampedTimeSeconds = SpeedSquared > UE_DOUBLE_SMALL_NUMBER
				? -FVector::DotProduct(Relative, RelativeVelocity) / SpeedSquared
				: -1.0;
			const bool bClosing = UnclampedTimeSeconds > 0.0;
			const double ClosestTimeSeconds = bClosing
				? FMath::Min(UnclampedTimeSeconds,
					static_cast<double>(Limits.TimeHorizonSeconds))
				: 0.0;
			Risk.TimeToClosestApproachSeconds = bClosing
				? ClosestTimeSeconds : TNumericLimits<double>::Max();
			Risk.NetSeparationCm = (Relative + RelativeVelocity * ClosestTimeSeconds).Size()
				- CombinedRadiusCm;
			return Risk;
		};
		const FRisk RiskA = ComputeRisk(A);
		const FRisk RiskB = ComputeRisk(B);
		if (RiskA.bOverlapping != RiskB.bOverlapping)
		{
			return RiskA.bOverlapping;
		}
		if (!FMath::IsNearlyEqual(RiskA.NetSeparationCm, RiskB.NetSeparationCm))
		{
			return RiskA.NetSeparationCm < RiskB.NetSeparationCm;
		}
		if (RiskA.TimeToClosestApproachSeconds != RiskB.TimeToClosestApproachSeconds)
		{
			return RiskA.TimeToClosestApproachSeconds < RiskB.TimeToClosestApproachSeconds;
		}
		return A.StableId < B.StableId;
	});

	TArray<FVelocityConstraintPlane, TInlineAllocator<72>> Planes;
	Planes.Reserve(SortedNeighbors.Num() + Limits.HorizontalPlaneCount * 3 + 16);

	for (const FAircraftAvoidanceAgentState* OtherPtr : SortedNeighbors)
	{
		const FAircraftAvoidanceAgentState& Other = *OtherPtr;
		// 加速度感知预测：邻居的 CommandedVelocity 与当前速度之差作为加速度估计，
		// 取视界前半段做匀加速外推（远期噪声大于价值），替代全期匀速假设。
		// 外推只用于"更早发现将来的冲突"（平面几何）——重叠判定与责任分配
		// 始终用当前真实距离：外推点被钳制在当前合并半径之外，防止把
		// "将来才会接近"误判成"现在已重叠"（那会让锚定责任归零逻辑失效，
		// 静止占位机被要求为移动机的假重叠分担逃避责任）。
		const double PredictionHorizonSeconds = FMath::Min(
			static_cast<double>(Limits.TimeHorizonSeconds) * 0.5, 1.0);
		const double CurrentCombinedRadiusCm = Self.BodyRadiusCm + Other.BodyRadiusCm
			+ Limits.SeparationPaddingCm + Self.TrackingReserveCm + Other.TrackingReserveCm;
		const FVector CurrentRelativePositionCm = Other.PositionCm - Self.PositionCm;
		// 重叠判定用当前真实距离（外推不参与）——真实重叠时锚定者按优先级分担
		// 分离责任是正确语义；假重叠会把锚定机错误地踢离锚点。
		const bool bOverlapping = CurrentRelativePositionCm.SizeSquared()
			<= FMath::Square(CurrentCombinedRadiusCm);
		FVector PredictedOtherPositionCm = Other.PositionCm;
		if (PredictionHorizonSeconds > 0.0)
		{
			const FVector EstimatedAcceleration = (Other.CommandedVelocityCmPerSec
				- Other.VelocityCmPerSec)
				.GetClampedToMaxSize(FMath::Max(Other.MaxHorizontalSpeedCmPerSec, 0.0f));
			PredictedOtherPositionCm = Other.PositionCm
				+ Other.VelocityCmPerSec * PredictionHorizonSeconds
				+ 0.5 * EstimatedAcceleration * FMath::Square(PredictionHorizonSeconds);
			// 当前未重叠、外推点却越过合并半径 → 收回到边界上：允许预测到
			// "将要重叠"，不允许外推出"已经重叠"（假重叠会绕过锚定责任归零，
			// 静止占位机被要求为移动机的假重叠分担逃避责任）。
			// 真实重叠时保留原始（更近的）位置——分离修正量依赖真实间距。
			if (!bOverlapping)
			{
				const FVector PredictedRelativePositionCm = PredictedOtherPositionCm
					- Self.PositionCm;
				const double PredictedDistanceCm = PredictedRelativePositionCm.Size();
				if (PredictedDistanceCm < CurrentCombinedRadiusCm
					&& PredictedDistanceCm > UE_DOUBLE_SMALL_NUMBER)
				{
					PredictedOtherPositionCm = Self.PositionCm
						+ PredictedRelativePositionCm / PredictedDistanceCm
							* CurrentCombinedRadiusCm;
				}
			}
		}
		const FVector RelativePositionCm = PredictedOtherPositionCm - Self.PositionCm;
		const FVector RelativeVelocityCmPerSec = Self.VelocityCmPerSec - Other.VelocityCmPerSec;
		const double DistanceSquaredCm = RelativePositionCm.SizeSquared();
		const double CombinedRadiusCm = CurrentCombinedRadiusCm;
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

		FVelocityConstraintPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.Normal = UnitW.GetSafeNormal(UE_SMALL_NUMBER,
			BuildDeterministicNormal(Self, Other, RelativePositionCm));
		const double Responsibility = ComputeResponsibility(Self, Other, bOverlapping);
		const FVector PlanePoint = Self.VelocityCmPerSec + Correction * Responsibility;
		Plane.MinimumProjectionCmPerSec = FVector::DotProduct(Plane.Normal, PlanePoint);
		if (Plane.ComputeViolation(PreferredVelocityCmPerSec) > PlaneToleranceCmPerSec)
		{
			Result.bAvoidanceRequired = true;
			const FVector PreferredClosingVelocityCmPerSec = Other.VelocityCmPerSec
				- PreferredVelocityCmPerSec;
			const double PreferredClosingSpeedSquared = PreferredClosingVelocityCmPerSec.SizeSquared();
			const double PreferredClosestTimeSeconds = PreferredClosingSpeedSquared > UE_DOUBLE_SMALL_NUMBER
				? FMath::Clamp(-FVector::DotProduct(RelativePositionCm,
					PreferredClosingVelocityCmPerSec) / PreferredClosingSpeedSquared,
					0.0, static_cast<double>(Limits.TimeHorizonSeconds))
				: 0.0;
			Result.EarliestConflictTimeSeconds = FMath::Min(
				Result.EarliestConflictTimeSeconds,
				static_cast<float>(PreferredClosestTimeSeconds));
		}
	}

	AddCapabilityPlanes(Self, PreviousCommandAccelerationCmPerSecSq, Limits, Planes);
	const double SmoothingAlpha = Limits.SmoothingWeight / (1.0 + Limits.SmoothingWeight);
	const FVector Objective = FMath::Lerp(PreferredVelocityCmPerSec,
		Self.CommandedVelocityCmPerSec, SmoothingAlpha);
	Result.ConstraintPlaneCount = Planes.Num();
	if (CorridorAlternatives.IsEmpty())
	{
		Result.bFeasible = SolveClosestPointWithCapsule(
			Objective, Self.PositionCm, Planes, nullptr, Result.TargetVelocityCmPerSec);
	}
	else
	{
		double BestCost = TNumericLimits<double>::Max();
		for (const FAircraftVelocityConstraintCapsule& Capsule : CorridorAlternatives)
		{
			FVector CandidateVelocityCmPerSec;
			if (!SolveClosestPointWithCapsule(
				Objective, Self.PositionCm, Planes, &Capsule, CandidateVelocityCmPerSec))
			{
				continue;
			}
			const double CandidateCost = FVector::DistSquared(
				CandidateVelocityCmPerSec, Objective);
			if (!Result.bFeasible || CandidateCost < BestCost - UE_DOUBLE_SMALL_NUMBER)
			{
				Result.bFeasible = true;
				BestCost = CandidateCost;
				Result.TargetVelocityCmPerSec = CandidateVelocityCmPerSec;
			}
		}
	}

	const auto ComputeUnionViolation = [&](const FVector& CandidateVelocityCmPerSec)
	{
		double Violation = ComputeMaximumViolation(CandidateVelocityCmPerSec, Planes);
		if (!CorridorAlternatives.IsEmpty())
		{
			double MinimumCapsuleViolation = TNumericLimits<double>::Max();
			for (const FAircraftVelocityConstraintCapsule& Capsule : CorridorAlternatives)
			{
				MinimumCapsuleViolation = FMath::Min(MinimumCapsuleViolation,
					Capsule.ComputeViolationCmPerSec(Self.PositionCm, CandidateVelocityCmPerSec));
			}
			Violation = FMath::Max(Violation, MinimumCapsuleViolation);
		}
		return Violation;
	};

	if (!Result.bFeasible)
	{
		const FVector PlaneFallback = SolveLeastViolation(FVector::ZeroVector, Planes);
		Result.TargetVelocityCmPerSec = PlaneFallback;
		double BestViolation = ComputeUnionViolation(PlaneFallback);
		double BestCost = FVector::DistSquared(PlaneFallback, Objective);
		for (const FAircraftVelocityConstraintCapsule& Capsule : CorridorAlternatives)
		{
			const FVector CandidateVelocityCmPerSec = ProjectVelocityIntoCapsule(
				Self.PositionCm, PlaneFallback, Capsule);
			const double CandidateViolation = ComputeUnionViolation(CandidateVelocityCmPerSec);
			const double CandidateCost = FVector::DistSquared(CandidateVelocityCmPerSec, Objective);
			if (CandidateViolation < BestViolation - UE_DOUBLE_SMALL_NUMBER
				|| (FMath::IsNearlyEqual(CandidateViolation, BestViolation)
					&& CandidateCost < BestCost - UE_DOUBLE_SMALL_NUMBER))
			{
				BestViolation = CandidateViolation;
				BestCost = CandidateCost;
				Result.TargetVelocityCmPerSec = CandidateVelocityCmPerSec;
			}
		}
	}
	Result.MaximumConstraintViolation = static_cast<float>(
		ComputeUnionViolation(Result.TargetVelocityCmPerSec));
	return Result;
}
