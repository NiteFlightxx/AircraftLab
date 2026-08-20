
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"

DEFINE_LOG_CATEGORY_STATIC(LogAircraftTrajectoryGen, Log, All);

bool FAircraftTrajectoryGenerator::SetPlan(const FAircraftTrajectoryPlan& Plan)
{
	Clear();
	FAircraftTrajectoryPlan EffectivePlan = Plan;
	ResolveEffectiveMotionLimits(EffectivePlan);
	const FTrajectoryMotionConstraints& Constraints = EffectivePlan.MotionConstraints;

	const bool bStationaryPath = EffectivePlan.Path.Geometry == EAircraftPathGeometry::Polyline
		&& EffectivePlan.Path.PointsCm.Num() == 1;

	if (!bStationaryPath && (Constraints.CruiseSpeedCmPerSec <= UE_SMALL_NUMBER
		|| Constraints.MaxAccelerationCmPerSecSq <= UE_SMALL_NUMBER
		|| Constraints.MaxDecelerationCmPerSecSq <= UE_SMALL_NUMBER))
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning,
			TEXT("TrajectoryGenerator: cruise speed, acceleration, and deceleration must be positive."));
		return false;
	}
	const float RequestedTerminalSpeed = EffectivePlan.TerminalVelocityCmPerSec.Size();
	if (!bStationaryPath && RequestedTerminalSpeed > Constraints.CruiseSpeedCmPerSec + UE_SMALL_NUMBER)
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning,
			TEXT("TrajectoryGenerator: terminal speed cannot exceed cruise speed."));
		return false;
	}

	FString Error;
	if (!bStationaryPath && !BuildGeometry(EffectivePlan, Error))
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning, TEXT("TrajectoryGenerator: build failed — %s"), *Error);
		bIsValid = false;
		return false;
	}

	if (bStationaryPath)
	{
		TotalArcLengthCm = 0.0f;
		CruiseSpeedCmPerSec = 0.0f;
		PlanningAccelCmPerSecSq = FMath::Max(Constraints.MaxAccelerationCmPerSecSq, UE_SMALL_NUMBER);
		PlanningDecelCmPerSecSq = FMath::Max(Constraints.MaxDecelerationCmPerSecSq, UE_SMALL_NUMBER);
		PlanningJerkCmPerSecCubed = FMath::Max(Constraints.MaxJerkCmPerSecCubed, 0.0f);
		InitialSpeedCmPerSec = 0.0f;
		TargetEndSpeedCmPerSec = 0.0f;
		AcceptanceRadiusCm = FMath::Max(EffectivePlan.AcceptanceRadiusCm, 1.0f);
		CurrentArcLength = 0.0f;
		CurrentSpeedCmPerSec = 0.0f;
		CurrentPathAccelerationCmPerSecSq = 0.0f;
		bIsValid = true;
		CurrentSetpoint.PositionCm = EffectivePlan.Path.PointsCm[0];
		CurrentSetpoint.VelocityCmPerSec = FVector::ZeroVector;
		CurrentSetpoint.AccelerationCmPerSecSq = FVector::ZeroVector;
		CurrentSetpoint.YawRateDegreesPerSec = 0.0f;
		CurrentSetpoint.ArcLengthCm = 0.0f;
		CurrentSetpoint.Curvature = 0.0f;
		CurrentSetpoint.bValid = true;
		return true;
	}

	RecomputeArcLengths();
	if (TotalArcLengthCm <= UE_SMALL_NUMBER)
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning, TEXT("TrajectoryGenerator: zero total arc length."));
		bIsValid = false;
		return false;
	}
	bUsesNativeTimeParameterization = Geometries.Num() == 1
		&& Geometries[0].IsValid() && Geometries[0]->UsesNativeTimeParameterization();
	TotalDurationSeconds = bUsesNativeTimeParameterization
		? Geometries[0]->GetTotalDurationSeconds() : 0.0f;

	CruiseSpeedCmPerSec = FMath::Max(Constraints.CruiseSpeedCmPerSec, UE_SMALL_NUMBER);
	PlanningAccelCmPerSecSq = FMath::Max(Constraints.MaxAccelerationCmPerSecSq, UE_SMALL_NUMBER);
	PlanningDecelCmPerSecSq = FMath::Max(Constraints.MaxDecelerationCmPerSecSq, UE_SMALL_NUMBER);
	PlanningJerkCmPerSecCubed = FMath::Max(Constraints.MaxJerkCmPerSecCubed, 0.0f);
	TargetEndSpeedCmPerSec = FMath::Clamp(
		EffectivePlan.TerminalVelocityCmPerSec.Size(),
		0.0f, CruiseSpeedCmPerSec);
	AcceptanceRadiusCm = FMath::Max(EffectivePlan.AcceptanceRadiusCm, 1.0f);
	bLooping = EffectivePlan.Traversal == EAircraftPathTraversal::Loop;

	const FVector StartTangent = Geometries.Num() > 0 && Geometries[0].IsValid()
		? Geometries[0]->GetFrenetAtArcLength(0.0f).Tangent.GetSafeNormal()
		: FVector::ZeroVector;
	InitialSpeedCmPerSec = StartTangent.IsNearlyZero()
		? 0.0f
		: FMath::Clamp(FVector::DotProduct(EffectivePlan.InitialVelocityCmPerSec, StartTangent), 0.0f, CruiseSpeedCmPerSec);
	CurrentPathAccelerationCmPerSecSq = StartTangent.IsNearlyZero()
		? 0.0f
		: FMath::Clamp(
			FVector::DotProduct(EffectivePlan.InitialAccelerationCmPerSecSq, StartTangent),
			-PlanningDecelCmPerSecSq,
			PlanningAccelCmPerSecSq);

	CurrentArcLength = 0.0f;
	CurrentTimeSeconds = 0.0f;
	CurrentSpeedCmPerSec = InitialSpeedCmPerSec;
	bIsValid = true;
	return true;
}

void FAircraftTrajectoryGenerator::Clear()
{
	Geometries.Reset();
	CumStartArc.Reset();
	TotalArcLengthCm = 0.0f;
	CurrentArcLength = 0.0f;
	CurrentSpeedCmPerSec = 0.0f;
	CurrentPathAccelerationCmPerSecSq = 0.0f;
	bLooping = false;
	CurrentTimeSeconds = 0.0f;
	TotalDurationSeconds = 0.0f;
	bUsesNativeTimeParameterization = false;
	CurrentSetpoint.Reset();
	bIsValid = false;
}

bool FAircraftTrajectoryGenerator::IsComplete() const
{
	if (!bIsValid)
	{
		return true;
	}
	if (bUsesNativeTimeParameterization)
	{
		return CurrentTimeSeconds + UE_SMALL_NUMBER >= TotalDurationSeconds;
	}
	if (bLooping)
	{
		return false;
	}
	return CurrentArcLength + UE_SMALL_NUMBER >= TotalArcLengthCm
		&& FMath::Abs(CurrentSpeedCmPerSec - TargetEndSpeedCmPerSec) <= 1.0f
		&& FMath::Abs(CurrentPathAccelerationCmPerSecSq) <= 1.0f;
}

float FAircraftTrajectoryGenerator::GetProgress() const
{
	if (!bIsValid || TotalArcLengthCm <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}
	if (bUsesNativeTimeParameterization && TotalDurationSeconds > UE_SMALL_NUMBER)
	{
		return FMath::Clamp(CurrentTimeSeconds / TotalDurationSeconds, 0.0f, 1.0f);
	}
	return FMath::Clamp(CurrentArcLength / TotalArcLengthCm, 0.0f, 1.0f);
}

bool FAircraftTrajectoryGenerator::UpdateSetpoint(
	float DeltaSeconds,
	const FVector& CurrentPosition,
	FTrajectoryPoint& OutSetpoint)
{
	OutSetpoint.Reset();
	if (!bIsValid || DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return false;
	}

	// 悬停退化态（零弧长）：直接输出驻留设定值，不推进游标
	if (TotalArcLengthCm <= UE_SMALL_NUMBER)
	{
		OutSetpoint = CurrentSetpoint;
		return true;
	}

	if (bUsesNativeTimeParameterization && Geometries.Num() == 1 && Geometries[0].IsValid())
	{
		CurrentTimeSeconds = FMath::Clamp(
			CurrentTimeSeconds + DeltaSeconds, 0.0f, TotalDurationSeconds);
		OutSetpoint = Geometries[0]->SampleAtTime(CurrentTimeSeconds);
		CurrentSetpoint = OutSetpoint;
		CurrentArcLength = Geometries[0]->GetArcLengthAtTime(CurrentTimeSeconds);
		CurrentSpeedCmPerSec = OutSetpoint.VelocityCmPerSec.Size();
		CurrentPathAccelerationCmPerSecSq = 0.0f;
		return OutSetpoint.bValid;
	}

	const bool bInfinite = bLooping;

	// MoveTo 的参考点必须跟随飞机实际进度，只领先一个更新步；否则参考轨迹会先到终点，
	// 真实飞机只能在零速目标下依靠位置误差缓慢补齐剩余距离。
	const float ProjectedS = ProjectToArcLength(CurrentPosition);
	const float ProgressArc = bInfinite
		? FMath::Max(CurrentArcLength, ProjectedS)
		: FMath::Clamp(ProjectedS, 0.0f, TotalArcLengthCm);
	CurrentArcLength = ProgressArc;

	if (bInfinite)
	{
		// 闭合路径只改变遍历策略；速度仍使用同一套 V/A/J 时序。
		CurrentSpeedCmPerSec = AdvanceSpeedToward(CruiseSpeedCmPerSec, DeltaSeconds);
		CurrentArcLength += CurrentSpeedCmPerSec * DeltaSeconds;

		const float SampleArc = bUseLookAhead
			? CurrentArcLength + LookAheadDistanceCm
			: CurrentArcLength;
		OutSetpoint = SampleAtGlobalArc(SampleArc, CurrentSpeedCmPerSec);
		CurrentSetpoint = OutSetpoint;
		return true;
	}

	// --- 2. 单一 S 曲线速度剖面：轨迹生成器独占 MoveTo 的平移 V/A/J 规划。 ---
	const float PreviousSpeedCmPerSec = CurrentSpeedCmPerSec;
	CurrentSpeedCmPerSec = ComputeConstrainedSpeed(ProgressArc, TotalArcLengthCm, DeltaSeconds);

	// --- 3. 推进游标：s += v·Δt（梯形积分）---
	const float IntegratedDistance = 0.5f * (PreviousSpeedCmPerSec + CurrentSpeedCmPerSec) * DeltaSeconds;
	CurrentArcLength = FMath::Clamp(CurrentArcLength + IntegratedDistance, 0.0f, TotalArcLengthCm);
	if (CurrentSpeedCmPerSec <= TargetEndSpeedCmPerSec + 1.0f
		&& TotalArcLengthCm - CurrentArcLength <= AcceptanceRadiusCm)
	{
		CurrentArcLength = TotalArcLengthCm;
	}

	// --- 4. 终点：保留减速段速度，不突跳到终点速度 ---
	const float RemainingDistance = FMath::Max(TotalArcLengthCm - CurrentArcLength, 0.0f);
	if (RemainingDistance <= UE_SMALL_NUMBER)
	{
		OutSetpoint = SampleGlobalArcLength(
			TotalArcLengthCm, CurrentSpeedCmPerSec, CurrentPathAccelerationCmPerSecSq);
		CurrentSetpoint = OutSetpoint;
		return true;
	}

	// --- 5. 采样设定值（含 LookAhead）---
	const float SampleArc = bUseLookAhead
		? FMath::Clamp(CurrentArcLength + LookAheadDistanceCm, 0.0f, TotalArcLengthCm)
		: CurrentArcLength;
	OutSetpoint = SampleGlobalArcLength(
		SampleArc, CurrentSpeedCmPerSec, CurrentPathAccelerationCmPerSecSq);
	CurrentSetpoint = OutSetpoint;
	return true;
}

bool FAircraftTrajectoryGenerator::BuildGeometry(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	if (Plan.Path.Geometry == EAircraftPathGeometry::Polyline)
	{
		return BuildPolyline(Plan, OutError);
	}

	TUniquePtr<FAircraftPathGeometry> Geometry;
	switch (Plan.Path.Geometry)
	{
	case EAircraftPathGeometry::Bezier:
		Geometry = MakeUnique<FAircraftBezierPathGeometry>();
		break;
	case EAircraftPathGeometry::Circle:
		Geometry = MakeUnique<FAircraftCirclePathGeometry>();
		break;
	case EAircraftPathGeometry::MinimumSnap:
		Geometry = MakeUnique<FAircraftMinimumSnapPathGeometry>();
		break;
	default:
		OutError = TEXT("Unsupported path geometry.");
		return false;
	}
	if (!Geometry->BuildPath(Plan, OutError)) return false;
	Geometries.Add(MoveTemp(Geometry));
	return true;
}

bool FAircraftTrajectoryGenerator::BuildPolyline(const FAircraftTrajectoryPlan& Plan, FString& OutError)
{
	if (Plan.Path.PointsCm.Num() < 2)
	{
		OutError = TEXT("Polyline requires at least two points.");
		return false;
	}
	for (int32 Index = 1; Index < Plan.Path.PointsCm.Num(); ++Index)
	{
		FAircraftTrajectoryPlan LegPlan = Plan;
		LegPlan.Path.PointsCm = {
			Plan.Path.PointsCm[Index - 1], Plan.Path.PointsCm[Index] };
		TUniquePtr<FAircraftLinePathGeometry> Leg = MakeUnique<FAircraftLinePathGeometry>();
		if (!Leg->BuildPath(LegPlan, OutError)) return false;
		Geometries.Add(MoveTemp(Leg));
	}
	return true;
}

void FAircraftTrajectoryGenerator::ResolveEffectiveMotionLimits(FAircraftTrajectoryPlan& InOutPlan) const
{
	FTrajectoryMotionConstraints& Limits = InOutPlan.MotionConstraints;
	auto ApplyDirection = [&Limits, &InOutPlan](const FVector& RawDirection)
	{
		const FVector Direction = RawDirection.GetSafeNormal();
		if (Direction.IsNearlyZero()) return;
		const float HorizontalFraction = FVector2D(Direction.X, Direction.Y).Size();
		const float VerticalFraction = FMath::Abs(Direction.Z);
		if (HorizontalFraction > UE_SMALL_NUMBER)
		{
			Limits.CruiseSpeedCmPerSec = FMath::Min(Limits.CruiseSpeedCmPerSec,
				InOutPlan.PhysicalMaxHorizontalSpeedCmPerSec / HorizontalFraction);
			Limits.MaxAccelerationCmPerSecSq = FMath::Min(Limits.MaxAccelerationCmPerSecSq,
				InOutPlan.PhysicalMaxHorizontalAccelerationCmPerSecSq / HorizontalFraction);
			Limits.MaxDecelerationCmPerSecSq = FMath::Min(Limits.MaxDecelerationCmPerSecSq,
				InOutPlan.PhysicalMaxHorizontalAccelerationCmPerSecSq / HorizontalFraction);
		}
		if (VerticalFraction > UE_SMALL_NUMBER)
		{
			const float VerticalSpeed = Direction.Z >= 0.0f
				? Limits.MaxClimbRateCmPerSec : Limits.MaxDescentRateCmPerSec;
			Limits.CruiseSpeedCmPerSec = FMath::Min(Limits.CruiseSpeedCmPerSec,
				VerticalSpeed / VerticalFraction);
			Limits.MaxAccelerationCmPerSecSq = FMath::Min(Limits.MaxAccelerationCmPerSecSq,
				Limits.MaxVerticalAccelerationCmPerSecSq / VerticalFraction);
			Limits.MaxDecelerationCmPerSecSq = FMath::Min(Limits.MaxDecelerationCmPerSecSq,
				Limits.MaxVerticalAccelerationCmPerSecSq / VerticalFraction);
			Limits.MaxJerkCmPerSecCubed = FMath::Min(Limits.MaxJerkCmPerSecCubed,
				Limits.MaxVerticalJerkCmPerSecCubed / VerticalFraction);
		}
	};

	if (InOutPlan.Path.Geometry == EAircraftPathGeometry::Circle)
	{
		ApplyDirection(FVector::ForwardVector);
		return;
	}
	for (int32 Index = 1; Index < InOutPlan.Path.PointsCm.Num(); ++Index)
	{
		ApplyDirection(InOutPlan.Path.PointsCm[Index] - InOutPlan.Path.PointsCm[Index - 1]);
	}
}

void FAircraftTrajectoryGenerator::RecomputeArcLengths()
{
	CumStartArc.Reset();
	CumStartArc.Add(0.0f);
	float Cum = 0.0f;
	for (const TUniquePtr<FAircraftPathGeometry>& Geometry : Geometries)
	{
		if (Geometry.IsValid())
		{
			Cum += Geometry->GetTotalArcLengthCm();
		}
		CumStartArc.Add(Cum);
	}
	TotalArcLengthCm = Cum;
}

float FAircraftTrajectoryGenerator::ComputeConstrainedSpeed(
	float CurrentS,
	float TotalS,
	float DeltaSeconds)
{
	const float Vc = CruiseSpeedCmPerSec;
	const float VEnd = TargetEndSpeedCmPerSec;
	if (DeltaSeconds <= UE_SMALL_NUMBER || TotalS <= UE_SMALL_NUMBER)
	{
		CurrentPathAccelerationCmPerSecSq = 0.0f;
		return VEnd;
	}

	const float RemainingForBraking = FMath::Max(TotalS - CurrentS, 0.0f);
	const float TargetSpeed = FMath::Min(
		Vc, ComputeBrakingSpeedLimit(RemainingForBraking));
	return AdvanceSpeedToward(TargetSpeed, DeltaSeconds);
}

float FAircraftTrajectoryGenerator::AdvanceSpeedToward(
	float TargetSpeedCmPerSec,
	float DeltaSeconds)
{
	const float TargetSpeed = FMath::Clamp(TargetSpeedCmPerSec, 0.0f, CruiseSpeedCmPerSec);
	const float SpeedError = TargetSpeed - CurrentSpeedCmPerSec;
	const float AccelerationLimit = SpeedError >= 0.0f
		? PlanningAccelCmPerSecSq : PlanningDecelCmPerSecSq;
	float DesiredAcceleration = 0.0f;
	if (PlanningJerkCmPerSecCubed > UE_SMALL_NUMBER)
	{
		const float ErrorDirection = FMath::Sign(SpeedError);
		const bool bAcceleratingTowardTarget =
			CurrentPathAccelerationCmPerSecSq * ErrorDirection > 0.0f;
		const float SpeedNeededToReleaseAcceleration =
			FMath::Square(CurrentPathAccelerationCmPerSecSq)
			/ (2.0f * PlanningJerkCmPerSecCubed);
		const bool bReleaseAcceleration = bAcceleratingTowardTarget
			&& SpeedNeededToReleaseAcceleration >= FMath::Abs(SpeedError);
		DesiredAcceleration = bReleaseAcceleration
			? 0.0f
			: ErrorDirection * AccelerationLimit;
		const float MaxAccelerationChange = PlanningJerkCmPerSecCubed * DeltaSeconds;
		CurrentPathAccelerationCmPerSecSq += FMath::Clamp(
			DesiredAcceleration - CurrentPathAccelerationCmPerSecSq,
			-MaxAccelerationChange,
			MaxAccelerationChange);
	}
	else
	{
		CurrentPathAccelerationCmPerSecSq = FMath::Clamp(
			SpeedError / DeltaSeconds, -PlanningDecelCmPerSecSq, PlanningAccelCmPerSecSq);
	}

	float NewSpeed = FMath::Clamp(
		CurrentSpeedCmPerSec + CurrentPathAccelerationCmPerSecSq * DeltaSeconds,
		0.0f,
		CruiseSpeedCmPerSec);
	if (!FMath::IsNearlyZero(SpeedError)
		&& SpeedError * (TargetSpeed - NewSpeed) <= 0.0f)
	{
		NewSpeed = TargetSpeed;
		if (PlanningJerkCmPerSecCubed <= UE_SMALL_NUMBER)
		{
			CurrentPathAccelerationCmPerSecSq = 0.0f;
		}
	}
	return NewSpeed;
}

float FAircraftTrajectoryGenerator::ComputeBrakingDistance(float StartSpeedCmPerSec, float EndSpeedCmPerSec) const
{
	const float StartSpeed = FMath::Max(StartSpeedCmPerSec, 0.0f);
	const float EndSpeed = FMath::Clamp(EndSpeedCmPerSec, 0.0f, StartSpeed);
	const float DeltaSpeed = StartSpeed - EndSpeed;
	const float Deceleration = FMath::Max(PlanningDecelCmPerSecSq, UE_SMALL_NUMBER);
	if (DeltaSpeed <= UE_SMALL_NUMBER
		&& CurrentPathAccelerationCmPerSecSq <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}
	if (PlanningJerkCmPerSecCubed <= UE_SMALL_NUMBER)
	{
		return (StartSpeed * StartSpeed - EndSpeed * EndSpeed)
			/ (2.0f * Deceleration);
	}

	const float Jerk = PlanningJerkCmPerSecCubed;
	float EffectiveStartSpeed = StartSpeed;
	float AccelerationReleaseDistance = 0.0f;
	if (CurrentPathAccelerationCmPerSecSq > UE_SMALL_NUMBER)
	{
		const float InitialAcceleration = FMath::Min(
			CurrentPathAccelerationCmPerSecSq, PlanningAccelCmPerSecSq);
		const float ReleaseTime = InitialAcceleration / Jerk;
		AccelerationReleaseDistance = StartSpeed * ReleaseTime
			+ 0.5f * InitialAcceleration * FMath::Square(ReleaseTime)
			- Jerk * ReleaseTime * ReleaseTime * ReleaseTime / 6.0f;
		EffectiveStartSpeed += 0.5f * FMath::Square(InitialAcceleration) / Jerk;
	}

	const float EffectiveDeltaSpeed = FMath::Max(EffectiveStartSpeed - EndSpeed, 0.0f);
	const float SpeedChangeInRamps = Deceleration * Deceleration / Jerk;
	float TotalBrakingTime = 0.0f;
	if (EffectiveDeltaSpeed >= SpeedChangeInRamps)
	{
		TotalBrakingTime = 2.0f * Deceleration / Jerk
			+ (EffectiveDeltaSpeed - SpeedChangeInRamps) / Deceleration;
	}
	else
	{
		TotalBrakingTime = 2.0f * FMath::Sqrt(EffectiveDeltaSpeed / Jerk);
	}
	return AccelerationReleaseDistance
		+ 0.5f * (EffectiveStartSpeed + EndSpeed) * TotalBrakingTime;
}

float FAircraftTrajectoryGenerator::ComputeBrakingSpeedLimit(float RemainingDistanceCm) const
{
	const float Remaining = FMath::Max(RemainingDistanceCm, 0.0f);
	float Low = TargetEndSpeedCmPerSec;
	float High = CruiseSpeedCmPerSec;
	for (int32 Iteration = 0; Iteration < 20; ++Iteration)
	{
		const float Candidate = 0.5f * (Low + High);
		if (ComputeBrakingDistance(Candidate, TargetEndSpeedCmPerSec) <= Remaining)
		{
			Low = Candidate;
		}
		else
		{
			High = Candidate;
		}
	}
	return Low;
}

void FAircraftTrajectoryGenerator::LocateSegment(float GlobalArc, int32& OutSegIndex, float& OutLocalArc) const
{
	OutSegIndex = INDEX_NONE;
	OutLocalArc = 0.0f;
	if (Geometries.Num() == 0)
	{
		return;
	}

	int32 Lo = 0;
	int32 Hi = CumStartArc.Num() - 1;
	while (Lo < Hi - 1)
	{
		const int32 Mid = (Lo + Hi) / 2;
		if (CumStartArc[Mid] <= GlobalArc) Lo = Mid;
		else Hi = Mid;
	}
	OutSegIndex = FMath::Clamp(Lo, 0, Geometries.Num() - 1);

	if (!Geometries[OutSegIndex].IsValid())
	{
		for (int32 i = OutSegIndex + 1; i < Geometries.Num(); ++i)
		{
			if (Geometries[i].IsValid())
			{
				OutSegIndex = i;
				break;
			}
		}
		if (!Geometries[OutSegIndex].IsValid())
		{
			return;
		}
	}

	OutLocalArc = FMath::Clamp(GlobalArc - CumStartArc[OutSegIndex], 0.0f, Geometries[OutSegIndex]->GetTotalArcLengthCm());
}

FTrajectoryPoint FAircraftTrajectoryGenerator::SampleGlobalArcLength(
	float GlobalArc,
	float Speed,
	float TangentialAccelerationCmPerSecSq) const
{
	FTrajectoryPoint Point;
	if (!bIsValid || Geometries.Num() == 0)
	{
		Point.bValid = false;
		return Point;
	}

	int32 SegIndex;
	float LocalArc;
	LocateSegment(GlobalArc, SegIndex, LocalArc);
	if (!Geometries.IsValidIndex(SegIndex))
	{
		Point.bValid = false;
		return Point;
	}

	Point = Geometries[SegIndex]->SampleAtArcLength(LocalArc, Speed);
	const FVector Tangent = Geometries[SegIndex]->GetFrenetAtArcLength(LocalArc).Tangent.GetSafeNormal();
	Point.AccelerationCmPerSecSq += Tangent * TangentialAccelerationCmPerSecSq;
	Point.ArcLengthCm = GlobalArc;
	return Point;
}

float FAircraftTrajectoryGenerator::ProjectToArcLength(const FVector& WorldPosition) const
{
	if (!bIsValid || Geometries.Num() == 0)
	{
		return 0.0f;
	}

	float BestS = CurrentArcLength;
	float BestDistSq = TNumericLimits<float>::Max();

	// 无限循环段（Orbit）：限制在当前游标附近 ±单圈 局部窗口搜索
	const bool bInfinite = bLooping;
	float SearchWindowMin = 0.0f;
	float SearchWindowMax = TNumericLimits<float>::Max();
	if (bInfinite)
	{
		int32 CurSegIdx;
		float CurLocalArc;
		LocateSegment(CurrentArcLength, CurSegIdx, CurLocalArc);
		if (Geometries.IsValidIndex(CurSegIdx) && Geometries[CurSegIdx].IsValid())
		{
			const float OneLap = Geometries[CurSegIdx]->GetTotalArcLengthCm();
			SearchWindowMin = FMath::Max(CurrentArcLength - OneLap, 0.0f);
			SearchWindowMax = CurrentArcLength + OneLap;
		}
	}

	for (int32 i = 0; i < Geometries.Num(); ++i)
	{
		const FAircraftPathGeometry* Seg = Geometries[i].Get();
		if (!Seg)
		{
			continue;
		}
		const float SegStart = CumStartArc[i];
		const float SegLen = Seg->GetTotalArcLengthCm();

		if (bInfinite && (SegStart + SegLen < SearchWindowMin || SegStart > SearchWindowMax))
		{
			continue;
		}

		const int32 ProjectionSamples = Seg->UsesNativeTimeParameterization() ? 128 : 16;
		const float Step = FMath::Max(SegLen / ProjectionSamples, 1.0f);
		float BestLocalS = 0.0f;
		float BestSegmentDistSq = TNumericLimits<float>::Max();
		for (float LocalS = 0.0f; LocalS <= SegLen; LocalS += Step)
		{
			float GlobalS = SegStart + LocalS;
			if (bInfinite && SegLen > UE_SMALL_NUMBER)
			{
				const float NearestLap = FMath::Max(
					FMath::RoundToFloat((CurrentArcLength - GlobalS) / SegLen), 0.0f);
				GlobalS += NearestLap * SegLen;
			}
			if (bInfinite && (GlobalS < SearchWindowMin || GlobalS > SearchWindowMax))
			{
				continue;
			}

			const FFrenetFrame Frame = Seg->GetFrenetAtArcLength(LocalS);
			const float DistSq = FVector::DistSquared(Frame.OriginCm, WorldPosition);
			if (DistSq < BestSegmentDistSq)
			{
				BestSegmentDistSq = DistSq;
				BestLocalS = LocalS;
			}
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestS = GlobalS;
			}
		}

		float RefineMin = FMath::Max(BestLocalS - Step, 0.0f);
		float RefineMax = FMath::Min(BestLocalS + Step, SegLen);
		for (int32 Iteration = 0; Iteration < 12; ++Iteration)
		{
			const float Third = (RefineMax - RefineMin) / 3.0f;
			const float Left = RefineMin + Third;
			const float Right = RefineMax - Third;
			const float LeftDistSq = FVector::DistSquared(
				Seg->GetFrenetAtArcLength(Left).OriginCm, WorldPosition);
			const float RightDistSq = FVector::DistSquared(
				Seg->GetFrenetAtArcLength(Right).OriginCm, WorldPosition);
			if (LeftDistSq <= RightDistSq)
			{
				RefineMax = Right;
			}
			else
			{
				RefineMin = Left;
			}
		}
		const float RefinedLocalS = 0.5f * (RefineMin + RefineMax);
		float RefinedGlobalS = SegStart + RefinedLocalS;
		if (bInfinite && SegLen > UE_SMALL_NUMBER)
		{
			const float NearestLap = FMath::Max(
				FMath::RoundToFloat((CurrentArcLength - RefinedGlobalS) / SegLen), 0.0f);
			RefinedGlobalS += NearestLap * SegLen;
		}
		const float RefinedDistSq = FVector::DistSquared(
			Seg->GetFrenetAtArcLength(RefinedLocalS).OriginCm, WorldPosition);
		if ((!bInfinite || (RefinedGlobalS >= SearchWindowMin && RefinedGlobalS <= SearchWindowMax))
			&& RefinedDistSq < BestDistSq)
		{
			BestDistSq = RefinedDistSq;
			BestS = RefinedGlobalS;
		}
	}

	return BestS;
}

FTrajectoryPoint FAircraftTrajectoryGenerator::SampleAtGlobalArc(float GlobalArc, float Speed) const
{
	FTrajectoryPoint Point;
	if (!bIsValid || Geometries.Num() == 0)
	{
		Point.bValid = false;
		return Point;
	}

	// 无限循环段（Orbit）：不 clamp 上界，段内部 Fmod 绕回单圈
	if (bLooping)
	{
		const float EffectiveArc = FMath::Max(GlobalArc, 0.0f);
		int32 CurSegIdx;
		float CurLocalArc;
		LocateSegment(CurrentArcLength, CurSegIdx, CurLocalArc);
		if (Geometries.IsValidIndex(CurSegIdx) && Geometries[CurSegIdx].IsValid())
		{
			const float SegStart = CumStartArc.IsValidIndex(CurSegIdx) ? CumStartArc[CurSegIdx] : 0.0f;
			const float LocalArc = EffectiveArc - SegStart;
			FTrajectoryPoint P = Geometries[CurSegIdx]->SampleAtArcLength(LocalArc, Speed);
			P.ArcLengthCm = EffectiveArc;
			return P;
		}
	}

	const float ClampedArc = FMath::Clamp(GlobalArc, 0.0f, TotalArcLengthCm);
	return SampleGlobalArcLength(ClampedArc, Speed);
}
