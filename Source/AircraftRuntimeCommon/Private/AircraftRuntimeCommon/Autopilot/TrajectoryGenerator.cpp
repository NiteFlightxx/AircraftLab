// 对应 NxGame AircraftAutopilot/Private/Trajectory/TrajectoryGenerator.cpp（逐函数对齐移植）。

#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"

DEFINE_LOG_CATEGORY_STATIC(LogAircraftTrajectoryGen, Log, All);

bool FAircraftTrajectoryGenerator::SetRequest(const FTrajectoryRequest& Request)
{
	Clear();

	// 悬停退化：Start≈Target（零长度 Waypoint/Line）时无需构造段，
	// 直接缓存目标位置为驻留设定值（避免 LineSegment coincident 检查刷屏）。
	const float StartToTargetDist = FVector::Dist(Request.StartPositionCm, Request.TargetPositionCm);
	const bool bHoverRequest = (Request.Type == ETrajectoryType::Waypoint || Request.Type == ETrajectoryType::Line)
		&& StartToTargetDist <= 1.0f;

	if (!bHoverRequest && (Request.PlanningAccelerationCmPerSecSq <= UE_SMALL_NUMBER
		|| Request.PlanningDecelerationCmPerSecSq <= UE_SMALL_NUMBER))
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning,
			TEXT("TrajectoryGenerator: acceleration and deceleration must both be positive."));
		return false;
	}
	const float RequestedTerminalSpeed = FVector2D(
		Request.TargetVelocityCmPerSec.X, Request.TargetVelocityCmPerSec.Y).Size();
	if (!bHoverRequest && RequestedTerminalSpeed > Request.CruiseSpeedCmPerSec + UE_SMALL_NUMBER)
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning,
			TEXT("TrajectoryGenerator: terminal speed cannot exceed cruise speed."));
		return false;
	}

	FString Error;
	if (!bHoverRequest && !BuildSegments(Request, Error))
	{
		UE_LOG(LogAircraftTrajectoryGen, Warning, TEXT("TrajectoryGenerator: build failed — %s"), *Error);
		bIsValid = false;
		return false;
	}

	if (bHoverRequest)
	{
		// 驻留态：零弧长，设定值=目标位置静止
		TotalArcLengthCm = 0.0f;
		CruiseSpeedCmPerSec = 0.0f;
		PlanningAccelCmPerSecSq = FMath::Max(Request.PlanningAccelerationCmPerSecSq, UE_SMALL_NUMBER);
		PlanningDecelCmPerSecSq = FMath::Max(Request.PlanningDecelerationCmPerSecSq, UE_SMALL_NUMBER);
		PlanningJerkCmPerSecCubed = FMath::Max(Request.PlanningJerkCmPerSecCubed, 0.0f);
		InitialSpeedCmPerSec = 0.0f;
		TargetEndSpeedCmPerSec = 0.0f;
		AcceptanceRadiusCm = FMath::Max(Request.AcceptanceRadiusCm, 1.0f);
		DecelTriggerDistanceCm = 0.0f;
		CurrentArcLength = 0.0f;
		CurrentSpeedCmPerSec = 0.0f;
		bIsValid = true;
		CurrentSetpoint.PositionCm = Request.TargetPositionCm;
		CurrentSetpoint.VelocityCmPerSec = FVector::ZeroVector;
		CurrentSetpoint.AccelerationCmPerSecSq = FVector::ZeroVector;
		CurrentSetpoint.YawDegrees = Request.TargetYawDegrees;
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
	bUsesNativeTimeParameterization = Segments.Num() == 1
		&& Segments[0].IsValid() && Segments[0]->UsesNativeTimeParameterization();
	TotalDurationSeconds = bUsesNativeTimeParameterization
		? Segments[0]->GetTotalDurationSeconds() : 0.0f;

	CruiseSpeedCmPerSec = FMath::Max(Request.CruiseSpeedCmPerSec, UE_SMALL_NUMBER);
	PlanningAccelCmPerSecSq = FMath::Max(Request.PlanningAccelerationCmPerSecSq, UE_SMALL_NUMBER);
	PlanningDecelCmPerSecSq = FMath::Max(Request.PlanningDecelerationCmPerSecSq, UE_SMALL_NUMBER);
	PlanningJerkCmPerSecCubed = FMath::Max(Request.PlanningJerkCmPerSecCubed, 0.0f);
	TargetEndSpeedCmPerSec = FMath::Clamp(
		FVector2D(Request.TargetVelocityCmPerSec.X, Request.TargetVelocityCmPerSec.Y).Size(),
		0.0f, CruiseSpeedCmPerSec);
	AcceptanceRadiusCm = FMath::Max(Request.AcceptanceRadiusCm, 1.0f);

	const FVector StartTangent = Segments.Num() > 0 && Segments[0].IsValid()
		? Segments[0]->GetFrenetAtArcLength(0.0f).Tangent.GetSafeNormal()
		: FVector::ZeroVector;
	InitialSpeedCmPerSec = StartTangent.IsNearlyZero()
		? 0.0f
		: FMath::Clamp(FVector::DotProduct(Request.StartVelocityCmPerSec, StartTangent), 0.0f, CruiseSpeedCmPerSec);

	DecelTriggerDistanceCm = ComputeBrakingDistance(CruiseSpeedCmPerSec, TargetEndSpeedCmPerSec);

	CurrentArcLength = 0.0f;
	CurrentTimeSeconds = 0.0f;
	CurrentSpeedCmPerSec = InitialSpeedCmPerSec;
	bIsValid = true;
	return true;
}

void FAircraftTrajectoryGenerator::Clear()
{
	Segments.Reset();
	CumStartArc.Reset();
	TotalArcLengthCm = 0.0f;
	CurrentArcLength = 0.0f;
	CurrentSpeedCmPerSec = 0.0f;
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
	if (IsCurrentSegmentInfiniteLoop())
	{
		return false;
	}
	return CurrentArcLength + UE_SMALL_NUMBER >= TotalArcLengthCm
		&& FMath::Abs(CurrentSpeedCmPerSec - TargetEndSpeedCmPerSec) <= 1.0f;
}

bool FAircraftTrajectoryGenerator::IsCurrentSegmentInfiniteLoop() const
{
	if (!bIsValid || Segments.Num() == 0)
	{
		return false;
	}
	int32 SegIndex;
	float LocalArc;
	LocateSegment(CurrentArcLength, SegIndex, LocalArc);
	if (Segments.IsValidIndex(SegIndex) && Segments[SegIndex].IsValid())
	{
		return Segments[SegIndex]->IsInfiniteLoop();
	}
	return false;
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

bool FAircraftTrajectoryGenerator::UpdateSetpoint(float DeltaSeconds, const FVector& CurrentPosition, const FVector& CurrentVelocity, FTrajectoryPoint& OutSetpoint)
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

	if (bUsesNativeTimeParameterization && Segments.Num() == 1 && Segments[0].IsValid())
	{
		CurrentTimeSeconds = FMath::Clamp(
			CurrentTimeSeconds + DeltaSeconds, 0.0f, TotalDurationSeconds);
		OutSetpoint = Segments[0]->SampleAtTime(CurrentTimeSeconds);
		CurrentSetpoint = OutSetpoint;
		CurrentArcLength = Segments[0]->GetArcLengthAtTime(CurrentTimeSeconds);
		CurrentSpeedCmPerSec = OutSetpoint.VelocityCmPerSec.Size();
		return OutSetpoint.bValid;
	}

	const bool bInfinite = IsCurrentSegmentInfiniteLoop();

	// --- 1. 用当前位置投影到轨迹，校正游标（防漂移；只前进不后退）---
	const float ProjectedS = ProjectToArcLength(CurrentPosition);
	CurrentArcLength = FMath::Max(CurrentArcLength, ProjectedS);

	if (bInfinite)
	{
		// --- 无限循环段（Orbit）：恒定巡航速，游标不 clamp、不触发完成 ---
		CurrentSpeedCmPerSec = CruiseSpeedCmPerSec;
		CurrentArcLength += CurrentSpeedCmPerSec * DeltaSeconds;

		const float SampleArc = bUseLookAhead
			? CurrentArcLength + LookAheadDistanceCm
			: CurrentArcLength;
		OutSetpoint = SampleAtGlobalArc(SampleArc, CurrentSpeedCmPerSec);
		CurrentSetpoint = OutSetpoint;
		return true;
	}

	// --- 2. 梯形速度剖面 ---
	const float PreviousSpeedCmPerSec = CurrentSpeedCmPerSec;
	CurrentSpeedCmPerSec = ComputeTrapezoidalSpeed(CurrentArcLength, TotalArcLengthCm, DeltaSeconds);

	// --- 3. 推进游标：s += v·Δt（梯形积分）---
	const float IntegratedDistance = 0.5f * (PreviousSpeedCmPerSec + CurrentSpeedCmPerSec) * DeltaSeconds;
	CurrentArcLength = FMath::Clamp(CurrentArcLength + IntegratedDistance, 0.0f, TotalArcLengthCm);

	// --- 4. 终点：保留减速段速度，不突跳到终点速度 ---
	const float RemainingDistance = FMath::Max(TotalArcLengthCm - CurrentArcLength, 0.0f);
	if (RemainingDistance <= UE_SMALL_NUMBER)
	{
		OutSetpoint = SampleGlobalArcLength(TotalArcLengthCm, CurrentSpeedCmPerSec);
		CurrentSetpoint = OutSetpoint;
		return true;
	}

	// --- 5. 采样设定值（含 LookAhead）---
	const float SampleArc = bUseLookAhead
		? FMath::Clamp(CurrentArcLength + LookAheadDistanceCm, 0.0f, TotalArcLengthCm)
		: CurrentArcLength;
	OutSetpoint = SampleGlobalArcLength(SampleArc, CurrentSpeedCmPerSec);
	CurrentSetpoint = OutSetpoint;
	return true;
}

bool FAircraftTrajectoryGenerator::BuildSegments(const FTrajectoryRequest& Request, FString& OutError)
{
	switch (Request.Type)
	{
	case ETrajectoryType::Waypoint:
		return BuildWaypointSegment(Request, OutError);
	case ETrajectoryType::FollowPath:
		return BuildFollowPathSegments(Request, OutError);
	case ETrajectoryType::Line:
	case ETrajectoryType::Bezier:
	case ETrajectoryType::Circle:
	case ETrajectoryType::Orbit:
	case ETrajectoryType::MinimumSnap:
	{
		TUniquePtr<FAircraftTrajectorySegment> Seg;
		switch (Request.Type)
		{
		case ETrajectoryType::Line:       Seg = MakeUnique<FAircraftLineTrajectorySegment>(); break;
		case ETrajectoryType::Bezier:     Seg = MakeUnique<FAircraftBezierTrajectorySegment>(); break;
		case ETrajectoryType::Circle:     Seg = MakeUnique<FAircraftCircleTrajectorySegment>(); break;
		case ETrajectoryType::Orbit:      Seg = MakeUnique<FAircraftOrbitTrajectorySegment>(); break;
		case ETrajectoryType::MinimumSnap: Seg = MakeUnique<FAircraftMinSnapTrajectorySegment>(); break;
		default: break;
		}
		if (!Seg.IsValid()) { OutError = TEXT("Unknown segment type."); return false; }
		if (!Seg->BuildSegment(Request, OutError)) { return false; }
		Segments.Add(MoveTemp(Seg));
		return true;
	}
	}
	return false;
}

bool FAircraftTrajectoryGenerator::BuildWaypointSegment(const FTrajectoryRequest& Request, FString& OutError)
{
	TUniquePtr<FAircraftLineTrajectorySegment> Seg = MakeUnique<FAircraftLineTrajectorySegment>();
	if (!Seg->BuildSegment(Request, OutError))
	{
		return false;
	}
	Segments.Add(MoveTemp(Seg));
	return true;
}

bool FAircraftTrajectoryGenerator::BuildFollowPathSegments(const FTrajectoryRequest& Request, FString& OutError)
{
	const TArray<FVector>& Points = Request.PathPointsCm;
	if (Points.Num() < 2)
	{
		OutError = TEXT("FollowPath: PathPointsCm needs >= 2 points.");
		return false;
	}

	// 起点与首个路径点不重合时，自动插入连接段
	const FVector Prev = Request.StartPositionCm;
	const bool bNeedConnectToFirst = FVector::Dist(Prev, Points[0]) > Request.AcceptanceRadiusCm * 0.5f;

	if (bNeedConnectToFirst)
	{
		FTrajectoryRequest ConnReq = Request;
		ConnReq.Type = ETrajectoryType::Line;
		ConnReq.StartPositionCm = Prev;
		ConnReq.TargetPositionCm = Points[0];
		TUniquePtr<FAircraftLineTrajectorySegment> Conn = MakeUnique<FAircraftLineTrajectorySegment>();
		if (!Conn->BuildSegment(ConnReq, OutError))
		{
			return false;
		}
		Segments.Add(MoveTemp(Conn));
	}

	for (int32 i = 1; i < Points.Num(); ++i)
	{
		FTrajectoryRequest LegReq = Request;
		LegReq.Type = ETrajectoryType::Line;
		LegReq.StartPositionCm = Points[i - 1];
		LegReq.TargetPositionCm = Points[i];
		TUniquePtr<FAircraftLineTrajectorySegment> Leg = MakeUnique<FAircraftLineTrajectorySegment>();
		if (!Leg->BuildSegment(LegReq, OutError))
		{
			return false;
		}
		Segments.Add(MoveTemp(Leg));
	}
	return true;
}

void FAircraftTrajectoryGenerator::RecomputeArcLengths()
{
	CumStartArc.Reset();
	CumStartArc.Add(0.0f);
	float Cum = 0.0f;
	for (const TUniquePtr<FAircraftTrajectorySegment>& Seg : Segments)
	{
		if (Seg.IsValid())
		{
			Cum += Seg->GetTotalArcLengthCm();
		}
		CumStartArc.Add(Cum);
	}
	TotalArcLengthCm = Cum;
}

// 冷启动死锁说明：纯公式 v=sqrt(2·a·s) 在 s=0 处 v=0 → 游标不推进 → deadlock。
// 修复语义由 ComputeTrapezoidalSpeed 的"初速 + 每步 a·dt 累进"保证（起步即有可观测位移）。
float FAircraftTrajectoryGenerator::ComputeTrapezoidalSpeed(float CurrentS, float TotalS, float DeltaSeconds) const
{
	const float Vc = CruiseSpeedCmPerSec;
	const float A = PlanningAccelCmPerSecSq;
	const float VEnd = TargetEndSpeedCmPerSec;
	const float D = PlanningDecelCmPerSecSq;
	if (DeltaSeconds <= UE_SMALL_NUMBER || TotalS <= UE_SMALL_NUMBER)
	{
		return VEnd;
	}

	const float RemainingForBraking = FMath::Max(TotalS - CurrentS, 0.0f);
	const float BrakingSpeedLimit = ComputeBrakingSpeedLimit(RemainingForBraking);
	const float TargetSpeed = FMath::Min(Vc, BrakingSpeedLimit);
	if (TargetSpeed >= CurrentSpeedCmPerSec)
	{
		return FMath::Min(CurrentSpeedCmPerSec + A * DeltaSeconds, TargetSpeed);
	}
	return FMath::Max(CurrentSpeedCmPerSec - D * DeltaSeconds, TargetSpeed);
}

float FAircraftTrajectoryGenerator::ComputeBrakingDistance(float StartSpeedCmPerSec, float EndSpeedCmPerSec) const
{
	const float StartSpeed = FMath::Max(StartSpeedCmPerSec, 0.0f);
	const float EndSpeed = FMath::Clamp(EndSpeedCmPerSec, 0.0f, StartSpeed);
	const float DeltaSpeed = StartSpeed - EndSpeed;
	const float Deceleration = FMath::Max(PlanningDecelCmPerSecSq, UE_SMALL_NUMBER);
	if (DeltaSpeed <= UE_SMALL_NUMBER)
	{
		return 0.0f;
	}
	if (PlanningJerkCmPerSecCubed <= UE_SMALL_NUMBER)
	{
		return (StartSpeed * StartSpeed - EndSpeed * EndSpeed)
			/ (2.0f * Deceleration);
	}

	const float Jerk = PlanningJerkCmPerSecCubed;
	const float SpeedChangeInRamps = Deceleration * Deceleration / Jerk;
	float TotalBrakingTime = 0.0f;
	if (DeltaSpeed >= SpeedChangeInRamps)
	{
		TotalBrakingTime = 2.0f * Deceleration / Jerk
			+ (DeltaSpeed - SpeedChangeInRamps) / Deceleration;
	}
	else
	{
		TotalBrakingTime = 2.0f * FMath::Sqrt(DeltaSpeed / Jerk);
	}
	return 0.5f * (StartSpeed + EndSpeed) * TotalBrakingTime;
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
	if (Segments.Num() == 0)
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
	OutSegIndex = FMath::Clamp(Lo, 0, Segments.Num() - 1);

	if (!Segments[OutSegIndex].IsValid())
	{
		for (int32 i = OutSegIndex + 1; i < Segments.Num(); ++i)
		{
			if (Segments[i].IsValid())
			{
				OutSegIndex = i;
				break;
			}
		}
		if (!Segments[OutSegIndex].IsValid())
		{
			return;
		}
	}

	OutLocalArc = FMath::Clamp(GlobalArc - CumStartArc[OutSegIndex], 0.0f, Segments[OutSegIndex]->GetTotalArcLengthCm());
}

FTrajectoryPoint FAircraftTrajectoryGenerator::SampleGlobalArcLength(float GlobalArc, float Speed) const
{
	FTrajectoryPoint Point;
	if (!bIsValid || Segments.Num() == 0)
	{
		Point.bValid = false;
		return Point;
	}

	int32 SegIndex;
	float LocalArc;
	LocateSegment(GlobalArc, SegIndex, LocalArc);
	if (!Segments.IsValidIndex(SegIndex))
	{
		Point.bValid = false;
		return Point;
	}

	Point = Segments[SegIndex]->SampleAtArcLength(LocalArc, Speed);
	Point.ArcLengthCm = GlobalArc;
	return Point;
}

float FAircraftTrajectoryGenerator::ProjectToArcLength(const FVector& WorldPosition) const
{
	if (!bIsValid || Segments.Num() == 0)
	{
		return 0.0f;
	}

	float BestS = CurrentArcLength;
	float BestDistSq = TNumericLimits<float>::Max();

	// 无限循环段（Orbit）：限制在当前游标附近 ±单圈 局部窗口搜索
	const bool bInfinite = IsCurrentSegmentInfiniteLoop();
	float SearchWindowMin = 0.0f;
	float SearchWindowMax = TNumericLimits<float>::Max();
	if (bInfinite)
	{
		int32 CurSegIdx;
		float CurLocalArc;
		LocateSegment(CurrentArcLength, CurSegIdx, CurLocalArc);
		if (Segments.IsValidIndex(CurSegIdx) && Segments[CurSegIdx].IsValid())
		{
			const float OneLap = Segments[CurSegIdx]->GetTotalArcLengthCm();
			SearchWindowMin = FMath::Max(CurrentArcLength - OneLap, 0.0f);
			SearchWindowMax = CurrentArcLength + OneLap;
		}
	}

	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		const FAircraftTrajectorySegment* Seg = Segments[i].Get();
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

		const int32 ProjectionSamples = Seg->UsesNativeTimeParameterization() ? 128 : 8;
		const float Step = FMath::Max(SegLen / ProjectionSamples, 1.0f);
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
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestS = GlobalS;
			}
		}
	}

	return BestS;
}

FTrajectoryPoint FAircraftTrajectoryGenerator::SampleAtGlobalArc(float GlobalArc, float Speed) const
{
	FTrajectoryPoint Point;
	if (!bIsValid || Segments.Num() == 0)
	{
		Point.bValid = false;
		return Point;
	}

	// 无限循环段（Orbit）：不 clamp 上界，段内部 Fmod 绕回单圈
	if (IsCurrentSegmentInfiniteLoop())
	{
		const float EffectiveArc = FMath::Max(GlobalArc, 0.0f);
		int32 CurSegIdx;
		float CurLocalArc;
		LocateSegment(CurrentArcLength, CurSegIdx, CurLocalArc);
		if (Segments.IsValidIndex(CurSegIdx) && Segments[CurSegIdx].IsValid())
		{
			const float SegStart = CumStartArc.IsValidIndex(CurSegIdx) ? CumStartArc[CurSegIdx] : 0.0f;
			const float LocalArc = EffectiveArc - SegStart;
			FTrajectoryPoint P = Segments[CurSegIdx]->SampleAtArcLength(LocalArc, Speed);
			P.ArcLengthCm = EffectiveArc;
			return P;
		}
	}

	const float ClampedArc = FMath::Clamp(GlobalArc, 0.0f, TotalArcLengthCm);
	return SampleGlobalArcLength(ClampedArc, Speed);
}
