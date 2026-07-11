// Copyright Epic Games, Inc. All Rights Reserved.

#include "Trajectory/TrajectoryGenerator.h"

#include "Trajectory/TrajectorySegment.h"
#include "Trajectory/LineTrajectorySegment.h"
#include "Trajectory/BezierTrajectorySegment.h"
#include "Trajectory/CircleTrajectorySegment.h"
#include "Trajectory/OrbitTrajectorySegment.h"
#include "Trajectory/MinSnapTrajectorySegment.h"

DEFINE_LOG_CATEGORY_STATIC(LogTrajectoryGen, Log, All);

UTrajectoryGenerator::UTrajectoryGenerator()
{
}

// ---------------------------------------------------------------------------
// SetRequest —— 入口：依据类型构造段、重置游标、计算剖面
// ---------------------------------------------------------------------------
bool UTrajectoryGenerator::SetRequest(const FTrajectoryRequest& Request)
{
	Clear();

	// 悬停退化：Start≈Target（零长度 Waypoint/Line）时无需构造段，
	// 直接缓存目标位置为驻留设定值。Hover 状态每帧生成"原地悬停"请求，
	// 若走 BuildSegments 会在 LineSegment 的 coincident 检查处失败并刷屏。
	// 此处提前拦截，使 bIsValid=true，下游 UpdateSetpoint 输出静止设定值。
	const float StartToTargetDist = FVector::Dist(Request.StartPositionCm, Request.TargetPositionCm);
	const bool bHoverRequest = (Request.Type == ETrajectoryType::Waypoint || Request.Type == ETrajectoryType::Line)
		&& StartToTargetDist <= 1.0f; // 1cm 容差（悬停微动）

	if (!bHoverRequest && (Request.PlanningAccelerationCmPerSecSq <= UE_SMALL_NUMBER
		|| Request.PlanningDecelerationCmPerSecSq <= UE_SMALL_NUMBER))
	{
		UE_LOG(LogTrajectoryGen, Warning,
			TEXT("TrajectoryGenerator: acceleration and deceleration must both be positive."));
		return false;
	}
	const float RequestedTerminalSpeed = FVector2D(
		Request.TargetVelocityCmPerSec.X, Request.TargetVelocityCmPerSec.Y).Size();
	if (!bHoverRequest && RequestedTerminalSpeed > Request.CruiseSpeedCmPerSec + UE_SMALL_NUMBER)
	{
		UE_LOG(LogTrajectoryGen, Warning,
			TEXT("TrajectoryGenerator: terminal speed cannot exceed cruise speed."));
		return false;
	}

	FString Error;
	if (!bHoverRequest && !BuildSegments(Request, Error))
	{
		UE_LOG(LogTrajectoryGen, Warning, TEXT("TrajectoryGenerator: build failed — %s"), *Error);
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
		InitialSpeedCmPerSec = 0.0f;
		TargetEndSpeedCmPerSec = 0.0f;
		AcceptanceRadiusCm = FMath::Max(Request.AcceptanceRadiusCm, 1.0f);
		DecelTriggerDistanceCm = 0.0f;
		CurrentArcLength = 0.0f;
		CurrentSpeedCmPerSec = 0.0f;
		bIsValid = true;
		// 预置驻留设定值（UpdateSetpoint 会复用）
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
		UE_LOG(LogTrajectoryGen, Warning, TEXT("TrajectoryGenerator: zero total arc length."));
		bIsValid = false;
		return false;
	}

	// 缓存速度剖面参数
	CruiseSpeedCmPerSec = FMath::Max(Request.CruiseSpeedCmPerSec, UE_SMALL_NUMBER);
	PlanningAccelCmPerSecSq = FMath::Max(Request.PlanningAccelerationCmPerSecSq, UE_SMALL_NUMBER);
	PlanningDecelCmPerSecSq = FMath::Max(Request.PlanningDecelerationCmPerSecSq, UE_SMALL_NUMBER);
	// 目标终点速度取 TargetVelocityCmPerSec 的水平幅值（默认 0 = 停在终点）
	TargetEndSpeedCmPerSec = FMath::Clamp(
		FVector2D(Request.TargetVelocityCmPerSec.X, Request.TargetVelocityCmPerSec.Y).Size(),
		0.0f, CruiseSpeedCmPerSec);
	AcceptanceRadiusCm = FMath::Max(Request.AcceptanceRadiusCm, 1.0f);

	const FVector StartTangent = Segments.Num() > 0 && Segments[0]
		? Segments[0]->GetFrenetAtArcLength(0.0f).Tangent.GetSafeNormal()
		: FVector::ZeroVector;
	InitialSpeedCmPerSec = StartTangent.IsNearlyZero()
		? 0.0f
		: FMath::Clamp(FVector::DotProduct(Request.StartVelocityCmPerSec, StartTangent), 0.0f, CruiseSpeedCmPerSec);

	// 减速触发距离 s_dec = (Vc² − V_end²)/(2a)
	DecelTriggerDistanceCm = FMath::Max(
		(CruiseSpeedCmPerSec * CruiseSpeedCmPerSec - TargetEndSpeedCmPerSec * TargetEndSpeedCmPerSec) / (2.0f * PlanningDecelCmPerSecSq),
		0.0f);

	CurrentArcLength = 0.0f;
	CurrentSpeedCmPerSec = InitialSpeedCmPerSec;
	bIsValid = true;

	UE_LOG(LogTrajectoryGen, Log,
		TEXT("TrajectoryGenerator: built %d segments, L=%.1fcm, Vc=%.1f, a=%.1f, V_end=%.1f, s_dec=%.1f"),
		Segments.Num(), TotalArcLengthCm, CruiseSpeedCmPerSec, PlanningAccelCmPerSecSq,
		TargetEndSpeedCmPerSec, DecelTriggerDistanceCm);
	return true;
}

void UTrajectoryGenerator::Clear()
{
	Segments.Reset();
	CumStartArc.Reset();
	TotalArcLengthCm = 0.0f;
	CurrentArcLength = 0.0f;
	CurrentSpeedCmPerSec = 0.0f;
	CurrentSetpoint.Reset();
	bIsValid = false;
}

bool UTrajectoryGenerator::IsComplete() const
{
	if (!bIsValid) return true;
	// 无限循环段（如 Orbit 持续盘旋）永不自动完成
	if (IsCurrentSegmentInfiniteLoop()) return false;
	return CurrentArcLength + UE_SMALL_NUMBER >= TotalArcLengthCm
		&& FMath::Abs(CurrentSpeedCmPerSec - TargetEndSpeedCmPerSec) <= 1.0f;
}

bool UTrajectoryGenerator::IsCurrentSegmentInfiniteLoop() const
{
	if (!bIsValid || Segments.Num() == 0) return false;
	int32 SegIndex;
	float LocalArc;
	LocateSegment(CurrentArcLength, SegIndex, LocalArc);
	if (Segments.IsValidIndex(SegIndex) && Segments[SegIndex])
	{
		return Segments[SegIndex]->IsInfiniteLoop();
	}
	return false;
}

float UTrajectoryGenerator::GetProgress() const
{
	if (!bIsValid || TotalArcLengthCm <= UE_SMALL_NUMBER) return 0.0f;
	return FMath::Clamp(CurrentArcLength / TotalArcLengthCm, 0.0f, 1.0f);
}

float UTrajectoryGenerator::GetTotalArcLength() const
{
	return TotalArcLengthCm;
}

// ---------------------------------------------------------------------------
// UpdateSetpoint —— 每周期推进游标、算速度、采样、输出设定值
// ---------------------------------------------------------------------------
bool UTrajectoryGenerator::UpdateSetpoint(float DeltaSeconds, const FVector& CurrentPosition, const FVector& CurrentVelocity, FTrajectoryPoint& OutSetpoint)
{
	OutSetpoint.Reset();
	if (!bIsValid || DeltaSeconds <= UE_SMALL_NUMBER)
	{
		return false;
	}

	// 悬停退化态（零弧长）：直接输出驻留设定值（目标位置静止），不推进游标
	if (TotalArcLengthCm <= UE_SMALL_NUMBER)
	{
		OutSetpoint = CurrentSetpoint;
		return true;
	}

	const bool bInfinite = IsCurrentSegmentInfiniteLoop();

	// --- 1. 用当前位置投影到轨迹，校正游标（防漂移） ---
	const float ProjectedS = ProjectToArcLength(CurrentPosition);
	// 取投影点与当前游标的较大者，防止因投影滞后导致游标倒退
	CurrentArcLength = FMath::Max(CurrentArcLength, ProjectedS);

	if (bInfinite)
	{
		// --- 无限循环段（Orbit 持续盘旋）：恒定巡航速，游标不 clamp、不触发完成 ---
		CurrentSpeedCmPerSec = CruiseSpeedCmPerSec;
		CurrentArcLength += CurrentSpeedCmPerSec * DeltaSeconds;

		// 采样设定值（含 LookAhead），弧长取模一圈以支持环绕段
		const float SampleArc = bUseLookAhead
			? CurrentArcLength + LookAheadDistanceCm
			: CurrentArcLength;
		OutSetpoint = SampleAtGlobalArc(SampleArc, CurrentSpeedCmPerSec);
		CurrentSetpoint = OutSetpoint;
		return true;
	}

	// --- 2. 梯形速度剖面：根据剩余距离决定本周期速度 ---
	const float PreviousSpeedCmPerSec = CurrentSpeedCmPerSec;
	CurrentSpeedCmPerSec = ComputeTrapezoidalSpeed(CurrentArcLength, TotalArcLengthCm, DeltaSeconds);

	// --- 3. 推进游标：s += v·Δt ---
	const float IntegratedDistance = 0.5f * (PreviousSpeedCmPerSec + CurrentSpeedCmPerSec) * DeltaSeconds;
	CurrentArcLength = FMath::Clamp(CurrentArcLength + IntegratedDistance, 0.0f, TotalArcLengthCm);

	// --- 4. 完成判定 ---
	const float RemainingDistance = FMath::Max(TotalArcLengthCm - CurrentArcLength, 0.0f);
	if (RemainingDistance <= UE_SMALL_NUMBER)
	{
		// At the endpoint, preserve the configured deceleration instead of
		// discontinuously snapping velocity to the requested terminal speed.
		OutSetpoint = SampleGlobalArcLength(TotalArcLengthCm, CurrentSpeedCmPerSec);
		CurrentSetpoint = OutSetpoint;
		return true;
	}

	// --- 5. 采样设定值（含 LookAhead） ---
	const float SampleArc = bUseLookAhead
		? FMath::Clamp(CurrentArcLength + LookAheadDistanceCm, 0.0f, TotalArcLengthCm)
		: CurrentArcLength;
	OutSetpoint = SampleGlobalArcLength(SampleArc, CurrentSpeedCmPerSec);
	CurrentSetpoint = OutSetpoint;
	return true;
}

// ---------------------------------------------------------------------------
// BuildSegments —— 按类型分发构造
// ---------------------------------------------------------------------------
bool UTrajectoryGenerator::BuildSegments(const FTrajectoryRequest& Request, FString& OutError)
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
			// 单段类型：构造对应段对象
			UTrajectorySegment* Seg = nullptr;
			switch (Request.Type)
			{
			case ETrajectoryType::Line:       Seg = NewObject<ULineTrajectorySegment>(this); break;
			case ETrajectoryType::Bezier:      Seg = NewObject<UBezierTrajectorySegment>(this); break;
			case ETrajectoryType::Circle:      Seg = NewObject<UCircleTrajectorySegment>(this); break;
			case ETrajectoryType::Orbit:       Seg = NewObject<UOrbitTrajectorySegment>(this); break;
			case ETrajectoryType::MinimumSnap: Seg = NewObject<UMinSnapTrajectorySegment>(this); break;
			default: break;
			}
			if (!Seg) { OutError = TEXT("Unknown segment type."); return false; }
			if (!Seg->BuildSegment(Request, OutError)) { return false; }
			Segments.Add(Seg);
			return true;
		}
	}
	return false;
}

bool UTrajectoryGenerator::BuildWaypointSegment(const FTrajectoryRequest& Request, FString& OutError)
{
	ULineTrajectorySegment* Seg = NewObject<ULineTrajectorySegment>(this);
	if (!Seg->BuildSegment(Request, OutError)) return false;
	Segments.Add(Seg);
	return true;
}

bool UTrajectoryGenerator::BuildFollowPathSegments(const FTrajectoryRequest& Request, FString& OutError)
{
	// FollowPath：PathPointsCm 为折线点串（含起止），展开为 N 个 Line 段
	const TArray<FVector>& Points = Request.PathPointsCm;
	if (Points.Num() < 2)
	{
		OutError = TEXT("FollowPath: PathPointsCm needs >= 2 points.");
		return false;
	}

	// 若起点与首个路径点不重合，自动插入一段连接当前位置 → 首个路径点
	FVector Prev = Request.StartPositionCm;
	const bool bNeedConnectToFirst = FVector::Dist(Prev, Points[0]) > Request.AcceptanceRadiusCm * 0.5f;

	if (bNeedConnectToFirst)
	{
		FTrajectoryRequest ConnReq = Request;
		ConnReq.Type = ETrajectoryType::Line;
		ConnReq.StartPositionCm = Prev;
		ConnReq.TargetPositionCm = Points[0];
		ULineTrajectorySegment* Conn = NewObject<ULineTrajectorySegment>(this);
		if (!Conn->BuildSegment(ConnReq, OutError)) return false;
		Segments.Add(Conn);
	}

	// 相邻点逐段构造 Line
	for (int32 i = 1; i < Points.Num(); ++i)
	{
		FTrajectoryRequest LegReq = Request;
		LegReq.Type = ETrajectoryType::Line;
		LegReq.StartPositionCm = Points[i - 1];
		LegReq.TargetPositionCm = Points[i];
		ULineTrajectorySegment* Leg = NewObject<ULineTrajectorySegment>(this);
		if (!Leg->BuildSegment(LegReq, OutError)) return false;
		Segments.Add(Leg);
	}
	return true;
}

// ---------------------------------------------------------------------------
// RecomputeArcLengths —— 累积弧长表
// ---------------------------------------------------------------------------
void UTrajectoryGenerator::RecomputeArcLengths()
{
	CumStartArc.Reset();
	CumStartArc.Add(0.0f);
	float Cum = 0.0f;
	for (UTrajectorySegment* Seg : Segments)
	{
		if (Seg)
		{
			Cum += Seg->GetTotalArcLengthCm();
		}
		CumStartArc.Add(Cum); // null 段也占位，保持 CumStartArc[i] 与 Segments[i] 索引对齐
	}
	TotalArcLengthCm = Cum;
}

// ---------------------------------------------------------------------------
// ComputeTrapezoidalSpeed —— 梯形/三角形速度剖面
// ---------------------------------------------------------------------------
// 从静止启动（V_init=0），加速到 Vc，巡航，减速到 VEnd。
// CurrentS = 从轨迹起点算的绝对弧长（加速段从 s=0 开始，增量距离 = CurrentS）。
//
// 冷启动死锁修复：
//   纯公式 v=sqrt(2·a·s) 在 s=0 处 v=0 → 游标推进 0 → 设定点不动 →
//   MotionProfile 位置闭合误差为 0 → 输出速度 0 → 无人机不动 → 投影游标不增长 →
//   死锁，仅靠物理扰动缓慢打破。表现为"下发了 CommandMoveTo 却纹丝不动"。
//   修复：加速段给一个起步保底速度 MinStartSpeed，使游标自推进、设定点前移，
//   位置环产生误差拉动无人机前进。保底速度由加速度推导（sqrt(2·a·dt_planning)），
//   dt_planning 取游戏线程典型帧时（0.02s=50Hz），保证起步即有可观测位移。
// ---------------------------------------------------------------------------
float UTrajectoryGenerator::ComputeTrapezoidalSpeed(float CurrentS, float TotalS, float DeltaSeconds) const
{
	const float Vc = CruiseSpeedCmPerSec;
	const float A = PlanningAccelCmPerSecSq;
	const float VEnd = TargetEndSpeedCmPerSec;
	const float D = PlanningDecelCmPerSecSq;
	if (DeltaSeconds <= UE_SMALL_NUMBER || TotalS <= UE_SMALL_NUMBER) return VEnd;

	const float RemainingForBraking = FMath::Max(TotalS - CurrentS, 0.0f);
	const float BrakingSpeedLimit = FMath::Sqrt(FMath::Max(
		VEnd * VEnd + 2.0f * D * RemainingForBraking, 0.0f));
	const float TargetSpeed = FMath::Min(Vc, BrakingSpeedLimit);
	if (TargetSpeed >= CurrentSpeedCmPerSec)
	{
		return FMath::Min(CurrentSpeedCmPerSec + A * DeltaSeconds, TargetSpeed);
	}
	return FMath::Max(CurrentSpeedCmPerSec - D * DeltaSeconds, TargetSpeed);

}

// ---------------------------------------------------------------------------
// LocateSegment —— 全局弧长 → 段索引 + 段内局部弧长
// ---------------------------------------------------------------------------
void UTrajectoryGenerator::LocateSegment(float GlobalArc, int32& OutSegIndex, float& OutLocalArc) const
{
	OutSegIndex = INDEX_NONE;
	OutLocalArc = 0.0f;
	if (Segments.Num() == 0) return;

	// 二分定位 CumStartArc[i] <= GlobalArc < CumStartArc[i+1]
	int32 Lo = 0;
	int32 Hi = CumStartArc.Num() - 1;
	while (Lo < Hi - 1)
	{
		const int32 Mid = (Lo + Hi) / 2;
		if (CumStartArc[Mid] <= GlobalArc) Lo = Mid;
		else Hi = Mid;
	}
	OutSegIndex = FMath::Clamp(Lo, 0, Segments.Num() - 1);

	// null 段保护：跳到下一个非空段（防御性，正常流程不产生 null 段）
	if (!Segments[OutSegIndex])
	{
		for (int32 i = OutSegIndex + 1; i < Segments.Num(); ++i)
		{
			if (Segments[i])
			{
				OutSegIndex = i;
				break;
			}
		}
		if (!Segments[OutSegIndex]) return; // 全部为 null，放弃
	}

	OutLocalArc = FMath::Clamp(GlobalArc - CumStartArc[OutSegIndex], 0.0f, Segments[OutSegIndex]->GetTotalArcLengthCm());
}

// ---------------------------------------------------------------------------
// SampleGlobalArcLength —— 全局弧长采样（含段间定位）
// ---------------------------------------------------------------------------
FTrajectoryPoint UTrajectoryGenerator::SampleGlobalArcLength(float GlobalArc, float Speed) const
{
	FTrajectoryPoint Point;
	if (!bIsValid || Segments.Num() == 0) { Point.bValid = false; return Point; }

	int32 SegIndex;
	float LocalArc;
	LocateSegment(GlobalArc, SegIndex, LocalArc);
	if (!Segments.IsValidIndex(SegIndex)) { Point.bValid = false; return Point; }

	Point = Segments[SegIndex]->SampleAtArcLength(LocalArc, Speed);
	// 累加全局弧长供下游诊断
	Point.ArcLengthCm = GlobalArc;
	return Point;
}

// ---------------------------------------------------------------------------
// ProjectToArcLength —— 世界位置投影到轨迹最近弧长（基础版，最近点法）
// ---------------------------------------------------------------------------
float UTrajectoryGenerator::ProjectToArcLength(const FVector& WorldPosition) const
{
	if (!bIsValid || Segments.Num() == 0) return 0.0f;

	// 基础投影：沿各段离散采样找最近点。段数少时（Waypoint/Line/FollowPath 折线）
	// 采样开销可控；后续第五部分用 LookAhead 段的解析投影优化。
	float BestS = CurrentArcLength;
	float BestDistSq = TNumericLimits<float>::Max();

	// 无限循环段（Orbit）：限制在当前游标附近 ±OneLap 局部窗口搜索，
	// 避免全局最近点导致游标错误前跳（圆周上处处都有几何最近点）
	const bool bInfinite = IsCurrentSegmentInfiniteLoop();
	float SearchWindowMin = 0.0f;
	float SearchWindowMax = TNumericLimits<float>::Max();
	if (bInfinite)
	{
		// 当前段的单圈弧长作为窗口半宽
		int32 CurSegIdx;
		float CurLocalArc;
		LocateSegment(CurrentArcLength, CurSegIdx, CurLocalArc);
		if (Segments.IsValidIndex(CurSegIdx) && Segments[CurSegIdx])
		{
			const float OneLap = Segments[CurSegIdx]->GetTotalArcLengthCm();
			SearchWindowMin = FMath::Max(CurrentArcLength - OneLap, 0.0f);
			SearchWindowMax = CurrentArcLength + OneLap;
		}
	}

	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		UTrajectorySegment* Seg = Segments[i];
		if (!Seg) continue;
		const float SegStart = CumStartArc[i];
		const float SegLen = Seg->GetTotalArcLengthCm();

		// 无限段：跳过窗口外的段
		if (bInfinite && (SegStart + SegLen < SearchWindowMin || SegStart > SearchWindowMax))
			continue;

		const float Step = FMath::Max(SegLen / 8.0f, 1.0f); // 每段 8 个采样点
		for (float LocalS = 0.0f; LocalS <= SegLen; LocalS += Step)
		{
			const float GlobalS = SegStart + LocalS;
			// 无限段：跳过窗口外的采样点
			if (bInfinite && (GlobalS < SearchWindowMin || GlobalS > SearchWindowMax))
				continue;

			const FFrenetFrame Frame = Seg->GetFrenetAtArcLength(LocalS);
			const float DistSq = FVector::DistSquared(Frame.OriginCm, WorldPosition);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestS = GlobalS;
			}
		}
	}

	// 防止投影点在游标之前（避免倒退），只在显著超前时采纳
	return BestS;
}

// ---------------------------------------------------------------------------
// SampleAtGlobalArc —— 公开采样接口（供 Path Following 制导律取前瞻点）
// ---------------------------------------------------------------------------
FTrajectoryPoint UTrajectoryGenerator::SampleAtGlobalArc(float GlobalArc, float Speed) const
{
	FTrajectoryPoint Point;
	if (!bIsValid || Segments.Num() == 0) { Point.bValid = false; return Point; }

	// 无限循环段（Orbit）：不 clamp 上界。段自身已用 Fmod 绕回单圈，
	// 故前瞻点超过一圈也能正确采样（连续盘旋）。有限轨迹会走到终点后
	// 被 LocateSegment 内的 clamp 截断，无法表达"绕回"，故二者需分别处理。
	if (IsCurrentSegmentInfiniteLoop())
	{
		const float EffectiveArc = FMath::Max(GlobalArc, 0.0f); // 仅 clamp 下界
		int32 CurSegIdx;
		float CurLocalArc;
		LocateSegment(CurrentArcLength, CurSegIdx, CurLocalArc);
		if (Segments.IsValidIndex(CurSegIdx) && Segments[CurSegIdx])
		{
			const float SegStart = CumStartArc.IsValidIndex(CurSegIdx) ? CumStartArc[CurSegIdx] : 0.0f;
			// 段内局部弧长（不 clamp 上界，由段内部 Fmod 绕回单圈）
			const float LocalArc = EffectiveArc - SegStart;
			FTrajectoryPoint P = Segments[CurSegIdx]->SampleAtArcLength(LocalArc, Speed);
			P.ArcLengthCm = EffectiveArc; // 全局弧长供下游诊断
			return P;
		}
	}

	// 有限轨迹：clamp 到 [0, TotalArcLength] 后采样
	const float ClampedArc = FMath::Clamp(GlobalArc, 0.0f, TotalArcLengthCm);
	return SampleGlobalArcLength(ClampedArc, Speed);
}
