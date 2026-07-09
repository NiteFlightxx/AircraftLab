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

	FString Error;
	if (!BuildSegments(Request, Error))
	{
		UE_LOG(LogTrajectoryGen, Warning, TEXT("TrajectoryGenerator: build failed — %s"), *Error);
		bIsValid = false;
		return false;
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
	// 目标终点速度取 TargetVelocityCmPerSec 的水平幅值（默认 0 = 停在终点）
	TargetEndSpeedCmPerSec = FMath::Max(FVector2D(Request.TargetVelocityCmPerSec.X, Request.TargetVelocityCmPerSec.Y).Size(), 0.0f);
	AcceptanceRadiusCm = FMath::Max(Request.AcceptanceRadiusCm, 1.0f);

	// 减速触发距离 s_dec = (Vc² − V_end²)/(2a)
	DecelTriggerDistanceCm = FMath::Max(
		(CruiseSpeedCmPerSec * CruiseSpeedCmPerSec - TargetEndSpeedCmPerSec * TargetEndSpeedCmPerSec) / (2.0f * PlanningAccelCmPerSecSq),
		0.0f);

	CurrentArcLength = 0.0f;
	CurrentSpeedCmPerSec = 0.0f;
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
	return CurrentArcLength + AcceptanceRadiusCm >= TotalArcLengthCm;
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

	// --- 1. 用当前位置投影到轨迹，校正游标（防漂移） ---
	const float ProjectedS = ProjectToArcLength(CurrentPosition);
	// 取投影点与当前游标的较大者，防止因投影滞后导致游标倒退
	CurrentArcLength = FMath::Max(CurrentArcLength, ProjectedS);

	// --- 2. 梯形速度剖面：根据剩余距离决定本周期速度 ---
	const float RemainingDistance = FMath::Max(TotalArcLengthCm - CurrentArcLength, 0.0f);
	CurrentSpeedCmPerSec = ComputeTrapezoidalSpeed(CurrentArcLength, TotalArcLengthCm, CurrentSpeedCmPerSec);

	// --- 3. 推进游标：s += v·Δt ---
	CurrentArcLength = FMath::Clamp(CurrentArcLength + CurrentSpeedCmPerSec * DeltaSeconds, 0.0f, TotalArcLengthCm);

	// --- 4. 完成判定 ---
	if (RemainingDistance <= AcceptanceRadiusCm)
	{
		// 到点：输出终点的零速设定值
		OutSetpoint = SampleGlobalArcLength(TotalArcLengthCm, TargetEndSpeedCmPerSec);
		CurrentSpeedCmPerSec = TargetEndSpeedCmPerSec;
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
		if (!Seg) continue;
		Cum += Seg->GetTotalArcLengthCm();
		CumStartArc.Add(Cum);
	}
	TotalArcLengthCm = Cum;
}

// ---------------------------------------------------------------------------
// ComputeTrapezoidalSpeed —— 梯形/三角形速度剖面
// ---------------------------------------------------------------------------
float UTrajectoryGenerator::ComputeTrapezoidalSpeed(float CurrentS, float TotalS, float V0) const
{
	const float Vc = CruiseSpeedCmPerSec;
	const float A = PlanningAccelCmPerSecSq;
	const float VEnd = TargetEndSpeedCmPerSec;
	const float Remaining = FMath::Max(TotalS - CurrentS, 0.0f);

	// 加速段距离 s_acc = (Vc² − V0²)/(2a)
	const float SAcc = (Vc * Vc - V0 * V0) / (2.0f * A);
	// 减速段距离 s_dec = (Vc² − V_end²)/(2a)（已缓存为 DecelTriggerDistanceCm）
	const float SDec = DecelTriggerDistanceCm;

	// 三角形退化：总距离不足以既加速到 Vc 又减速到 V_end
	// 退化为峰值速度 V_peak = sqrt(2·a·L/(1 + (a/(a))))，此处用简化：
	// 取 V_peak = min(Vc, sqrt(2·a·(L − s_acc_min)))
	bool bTriangle = (SAcc + SDec) > TotalS;
	float EffectiveVc = Vc;
	if (bTriangle)
	{
		// 退化为三角形：峰值速度由剩余对称距离反解
		// 此处用当前剩余距离粗略估计，保证减速触发早于冲过终点
		EffectiveVc = FMath::Sqrt(2.0f * A * Remaining * 0.5f) + VEnd * 0.5f;
		EffectiveVc = FMath::Min(EffectiveVc, Vc);
	}

	// 减速段优先：距终点 < s_dec → 减速
	const float DecelStartS = FMath::Max(TotalS - SDec, 0.0f);
	if (CurrentS >= DecelStartS)
	{
		// v = sqrt(V_end² + 2·a·(Remaining))，但不超过 EffectiveVc
		float VDecel = FMath::Sqrt(VEnd * VEnd + 2.0f * A * Remaining);
		return FMath::Min(VDecel, EffectiveVc);
	}

	// 加速段：v = V0 + a·Δt ≈ 用 sqrt(V0² + 2·a·s_acc_progress) 估算
	if (CurrentS < SAcc)
	{
		float VAccel = FMath::Sqrt(V0 * V0 + 2.0f * A * CurrentS);
		return FMath::Min(VAccel, EffectiveVc);
	}

	// 巡航段
	return EffectiveVc;
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

	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		UTrajectorySegment* Seg = Segments[i];
		if (!Seg) continue;
		const float SegLen = Seg->GetTotalArcLengthCm();
		const float Step = FMath::Max(SegLen / 8.0f, 1.0f); // 每段 8 个采样点
		for (float LocalS = 0.0f; LocalS <= SegLen; LocalS += Step)
		{
			const FFrenetFrame Frame = Seg->GetFrenetAtArcLength(LocalS);
			const float DistSq = FVector::DistSquared(Frame.OriginCm, WorldPosition);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestS = CumStartArc[i] + LocalS;
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
	// 复用内部 SampleGlobalArcLength，弧长 clamp 到 [0, TotalArcLength]
	const float ClampedArc = FMath::Clamp(GlobalArc, 0.0f, TotalArcLengthCm);
	return SampleGlobalArcLength(ClampedArc, Speed);
}
