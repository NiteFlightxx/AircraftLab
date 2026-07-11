// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutopilotDebugDraw.h"

#include "Trajectory/TrajectoryGenerator.h"
#include "Trajectory/AutopilotTrajectoryTypes.h"
#include "PathFollowing/PathFollowingTypes.h"

#include "DrawDebugHelpers.h"

#if ENABLE_DRAW_DEBUG
// ---------------------------------------------------------------------------
// 控制台命令（CVar）定义 —— 参考 PoseSearchHistory.cpp 的 FAutoConsoleVariableRef 模式。
// 命名前缀 "Autopilot." 与 "a.AnimNode.PoseHistory." 同为点分式对象路径风格。
// ---------------------------------------------------------------------------

static bool GVarAutopilotDebugDrawTrajectory = true;
static FAutoConsoleVariableRef CVarAutopilotDebugDrawTrajectory(
	TEXT("Autopilot.DebugDrawTrajectory"), GVarAutopilotDebugDrawTrajectory,
	TEXT("Enable/Disable Autopilot trajectory path debug draw (0=off, 1=on)."));

static bool GVarAutopilotDebugDrawSetpoint = true;
static FAutoConsoleVariableRef CVarAutopilotDebugDrawSetpoint(
	TEXT("Autopilot.DebugDrawSetpoint"), GVarAutopilotDebugDrawSetpoint,
	TEXT("Enable/Disable Autopilot current setpoint + yaw arrow debug draw (0=off, 1=on)."));

static bool GVarAutopilotDebugDrawLookAhead = true;
static FAutoConsoleVariableRef CVarAutopilotDebugDrawLookAhead(
	TEXT("Autopilot.DebugDrawLookAhead"), GVarAutopilotDebugDrawLookAhead,
	TEXT("Enable/Disable Autopilot look-ahead point + cross-track error debug draw (0=off, 1=on)."));

static bool GVarAutopilotDebugDrawVelocity = true;
static FAutoConsoleVariableRef CVarAutopilotDebugDrawVelocity(
	TEXT("Autopilot.DebugDrawVelocity"), GVarAutopilotDebugDrawVelocity,
	TEXT("Enable/Disable Autopilot desired velocity vector debug draw (0=off, 1=on)."));

static float GVarAutopilotDebugDrawThickness = 1.0f;
static FAutoConsoleVariableRef CVarAutopilotDebugDrawThickness(
	TEXT("Autopilot.DebugDrawThickness"), GVarAutopilotDebugDrawThickness,
	TEXT("Line thickness for Autopilot debug draw (default 0.0)."));

static int32 GVarAutopilotDebugDrawMaxSamples = 256;
static FAutoConsoleVariableRef CVarAutopilotDebugDrawMaxSamples(
	TEXT("Autopilot.DebugDrawMaxSamples"), GVarAutopilotDebugDrawMaxSamples,
	TEXT("Max number of samples for trajectory path debug draw (default 256)."));
#endif // ENABLE_DRAW_DEBUG

// ---------------------------------------------------------------------------
// CVar 查询
// ---------------------------------------------------------------------------
bool FAutopilotDebugDraw::IsDrawTrajectoryEnabled()
{
#if ENABLE_DRAW_DEBUG
	return GVarAutopilotDebugDrawTrajectory;
#else
	return false;
#endif
}

bool FAutopilotDebugDraw::IsDrawSetpointEnabled()
{
#if ENABLE_DRAW_DEBUG
	return GVarAutopilotDebugDrawSetpoint;
#else
	return false;
#endif
}

bool FAutopilotDebugDraw::IsDrawLookAheadEnabled()
{
#if ENABLE_DRAW_DEBUG
	return GVarAutopilotDebugDrawLookAhead;
#else
	return false;
#endif
}

bool FAutopilotDebugDraw::IsDrawVelocityEnabled()
{
#if ENABLE_DRAW_DEBUG
	return GVarAutopilotDebugDrawVelocity;
#else
	return false;
#endif
}

float FAutopilotDebugDraw::GetThickness()
{
#if ENABLE_DRAW_DEBUG
	return GVarAutopilotDebugDrawThickness;
#else
	return 0.0f;
#endif
}

int32 FAutopilotDebugDraw::GetMaxSamples()
{
#if ENABLE_DRAW_DEBUG
	return GVarAutopilotDebugDrawMaxSamples;
#else
	return 0;
#endif
}

// ---------------------------------------------------------------------------
// 单项绘制
// ---------------------------------------------------------------------------

void FAutopilotDebugDraw::DrawTrajectory(UWorld* World, const UTrajectoryGenerator* TrajGen, const FVector& CurrentPositionCm)
{
#if ENABLE_DRAW_DEBUG
	if (!World || !TrajGen || !TrajGen->IsValid()) return;

	const float TotalArc = TrajGen->GetTotalArcLength();
	if (TotalArc <= UE_SMALL_NUMBER) return; // 悬停零弧长：无路径可画

	const int32 MaxSamples = FMath::Max(GVarAutopilotDebugDrawMaxSamples, 2);
	const float Step = TotalArc / static_cast<float>(MaxSamples);
	const float CurrentArc = TrajGen->GetCurrentArcLength();
	const float Thickness = GVarAutopilotDebugDrawThickness;

	// 沿弧长离散采样并连线：已走过段=绿，未走段=青
	FVector PrevPos = FVector::ZeroVector;
	bool bHasPrev = false;
	for (int32 i = 0; i <= MaxSamples; ++i)
	{
		const float S = FMath::Min(static_cast<float>(i) * Step, TotalArc);
		const FTrajectoryPoint P = TrajGen->SampleAtGlobalArc(S, 0.0f);
		if (!P.bValid) { bHasPrev = false; continue; }

		const FVector Pos = P.PositionCm;
		if (bHasPrev)
		{
			const FColor Color = (S <= CurrentArc) ? FColor::Green : FColor::Cyan;
			DrawDebugLine(World, PrevPos, Pos, Color, false, -1.0f, SDPG_Foreground, Thickness);
		}
		PrevPos = Pos;
		bHasPrev = true;
	}

	// 起点（蓝）/终点（红）
	const FTrajectoryPoint StartP = TrajGen->SampleAtGlobalArc(0.0f, 0.0f);
	if (StartP.bValid)
	{
		DrawDebugPoint(World, StartP.PositionCm, 16.0f, FColor::Blue, false, -1.0f, SDPG_Foreground);
	}
	const FTrajectoryPoint EndP = TrajGen->SampleAtGlobalArc(TotalArc, 0.0f);
	if (EndP.bValid)
	{
		DrawDebugPoint(World, EndP.PositionCm, 16.0f, FColor::Red, false, -1.0f, SDPG_Foreground);
	}

	// 当前位置在轨迹上的最近投影点（白）
	const float ProjS = TrajGen->ProjectToArcLength(CurrentPositionCm);
	const FTrajectoryPoint ProjP = TrajGen->SampleAtGlobalArc(ProjS, 0.0f);
	if (ProjP.bValid)
	{
		DrawDebugPoint(World, ProjP.PositionCm, 10.0f, FColor::White, false, -1.0f, SDPG_Foreground);
	}
#endif // ENABLE_DRAW_DEBUG
}

void FAutopilotDebugDraw::DrawSetpoint(UWorld* World, const FTrajectoryPoint& Setpoint)
{
#if ENABLE_DRAW_DEBUG
	if (!World || !Setpoint.bValid) return;

	const float Thickness = GVarAutopilotDebugDrawThickness;
	// 设定点：黄色球
	DrawDebugSphere(World, Setpoint.PositionCm, 16.0f, 12, FColor::Yellow, false, -1.0f, SDPG_Foreground, Thickness);

	// 航向箭头：沿 YawDegrees 方向
	const float YawRad = FMath::DegreesToRadians(Setpoint.YawDegrees);
	const FVector YawDir(FMath::Cos(YawRad), FMath::Sin(YawRad), 0.0f);
	DrawDebugDirectionalArrow(
		World, Setpoint.PositionCm, Setpoint.PositionCm + YawDir * 80.0f, 30.0f,
		FColor::Yellow, false, -1.0f, SDPG_Foreground, Thickness);
#endif // ENABLE_DRAW_DEBUG
}

void FAutopilotDebugDraw::DrawLookAhead(UWorld* World, const FGuidanceCommand& GuidanceCmd, const FVector& CurrentPositionCm, const UTrajectoryGenerator* TrajGen)
{
#if ENABLE_DRAW_DEBUG
	if (!World || !GuidanceCmd.bValid) return;

	const float Thickness = GVarAutopilotDebugDrawThickness;
	// 前瞻点：紫色
	DrawDebugPoint(World, GuidanceCmd.LookAheadPointCm, 14.0f, FColor::Magenta, false, -1.0f, SDPG_Foreground);

	// 横向误差：当前位置 → 最近投影点，红色虚线
	if (TrajGen && TrajGen->IsValid())
	{
		const float ProjS = TrajGen->ProjectToArcLength(CurrentPositionCm);
		const FTrajectoryPoint ProjP = TrajGen->SampleAtGlobalArc(ProjS, 0.0f);
		if (ProjP.bValid)
		{
			DrawDashedLine(World, CurrentPositionCm, ProjP.PositionCm, FColor::Red, Thickness);
		}
	}
#endif // ENABLE_DRAW_DEBUG
}

void FAutopilotDebugDraw::DrawVelocityVector(UWorld* World, const FTrajectoryPoint& Setpoint)
{
#if ENABLE_DRAW_DEBUG
	if (!World || !Setpoint.bValid) return;

	const float Speed = Setpoint.VelocityCmPerSec.Size();
	if (Speed < UE_SMALL_NUMBER) return; // 静止（悬停）不画

	const float Thickness = GVarAutopilotDebugDrawThickness;
	DrawDebugDirectionalArrow(
		World, Setpoint.PositionCm, Setpoint.PositionCm + Setpoint.VelocityCmPerSec, 20.0f,
		FColor::White, false, -1.0f, SDPG_Foreground, Thickness);
#endif // ENABLE_DRAW_DEBUG
}

// ---------------------------------------------------------------------------
// 便捷入口：按开关一次性绘制全部
// ---------------------------------------------------------------------------
void FAutopilotDebugDraw::DrawAll(
	UWorld* World,
	const UTrajectoryGenerator* TrajGen,
	const FTrajectoryPoint& CurrentSetpoint,
	const FGuidanceCommand& GuidanceCmd,
	const FVector& CurrentPositionCm)
{
#if ENABLE_DRAW_DEBUG
	if (!World) return;

	if (GVarAutopilotDebugDrawTrajectory)
	{
		DrawTrajectory(World, TrajGen, CurrentPositionCm);
	}
	if (GVarAutopilotDebugDrawSetpoint)
	{
		DrawSetpoint(World, CurrentSetpoint);
	}
	if (GVarAutopilotDebugDrawLookAhead)
	{
		DrawLookAhead(World, GuidanceCmd, CurrentPositionCm, TrajGen);
	}
	if (GVarAutopilotDebugDrawVelocity)
	{
		DrawVelocityVector(World, CurrentSetpoint);
	}
#endif // ENABLE_DRAW_DEBUG
}

// ---------------------------------------------------------------------------
// 内部绘制工具
// ---------------------------------------------------------------------------
void FAutopilotDebugDraw::DrawDashedLine(UWorld* World, const FVector& Start, const FVector& End, const FColor& Color, float Thickness, float DashLenCm, float GapLenCm)
{
#if ENABLE_DRAW_DEBUG
	if (!World) return;

	const FVector ToEnd = End - Start;
	const float Len = ToEnd.Size();
	if (Len <= UE_SMALL_NUMBER) return;

	const FVector Dir = ToEnd / Len;
	const float SegLen = DashLenCm + GapLenCm;
	float Traveled = 0.0f;
	while (Traveled < Len)
	{
		const float DashEnd = FMath::Min(Traveled + DashLenCm, Len);
		DrawDebugLine(World,
			Start + Dir * Traveled,
			Start + Dir * DashEnd,
			Color, false, -1.0f, SDPG_Foreground, Thickness);
		Traveled += SegLen;
	}
#endif // ENABLE_DRAW_DEBUG
}
