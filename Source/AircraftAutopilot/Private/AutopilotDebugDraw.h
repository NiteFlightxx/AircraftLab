// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class UWorld;
class UTrajectoryGenerator;
struct FTrajectoryPoint;
struct FGuidanceCommand;

/**
 * Autopilot 调试绘制与诊断工具集
 *
 * 参考 UE PoseSearchHistory 的调试绘制模式（PoseSearchHistory.cpp）：通过控制台命令
 * （CVar，FAutoConsoleVariableRef）开关控制各类可视化元素的启停，所有绘制在
 * AutopilotComponent Tick 末尾统一调用，每物理帧重绘。
 *
 * 控制台命令（编辑器 ~ 控制台输入，类似 a.AnimNode.PoseHistory.DebugDrawTrajectory）：
 *   Autopilot.DebugDrawTrajectory 1/0     —— 绘制完整轨迹路径（已走过=绿，未走=青）
 *   Autopilot.DebugDrawSetpoint 1/0       —— 绘制当前设定点（黄球）+ 航向箭头
 *   Autopilot.DebugDrawLookAhead 1/0      —— 绘制前瞻点（紫）+ 横向误差线（红虚线）
 *   Autopilot.DebugDrawVelocity 1/0       —— 绘制期望速度向量（白箭头）
 *   Autopilot.DebugDrawThickness <float>  —— 线条粗细（默认 0）
 *   Autopilot.DebugDrawMaxSamples <int>   —— 轨迹路径最大采样点数（默认 256）
 *
 * 颜色约定：
 *   蓝点  —— 轨迹起点
 *   红点  —— 轨迹终点
 *   绿线  —— 已走过的轨迹段
 *   青线  —— 未走过的轨迹段
 *   白点  —— 当前位置在轨迹上的最近投影点
 *   黄球  —— 当前设定点
 *   黄箭头 —— 设定点航向
 *   紫点  —— 前瞻点（PurePursuit 等）
 *   红虚线 —— 横向误差（当前位置 → 最近投影点）
 *   白箭头 —— 期望速度向量
 *
 * 线程安全：仅游戏线程调用（AutopilotComponent Tick 在 TG_PrePhysics）。
 * 性能：全部受 ENABLE_DRAW_DEBUG 保护，Shipping/无 DrawDebug 的构建自动剔除为零开销。
 *
 * 扩展：本文件集中所有调试/绘制能力。后续新增 debug 功能（如速度剖面曲线、
 *   PID 误差示波、行为状态机可视化）在此结构体追加静态方法即可。
 */
struct AIRCRAFTAUTOPILOT_API FAutopilotDebugDraw
{
	// -----------------------------------------------------------------------
	// CVar 开关查询（!ENABLE_DRAW_DEBUG 构建恒返回 false）
	// -----------------------------------------------------------------------

	/** 是否绘制完整轨迹路径 */
	static bool IsDrawTrajectoryEnabled();
	/** 是否绘制当前设定点 + 航向箭头 */
	static bool IsDrawSetpointEnabled();
	/** 是否绘制前瞻点 + 横向误差 */
	static bool IsDrawLookAheadEnabled();
	/** 是否绘制期望速度向量 */
	static bool IsDrawVelocityEnabled();
	/** 线条粗细 */
	static float GetThickness();
	/** 轨迹路径最大采样点数 */
	static int32 GetMaxSamples();

	// -----------------------------------------------------------------------
	// 单项绘制
	// -----------------------------------------------------------------------

	/**
	 * 绘制完整轨迹路径：沿弧长 [0, TotalArc] 离散采样并连线。
	 * 已走过段（s <= CurrentArc）画绿色，未走段画青色；标记起点/终点/当前投影点。
	 * 悬停零弧长轨迹不画路径（仅设定点）。
	 */
	static void DrawTrajectory(UWorld* World, const UTrajectoryGenerator* TrajGen, const FVector& CurrentPositionCm);

	/**
	 * 绘制当前设定点：黄色球 + 沿 YawDegrees 方向的航向箭头。
	 */
	static void DrawSetpoint(UWorld* World, const FTrajectoryPoint& Setpoint);

	/**
	 * 绘制前瞻点（紫色）+ 横向误差红虚线（当前位置 → 最近投影点）。
	 */
	static void DrawLookAhead(UWorld* World, const FGuidanceCommand& GuidanceCmd, const FVector& CurrentPositionCm, const UTrajectoryGenerator* TrajGen);

	/**
	 * 绘制期望速度向量：从设定点位置出发的白色箭头，长度=速度幅值（cm/s 直接映射 cm）。
	 */
	static void DrawVelocityVector(UWorld* World, const FTrajectoryPoint& Setpoint);

	// -----------------------------------------------------------------------
	// 便捷入口：根据 CVar 开关一次性绘制全部（AutopilotComponent Tick 末尾调用）
	// -----------------------------------------------------------------------
	static void DrawAll(
		UWorld* World,
		const UTrajectoryGenerator* TrajGen,
		const FTrajectoryPoint& CurrentSetpoint,
		const FGuidanceCommand& GuidanceCmd,
		const FVector& CurrentPositionCm);

private:
	// -----------------------------------------------------------------------
	// 内部绘制工具
	// -----------------------------------------------------------------------

	/**
	 * 绘制虚线（用交替短段模拟，引擎无 DrawDebugDashedLine 时的自实现）。
	 * @param World       世界
	 * @param Start       起点（世界系 cm）
	 * @param End         终点（世界系 cm）
	 * @param Color       颜色
	 * @param Thickness   线条粗细
	 * @param DashLenCm   实线段长度（cm，默认 12）
	 * @param GapLenCm    间隔段长度（cm，默认 8）
	 */
	static void DrawDashedLine(UWorld* World, const FVector& Start, const FVector& End, const FColor& Color, float Thickness, float DashLenCm = 12.0f, float GapLenCm = 8.0f);
};
