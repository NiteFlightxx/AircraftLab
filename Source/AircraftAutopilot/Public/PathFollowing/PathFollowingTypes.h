// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "PathFollowingTypes.generated.h"

/**
 * 路径跟踪（Path Following）数据类型
 *
 * 第五部分：把"沿轨迹飞行"从"位置 PID 硬追"升级为"几何制导律"。
 *
 * 在 12 层架构中的位置：
 *   Trajectory Generator（给路径几何 + 名义速度剖面）
 *     → Path Following（制导律：决定"朝哪飞、飞多快"）
 *       → Motion Profile（整形为物理可达设定值）
 *         → 控制器金字塔
 *
 * 制导律 vs 轨迹跟踪的区别：
 *   - 轨迹跟踪（TrajectoryGenerator 本身）：时间参数化，要求 t 时刻在 s(t) 处。
 *   - 路径跟踪（Path Following）：只关心"贴着路径走"，不强求时间对齐，
 *     允许无人机因风/扰动落后于名义进度，但仍贴路径 —— 更鲁棒、更自然。
 *
 * 本模块输出 FGuidanceCommand（期望速度向量 + 期望航向 + 横向误差），
 * 交给 Motion Profile 整形，不直接驱动控制器（严格分层）。
 */

/**
 * 制导律输出
 *
 * 期望速度向量是世界系的"想往哪飞、飞多快"，已含横向修正；
 * 期望航向由制导律决定（跟随路径切向 或 指向前瞻点）。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FGuidanceCommand
{
	GENERATED_BODY()

	/** 期望速度（cm/s，世界系）—— 含横向修正的方向 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	FVector DesiredVelocityCmPerSec = FVector::ZeroVector;

	/** 期望航向（°，世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	float DesiredYawDegrees = 0.0f;

	/** 期望偏航角速度（°/s）—— 制导律可直接给出转弯速率 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	float DesiredYawRateDegPerSec = 0.0f;

	/** 横向误差（cm，带符号：正在路径左侧为正）—— 诊断/限幅用 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	float CrossTrackErrorCm = 0.0f;

	/** 前瞻点世界位置（cm）—— 诊断/可视化用 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	FVector LookAheadPointCm = FVector::ZeroVector;

	/** 是否有效 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|PathFollowing")
	bool bValid = false;
};

/** 制导策略枚举 */
UENUM(BlueprintType, meta = (ScriptName = "AutopilotGuidanceStrategy"))
enum class EPathFollowingStrategy : uint8
{
	/** 纯追踪：朝路径上的前瞻点飞 */
	PurePursuit UMETA(DisplayName = "纯追踪"),
	/** 向量场：用切向 + 横向误差反馈构造期望速度场 */
	VectorField UMETA(DisplayName = "向量场"),
	/** 直接跟踪：用轨迹名义设定值（无额外制导修正） */
	Direct UMETA(DisplayName = "直接跟踪")
};
