// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "MissionTypes.generated.h"

/**
 * Mission Layer（任务层）数据类型
 *
 * 第八部分：把"任务"从 Behavior 无法表达的"多行为序列"提升为显式任务对象。
 *
 * 架构定位（12 层最顶层）：
 *   Mission Layer（任务序列，每个任务项 = 一个 Behavior 指令）
 *     → Behavior Layer（状态机执行单个行为）
 *       → Trajectory → Motion Profile → 控制器金字塔
 *
 * Mission vs Behavior 的区别：
 *   - Behavior 回答"现在做哪个动作"（单步）
 *   - Mission 回答"按什么顺序完成目标"（多步序列）
 *   例如"巡检任务" = TakeOff → FollowPath(航点1→2→3) → Orbit(重点) → FollowPath(返航) → Land
 *   Mission 只管"切到哪个 Behavior"，不关心 Behavior 内部如何实现。
 *
 * 严格单向依赖：Mission 只调用 BehaviorPlanner 的 Command* 接口，
 *   绝不直接构造轨迹、绝不直接驱动控制器、绝不写物理状态。
 */

/** 任务项类型 */
UENUM(BlueprintType)
enum class EMissionItemType : uint8
{
	/** 起飞 */
	TakeOff UMETA(DisplayName = "Take Off"),
	/** 飞到航点 */
	Waypoint UMETA(DisplayName = "Waypoint"),
	/** 沿路径飞行（多航点） */
	Path UMETA(DisplayName = "Path"),
	/** 环绕点 */
	Orbit UMETA(DisplayName = "Orbit"),
	/** 悬停等待（延时） */
	Loiter UMETA(DisplayName = "Loiter"),
	/** 返航 */
	ReturnHome UMETA(DisplayName = "Return Home"),
	/** 降落 */
	Land UMETA(DisplayName = "Land")
};

/**
 * 单个任务项（Mission Item）
 *
 * 一个任务项 = 一个 Behavior 指令 + 完成条件。
 * Mission 按序执行，前一项完成才进下一项。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTAUTOPILOT_API FMissionItem
{
	GENERATED_BODY()

	/** 任务项类型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission")
	EMissionItemType Type = EMissionItemType::Waypoint;

	/** 目标位置（cm，世界系）—— Waypoint/Orbit/TakeOff 用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission")
	FVector TargetPositionCm = FVector::ZeroVector;

	/** 目标航向（°） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission")
	float TargetYawDegrees = 0.0f;

	/** 路径点串（cm）—— Path 类型用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission")
	TArray<FVector> PathPointsCm;

	/** 环绕半径（cm）—— Orbit 用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float OrbitRadiusCm = 500.0f;

	/** 环绕角速度（°/s）—— Orbit 用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission")
	float OrbitAngularRateDegPerSec = 45.0f;

	/** 巡航速度（cm/s） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float CruiseSpeedCmPerSec = 800.0f;

	/** 起飞高度（cm）—— TakeOff 用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float TakeOffAltitudeCm = 1000.0f;

	/** 悬停时长（s）—— Loiter 用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float LoiterDurationSeconds = 5.0f;

	/** 返航高度（cm）—— ReturnHome 用 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float ReturnAltitudeCm = 2000.0f;

	/**
	 * 航点到达水平容差（cm）—— 第 7 批：航点判定 XY/Z 分离。
	 * Waypoint/Path/ReturnHome 完成判定：水平距离² < max(AcceptanceRadiusCm, 50)²
	 *                                       且 垂直距离 < AcceptanceRadiusZCm。
	 * 默认 50cm（沿用原 100cm³ 球体判定的体感下限）。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusCm = 50.0f;

	/**
	 * 航点到达垂直容差（cm）—— 第 7 批：与水平容差分离。
	 * 垂直精度通常比水平要求更宽（高度保持误差大），默认 100cm。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Mission", meta = (ClampMin = "0.0"))
	float AcceptanceRadiusZCm = 100.0f;
};

/** 任务执行状态 */
UENUM(BlueprintType)
enum class EMissionStatus : uint8
{
	/** 未开始 */
	Pending UMETA(DisplayName = "Pending"),
	/** 执行中 */
	Running UMETA(DisplayName = "Running"),
	/** 已完成 */
	Completed UMETA(DisplayName = "Completed"),
	/** 已中止 */
	Aborted UMETA(DisplayName = "Aborted")
};
