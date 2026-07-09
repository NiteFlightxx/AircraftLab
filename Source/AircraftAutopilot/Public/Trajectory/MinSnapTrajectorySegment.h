// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/TrajectorySegment.h"

#include "MinSnapTrajectorySegment.generated.h"

/**
 * Minimum Snap 轨迹段（接口预留，尚未实现）
 *
 * 理论：Minimum Snap 最小化加加速度的平方积分（snap = d⁴x/dt⁴），
 *   在给定航点与边界条件（位置/速度/加速度连续）下求最优多项式轨迹，
 *   是动态可行的最平滑轨迹。常用于穿越多个航点的高速飞行。
 *
 * 求解：对每段构造 7 次多项式，建立 QP（二次规划），
 *   约束 = 航点位置 + 段间 P/V/A/Jerk 连续 + 边界条件。
 *
 * 当前状态：仅占位接口，BuildSegment 返回 false 并提示未实现。
 *   计划在第四部分（FeedForward）之后、控制器可消费复杂设定值时实现。
 *
 * 扩展钩子：将来实现时只需补全 BuildSegment / Sample / GetFrenet 三个函数，
 *   不影响 Generator 与下游控制器接口。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API UMinSnapTrajectorySegment : public UTrajectorySegment
{
	GENERATED_BODY()

public:
	UMinSnapTrajectorySegment();

	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

	/** 预留：航点序列（实现后用于构造多项式段） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	TArray<FVector> Waypoints;
};
