// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/TrajectorySegment.h"

#include "LineTrajectorySegment.generated.h"

/**
 * 直线轨迹段
 *
 * 几何：从 StartCm 到 EndCm 的空间直线（可含高度变化，3D）。
 * 弧长 s ∈ [0, |End−Start|]。
 *
 * 运动学：
 *   - 切向 T = (End−Start)/L 恒定
 *   - 曲率 κ = 0 → 无向心加速度、航向恒定
 *   - 速度幅值由生成器注入：Vel = Speed·T
 *   - 几何加速度 = 0（切向加速度由生成器速度剖面另行计算）
 *
 * 用途：Waypoint 直飞、FollowPath 折线的每一段。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API ULineTrajectorySegment : public UTrajectorySegment
{
	GENERATED_BODY()

public:
	ULineTrajectorySegment();

	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

	/** 起点世界坐标（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector StartCm = FVector::ZeroVector;

	/** 终点世界坐标（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector EndCm = FVector::ZeroVector;

	/** 单位切向（世界系） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector Tangent = FVector::ForwardVector;
};
