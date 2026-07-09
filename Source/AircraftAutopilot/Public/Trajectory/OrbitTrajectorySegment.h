// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/TrajectorySegment.h"

#include "OrbitTrajectorySegment.generated.h"

/**
 * 环绕（盘旋）轨迹段
 *
 * 几何：以 OrbitCenterCm 为圆心、OrbitRadiusCm 为半径的水平连续圆周，
 *   按 OrbitAngularRateDegPerSec 持续旋转。与 Circle 的区别：
 *   - Circle 是有限弧段（到 EndAngle 终止）。
 *   - Orbit 是连续盘旋：s 超过一圈后环绕回 0 继续，IsComplete 默认返回 false
 *     （由 Behavior 主动取消，而非自动到点结束）。
 *
 * 起始角由 Build 时根据当前位置相对圆心的方位自动计算，保证从当前位置平滑接入圆周。
 *
 * 运动学与 Circle 相同（T/N/κ/向心加速度/偏航率），但方向由角速度符号决定。
 *
 * 用途：目标环绕、巡检盘旋、等待盘旋、返航前盘旋。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API UOrbitTrajectorySegment : public UTrajectorySegment
{
	GENERATED_BODY()

public:
	UOrbitTrajectorySegment();

	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

	/** 环绕不自动完成（除非 Behavior 取消），覆写为 false */
	virtual bool IsComplete(float CurrentS) const override { return bLoopLimitEnabled && Super::IsComplete(CurrentS); }

protected:
	/** 圆心（世界系 cm，Z = 盘旋高度） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector CenterCm = FVector::ZeroVector;

	/** 半径（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float RadiusCm = 500.0f;

	/** 起始角（°，由当前位置自动计算） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float StartAngleDeg = 0.0f;

	/** 旋转方向符号：角速度>0 → +1(逆时针)，<0 → −1(顺时针) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float SpinSign = 1.0f;

	/** 是否启用圈数上限（否则无限盘旋） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory")
	bool bLoopLimitEnabled = false;

	/** 限定圈数（bLoopLimitEnabled=true 时有效） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Trajectory", meta = (ClampMin = "1"))
	int32 LoopCount = 1;
};
