// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/TrajectorySegment.h"

#include "CircleTrajectorySegment.generated.h"

/**
 * 圆弧轨迹段
 *
 * 几何：以 OrbitCenterCm 为圆心、OrbitRadiusCm 为半径的水平圆，
 *   从 ArcStartAngleDegrees 扫到 ArcEndAngleDegrees（绕 +Z 轴）。
 *   角度约定：0° = +X，逆时针为正，世界系顶视图。
 *   扫角方向：EndAngle > StartAngle → 逆时针(CCW)，反之为顺时针(CW)。
 *
 * 弧长 s ∈ [0, R·|Δθ|]，s 线性映射到角度。
 * 运动学：
 *   - 切向 T：沿圆周切线方向（含旋转方向符号）
 *   - 法向 N：指向圆心（向心）
 *   - 曲率 κ = 1/R
 *   - 向心加速度 a_n = v²·κ·N
 *   - 偏航角速度 ω = κ·v（含方向符号）
 *
 * 用途：弧线过渡、固定半径转弯、巡检绕弧。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API UCircleTrajectorySegment : public UTrajectorySegment
{
	GENERATED_BODY()

public:
	UCircleTrajectorySegment();

	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

protected:
	/** 圆心（世界系 cm，Z 分量为盘旋高度） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	FVector CenterCm = FVector::ZeroVector;

	/** 半径（cm） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float RadiusCm = 500.0f;

	/** 起始角（°，绕 +Z） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float StartAngleDeg = 0.0f;

	/** 终止角（°，绕 +Z） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float EndAngleDeg = 360.0f;

	/** 旋转方向符号：+1=逆时针，−1=顺时针 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float SpinSign = 1.0f;

	/** 扫角弧度（|Δθ|，rad，正） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	float SweepRad = 0.0f;
};
