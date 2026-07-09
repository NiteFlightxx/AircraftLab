// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Trajectory/TrajectorySegment.h"

#include "BezierTrajectorySegment.generated.h"

/**
 * 贝塞尔曲线轨迹段
 *
 * 几何：de Casteljau 算法求值，支持任意阶（由 BezierDegree 决定，控制点数 = 阶数+1）。
 * 弧长参数化：Bezier 的参数 u∈[0,1] 与弧长 s 非线性相关，因此 Build 时预建
 *   "累积弧长表" CumArcLengths[i]（i 对应 u_i = i/N），Sample 时按 s 二分查表得到 u。
 *
 * 运动学：
 *   - 切向 T = 数值差分 (P(u+du) − P(u−du)) 归一化
 *   - 曲率 κ = |dT/ds|（数值差分），用于向心加速度与前馈偏航率
 *   - 速度幅值由生成器注入
 *
 * 用途：平滑过渡航点、避开直角折线、生成曲线进场/离场段。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class AIRCRAFTAUTOPILOT_API UBezierTrajectorySegment : public UTrajectorySegment
{
	GENERATED_BODY()

public:
	UBezierTrajectorySegment();

	virtual bool BuildSegment_Implementation(const FTrajectoryRequest& Request, FString& OutError) override;
	virtual FTrajectoryPoint SampleAtArcLength(float S, float SpeedCmPerSec) const override;
	virtual FFrenetFrame GetFrenetAtArcLength(float S) const override;

	/** 在参数 u∈[0,1] 处用 de Casteljau 求位置 */
	FVector EvaluatePosition(float U) const;

	/** 在参数 u 处用数值差分求单位切向 */
	FVector EvaluateTangent(float U) const;

	/** 由弧长 s 反查参数 u（累积弧长表二分） */
	float ArcLengthToParameter(float S) const;

protected:
	/** 控制点（世界系 cm），数量 = BezierDegree+1 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	TArray<FVector> ControlPoints;

	/** 累积弧长表：CumArcLengths[i] = u=i/N 处的累积弧长（cm），末元素 = TotalArcLength */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|Trajectory")
	TArray<float> CumArcLengths;

	/** 弧长表采样数（越大越精确） */
	static constexpr int32 ArcTableResolution = 64;
};
