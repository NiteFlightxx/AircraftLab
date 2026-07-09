// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "PathFollowing/PathFollowingStrategy.h"

#include "PurePursuitGuidance.generated.h"

/**
 * 纯追踪制导（Pure Pursuit）
 *
 * 几何核心：在路径上取一个"前瞻点"（距最近投影点 L_la 处），
 * 令无人机速度方向始终指向前瞻点。横向误差越大，指向越偏回路径，
 * 天然收敛 —— 这是移动机器人最经典的几何路径跟踪律。
 *
 * 自适应前瞻距离：L_la = k·|v| + L_min
 *   - 高速时 L_la 大 → 转弯平滑、不抖动
 *   - 低速时 L_la 小 → 紧贴路径、转弯锐利
 *
 * 速度幅值：沿用轨迹的梯形速度剖面（保留提前减速），仅替换方向。
 *   desiredVel = normalize(lookaheadPoint − currentPos) · |nominalVel|
 *
 * 期望航向：指向前瞻点（即速度方向）。
 *
 * 频率：与 Trajectory Generator 同频。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UPurePursuitGuidance : public UPathFollowingStrategy
{
	GENERATED_BODY()

public:
	UPurePursuitGuidance();

	// UPathFollowingStrategy
	virtual bool Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec, float DeltaSeconds, FGuidanceCommand& OutCommand) override;
	virtual EPathFollowingStrategy GetStrategyType() const override { return EPathFollowingStrategy::PurePursuit; }

protected:
	/** 前瞻距离速度增益：L_la += LookAheadGain·|v| */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing|PurePursuit", meta = (ClampMin = "0.0"))
	float LookAheadGain = 0.5f;

	/** 最小前瞻距离（cm）—— 悬停时也保证有前瞻点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing|PurePursuit", meta = (ClampMin = "0.0"))
	float MinLookAheadCm = 100.0f;

	/** 最大前瞻距离（cm）—— 高速限幅，避免前瞻过远导致切角 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing|PurePursuit", meta = (ClampMin = "0.0"))
	float MaxLookAheadCm = 1000.0f;
};
