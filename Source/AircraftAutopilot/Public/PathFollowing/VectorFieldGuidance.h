// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "PathFollowing/PathFollowingStrategy.h"

#include "VectorFieldGuidance.generated.h"

/**
 * 向量场制导（Vector Field Guidance）
 *
 * 几何核心：在路径附近构造一个"期望速度场"，无人机速度收敛到场方向。
 *   field(P) = Tangent(s*) + K_cte · CrossTrackError · Normal(s*)
 *
 *   - Tangent(s*)：最近路径点 s* 处的切向（前进方向）
 *   - Normal(s*)：该处法向（指向曲率圆心）
 *   - CrossTrackError：当前位置在 Normal 方向的投影（带符号横向误差）
 *   - K_cte：横向误差反馈增益，越大越快回正但易振荡
 *
 * 收敛性：横向误差大时，场方向偏回路径 → 速度跟场 → 误差减小 → 收敛。
 *   这是控制论里经典的"势场/向量场"路径跟踪，比纯追踪更平滑、
 *   在高速/大曲率下不易切角。
 *
 * 速度幅值：沿用轨迹梯形剖面。期望航向 = 场方向。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UVectorFieldGuidance : public UPathFollowingStrategy
{
	GENERATED_BODY()

public:
	UVectorFieldGuidance();

	// UPathFollowingStrategy
	virtual bool Update(const FVector& CurrentPositionCm, const FVector& CurrentVelocityCmPerSec, float DeltaSeconds, FGuidanceCommand& OutCommand) override;
	virtual EPathFollowingStrategy GetStrategyType() const override { return EPathFollowingStrategy::VectorField; }

protected:
	/** 横向误差反馈增益（1/cm）。典型 0.005~0.02 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing|VectorField", meta = (ClampMin = "0.0"))
	float CrossTrackGain = 0.01f;

	/** 横向误差硬限幅（cm），超过此值不继续增大回正项，防过冲 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|PathFollowing|VectorField", meta = (ClampMin = "0.0"))
	float MaxCrossTrackCorrectionCm = 500.0f;
};
