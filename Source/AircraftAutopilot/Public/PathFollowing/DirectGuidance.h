// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "PathFollowing/PathFollowingStrategy.h"

#include "DirectGuidance.generated.h"

/**
 * 直连制导（Direct）
 *
 * 最简策略：直接转发轨迹生成器的名义设定值作为制导指令，
 * 不做几何修正（无前瞻、无横向误差校正）。
 *
 * 用途：调试基线、或轨迹自身已足够精确（如悬停/垂直起降）无需制导律介入的场景。
 * 实现完全复用基类 UPathFollowingStrategy::Update 的默认行为（UpdateDirect）。
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced, ClassGroup = (AircraftAutopilot))
class AIRCRAFTAUTOPILOT_API UDirectGuidance : public UPathFollowingStrategy
{
	GENERATED_BODY()

public:
	UDirectGuidance();

	// Update 不覆写：复用基类默认实现（直接转发轨迹名义设定值）
	virtual EPathFollowingStrategy GetStrategyType() const override { return EPathFollowingStrategy::Direct; }
};
