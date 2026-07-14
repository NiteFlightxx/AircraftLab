// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "HoverThrust/HoverThrustEstimator.h"
#include "PathFollowing/PathFollowingTypes.h"
#include "PathFollowing/PurePursuitGuidance.h"
#include "PathFollowing/VectorFieldGuidance.h"
#include "Turn/TurnBehavior.h"
#include "AutopilotProfileAsset.generated.h"

UCLASS(BlueprintType)
class AIRCRAFTAUTOPILOT_API UAutopilotProfileAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn", meta = (DisplayName = "启用协调转弯"))
	bool bEnableCoordinatedTurns = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn",
		meta = (EditCondition = "bEnableCoordinatedTurns", EditConditionHides, DisplayName = "转弯限幅"))
	FTurnLimits TurnLimits;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path", meta = (DisplayName = "制导策略"))
	EPathFollowingStrategy GuidanceStrategy = EPathFollowingStrategy::PurePursuit;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path",
		meta = (EditCondition = "GuidanceStrategy == EPathFollowingStrategy::PurePursuit", EditConditionHides, DisplayName = "纯追踪配置"))
	FPurePursuitGuidanceConfig PurePursuit;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path",
		meta = (EditCondition = "GuidanceStrategy == EPathFollowingStrategy::VectorField", EditConditionHides, DisplayName = "向量场配置"))
	FVectorFieldGuidanceConfig VectorField;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|HoverThrust", meta = (DisplayName = "启用悬停推力估计器"))
	bool bEnableHoverThrustEstimator = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|HoverThrust",
		meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, DisplayName = "悬停推力估计器"))
	FHoverThrustEstimatorConfig HoverThrustEstimator;
};
