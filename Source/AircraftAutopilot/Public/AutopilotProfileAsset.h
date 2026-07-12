// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FeedForward/FeedForwardCalculator.h"
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
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|FeedForward")
	FFeedForwardParams FeedForward;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	FTurnLimits TurnLimits;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Turn")
	bool bEnableCoordinatedTurns = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path")
	EPathFollowingStrategy GuidanceStrategy = EPathFollowingStrategy::PurePursuit;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path")
	bool bEnablePathFollowing = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path")
	FPurePursuitGuidanceConfig PurePursuit;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|Path")
	FVectorFieldGuidanceConfig VectorField;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|HoverThrust")
	FHoverThrustEstimatorConfig HoverThrustEstimator;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Autopilot|HoverThrust")
	bool bEnableHoverThrustEstimator = true;
};
