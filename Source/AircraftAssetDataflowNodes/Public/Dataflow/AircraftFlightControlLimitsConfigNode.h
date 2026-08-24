#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftFlightControlLimitsConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftFlightControlLimitsConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxTiltAngleDegrees = 25.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxYawRateDegreesPerSec = 90.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxRollRateDegreesPerSec = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxPitchRateDegreesPerSec = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxClimbRateCmPerSec = 300.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxDescentRateCmPerSec = 200.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxHorizontalSpeedCmPerSec = 800.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxHorizontalAccelerationCmPerSecSq = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxHorizontalDecelerationCmPerSecSq = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxHorizontalJerkCmPerSecCubed = 2000.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxVerticalAccelerationCmPerSecSq = 500.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxVerticalJerkCmPerSecCubed = 1500.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxYawAccelerationDegPerSecSq = 180.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxYawJerkDegPerSecCubed = 600.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinCollectiveCommand = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HoverCollectiveCommand = 0.5f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MaxCollectiveCommand = 1.0f;

	/** 悬停推力 EKF：默认开启，载荷/电压/旋翼效率变化时自动修正悬停基准。 */
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator") bool bEnableHoverThrustEstimator = true;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (ClampMin = "0.0")) float HoverThrustInitialStateVariance = 0.01f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (ClampMin = "0.0")) float HoverThrustProcessNoiseVariance = 12.5e-6f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (ClampMin = "0.001")) float HoverThrustAccelNoiseVariance = 5.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (ClampMin = "1.0")) float HoverThrustGateSize = 3.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HoverThrustMin = 0.1f;
	UPROPERTY(EditAnywhere, Category = "HoverThrustEstimator", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HoverThrustMax = 0.9f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftFlightControlLimitsConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftFlightControlLimitsConfigNode, "AircraftFlightControlLimitsConfig", "Aircraft|Flight Controller", "Flight Control Limits")
public:
	FAircraftFlightControlLimitsConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftFlightControlLimitsConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
