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
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0")) float MaxVerticalAccelerationCmPerSecSq = 500.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinCollectiveCommand = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HoverCollectiveCommand = 0.5f;
	UPROPERTY(EditAnywhere, Category = "Limits", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MaxCollectiveCommand = 1.0f;
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
