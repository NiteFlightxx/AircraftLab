#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftControllerInputConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftControllerInputConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0", ClampMax = "1.0")) float HorizontalHoldStickDeadband = 0.08f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0", ClampMax = "1.0")) float VerticalHoldStickDeadband = 0.08f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0", ClampMax = "1.0")) float YawHoldStickDeadband = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Hold", meta = (ClampMin = "0.0")) float HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Execution") bool bControllerEnabledByDefault = true;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftControllerInputConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftControllerInputConfigNode, "AircraftControllerInputConfig", "Aircraft|Flight Controller", "Controller Input")
public:
	FAircraftControllerInputConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftControllerInputConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
