#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "Dataflow/AircraftPidConfigNodeTypes.h"

#include "AircraftPositionControllerConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftPositionControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Position", meta = (ShowOnlyInnerProperties)) FAircraftPidChannelConfig PositionX { 0.40f, 0.0f, 0.30f, 1.0f, 0.0f, 800.0f, 0.0f, true };
	UPROPERTY(EditAnywhere, Category = "Position", meta = (ShowOnlyInnerProperties)) FAircraftPidChannelConfig PositionY { 0.40f, 0.0f, 0.30f, 1.0f, 0.0f, 800.0f, 0.0f, true };
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ShowOnlyInnerProperties)) FAircraftPidChannelConfig VelocityX { 1.50f, 0.01f, 0.60f, 1.0f, 3000.0f, 600.0f, 12.0f, true };
	UPROPERTY(EditAnywhere, Category = "Velocity", meta = (ShowOnlyInnerProperties)) FAircraftPidChannelConfig VelocityY { 1.50f, 0.01f, 0.60f, 1.0f, 3000.0f, 600.0f, 12.0f, true };
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float LinearDampingFeedForwardScale = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0", ClampMax = "0.9")) float DampingAccelerationReserveFraction = 0.2f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftPositionControllerConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftPositionControllerConfigNode, "AircraftPositionControllerConfig", "Aircraft|Flight Controller", "Position Controller")
public:
	FAircraftPositionControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftPositionControllerConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
