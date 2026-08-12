#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"
#include "Dataflow/AircraftPidConfigNodeTypes.h"

#include "AircraftAltitudeControllerConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAltitudeControllerConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Altitude", meta = (ShowOnlyInnerProperties)) FAircraftPidChannelConfig Altitude { 1.20f, 0.0f, 0.20f, 1.0f, 0.0f, 300.0f, 0.0f, true };
	UPROPERTY(EditAnywhere, Category = "Vertical Velocity", meta = (ShowOnlyInnerProperties)) FAircraftFeedbackPidChannelConfig VerticalVelocity { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f, 10.0f, true };
	UPROPERTY(EditAnywhere, Category = "Damping Feed Forward", meta = (ClampMin = "0.0")) float VerticalDampingFeedForwardScale = 1.0f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAltitudeControllerConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAltitudeControllerConfigNode, "AircraftAltitudeControllerConfig", "Aircraft|Flight Controller", "Altitude Controller")
public:
	FAircraftAltitudeControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftAltitudeControllerConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
