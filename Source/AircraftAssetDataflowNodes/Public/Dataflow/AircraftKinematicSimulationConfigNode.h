#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftKinematicSimulationConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftKinematicSimulationConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (DisplayName = "Sweep Movement"))
	bool bSweepMovement = true;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftKinematicSimulationConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftKinematicSimulationConfigNode, "AircraftKinematicSimulationConfig", "Aircraft|Flight Controller", "Kinematic Simulation")
public:
	FAircraftKinematicSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftKinematicSimulationConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
