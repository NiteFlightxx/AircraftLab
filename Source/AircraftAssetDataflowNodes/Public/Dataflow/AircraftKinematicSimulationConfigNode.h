#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftKinematicSimulationConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftKinematicSimulationConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (DisplayName = "Sweep Movement"))
	bool bSweepMovement = true;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftKinematicSimulationConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftKinematicSimulationConfigNode, "AircraftKinematicSimulationConfig", "Aircraft|Flight Controller", "Kinematic Simulation")
public:
	FAircraftKinematicSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftKinematicSimulationConfig Config;
protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
