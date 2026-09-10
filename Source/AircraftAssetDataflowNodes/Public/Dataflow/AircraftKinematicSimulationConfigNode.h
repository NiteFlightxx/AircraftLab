#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"
#include "Dataflow/AircraftAlternativeAttitudeConfig.h"

#include "AircraftKinematicSimulationConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftKinematicSimulationConfig
{
	GENERATED_BODY()

	FAircraftKinematicSimulationConfig()
	{
		AttitudeReference.NaturalFrequencyHz = 1.5f;
		AttitudeReference.MaxAngularAccelerationDegPerSecSq = FVector(480.0, 480.0, 240.0);
		AttitudeReference.MaxAngularJerkDegPerSecCubed = FVector(1920.0, 1920.0, 960.0);
		AttitudeReference.DynamicsFeedForwardScale = 0.0f;
	}

	UPROPERTY(EditAnywhere, Category = "Kinematic|Attitude Reference", meta = (ShowOnlyInnerProperties))
	FAircraftAlternativeAttitudeConfig AttitudeReference;

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
