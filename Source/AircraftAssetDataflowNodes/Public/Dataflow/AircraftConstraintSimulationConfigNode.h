#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftConstraintSimulationConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftConstraintSimulationConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float LinearPositionStrength = 100.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float LinearVelocityStrength = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float LinearForceLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float AngularPositionStrength = 100.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float AngularVelocityStrength = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint", meta = (ClampMin = "0.0")) float AngularTorqueLimit = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint") bool bAccelerationMode = true;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftConstraintSimulationConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftConstraintSimulationConfigNode, "AircraftConstraintSimulationConfig", "Aircraft|Flight Controller", "Constraint Simulation")
public:
	FAircraftConstraintSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftConstraintSimulationConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
