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

	UPROPERTY(EditAnywhere, Category = "Kinematic") bool bSweepMovement = true;
	/** 速度积分位置向轨迹位置收敛的一阶响应率；0 表示只按速度积分。 */
	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (ClampMin = "0.0")) float PositionCorrectionRate = 8.0f;
	/** 偏航积分结果向轨迹偏航收敛的一阶响应率；0 表示只按偏航角速度积分。 */
	UPROPERTY(EditAnywhere, Category = "Kinematic", meta = (ClampMin = "0.0")) float RotationInterpSpeed = 8.0f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftKinematicSimulationConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftKinematicSimulationConfigNode, "AircraftKinematicSimulationConfig", "Aircraft|Flight Controller", "Kinematic Simulation")
public:
	FAircraftKinematicSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftKinematicSimulationConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
