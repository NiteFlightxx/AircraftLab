#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAutopilotPathConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAutopilotPathConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Sampling", meta = (ClampMin = "1.0", Units = "cm"))
	float ResampleSpacingCm = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Sampling", meta = (ClampMin = "0.01", Units = "cm"))
	float MinimumSegmentLengthCm = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Corridor", meta = (ClampMin = "0.0", Units = "cm"))
	float CorridorSafetyMarginCm = 20.0f;

	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0"))
	float CenterlineWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0"))
	float CurvatureWeight = 0.25f;

	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0"))
	float SnapWeight = 0.05f;

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (ClampMin = "1"))
	int32 MaxIterations = 24;

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (ClampMin = "0.0001", Units = "cm"))
	float ConvergenceToleranceCm = 0.1f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAutopilotPathConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAutopilotPathConfigNode, "AircraftAutopilotPathConfig", "Aircraft|Autopilot", "Spatial Path Optimization")

public:
	FAircraftAutopilotPathConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties))
	FAircraftAutopilotPathConfig Config;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
