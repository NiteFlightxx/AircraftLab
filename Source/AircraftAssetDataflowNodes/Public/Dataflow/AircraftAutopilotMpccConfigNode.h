#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAutopilotMpccConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAutopilotMpccConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Horizon", meta = (ClampMin = "1.0", Units = "Hz")) float UpdateRateHz = 50.0f;
	UPROPERTY(EditAnywhere, Category = "Horizon", meta = (ClampMin = "0.1", Units = "s")) float HorizonSeconds = 1.5f;
	UPROPERTY(EditAnywhere, Category = "Horizon", meta = (ClampMin = "2")) int32 HorizonSteps = 30;
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (ClampMin = "1")) int32 MaxOptimizationIterations = 2;
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (ClampMin = "0.01", Units = "ms")) float SolveTimeBudgetMilliseconds = 2.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float ContourErrorWeight = 8.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float LagErrorWeight = 2.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float ProgressWeight = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float SpeedTrackingWeight = 1.5f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float AccelerationWeight = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float JerkWeight = 0.02f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float YawTrackingWeight = 0.5f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float TerminalPositionWeight = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (ClampMin = "0.0")) float TerminalVelocityWeight = 10.0f;
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (ClampMin = "0.0000001")) float Regularization = 1.0e-5f;
	UPROPERTY(EditAnywhere, Category = "Safety", meta = (ClampMin = "1")) int32 MaxConsecutiveFailures = 3;
	UPROPERTY(EditAnywhere, Category = "Safety", meta = (ClampMin = "0.01", Units = "s")) float MaximumReferenceAgeSeconds = 0.15f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAutopilotMpccConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAutopilotMpccConfigNode, "AircraftAutopilotMpccConfig", "Aircraft|Autopilot", "Model Predictive Contouring Control")

public:
	FAircraftAutopilotMpccConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties))
	FAircraftAutopilotMpccConfig Config;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
