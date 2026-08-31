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

	UPROPERTY(EditAnywhere, Category = "Horizon", meta = (DisplayName = "Update Rate (Hz)", ClampMin = "1.0", Units = "Hz"))
	float UpdateRateHz = 50.0f;
	UPROPERTY(EditAnywhere, Category = "Horizon", meta = (DisplayName = "Horizon (s)", ClampMin = "0.1", Units = "s"))
	float HorizonSeconds = 1.5f;
	UPROPERTY(EditAnywhere, Category = "Horizon", meta = (DisplayName = "Horizon Steps", ClampMin = "2"))
	int32 HorizonSteps = 30;
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Max Optimization Iterations", ClampMin = "1"))
	int32 MaxOptimizationIterations = 2;
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Solve Time Budget (ms)", ClampMin = "0.01", Units = "ms"))
	float SolveTimeBudgetMilliseconds = 2.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Contour Error Weight", ClampMin = "0.0"))
	float ContourErrorWeight = 8.0f;
	UPROPERTY(EditAnywhere, Category = "Safety", meta = (DisplayName = "Corridor Violation Weight", ClampMin = "0.0"))
	float CorridorViolationWeight = 1000.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Lag Error Weight", ClampMin = "0.0"))
	float LagErrorWeight = 2.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Speed Tracking Weight", ClampMin = "0.0"))
	float SpeedTrackingWeight = 1.5f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Acceleration Weight", ClampMin = "0.0"))
	float AccelerationWeight = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Jerk Weight", ClampMin = "0.0"))
	float JerkWeight = 0.02f;
	UPROPERTY(EditAnywhere, Category = "Tracking", meta = (DisplayName = "Yaw Response Time (s)", ClampMin = "0.001", Units = "s"))
	float YawResponseTimeSeconds = 0.04f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Terminal Position Weight", ClampMin = "0.0"))
	float TerminalPositionWeight = 20.0f;
	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Terminal Velocity Weight", ClampMin = "0.0"))
	float TerminalVelocityWeight = 10.0f;
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Regularization", ClampMin = "0.0000001"))
	float Regularization = 1.0e-5f;
	UPROPERTY(EditAnywhere, Category = "Safety", meta = (DisplayName = "Max Consecutive Failures", ClampMin = "1"))
	int32 MaxConsecutiveFailures = 3;
	UPROPERTY(EditAnywhere, Category = "Safety", meta = (DisplayName = "Maximum Reference Age (s)", ClampMin = "0.01", Units = "s"))
	float MaximumReferenceAgeSeconds = 0.15f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAutopilotMpccConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAutopilotMpccConfigNode, "AircraftAutopilotMpccConfig", "Aircraft|Autopilot", "Model Predictive Contouring Control")

public:
	FAircraftAutopilotMpccConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftAutopilotMpccConfig Config;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
