#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAutopilotTimingConfigNode.generated.h"

USTRUCT(BlueprintType)
struct FAircraftAutopilotTimingConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Sampling", meta = (DisplayName = "Sample Spacing (cm)", ClampMin = "1.0", Units = "cm"))
	float SampleSpacingCm = 50.0f;

	UPROPERTY(EditAnywhere, Category = "Reserve", meta = (DisplayName = "Thrust Reserve Fraction", ClampMin = "0.0", ClampMax = "0.9"))
	float ThrustReserveFraction = 0.15f;

	UPROPERTY(EditAnywhere, Category = "Reserve", meta = (DisplayName = "Curvature Accel Reserve Fraction", ClampMin = "0.0", ClampMax = "0.9"))
	float CurvatureAccelerationReserveFraction = 0.15f;

	UPROPERTY(EditAnywhere, Category = "Reserve", meta = (DisplayName = "Braking Reserve Fraction", ClampMin = "0.0", ClampMax = "0.9"))
	float BrakingReserveFraction = 0.10f;

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Max Iterations", ClampMin = "1"))
	int32 MaxIterations = 12;

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Speed Convergence Tolerance (cm/s)", ClampMin = "0.000001", Units = "cm/s"))
	float SpeedConvergenceToleranceCmPerSec = 1.0e-3f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAutopilotTimingConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAutopilotTimingConfigNode, "AircraftAutopilotTimingConfig", "Aircraft|Autopilot", "Dynamic Trajectory Timing")

public:
	FAircraftAutopilotTimingConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftAutopilotTimingConfig Config;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
