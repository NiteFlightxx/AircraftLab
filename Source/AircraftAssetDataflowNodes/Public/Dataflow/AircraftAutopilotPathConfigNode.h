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

	UPROPERTY(EditAnywhere, Category = "Sampling", meta = (DisplayName = "Resample Spacing (cm)", ClampMin = "1.0", Units = "cm"))
	float ResampleSpacingCm = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Sampling", meta = (DisplayName = "Minimum Segment Length (cm)", ClampMin = "0.01", Units = "cm"))
	float MinimumSegmentLengthCm = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Corridor", meta = (DisplayName = "Corridor Safety Margin (cm)", ClampMin = "0.0", Units = "cm"))
	float CorridorSafetyMarginCm = 20.0f;

	/** 防止自交或邻近路径把进度投影到已经走过的远处路段。 */
	UPROPERTY(EditAnywhere, Category = "Projection", meta = (DisplayName = "Projection Backtrack Tolerance (cm)", ClampMin = "0.0", Units = "cm"))
	float ProjectionBacktrackToleranceCm = 25.0f;

	UPROPERTY(EditAnywhere, Category = "Projection", meta = (DisplayName = "Projection Search Distance (cm)", ClampMin = "1.0", Units = "cm"))
	float ProjectionSearchDistanceCm = 2000.0f;

	/** MPCC 与物理约束进度调节器共享的轮廓误差标度。 */
	UPROPERTY(EditAnywhere, Category = "Tracking", meta = (DisplayName = "Contour Error Governor Scale (cm)", ClampMin = "1.0", Units = "cm"))
	float ContourErrorGovernorScaleCm = 100.0f;

	/** 物理约束进度比例向目标比例收敛的响应率。 */
	UPROPERTY(EditAnywhere, Category = "Tracking", meta = (DisplayName = "Progress Scale Response Rate (/s)", ClampMin = "0.01"))
	float ProgressScaleResponseRatePerSecond = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Centerline Weight", ClampMin = "0.0"))
	float CenterlineWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Curvature Weight", ClampMin = "0.0"))
	float CurvatureWeight = 0.25f;

	UPROPERTY(EditAnywhere, Category = "Objective", meta = (DisplayName = "Snap Weight", ClampMin = "0.0"))
	float SnapWeight = 0.05f;

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Max Iterations", ClampMin = "1"))
	int32 MaxIterations = 24;

	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Convergence Tolerance (cm)", ClampMin = "0.0001", Units = "cm"))
	float ConvergenceToleranceCm = 0.1f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAutopilotPathConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAutopilotPathConfigNode, "AircraftAutopilotPathConfig", "Aircraft|Autopilot", "Spatial Path and Tracking")

public:
	FAircraftAutopilotPathConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftAutopilotPathConfig Config;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
