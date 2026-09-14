#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftSolverConfigNode.generated.h"

USTRUCT(meta = (DataflowAircraft))
struct FAircraftSolverConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftSolverConfigNode,
		"AircraftSolverConfig",
		"Aircraft",
		"Aircraft Global Solver Config")

public:
	FAircraftSolverConfigNode(
		const UE::Dataflow::FNodeParameters& InParam,
		FGuid InGuid = FGuid::NewGuid());

	/** 是否按当前无人机刚体覆盖项目级求解器迭代次数。 */
	UPROPERTY(EditAnywhere, Category = "Solver|Iterations", meta = (DisplayName = "Override Iteration Counts"))
	bool bOverrideIterationCounts = false;

	/** 位置求解迭代：影响穿透、静摩擦和约束位置稳定性。 */
	UPROPERTY(EditAnywhere, Category = "Solver|Iterations", meta = (DisplayName = "Position Solver Iterations", EditCondition = "bOverrideIterationCounts", ClampMin = "0", ClampMax = "255"))
	int32 PositionSolverIterationCount = 8;

	/** 速度求解迭代：影响反弹、动态摩擦和速度约束。 */
	UPROPERTY(EditAnywhere, Category = "Solver|Iterations", meta = (DisplayName = "Velocity Solver Iterations", EditCondition = "bOverrideIterationCounts", ClampMin = "0", ClampMax = "255"))
	int32 VelocitySolverIterationCount = 2;

	/** 投影迭代：修正约束漂移，通常使用 0 到 2。 */
	UPROPERTY(EditAnywhere, Category = "Solver|Iterations", meta = (DisplayName = "Projection Solver Iterations", EditCondition = "bOverrideIterationCounts", ClampMin = "0", ClampMax = "255"))
	int32 ProjectionSolverIterationCount = 1;

protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
