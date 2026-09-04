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

	/**
	 * Chaos 异步物理解算的固定时间步（秒）。
	 * 节点存在时为当前机体请求该步长；若低于所属 Chaos Solver 当前异步步长，
	 * 会降低该共享 Solver 的异步步长，因此同一 Solver 中的其他刚体也会受到影响。
	 * 删除节点时完全使用项目物理设置。
	 */
	UPROPERTY(EditAnywhere, Category = "Solver", meta = (DisplayName = "Async Fixed Time Step (s)", ClampMin = "0.001", ClampMax = "0.066667", Units = "s"))
	float AsyncFixedTimeStepSize = 1.0f / 60.0f;

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
