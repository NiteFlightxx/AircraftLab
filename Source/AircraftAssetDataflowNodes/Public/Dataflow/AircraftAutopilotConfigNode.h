#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAutopilotConfigNode.generated.h"

/** 路径跟随制导策略（节点侧枚举，值序与 EAircraftGuidanceStrategy 一致）。 */
UENUM()
enum class EAircraftGuidanceStrategyNode : uint8
{
	/** 纯追踪：朝路径上的前瞻点飞。 */
	PurePursuit UMETA(DisplayName = "纯追踪"),
	/** 向量场：切向 + 横向误差反馈构造期望速度场。 */
	VectorField UMETA(DisplayName = "向量场"),
	/** 直接跟踪：用轨迹名义设定值（无额外制导修正）。 */
	Direct UMETA(DisplayName = "直接跟踪")
};

/**
 * 协调转弯 / 路径制导 / 悬停推力估计器三段。
 */
USTRUCT(BlueprintType)
struct FAircraftAutopilotConfig
{
	GENERATED_BODY()

	/* ---- 协调转弯 ---- */
	UPROPERTY(EditAnywhere, Category = "Turn") bool bEnableCoordinatedTurns = true;
	/** 协调转弯启用速度阈值（cm/s）：|v| < 此值用偏航跟踪，≥ 此值用滚转协调转弯。 */
	UPROPERTY(EditAnywhere, Category = "Turn", meta = (EditCondition = "bEnableCoordinatedTurns", EditConditionHides, ClampMin = "0.0")) float CoordinatedTurnSpeedThresholdCmPerSec = 300.0f;
	/** 最大滚转角（°）——bank turn 上限。 */
	UPROPERTY(EditAnywhere, Category = "Turn", meta = (EditCondition = "bEnableCoordinatedTurns", EditConditionHides, ClampMin = "0.0", ClampMax = "60.0")) float MaxBankAngleDegrees = 35.0f;
	/** 最大横向加速度（cm/s²）——向心加速度上限。 */
	UPROPERTY(EditAnywhere, Category = "Turn", meta = (EditCondition = "bEnableCoordinatedTurns", EditConditionHides, ClampMin = "0.0")) float MaxLateralAccelCmPerSecSq = 500.0f;

	/* ---- 路径制导 ---- */
	UPROPERTY(EditAnywhere, Category = "Path") EAircraftGuidanceStrategyNode GuidanceStrategy = EAircraftGuidanceStrategyNode::PurePursuit;
	UPROPERTY(EditAnywhere, Category = "Path", meta = (EditCondition = "GuidanceStrategy == EAircraftGuidanceStrategyNode::PurePursuit", EditConditionHides, ClampMin = "0.0")) float PurePursuitLookAheadGain = 0.5f;
	UPROPERTY(EditAnywhere, Category = "Path", meta = (EditCondition = "GuidanceStrategy == EAircraftGuidanceStrategyNode::PurePursuit", EditConditionHides, ClampMin = "0.0")) float PurePursuitMinLookAheadCm = 100.0f;
	UPROPERTY(EditAnywhere, Category = "Path", meta = (EditCondition = "GuidanceStrategy == EAircraftGuidanceStrategyNode::PurePursuit", EditConditionHides, ClampMin = "0.0")) float PurePursuitMaxLookAheadCm = 1000.0f;
	UPROPERTY(EditAnywhere, Category = "Path", meta = (EditCondition = "GuidanceStrategy == EAircraftGuidanceStrategyNode::VectorField", EditConditionHides, ClampMin = "0.0")) float VectorFieldCrossTrackGain = 0.01f;
	UPROPERTY(EditAnywhere, Category = "Path", meta = (EditCondition = "GuidanceStrategy == EAircraftGuidanceStrategyNode::VectorField", EditConditionHides, ClampMin = "0.0")) float VectorFieldMaxCrossTrackCorrectionCm = 500.0f;

	/* ---- 悬停推力估计器（零阶 EKF） ---- */
	UPROPERTY(EditAnywhere, Category = "HoverThrust") bool bEnableHoverThrustEstimator = true;
	UPROPERTY(EditAnywhere, Category = "HoverThrust", meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, ClampMin = "0.0")) float InitialStateVariance = 0.01f;
	UPROPERTY(EditAnywhere, Category = "HoverThrust", meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, ClampMin = "0.0")) float ProcessNoiseVariance = 12.5e-6f;
	UPROPERTY(EditAnywhere, Category = "HoverThrust", meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, ClampMin = "0.001")) float AccelNoiseVariance = 5.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrust", meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, ClampMin = "1.0")) float GateSize = 3.0f;
	UPROPERTY(EditAnywhere, Category = "HoverThrust", meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, ClampMin = "0.0", ClampMax = "1.0")) float MinHoverThrust = 0.1f;
	UPROPERTY(EditAnywhere, Category = "HoverThrust", meta = (EditCondition = "bEnableHoverThrustEstimator", EditConditionHides, ClampMin = "0.0", ClampMax = "1.0")) float MaxHoverThrust = 0.9f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftAutopilotConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftAutopilotConfigNode, "AircraftAutopilotConfig", "Aircraft|Autopilot", "Autopilot Config")
public:
	FAircraftAutopilotConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftAutopilotConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
