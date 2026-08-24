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

	/** 线性弹簧自然频率（Hz）。运行时转换为 Stiffness=(Strength*2π)^2。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (ClampMin = "0.0", Units = "Hz")) float LinearStrength = 1.59154943f;
	/** 线性阻尼比；1 为临界阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (ClampMin = "0.0")) float LinearDampingRatio = 1.0f;
	/** 不依赖 Strength 的附加线性阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (ClampMin = "0.0")) float LinearExtraDamping = 0.0f;
	/** 合力上限（N）；0 表示不限制。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (ClampMin = "0.0", Units = "N")) float LinearForceLimitN = 0.0f;
	/** 重力前馈系数；0 禁用，1 完整补偿世界重力。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Feed Forward", meta = (ClampMin = "0.0")) float GravityFeedForwardScale = 1.0f;
	/** 预测器动力学前馈系数；0 禁用，1 完整消费阻尼或显式空气动力学补偿。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Feed Forward", meta = (ClampMin = "0.0")) float DynamicsFeedForwardScale = 1.0f;
	/** 角度弹簧自然频率（Hz）。运行时转换为 Stiffness=(Strength*2π)^2。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular", meta = (ClampMin = "0.0", Units = "Hz")) float AngularStrength = 1.59154943f;
	/** 角度阻尼比；1 为临界阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular", meta = (ClampMin = "0.0")) float AngularDampingRatio = 1.0f;
	/** 不依赖 Strength 的附加角度阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular", meta = (ClampMin = "0.0")) float AngularExtraDamping = 0.0f;
	/** 驱动力矩上限（N·m）；0 表示不限制。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular", meta = (ClampMin = "0.0", Units = "Nm")) float AngularTorqueLimitNm = 0.0f;
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
