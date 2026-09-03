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
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (DisplayName = "Linear Natural Frequency (Hz)", ClampMin = "0.0", Units = "Hz"))
	float LinearNaturalFrequencyHz = 1.59154943f;
	/** 线性阻尼比；1 为临界阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (DisplayName = "Linear Damping Ratio", ClampMin = "0.0"))
	float LinearDampingRatio = 1.0f;
	/** 不依赖 Strength 的附加线性阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (DisplayName = "Linear Extra Damping (/s)", ClampMin = "0.0"))
	float LinearExtraDampingPerSecond = 0.0f;
	/** 合力上限（N）；0 表示不限制。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (DisplayName = "Linear Force Limit (N)", ClampMin = "0.0", Units = "N"))
	float LinearForceLimitN = 0.0f;
	/** 重力前馈系数；0 禁用，1 完整补偿世界重力。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Feed Forward", meta = (DisplayName = "Gravity Feedforward Scale", ClampMin = "0.0"))
	float GravityFeedForwardScale = 1.0f;
	/** 预测器动力学前馈系数；0 禁用，1 完整消费阻尼或显式空气动力学补偿。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Feed Forward", meta = (DisplayName = "Dynamics Feedforward Scale", ClampMin = "0.0"))
	float DynamicsFeedForwardScale = 1.0f;
	/** 显式姿态扭矩控制器自然频率（Hz）。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Torque", meta = (DisplayName = "Attitude Natural Frequency (Hz)", ClampMin = "0.0", Units = "Hz"))
	float AttitudeNaturalFrequencyHz = 1.59154943f;
	/** 姿态扭矩控制器阻尼比；1 为临界阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Torque", meta = (DisplayName = "Attitude Damping Ratio", ClampMin = "0.0"))
	float AttitudeDampingRatio = 1.0f;
	/** 姿态扭矩控制器附加角速度阻尼（s^-1）。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Torque", meta = (DisplayName = "Attitude Extra Damping (/s)", ClampMin = "0.0"))
	float AttitudeExtraDampingPerSecond = 0.0f;
	/** 物理线程施加的姿态力矩上限（N·m）；0 表示不限制。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Torque", meta = (DisplayName = "Attitude Torque Limit (N·m)", ClampMin = "0.0", Units = "Nm"))
	float AttitudeTorqueLimitNm = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (DisplayName = "Linear Acceleration Mode"))
	bool bLinearAccelerationMode = true;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftConstraintSimulationConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftConstraintSimulationConfigNode, "AircraftConstraintSimulationConfig", "Aircraft|Flight Controller", "Constraint Simulation")
public:
	FAircraftConstraintSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DisplayName = "Collection", DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftConstraintSimulationConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
