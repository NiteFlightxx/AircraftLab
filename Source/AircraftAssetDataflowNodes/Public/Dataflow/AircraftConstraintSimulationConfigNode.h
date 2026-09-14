#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"

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
	/** 原生 SLERP Angular Drive 的自然频率（Hz）。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular Drive", meta = (DisplayName = "Attitude Natural Frequency (Hz)", ClampMin = "0.0", Units = "Hz"))
	float AttitudeNaturalFrequencyHz = 1.59154943f;
	/** 原生 SLERP Angular Drive 的阻尼比；1 为临界阻尼。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular Drive", meta = (DisplayName = "Attitude Damping Ratio", ClampMin = "0.0"))
	float AttitudeDampingRatio = 1.0f;
	/** 原生 SLERP Angular Drive 的附加角速度阻尼（s^-1）。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular Drive", meta = (DisplayName = "Attitude Extra Damping (/s)", ClampMin = "0.0"))
	float AttitudeExtraDampingPerSecond = 0.0f;
	/** 原生 SLERP Angular Drive 的力矩上限（N·m）；0 表示不限制。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Angular Drive", meta = (DisplayName = "Attitude Torque Limit (N·m)", ClampMin = "0.0", Units = "Nm"))
	float AttitudeTorqueLimitNm = 0.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint|Linear", meta = (DisplayName = "Linear Acceleration Mode"))
	bool bLinearAccelerationMode = true;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftConstraintSimulationConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftConstraintSimulationConfigNode, "AircraftConstraintSimulationConfig", "Aircraft|Flight Controller", "Constraint Simulation")
public:
	FAircraftConstraintSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Config", meta = (DisplayName = "Config", ShowOnlyInnerProperties))
	FAircraftConstraintSimulationConfig Config;
protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
};
