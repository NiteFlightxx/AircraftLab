#pragma once

#include "CoreMinimal.h"
#include "Dataflow/AircraftConfigNodeBase.h"
#include "Dataflow/AircraftAlternativeAttitudeConfig.h"

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
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Reference", meta = (ShowOnlyInnerProperties))
	FAircraftAlternativeAttitudeConfig AttitudeReference;

	/** 物理刚体跟随整形姿态参考的伺服自然频率。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Servo", meta = (ClampMin = "0.0", Units = "Hz"))
	float AttitudeServoNaturalFrequencyHz = 1.59154943f;
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Servo", meta = (ClampMin = "0.0"))
	float AttitudeServoDampingRatio = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Servo", meta = (ClampMin = "0.0"))
	float AttitudeServoExtraDampingPerSecond = 0.0f;
	/** 物理线程施加的姿态力矩上限（N·m）；0 表示不限制。 */
	UPROPERTY(EditAnywhere, Category = "Constraint|Attitude Servo", meta = (DisplayName = "Attitude Torque Limit (N·m)", ClampMin = "0.0", Units = "Nm"))
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
