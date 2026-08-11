#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftRotorFailurePolicyConfigNode.generated.h"

UENUM()
enum class EAircraftFailurePolicyActionNode : uint8
{
	WarningOnly UMETA(DisplayName = "仅警告"),
	SwitchFlightMode UMETA(DisplayName = "切换飞行模式"),
	Failsafe UMETA(DisplayName = "故障保护 / 停桨"),
	EmergencyStop UMETA(DisplayName = "紧急停止")
};

/** 降级飞行模式（节点侧枚举，值序与 EAircraftFlightMode 一致）。 */
UENUM()
enum class EAircraftDegradedFlightModeNode : uint8
{
	Manual UMETA(DisplayName = "Manual"),
	Acro UMETA(DisplayName = "Acro"),
	Angle UMETA(DisplayName = "Angle"),
	AltitudeHold UMETA(DisplayName = "Altitude Hold"),
	PositionHold UMETA(DisplayName = "Position Hold"),
	VelocityHold UMETA(DisplayName = "Velocity Hold"),
	Mission UMETA(DisplayName = "Mission"),
	ReturnToHome UMETA(DisplayName = "Return To Home"),
	AutoLand UMETA(DisplayName = "Auto Land")
};

/**
 * 旋翼故障与控制权限降级策略配置。
 * 任一启用的阈值不满足并持续 ConfirmationTimeSeconds 后触发 Action。
 */
USTRUCT(BlueprintType)
struct FAircraftRotorFailurePolicyConfig
{
	GENERATED_BODY()

	/** 默认关闭，确保资产化不改变现有飞行行为。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy") bool bEnabled = false;
	/** 默认只在已解锁飞行时评估，避免地面维护阶段触发降级。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy") bool bEvaluateOnlyWhenArmed = true;
	/** 最少健康旋翼数；0 表示不检查。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0")) int32 MinimumHealthyRotorCount = 4;
	/** 各轴最小剩余控制权限；0 表示不检查该轴。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinimumCollectiveAuthority = 0.25f;
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinimumRollAuthority = 0.25f;
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinimumPitchAuthority = 0.25f;
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0")) float MinimumYawAuthority = 0.25f;
	/** 故障条件必须连续成立的时间，防止单帧抖动。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0.0")) float ConfirmationTimeSeconds = 0.10f;
	/** 未锁存时，条件恢复后必须连续健康的时间。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (ClampMin = "0.0")) float RecoveryConfirmationTimeSeconds = 1.0f;
	/** 触发后保持锁存，必须显式复位。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy") bool bLatchTriggeredAction = true;
	UPROPERTY(EditAnywhere, Category = "FailurePolicy") EAircraftFailurePolicyActionNode Action = EAircraftFailurePolicyActionNode::WarningOnly;
	/** Action=SwitchFlightMode 时切换到的模式。 */
	UPROPERTY(EditAnywhere, Category = "FailurePolicy", meta = (EditCondition = "Action == EAircraftFailurePolicyActionNode::SwitchFlightMode", EditConditionHides)) EAircraftDegradedFlightModeNode DegradedFlightMode = EAircraftDegradedFlightModeNode::Angle;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftRotorFailurePolicyConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(FAircraftRotorFailurePolicyConfigNode, "AircraftRotorFailurePolicyConfig", "Aircraft|Flight Controller", "Rotor Failure Policy")
public:
	FAircraftRotorFailurePolicyConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());
	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection")) FManagedArrayCollection Collection;
	UPROPERTY(EditAnywhere, Category = "Config", meta = (ShowOnlyInnerProperties)) FAircraftRotorFailurePolicyConfig Config;
	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
