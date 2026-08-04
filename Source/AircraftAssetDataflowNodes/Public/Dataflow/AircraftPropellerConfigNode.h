// 单个旋翼参数 USTRUCT 数组节点：写入 Propellers 多元素组。
//
// 推力 / 反扭矩模型（基于螺旋桨气动经验关系）：
//     F_thrust = kT · ω²
//     τ_drag   = kQ · ω²
// ThrustCoefficient 对应 kT；ReactionTorqueCoefficient 等价于 kQ/kT 比值。
// SpinDirection 决定反扭矩方向（顺/逆桨配对抵消机体偏航力矩）。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftPropellerConfigNode.generated.h"

UENUM()
enum class EAircraftRotorSpinDirectionNode : uint8
{
	Clockwise UMETA(DisplayName = "Clockwise"),
	CounterClockwise UMETA(DisplayName = "Counter-Clockwise"),
};

/**
 * 单个旋翼的配置项（与 FDroneRotorDefinition 字段一一对应）。
 */
USTRUCT(BlueprintType)
struct FAircraftPropellerEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Propeller")
	FName Name = NAME_None;

	/** 引用 Motors 组中某行的 Name；Terminal 构建时做交叉验证。 */
	UPROPERTY(EditAnywhere, Category = "Propeller")
	FName MotorName = NAME_None;

	/** 骨骼插槽名（若使用 SocketTransform）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller")
	FName SocketName = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Propeller")
	bool bUseSocketTransform = true;

	/** 旋翼在机体坐标系的位置（厘米，bUseSocketTransform=false 时使用）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Transform")
	FVector3f PositionLocalCm = FVector3f::ZeroVector;

	/** 旋翼局部欧拉角（度）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Transform")
	FVector3f RotationLocalEulerDeg = FVector3f::ZeroVector;

	/** 推力轴方向（机体坐标系）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Transform")
	FVector3f ThrustAxisLocal = FVector3f(0.f, 0.f, 1.f);

	UPROPERTY(EditAnywhere, Category = "Propeller")
	EAircraftRotorSpinDirectionNode SpinDirection = EAircraftRotorSpinDirectionNode::CounterClockwise;

	UPROPERTY(EditAnywhere, Category = "Propeller|Aero", meta = (ClampMin = "0.0"))
	float RadiusCm = 12.f;

	/** 最大推力（牛顿）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Aero", meta = (ClampMin = "0.0"))
	float MaxThrustForce = 900.f;

	/** 推力系数 kT。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Aero", meta = (ClampMin = "0.0"))
	float ThrustCoefficient = 1.f;

	/** 反扭矩系数（τ_drag = 系数 · F_thrust，等价于 kQ/kT 比值）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Aero", meta = (ClampMin = "0.0"))
	float ReactionTorqueCoefficient = 0.03f;

	UPROPERTY(EditAnywhere, Category = "Propeller|Aero", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Efficiency = 1.f;

	/** 控制分配可用推力缩放（0~1）。 */
	UPROPERTY(EditAnywhere, Category = "Propeller|Allocation", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ControlAuthorityScale = 1.f;
};

USTRUCT(meta = (DataflowAircraft))
struct FAircraftPropellerConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftPropellerConfigNode,
		"AircraftPropellerConfig",
		"Aircraft",
		"Aircraft Propellers / Rotors Config")

public:
	FAircraftPropellerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	UPROPERTY(EditAnywhere, Category = "Propellers")
	TArray<FAircraftPropellerEntry> Propellers;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
