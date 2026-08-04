// 电池单元素组节点。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftBatteryConfigNode.generated.h"

/**
 * 电池配置节点：写入 Battery 单元素组。
 *
 * 电池模型默认为内阻线性模型：
 *     V_terminal = V_oc - I · R_internal
 *     V_oc 在 [MinVoltageV, NominalVoltageV] 之间随 SoC 线性下降
 * SimulationProxy 消费该模型，用于电量、压降与可用推力衰减。
 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftBatteryConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftBatteryConfigNode,
		"AircraftBatteryConfig",
		"Aircraft",
		"Aircraft Battery Config")

public:
	FAircraftBatteryConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	/** 容量（毫安时）。 */
	UPROPERTY(EditAnywhere, Category = "Battery", meta = (DataflowInput, ClampMin = "0.0"))
	float CapacityMilliAmpHour = 2200.f;

	/** 标称电压（伏特）。 */
	UPROPERTY(EditAnywhere, Category = "Battery", meta = (DataflowInput, ClampMin = "0.0"))
	float NominalVoltageV = 14.8f;

	/** 满放电状态下的开路电压（伏特）。 */
	UPROPERTY(EditAnywhere, Category = "Battery", meta = (DataflowInput, ClampMin = "0.0"))
	float MinVoltageV = 13.2f;

	/** 最大放电倍率 C。 */
	UPROPERTY(EditAnywhere, Category = "Battery", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxDischargeC = 75.f;

	/** 内阻（欧姆）。 */
	UPROPERTY(EditAnywhere, Category = "Battery", meta = (DataflowInput, ClampMin = "0.0"))
	float InternalResistanceOhm = 0.012f;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
