// 单个电机参数 USTRUCT 数组节点：写入 Motors 多元素组（每个数组元素 → 一行 Group 数据）。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftMotorConfigNode.generated.h"

/**
 * 单个电机的配置项（与 FDroneMotorModelConfig 字段一一对应）。
 *
 * 电机一阶滞后动力学：
 *     τ_motor · dω/dt + ω = ω_cmd
 * 离散化（前向欧拉）：
 *     α = Δt / (τ_motor + Δt),  ω_k = ω_{k-1} + α · (ω_cmd - ω_{k-1})
 * SpinUp/SpinDown 是不对称时间常数（加/减速过程不同）。
 */
USTRUCT(BlueprintType)
struct FAircraftMotorEntry
{
	GENERATED_BODY()

	/** 电机名称（用于 Propeller 引用）。 */
	UPROPERTY(EditAnywhere, Category = "Motor")
	FName Name = NAME_None;

	UPROPERTY(EditAnywhere, Category = "Motor")
	bool bEnabled = true;

	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0"))
	float MinRpm = 0.f;

	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0"))
	float IdleRpm = 1500.f;

	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0"))
	float MaxRpm = 12000.f;

	/** 加速时间常数 τ_up（秒）。 */
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.001"))
	float SpinUpTimeSeconds = 0.06f;

	/** 减速时间常数 τ_down（秒）。 */
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.001"))
	float SpinDownTimeSeconds = 0.10f;

	/** 指令到推力的指数（≈2 模拟 F = kT·ω²）。 */
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.1"))
	float CommandExponent = 2.f;

	/** 指令变化率上限（每秒归一化指令变化量）。 */
	UPROPERTY(EditAnywhere, Category = "Motor", meta = (ClampMin = "0.0"))
	float MaxCommandSlewPerSecond = 8.f;
};

/**
 * 电机配置节点：批量写入 Motors 组。Resize(N)，把数组中每条 Entry 写入对应行。
 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftMotorConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftMotorConfigNode,
		"AircraftMotorConfig",
		"Aircraft",
		"Aircraft Motors Config")

public:
	FAircraftMotorConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	/** 全部电机条目；数量决定 Motors 组元素数量。 */
	UPROPERTY(EditAnywhere, Category = "Motors")
	TArray<FAircraftMotorEntry> Motors;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
