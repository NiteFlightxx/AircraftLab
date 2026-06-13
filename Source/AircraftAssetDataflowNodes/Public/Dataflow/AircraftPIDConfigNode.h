// 串级 PID + 限幅 + 分配阻尼配置节点。
//
// 串级 PID 顺序：Position → Velocity → Angle → Rate
//   - Position 外环：误差 = 期望位置 - 估计位置 → 期望速度
//   - Velocity 内环：误差 = 期望速度 - 估计速度 → 期望倾斜角度（送入 Angle 外环）
//   - Angle 外环：误差 = 期望姿态 - 估计姿态 → 期望机体角速率
//   - Rate 内环：误差 = 期望角速率 - 实测角速率 → 期望机体力矩 τ
// Altitude / VerticalVelocity 是与水平 Cartesian PID 解耦的额外两环（独立通道）。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftPIDConfigNode.generated.h"

/**
 * 飞控配置节点：写入 FlightController 单元素组（12 路串级 PID + 6 路高度 PID + 7 路限幅）。
 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftPIDConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftPIDConfigNode,
		"AircraftPIDConfig",
		"Aircraft",
		"Aircraft Cascaded PID Config")

public:
	FAircraftPIDConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	/* ----------- 位置外环（笛卡尔 X/Y/Z） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f PositionKp = FVector3f(2.f, 2.f, 2.f);

	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f PositionKi = FVector3f::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f PositionKd = FVector3f::ZeroVector;

	/* ----------- 速度内环（笛卡尔 X/Y/Z） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f VelocityKp = FVector3f(3.f, 3.f, 3.f);

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f VelocityKi = FVector3f(0.5f, 0.5f, 0.5f);

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f VelocityKd = FVector3f(0.1f, 0.1f, 0.1f);

	/* ----------- 角度外环（欧拉角 Roll/Pitch/Yaw） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngleKp = FVector3f(6.f, 6.f, 4.f);

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngleKi = FVector3f::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngleKd = FVector3f::ZeroVector;

	/* ----------- 角速率内环（机体角速率 Roll/Pitch/Yaw） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f RateKp = FVector3f(0.15f, 0.15f, 0.20f);

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f RateKi = FVector3f(0.10f, 0.10f, 0.15f);

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f RateKd = FVector3f(0.005f, 0.005f, 0.f);

	/* ----------- Altitude / VerticalVelocity（独立通道） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float AltitudeKp = 2.f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float AltitudeKi = 0.f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float AltitudeKd = 0.f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float VerticalVelocityKp = 3.f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float VerticalVelocityKi = 0.5f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float VerticalVelocityKd = 0.1f;

	/* ----------- 限幅 ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxTiltAngleDegrees = 35.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxYawRateDegreesPerSec = 180.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 400.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 250.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 1200.f;

	/** 微分项一阶低通截止频率（Hz）；0 表示不滤波。 */
	UPROPERTY(EditAnywhere, Category = "PID|Filter", meta = (DataflowInput, ClampMin = "0.0"))
	float DerivativeCutoffHz = 80.f;

	/** 阻尼伪逆控制分配的 λ：u = (BᵀB + λI)⁻¹ Bᵀ τ_des。 */
	UPROPERTY(EditAnywhere, Category = "PID|Allocation", meta = (DataflowInput, ClampMin = "0.0"))
	float AllocationDamping = 1e-3f;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
