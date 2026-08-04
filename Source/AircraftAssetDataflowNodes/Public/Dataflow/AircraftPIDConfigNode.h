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

/** 模型局部前向轴。飞控内部始终使用 X=Forward、Y=Right、Z=Up。 */
UENUM()
enum class EAircraftForwardAxisNode : uint8
{
	PositiveX UMETA(DisplayName = "+X"),
	PositiveY UMETA(DisplayName = "+Y"),
	NegativeX UMETA(DisplayName = "-X"),
	NegativeY UMETA(DisplayName = "-Y"),
};

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

	/** 美术模型机头方向；默认与权威飞控一致为局部 +Y。 */
	UPROPERTY(EditAnywhere, Category = "Axes")
	EAircraftForwardAxisNode ForwardAxis = EAircraftForwardAxisNode::PositiveY;

	/* ----------- 位置外环（笛卡尔 X/Y/Z） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f PositionKp = FVector3f(0.40f, 0.40f, 0.0f);

	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f PositionKi = FVector3f::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Position", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f PositionKd = FVector3f(0.30f, 0.30f, 0.0f);

	/* ----------- 速度内环（笛卡尔 X/Y/Z） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f VelocityKp = FVector3f(1.50f, 1.50f, 0.0f);

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f VelocityKi = FVector3f(0.01f, 0.01f, 0.0f);

	UPROPERTY(EditAnywhere, Category = "PID|Velocity", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f VelocityKd = FVector3f(0.60f, 0.60f, 0.0f);

	/* ----------- 角度外环（欧拉角 Roll/Pitch/Yaw） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngleKp = FVector3f(4.5f, 4.5f, 3.0f);

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngleKi = FVector3f::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "PID|Angle", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngleKd = FVector3f::ZeroVector;

	/* ----------- 角速率内环（机体角速率 Roll/Pitch/Yaw） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f RateKp = FVector3f(0.0080f, 0.0080f, 0.0012f);

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f RateKi = FVector3f(0.0010f, 0.0010f, 0.00015f);

	UPROPERTY(EditAnywhere, Category = "PID|Rate", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f RateKd = FVector3f(0.00040f, 0.00040f, 0.00008f);

	/* ----------- Altitude / VerticalVelocity（独立通道） ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float AltitudeKp = 1.20f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float AltitudeKi = 0.f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float AltitudeKd = 0.20f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float VerticalVelocityKp = 0.0015f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float VerticalVelocityKi = 0.00020f;

	UPROPERTY(EditAnywhere, Category = "PID|Altitude", meta = (DataflowInput, ClampMin = "0.0"))
	float VerticalVelocityKd = 0.00050f;

	/* ----------- 限幅 ----------- */

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxTiltAngleDegrees = 25.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxYawRateDegreesPerSec = 90.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxRollRateDegreesPerSec = 180.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxPitchRateDegreesPerSec = 180.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxClimbRateCmPerSec = 300.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxDescentRateCmPerSec = 200.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxHorizontalSpeedCmPerSec = 800.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxHorizontalAccelerationCmPerSecSq = 600.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0"))
	float MaxVerticalAccelerationCmPerSecSq = 500.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float MinCollectiveCommand = 0.f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float HoverCollectiveCommand = 0.5f;

	UPROPERTY(EditAnywhere, Category = "PID|Limits", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float MaxCollectiveCommand = 1.f;

	/** 微分项一阶低通截止频率（Hz）；0 表示不滤波。 */
	UPROPERTY(EditAnywhere, Category = "PID|Filter", meta = (DataflowInput, ClampMin = "0.0"))
	float DerivativeCutoffHz = 15.f;

	/** 阻尼伪逆控制分配的 λ：u = (BᵀB + λI)⁻¹ Bᵀ τ_des。 */
	UPROPERTY(EditAnywhere, Category = "PID|Allocation", meta = (DataflowInput, ClampMin = "0.0"))
	float AllocationDamping = 0.05f;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
