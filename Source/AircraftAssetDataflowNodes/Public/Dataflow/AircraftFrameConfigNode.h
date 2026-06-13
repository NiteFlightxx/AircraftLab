// 对齐 ChaosClothAssetDataflowNodes 中"配置类"节点（如 ClothPhysicalMeshConfigNode）：
// 写入单元素 Frame group 的全部静态字段。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowEngine.h"
#include "Dataflow/DataflowNode.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftFrameConfigNode.generated.h"

/**
 * 多旋翼机架类型 Dataflow 端枚举
 *
 * 与 EDroneFrameType 一一映射；这里用一个独立的 USTRUCT 范围内枚举是为了让 Dataflow 节点的
 * UPROPERTY 直接绑定下拉框（避免外部模块的 UENUM 头依赖在节点头里出现）。
 */
UENUM()
enum class EAircraftFrameTypeNode : uint8
{
	QuadX UMETA(DisplayName = "Quad X"),
	QuadPlus UMETA(DisplayName = "Quad Plus"),
	HexX UMETA(DisplayName = "Hex X"),
	OctoX UMETA(DisplayName = "Octo X"),
	Custom UMETA(DisplayName = "Custom"),
};

/**
 * 机架配置节点：写入 Frame 单元素组的全部字段（机架类型 + 质量惯性 + 气动 + 风场）。
 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftFrameConfigNode : public FDataflowNode
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftFrameConfigNode,
		"AircraftFrameConfig",
		"Aircraft",
		"Aircraft Frame & Body Config")

public:
	FAircraftFrameConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	UPROPERTY(EditAnywhere, Category = "Aircraft", meta = (DataflowInput, DataflowOutput, DataflowPassthrough = "Collection"))
	FManagedArrayCollection Collection;

	/** 机体根骨骼，用于物理刚体绑定。 */
	UPROPERTY(EditAnywhere, Category = "Frame", meta = (DataflowInput))
	FName RootBone = NAME_None;

	/** 机架布局类型（决定默认旋翼放置）。 */
	UPROPERTY(EditAnywhere, Category = "Frame")
	EAircraftFrameTypeNode FrameType = EAircraftFrameTypeNode::QuadX;

	/** 机体总质量（千克）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (DataflowInput, ClampMin = "0.01"))
	float MassKg = 1.2f;

	/** 质心相对于根骨骼原点的偏移（厘米）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (DataflowInput))
	FVector3f CenterOfMassOffsetCm = FVector3f::ZeroVector;

	/** 主惯量对角线 Ixx, Iyy, Izz（千克·厘米²）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f InertiaDiagonalKgCmSq = FVector3f(5000.f, 5000.f, 9000.f);

	/** 线性阻尼系数（X/Y/Z），力 = -D · v。 */
	UPROPERTY(EditAnywhere, Category = "Frame|Aero", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f LinearDragPerAxis = FVector3f(0.12f, 0.12f, 0.18f);

	/** 角阻尼系数（Roll/Pitch/Yaw），力矩 = -A · ω。 */
	UPROPERTY(EditAnywhere, Category = "Frame|Aero", meta = (DataflowInput, ClampMin = "0.0"))
	FVector3f AngularDragPerAxis = FVector3f(0.02f, 0.02f, 0.03f);

	/** 外部风场速度（厘米/秒，世界系）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|Aero", meta = (DataflowInput))
	FVector3f WindVelocityCmPerSec = FVector3f::ZeroVector;

	/** 地面效应起始高度（厘米）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|Aero", meta = (DataflowInput, ClampMin = "0.0"))
	float GroundEffectStartHeightCm = 80.f;

	/** 地面效应额外推力比例（0~1）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|Aero", meta = (DataflowInput, ClampMin = "0.0", ClampMax = "1.0"))
	float GroundEffectStrength = 0.15f;

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
