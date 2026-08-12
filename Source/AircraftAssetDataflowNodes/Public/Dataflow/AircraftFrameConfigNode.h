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

/** 模型局部前向轴；飞控标准坐标始终为 X=Forward、Y=Right、Z=Up。 */
UENUM()
enum class EAircraftForwardAxisNode : uint8
{
	PositiveX UMETA(DisplayName = "+X"),
	PositiveY UMETA(DisplayName = "+Y"),
	NegativeX UMETA(DisplayName = "-X"),
	NegativeY UMETA(DisplayName = "-Y"),
};

/** 机架配置节点：写入机架类型、控制轴约定、质量、质心和惯量。 */
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
	UPROPERTY(EditAnywhere, Category = "Frame")
	FName RootBone = NAME_None;

	/** 机架布局类型（决定默认旋翼放置）。 */
	UPROPERTY(EditAnywhere, Category = "Frame")
	EAircraftFrameTypeNode FrameType = EAircraftFrameTypeNode::QuadX;

	/** 视觉模型的局部机头方向。 */
	UPROPERTY(EditAnywhere, Category = "Frame")
	EAircraftForwardAxisNode ForwardAxis = EAircraftForwardAxisNode::PositiveY;

	/** 机体总质量（千克）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (ClampMin = "0.01"))
	float MassKg = 1.2f;

	/** 质心相对于根骨骼原点的偏移（厘米）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia")
	FVector3f CenterOfMassOffsetCm = FVector3f::ZeroVector;

	/** 主惯量对角线 Ixx, Iyy, Izz（千克·厘米²）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (ClampMin = "0.0"))
	FVector3f InertiaDiagonalKgCmSq = FVector3f(5000.f, 5000.f, 9000.f);

	virtual void Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const override;
};
