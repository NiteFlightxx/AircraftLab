// 写入单元素 Frame group 的全部静态字段。

#pragma once

#include "CoreMinimal.h"
#include "Aircraft/AircraftFrameBinding.h"
#include "Dataflow/AircraftConfigNodeBase.h"

#include "AircraftFrameConfigNode.generated.h"

/** 机架配置节点：写入物理绑定、控制轴约定、质量、质心和惯量。 */
USTRUCT(meta = (DataflowAircraft))
struct FAircraftFrameConfigNode : public FAircraftConfigNodeBase
{
	GENERATED_BODY()
	DATAFLOW_NODE_DEFINE_INTERNAL(
		FAircraftFrameConfigNode,
		"AircraftFrameConfig",
		"Aircraft",
		"Aircraft Frame & Body Config")

public:
	FAircraftFrameConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid = FGuid::NewGuid());

	/** 机体根骨骼，用于物理刚体绑定。 */
	UPROPERTY(EditAnywhere, Category = "Frame", meta = (DisplayName = "Root Bone"))
	FName RootBone = NAME_None;

	/** 蒙皮模型局部空间中的机头方向；模型 Up 固定为 +Z。 */
	UPROPERTY(EditAnywhere, Category = "Frame", meta = (DisplayName = "Forward Axis"))
	EAircraftModelForwardAxis ForwardAxis = EAircraftModelForwardAxis::PositiveY;

	/** 机体总质量（千克）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (DisplayName = "Total Mass (kg)", ClampMin = "0.01"))
	float MassKg = 1.2f;

	/** 在 PhysicsAsset 计算质心基础上施加的局部偏移（厘米）。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (DisplayName = "Center Of Mass Nudge (cm)"))
	FVector3f CenterOfMassNudgeCm = FVector3f::ZeroVector;

	/** PhysicsAsset 计算出的惯性张量逐轴缩放；(1,1,1) 保持原始惯性。 */
	UPROPERTY(EditAnywhere, Category = "Frame|MassInertia", meta = (DisplayName = "Inertia Tensor Scale", ClampMin = "0.01"))
	FVector3f InertiaTensorScale = FVector3f::OneVector;

protected:
	virtual bool ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const override;
#if WITH_EDITOR
	virtual bool HighlightsRootBodyInConstruction() const override { return true; }
#endif
};
