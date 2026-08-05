#include "AircraftAsset/AircraftDataflowPreviewActor.h"

#include "Animation/AnimationAsset.h"
#include "Engine/SkeletalMesh.h"

#include "AircraftAsset/AircraftComponent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftDataflowPreviewActor)

AAircraftDataflowPreviewActor::AAircraftDataflowPreviewActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	AircraftComponent = CreateDefaultSubobject<UAircraftComponent>(TEXT("AircraftComponent0"));
	RootComponent = AircraftComponent;
}

void AAircraftDataflowPreviewActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 引擎 spawn 路径（UE::Dataflow::SpawnSimulatedActor）以 bDeferConstruction=true 生成，
	// SetActorProperties 覆写完三个注入属性之后才 FinishSpawning → 到达这里时
	// DataflowAsset / SkeletalMesh / AnimationAsset 均已就位。
	SyncComponentFromInjectedProperties();
}

#if WITH_EDITOR
void AAircraftDataflowPreviewActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	SyncComponentFromInjectedProperties();
}
#endif

void AAircraftDataflowPreviewActor::SyncComponentFromInjectedProperties()
{
	if (!AircraftComponent)
	{
		return;
	}

	// 1) 资产绑定：UAircraftComponent::SetAsset 内部会同步骨骼网格/物理资产/仿真模型。
	if (AircraftComponent->GetAsset() != DataflowAsset)
	{
		AircraftComponent->SetAsset(DataflowAsset);
	}

	// 2) 预览网格覆盖：编辑器内容面板上用户可另行指定预览网格（与布料的 SkeletalMesh 变量语义一致）。
	if (SkeletalMesh && AircraftComponent->GetSkeletalMeshAsset() != SkeletalMesh)
	{
		AircraftComponent->SetSkeletalMeshAsset(SkeletalMesh);
	}

	// 3) 预览动画（无人机通常不播动画，但保持与布料预览一致的通路）。
	if (AnimationAsset && AircraftComponent->AnimationData.AnimToPlay != AnimationAsset)
	{
		AircraftComponent->SetAnimationMode(EAnimationMode::Type::AnimationSingleNode);
		AircraftComponent->SetAnimation(AnimationAsset);
	}
}
