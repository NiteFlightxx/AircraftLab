// 对齐 ChaosClothAssetEditor/Public/ChaosClothAsset/ClothEditorPreviewScene.h 中的
// UChaosClothPreviewSceneDescription：预览场景内容的可编辑描述对象，
// 供 FAdvancedPreviewSettingsWidget（Preview Scene Details 页签）编辑。

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"

#include "AircraftPreviewSceneDescription.generated.h"

class USkeletalMesh;
class UAnimationAsset;
class FAircraftAssetEditorPreviewScene;

DECLARE_EVENT(UAircraftPreviewSceneDescription, FAircraftPreviewSceneDescriptionChanged)

/**
 * 预览场景内容的描述对象，供 FAdvancedPreviewSettingsWidget 编辑。
 */
UCLASS(MinimalAPI)
class UAircraftPreviewSceneDescription : public UObject
{
public:
	GENERATED_BODY()

	FAircraftPreviewSceneDescriptionChanged AircraftPreviewSceneDescriptionChanged;

	UAircraftPreviewSceneDescription()
	{
		SetFlags(RF_Transactional);
	}

	AIRCRAFTASSETEDITOR_API void SetPreviewScene(FAircraftAssetEditorPreviewScene* PreviewScene);

	/** PIE/SIE 激活时预览视口是否暂停动画与仿真。 */
	UPROPERTY(EditAnywhere, Transient, Category = "Viewport")
	bool bPauseWhilePlayingInEditor = true;

	/** 预览用骨骼网格资产（由资产的 PreviewSceneSkeletalMesh 初始化）。 */
	UPROPERTY(EditAnywhere, Transient, Category = "SkeletalMesh")
	TObjectPtr<USkeletalMesh> SkeletalMeshAsset;

	/** 预览动画资产（在预览网格上以单节点实例播放）。 */
	UPROPERTY(EditAnywhere, Transient, Category = "SkeletalMesh")
	TObjectPtr<UAnimationAsset> AnimationAsset;

	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;

private:
	virtual void PostTransacted(const FTransactionObjectEvent& TransactionEvent) override;

	FAircraftAssetEditorPreviewScene* PreviewScene = nullptr;
};
