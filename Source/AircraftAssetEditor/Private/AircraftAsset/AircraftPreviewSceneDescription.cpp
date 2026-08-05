// 对齐 ChaosClothAssetEditor ClothEditorPreviewScene.cpp 中
// UChaosClothPreviewSceneDescription 的实现。

#include "AircraftAsset/AircraftPreviewSceneDescription.h"

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "Misc/TransactionObjectEvent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPreviewSceneDescription)

void UAircraftPreviewSceneDescription::SetPreviewScene(FAircraftAssetEditorPreviewScene* InPreviewScene)
{
	PreviewScene = InPreviewScene;
}

void UAircraftPreviewSceneDescription::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PreviewScene)
	{
		PreviewScene->SceneDescriptionPropertyChanged(PropertyChangedEvent.GetMemberPropertyName());
	}

	AircraftPreviewSceneDescriptionChanged.Broadcast();
}

void UAircraftPreviewSceneDescription::PostTransacted(const FTransactionObjectEvent& TransactionEvent)
{
	Super::PostTransacted(TransactionEvent);

	// Undo/Redo 时 PostEditChangeProperty 只收到空的 FPropertyChangedEvent；
	// 这里有足够的信息判断哪些属性变了。
	if (TransactionEvent.GetEventType() == ETransactionObjectEventType::UndoRedo && TransactionEvent.HasPropertyChanges())
	{
		if (PreviewScene)
		{
			const TArray<FName>& PropertyNames = TransactionEvent.GetChangedProperties();
			for (const FName& PropertyName : PropertyNames)
			{
				PreviewScene->SceneDescriptionPropertyChanged(PropertyName);
			}
		}
	}
}
