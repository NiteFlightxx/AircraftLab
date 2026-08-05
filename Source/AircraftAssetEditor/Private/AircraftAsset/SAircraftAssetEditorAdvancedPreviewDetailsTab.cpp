// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/SClothEditorAdvancedPreviewDetailsTab.cpp。

#include "AircraftAsset/SAircraftAssetEditorAdvancedPreviewDetailsTab.h"

#include "AircraftAsset/AircraftPreviewSceneDescription.h"
#include "IDetailsView.h"

SAircraftAssetEditorAdvancedPreviewDetailsTab::SAircraftAssetEditorAdvancedPreviewDetailsTab()
	: SAdvancedPreviewDetailsTab()
{
	PropertyChangedDelegateHandle = FCoreUObjectDelegates::OnObjectPropertyChanged.AddLambda([this](UObject* Object, struct FPropertyChangedEvent& Event)
	{
		if (Object->IsA<UAircraftPreviewSceneDescription>())
		{
			SettingsView->InvalidateCachedState();
		}
	});
}

SAircraftAssetEditorAdvancedPreviewDetailsTab::~SAircraftAssetEditorAdvancedPreviewDetailsTab()
{
	FCoreUObjectDelegates::OnObjectPropertyChanged.Remove(PropertyChangedDelegateHandle);
}
