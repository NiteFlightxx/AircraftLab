// 对齐 ChaosClothAssetEditor/Public/ChaosClothAsset/SClothEditorAdvancedPreviewDetailsTab.h：
// SAdvancedPreviewDetailsTab 的子类，用于在预览场景描述对象变化时刷新 SettingsView 缓存。

#pragma once

#include "SAdvancedPreviewDetailsTab.h"

class SAircraftAssetEditorAdvancedPreviewDetailsTab : public SAdvancedPreviewDetailsTab
{
public:

	SAircraftAssetEditorAdvancedPreviewDetailsTab();
	virtual ~SAircraftAssetEditorAdvancedPreviewDetailsTab() override;

private:

	FDelegateHandle PropertyChangedDelegateHandle;
};
