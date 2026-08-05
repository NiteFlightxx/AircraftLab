#pragma once

#include "BaseCharacterFXEditorModule.h"
#include "ComponentAssetBroker.h"

class UAircraftAssetBase;

namespace UE::AircraftLab::AircraftAsset
{
	class IAircraftDataflowTemplateProvider;
}

class FAircraftAssetEditorModule : public FBaseCharacterFXEditorModule
{
public:
	FAircraftAssetEditorModule() = default;
	// 外联析构：TUniquePtr 持有前置声明的 IAircraftDataflowTemplateProvider，
	// 在 cpp（类型完整处）实例化删除器，避免 C5205。
	virtual ~FAircraftAssetEditorModule() override;

	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	TWeakObjectPtr<UAircraftAssetBase> ActiveAircraftAsset;
	TSharedPtr<IComponentAssetBroker> AircraftAssetComponentBroker;
	TUniquePtr<UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider> TemplateProvider;
	FDelegateHandle DataflowAssetMenusHandle;
};
