#pragma once

#include "BaseCharacterFXEditorModule.h"
#include "ComponentAssetBroker.h"

class UAircraftAssetBase;

class FAircraftAssetEditorModule : public FBaseCharacterFXEditorModule
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	TWeakObjectPtr<UAircraftAssetBase> ActiveAircraftAsset;
	TSharedPtr<IComponentAssetBroker> AircraftAssetComponentBroker;
};
