#pragma once

#include "Modules/ModuleManager.h"

class FAircraftAssetDataflowNodesModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
