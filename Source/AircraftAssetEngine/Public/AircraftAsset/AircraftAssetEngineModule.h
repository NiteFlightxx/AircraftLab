
#pragma once

#include "Modules/ModuleManager.h"

class IConsoleObject;

class FAircraftAssetEngineModule : public IModuleInterface
{
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	void HandlePreExit();

	FDelegateHandle PreExitDelegateHandle;
	IConsoleObject* SimulationResetCommand = nullptr;
};
