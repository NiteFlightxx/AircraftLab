#pragma once

#include "Framework/Commands/Commands.h"

class FAircraftAssetEditorCommands final : public TCommands<FAircraftAssetEditorCommands>
{
public:
	FAircraftAssetEditorCommands();

	virtual void RegisterCommands() override;

	TSharedPtr<FUICommandInfo> SoftResetSimulation;
	TSharedPtr<FUICommandInfo> HardResetSimulation;
	TSharedPtr<FUICommandInfo> ToggleSimulationSuspended;
};
