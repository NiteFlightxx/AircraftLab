#pragma once

#include "Framework/Commands/Commands.h"

class FAircraftAssetEditorCommands final : public TCommands<FAircraftAssetEditorCommands>
{
public:
	FAircraftAssetEditorCommands();

	virtual void RegisterCommands() override;

	// Preview viewport commands
	const static FString TogglePreviewWireframeIdentifier;
	TSharedPtr<FUICommandInfo> TogglePreviewWireframe;

	const static FString SoftResetSimulationIdentifier;
	TSharedPtr<FUICommandInfo> SoftResetSimulation;

	const static FString HardResetSimulationIdentifier;
	TSharedPtr<FUICommandInfo> HardResetSimulation;

	const static FString ToggleSimulationSuspendedIdentifier;
	TSharedPtr<FUICommandInfo> ToggleSimulationSuspended;

};
