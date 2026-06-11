#include "AircraftAsset/AircraftAssetEditorCommands.h"

#include "Styling/AppStyle.h"

#define LOCTEXT_NAMESPACE "AircraftAssetEditorCommands"

FAircraftAssetEditorCommands::FAircraftAssetEditorCommands()
	: TCommands<FAircraftAssetEditorCommands>(
		TEXT("AircraftAssetEditor"),
		LOCTEXT("ContextDescription", "Aircraft Asset Editor"),
		NAME_None,
		FAppStyle::GetAppStyleSetName())
{
}

void FAircraftAssetEditorCommands::RegisterCommands()
{
	UI_COMMAND(SoftResetSimulation, "Soft Reset Simulation", "Soft reset the Aircraft simulation state.", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(HardResetSimulation, "Hard Reset Simulation", "Hard reset the Aircraft preview simulation.", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control | EModifierKey::Alt, EKeys::V));
	UI_COMMAND(ToggleSimulationSuspended, "Toggle Simulation", "Pause or resume the Aircraft preview simulation.", EUserInterfaceActionType::ToggleButton, FInputChord());
}

#undef LOCTEXT_NAMESPACE
