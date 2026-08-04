#include "AircraftAsset/AircraftAssetEditorCommands.h"

#include "AircraftAsset/AircraftAssetEditorStyle.h"

#define LOCTEXT_NAMESPACE "AircraftAssetEditorCommands"

const FString FAircraftAssetEditorCommands::ToggleSimulationSuspendedIdentifier = TEXT("ToggleSimulationSuspended");
const FString FAircraftAssetEditorCommands::SoftResetSimulationIdentifier = TEXT("SoftResetSimulation");
const FString FAircraftAssetEditorCommands::HardResetSimulationIdentifier = TEXT("HardResetSimulation");
const FString FAircraftAssetEditorCommands::TogglePreviewWireframeIdentifier = TEXT("TogglePreviewWireframe");
const FString FAircraftAssetEditorCommands::MotorPlacementIdentifier = TEXT("AircraftMotorPlacement");
const FString FAircraftAssetEditorCommands::PidTuningIdentifier = TEXT("AircraftPidTuning");
const FString FAircraftAssetEditorCommands::ThrustVectorOrientationIdentifier = TEXT("AircraftThrustVectorOrientation");

FAircraftAssetEditorCommands::FAircraftAssetEditorCommands()
	: TCommands<FAircraftAssetEditorCommands>(
		// 命令上下文名，必须与 FAircraftAssetEditorStyle 的图标 key 前缀完全一致
		// （即 "AircraftAssetEditor.SoftResetSimulation" 中的 "AircraftAssetEditor"）。
		TEXT("AircraftAssetEditor"),
		LOCTEXT("ContextDescription", "Aircraft Asset Editor"),
		NAME_None,
		// 关键：StyleSetName 指向我们自家 StyleSet 的 GetStyleSetName()，让 FCommandInfo::GetIcon()
		// 能从 FAircraftAssetEditorStyle 中查到 brush；这是按钮在 SetShowInToolbarTopLevel(true)
		// 模式下能渲染图标的必要前提。
		FAircraftAssetEditorStyle::GetStyleName())
{
}

void FAircraftAssetEditorCommands::RegisterCommands()
{
	UI_COMMAND(TogglePreviewWireframe, "Preview Wireframe", "Toggle preview wireframe.", EUserInterfaceActionType::ToggleButton, FInputChord());
	UI_COMMAND(SoftResetSimulation, "Soft Reset Simulation", "Soft reset the Aircraft simulation state.", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(HardResetSimulation, "Hard Reset Simulation", "Hard reset the Aircraft preview simulation.", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control | EModifierKey::Alt, EKeys::V));
	UI_COMMAND(ToggleSimulationSuspended, "Toggle Simulation", "Pause or resume the Aircraft preview simulation.", EUserInterfaceActionType::ToggleButton, FInputChord());
	UI_COMMAND(MotorPlacement, "Motor Placement", "Edit rotor application points in the preview viewport.", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(PidTuning, "PID Tuning", "Tune flight-controller gains while previewing the aircraft.", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ThrustVectorOrientation, "Thrust Orientation", "Edit rotor thrust-vector orientation in the preview viewport.", EUserInterfaceActionType::Button, FInputChord());
}

#undef LOCTEXT_NAMESPACE
