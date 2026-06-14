#include "AircraftAsset/AircraftAssetEditorStyle.h"

#include "Styling/AppStyle.h"
#include "Styling/SlateStyleRegistry.h"

const FName FAircraftAssetEditorStyle::StyleName(TEXT("AircraftStyle"));

FAircraftAssetEditorStyle::FAircraftAssetEditorStyle()
	: FSlateStyleSet(StyleName)
{
	// 命令 key 必须与 FAircraftAssetEditorCommands::TCommands 第一个参数 "AircraftAssetEditor"
	// 完全匹配，UE FCommandInfo::GetIcon() 会按 "ContextName.CommandName" 查 brush。
	const FSlateBrush* const RefreshBrush = FAppStyle::Get().GetBrush(TEXT("Icons.Refresh"));
	const FSlateBrush* const ResetBrush = FAppStyle::Get().GetBrush(TEXT("Icons.Toolbar.Settings"));
	const FSlateBrush* const PauseBrush = FAppStyle::Get().GetBrush(TEXT("GenericPlay.Pause"));

	if (RefreshBrush)
	{
		Set("AircraftAssetEditor.SoftResetSimulation", new FSlateBrush(*RefreshBrush));
	}
	if (ResetBrush)
	{
		Set("AircraftAssetEditor.HardResetSimulation", new FSlateBrush(*ResetBrush));
	}
	if (PauseBrush)
	{
		Set("AircraftAssetEditor.ToggleSimulationSuspended", new FSlateBrush(*PauseBrush));
	}

	FSlateStyleRegistry::RegisterSlateStyle(*this);
}

FAircraftAssetEditorStyle::~FAircraftAssetEditorStyle()
{
	FSlateStyleRegistry::UnRegisterSlateStyle(*this);
}

FAircraftAssetEditorStyle& FAircraftAssetEditorStyle::Get()
{
	static FAircraftAssetEditorStyle Instance;
	return Instance;
}

const FName& FAircraftAssetEditorStyle::GetStyleSetName()
{
	return StyleName;
}
