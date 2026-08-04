#include "AircraftAsset/AircraftAssetEditorStyle.h"

#include "AircraftAsset/AircraftAssetEditorCommands.h"
#include "Interfaces/IPluginManager.h"
#include "Styling/AppStyle.h"
#include "Styling/SlateStyleMacros.h"
#include "Styling/SlateStyleRegistry.h"

const FName FAircraftAssetEditorStyle::StyleName(TEXT("AircraftStyle"));

FAircraftAssetEditorStyle::FAircraftAssetEditorStyle()
	: FSlateStyleSet(StyleName)
{
	// 命令 key 必须与 FAircraftAssetEditorCommands::TCommands 第一个参数 "AircraftAssetEditor"
	// 完全匹配，UE FCommandInfo::GetIcon() 会按 "ContextName.CommandName" 查 brush。
	SetCoreContentRoot(FPaths::EngineContentDir() / TEXT("Editor/Slate"));
	
	if (const TSharedPtr<IPlugin> AircraftAssetAssetEditorPlugin = IPluginManager::Get().GetModuleOwnerPlugin(UE_MODULE_NAME))
	{
		SetContentRoot(AircraftAssetAssetEditorPlugin->GetContentDir());
		
		auto RegisterAppStyleBrush = [this](const FString& CommandIdentifier, const FName AppStyleBrushName)
		{
			const FString PropertyName = TEXT("AircraftAssetEditor.") + CommandIdentifier;
			Set(*PropertyName, new FSlateBrush(*FAppStyle::Get().GetBrush(AppStyleBrushName)));
		};

		RegisterAppStyleBrush(FAircraftAssetEditorCommands::TogglePreviewWireframeIdentifier, TEXT("EditorViewport.WireframeMode"));
		RegisterAppStyleBrush(FAircraftAssetEditorCommands::ToggleSimulationSuspendedIdentifier, TEXT("Icons.Pause"));
		RegisterAppStyleBrush(FAircraftAssetEditorCommands::SoftResetSimulationIdentifier, TEXT("Icons.Refresh"));
		RegisterAppStyleBrush(FAircraftAssetEditorCommands::HardResetSimulationIdentifier, TEXT("Icons.Reset"));
		RegisterAppStyleBrush(FAircraftAssetEditorCommands::MotorPlacementIdentifier, TEXT("Icons.Transform"));
		RegisterAppStyleBrush(FAircraftAssetEditorCommands::PidTuningIdentifier, TEXT("Icons.Settings"));
		RegisterAppStyleBrush(FAircraftAssetEditorCommands::ThrustVectorOrientationIdentifier, TEXT("Icons.Rotate"));
		
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

const FName& FAircraftAssetEditorStyle::GetStyleName()
{
	return StyleName;
}
