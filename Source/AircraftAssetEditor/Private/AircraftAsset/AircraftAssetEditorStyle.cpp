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
		
		// Some standard icon sizes used elsewhere in the editor
		const FVector2D Icon8x8(8.0f, 8.0f);
		const FVector2D Icon16x16(16.0f, 16.0f);
		const FVector2D Icon20x20(20.0f, 20.0f);
		const FVector2D Icon28x28(28.0f, 28.0f);
		const FVector2D Icon40x40(40.0f, 40.0f);
		const FVector2D Icon120(120.0f, 120.0f);

		// Icon sizes used in this style set
		const FVector2D ViewportToolbarIconSize = Icon16x16;
		const FVector2D ToolbarIconSize = Icon20x20;

		FString PropertyNameString = "ChaosClothAssetEditor." + FAircraftAssetEditorCommands::TogglePreviewWireframeIdentifier;
		
		PropertyNameString = "ChaosClothAssetEditor." + FAircraftAssetEditorCommands::TogglePreviewWireframeIdentifier;
		Set(*PropertyNameString, new IMAGE_BRUSH_SVG("Icons/ClothWireframe_16", ViewportToolbarIconSize));

		PropertyNameString = "ChaosClothAssetEditor." + FAircraftAssetEditorCommands::ToggleSimulationSuspendedIdentifier;
		Set(*PropertyNameString, new IMAGE_BRUSH_SVG("Icons/ClothSimSuspend_16", ViewportToolbarIconSize));

		PropertyNameString = "ChaosClothAssetEditor." + FAircraftAssetEditorCommands::SoftResetSimulationIdentifier;
		Set(*PropertyNameString, new IMAGE_BRUSH_SVG("Icons/ResetSoft_16", ViewportToolbarIconSize));

		PropertyNameString = "ChaosClothAssetEditor." + FAircraftAssetEditorCommands::HardResetSimulationIdentifier;
		Set(*PropertyNameString, new IMAGE_BRUSH_SVG("Icons/ResetHard_16", ViewportToolbarIconSize));
		
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
