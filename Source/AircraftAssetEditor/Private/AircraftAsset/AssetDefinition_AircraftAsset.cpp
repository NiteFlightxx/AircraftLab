// Fill out your copyright notice in the Description page of Project Settings.


#include "AircraftAsset/AssetDefinition_AircraftAsset.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/ColorScheme.h"
#include "ThumbnailRendering/SceneThumbnailInfo.h"
#include "Toolkits/SimpleAssetEditor.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftAssetThumbnailRenderer.h"
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"
#include "HAL/IConsoleManager.h"

class UDataflowEditor;

FText UAssetDefinition_AircraftAsset::GetAssetDisplayName() const
{
	return NSLOCTEXT("AssetTypeActions", "AssetTypeActions_AircraftDataflowAsset", "Aircraft Dataflow Asset");
}

TSoftClassPtr<UObject> UAssetDefinition_AircraftAsset::GetAssetClass() const
{
	return UAircraftAssetBase::StaticClass();
}

FLinearColor UAssetDefinition_AircraftAsset::GetAssetColor() const
{
	// 资产色统一取自 FColorScheme（对齐 ChaosClothAssetTools/ColorScheme.h）
	return UE::AircraftLab::AircraftAsset::FColorScheme::Asset;
}

TConstArrayView<FAssetCategoryPath> UAssetDefinition_AircraftAsset::GetAssetCategories() const
{
	static const auto Categories = { EAssetCategoryPaths::Physics };
	return Categories;
}

UThumbnailInfo* UAssetDefinition_AircraftAsset::LoadThumbnailInfo(const FAssetData& InAssetData) const
{
	return UE::Editor::FindOrCreateThumbnailInfo(InAssetData.GetAsset(), USceneThumbnailInfo::StaticClass());
}

EAssetCommandResult UAssetDefinition_AircraftAsset::OpenAssets(const FAssetOpenArgs& OpenArgs) const
{
	TArray<UAircraftAsset*> AircraftAssets = OpenArgs.LoadObjects<UAircraftAsset>();

	ensure(AircraftAssets.Num() == 0 || AircraftAssets.Num() == 1);
	if (AircraftAssets.Num() > 0)
	{
		// 编辑器选项（Aircraft.EnableDataflowEditor CVar，经 UAircraftEditorOptions 双向绑定）分流，
		// 默认打开引擎统一的 FDataflowEditorToolkit（对齐 ChaosCloth 5.9：该面板从 5.8 起就是
		// 布料资产的正式编辑器，旧的自制 Panel Editor 路线已废弃）。
		// 直接读 CVar 以避免 AircraftAssetEditor → AircraftEditor 的模块依赖。
		bool bOpenInDataflowEditor = true;
		if (const IConsoleVariable* const CVar =
			IConsoleManager::Get().FindConsoleVariable(TEXT("Aircraft.EnableDataflowEditor")))
		{
			bOpenInDataflowEditor = CVar->GetBool();
		}

		if (bOpenInDataflowEditor
			&& UE::AircraftDataflowAssetEditor::Private::OpenAircraftAssetEditor(AircraftAssets[0]))
		{
			return EAssetCommandResult::Handled;
		}

		FSimpleAssetEditor::CreateEditor(EToolkitMode::Standalone, OpenArgs.ToolkitHost, AircraftAssets[0]);
		return EAssetCommandResult::Handled;
	}

	return EAssetCommandResult::Unhandled;
}
