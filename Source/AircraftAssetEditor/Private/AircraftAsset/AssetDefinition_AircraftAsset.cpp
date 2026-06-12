// Fill out your copyright notice in the Description page of Project Settings.


#include "AircraftAsset/AssetDefinition_AircraftAsset.h"

#include "AircraftAsset/AircraftAsset.h"
#include "ThumbnailRendering/SceneThumbnailInfo.h"
#include "Toolkits/SimpleAssetEditor.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftAssetThumbnailRenderer.h"
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"

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
	return FLinearColor(0.09f, 0.39f, 0.24f);
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
	if (AircraftAssets.Num() > 0 )
	{
		if (UE::AircraftDataflowAssetEditor::Private::OpenAircraftAssetEditor(AircraftAssets[0]))
		{
			return EAssetCommandResult::Handled;
		}
		
		FSimpleAssetEditor::CreateEditor(EToolkitMode::Standalone, OpenArgs.ToolkitHost, AircraftAssets[0]);
		return EAssetCommandResult::Handled;
	}

	return EAssetCommandResult::Unhandled;
}
