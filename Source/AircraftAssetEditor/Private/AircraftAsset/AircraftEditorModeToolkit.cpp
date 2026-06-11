#include "AircraftAsset/AircraftEditorModeToolkit.h"

#include "Styling/AppStyle.h"

#define LOCTEXT_NAMESPACE "AircraftAssetEditorModeToolkit"

FName FAircraftAssetEditorModeToolkit::GetToolkitFName() const
{
	return FName("AircraftAssetEditorMode");
}

FText FAircraftAssetEditorModeToolkit::GetBaseToolkitName() const
{
	return LOCTEXT("DisplayName", "Aircraft Asset Editor Mode");
}

const FSlateBrush* FAircraftAssetEditorModeToolkit::GetActiveToolIcon(const FString& Identifier) const
{
	return FAppStyle::GetBrush("NoBrush");
}

#undef LOCTEXT_NAMESPACE
