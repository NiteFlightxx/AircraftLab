#pragma once

#include "BaseCharacterFXEditorModeUILayer.h"
#include "AircraftEditorModeUILayer.generated.h"

UCLASS()
class UAircraftAssetEditorUISubsystem : public UBaseCharacterFXEditorUISubsystem
{
	GENERATED_BODY()

protected:
	virtual FName GetModuleName() const override
	{
		return TEXT("AircraftAssetEditor");
	}
};

class FAircraftAssetEditorModeUILayer : public FBaseCharacterFXEditorModeUILayer
{
public:
	explicit FAircraftAssetEditorModeUILayer(const IToolkitHost* InToolkitHost)
		: FBaseCharacterFXEditorModeUILayer(InToolkitHost)
	{
	}
};
