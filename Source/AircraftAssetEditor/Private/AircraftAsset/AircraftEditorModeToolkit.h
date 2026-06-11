#pragma once

#include "BaseCharacterFXEditorModeToolkit.h"

class FAircraftAssetEditorModeToolkit : public FBaseCharacterFXEditorModeToolkit
{
public:
	virtual FName GetToolkitFName() const override;
	virtual FText GetBaseToolkitName() const override;

protected:
	virtual const FSlateBrush* GetActiveToolIcon(const FString& Identifier) const override;
};
