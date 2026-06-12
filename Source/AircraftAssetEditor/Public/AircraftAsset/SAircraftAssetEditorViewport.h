#pragma once

#include "SBaseCharacterFXEditorViewport.h"
#include "SCommonEditorViewportToolbarBase.h"

class FAircraftAssetEditorPreviewScene;

class SAircraftAssetEditorViewport : public SBaseCharacterFXEditorViewport, public ICommonEditorViewportToolbarInfoProvider
{
public:
	SLATE_BEGIN_ARGS(SAircraftAssetEditorViewport) {}
		SLATE_ARGUMENT(TSharedPtr<FEditorViewportClient>, EditorViewportClient)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs);

	virtual void BindCommands() override;
	virtual TSharedPtr<SWidget> BuildViewportToolbar() override;
	virtual TSharedPtr<IPreviewProfileController> CreatePreviewProfileController() override;
	virtual bool IsVisible() const override;
	virtual void OnFocusViewportToSelection() override;

	virtual TSharedRef<SEditorViewport> GetViewportWidget() override;
	virtual TSharedPtr<FExtender> GetExtenders() const override;
	virtual void OnFloatingButtonClicked() override {}

private:
	TWeakPtr<FAircraftAssetEditorPreviewScene> GetPreviewScene() const;
};
