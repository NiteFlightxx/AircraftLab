#pragma once

#include "SBaseCharacterFXEditorViewport.h"
#include "SCommonEditorViewportToolbarBase.h"

class FAircraftAssetEditorPreviewScene;
class SAircraftAnimationScrubPanel;

class SAircraftAssetEditorViewport : public SBaseCharacterFXEditorViewport, public ICommonEditorViewportToolbarInfoProvider
{
public:
	SLATE_BEGIN_ARGS(SAircraftAssetEditorViewport) {}
		SLATE_ARGUMENT(TSharedPtr<FEditorViewportClient>, EditorViewportClient)
		SLATE_ARGUMENT(TSharedPtr<FUICommandList>, ToolkitCommandList)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs);

	virtual void BindCommands() override;
	virtual TSharedPtr<SWidget> BuildViewportToolbar() override;
	virtual TSharedPtr<IPreviewProfileController> CreatePreviewProfileController() override;
	virtual bool IsVisible() const override;
	virtual void OnFocusViewportToSelection() override;
	virtual void PopulateViewportOverlays(TSharedRef<SOverlay> Overlay) override;

	virtual TSharedRef<SEditorViewport> GetViewportWidget() override;
	virtual TSharedPtr<FExtender> GetExtenders() const override;
	virtual void OnFloatingButtonClicked() override {}

private:
	TWeakPtr<FAircraftAssetEditorPreviewScene> GetPreviewScene() const;

	/** 底部动画控制条可见性（有预览动画实例时显示）。 */
	EVisibility GetAnimControlVisibility() const;

	/** 视口左上角状态文本（来自 FAircraftEditorSimulationVisualization）。 */
	FText GetViewportDisplayString() const;

	/** 动画拖动条输入范围。 */
	float GetViewMinInput() const;
	float GetViewMaxInput() const;

	TSharedPtr<FUICommandList> ToolkitCommandList;
};
