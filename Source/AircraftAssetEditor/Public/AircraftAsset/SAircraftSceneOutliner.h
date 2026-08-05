// 预览场景大纲：以 STreeView 展示预览场景中的 Actor/组件层级。
// 参考 Persona / LevelEditor 的 Scene Outliner 交互，内容来自 FAircraftAssetEditorPreviewScene。

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/STreeView.h"

class FAircraftAssetEditorPreviewScene;
class UActorComponent;

class SAircraftSceneOutliner : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAircraftSceneOutliner) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, const TWeakPtr<FAircraftAssetEditorPreviewScene>& InPreviewScene);

	/** 场景内容变化（换资产/换网格）后刷新树。 */
	void Refresh();

	/** 点击组件行时请求视口聚焦（由 Toolkit 绑定到视口的 Focus 逻辑）。 */
	FSimpleDelegate& OnFocusRequested() { return FocusRequestedDelegate; }

private:
	struct FOutlinerItem
	{
		FString Label;
		FString TypeName;
		TArray<TSharedRef<FOutlinerItem>> Children;
	};

	void RepopulateTree();
	TSharedRef<ITableRow> OnGenerateRow(TSharedRef<FOutlinerItem> Item, const TSharedRef<STableViewBase>& OwnerTable);
	void OnGetChildren(TSharedRef<FOutlinerItem> Item, TArray<TSharedRef<FOutlinerItem>>& OutChildren);
	void OnSelectionChanged(TSharedPtr<FOutlinerItem> Item, ESelectInfo::Type SelectInfo);

	TWeakPtr<FAircraftAssetEditorPreviewScene> PreviewSceneWeak;
	TSharedPtr<STreeView<TSharedRef<FOutlinerItem>>> TreeView;
	TSharedPtr<FOutlinerItem> RootItem;
	TSharedPtr<FOutlinerItem> SelectedItem;
	FSimpleDelegate FocusRequestedDelegate;
};
