// 预览场景大纲实现：以 FAircraftAssetEditorPreviewScene 内容为数据源构建树。

#include "AircraftAsset/SAircraftSceneOutliner.h"

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "Widgets/Views/STreeView.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/AppStyle.h"

#include "Engine/Engine.h"

#define LOCTEXT_NAMESPACE "SAircraftSceneOutliner"

namespace
{
	const FText UnknownLabel = LOCTEXT("UnknownLabel", "<none>");
}

void SAircraftSceneOutliner::Construct(const FArguments& InArgs, const TWeakPtr<FAircraftAssetEditorPreviewScene>& InPreviewScene)
{
	PreviewSceneWeak = InPreviewScene;

	RootItem = MakeShared<FOutlinerItem>();
	RootItem->Label = TEXT("Root");
	RootItem->TypeName = TEXT("Scene");

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.FillHeight(1.0f)
		[
			SAssignNew(TreeView, STreeView<TSharedRef<FOutlinerItem>>)
			.TreeItemsSource(&RootItem->Children)
			.OnGenerateRow(this, &SAircraftSceneOutliner::OnGenerateRow)
			.OnGetChildren(this, &SAircraftSceneOutliner::OnGetChildren)
			.OnSelectionChanged(this, &SAircraftSceneOutliner::OnSelectionChanged)
		]
		+ SVerticalBox::Slot()
		.AutoHeight()
		[
			SNew(STextBlock)
			.Text(LOCTEXT("SceneHint", "Preview scene hierarchy. Select a row to focus it in the viewport."))
			.ColorAndOpacity(FSlateColor::UseSubduedForeground())
			.AutoWrapText(true)
		]
	];

	RepopulateTree();
}

void SAircraftSceneOutliner::Refresh()
{
	RepopulateTree();
}

void SAircraftSceneOutliner::RepopulateTree()
{
	if (!RootItem.IsValid())
	{
		return;
	}
	RootItem->Children.Reset();

	const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = PreviewSceneWeak.Pin();
	if (!PreviewScene.IsValid())
	{
		TreeView->RequestTreeRefresh();
		return;
	}

	UAircraftComponent* AircraftComponent = PreviewScene->GetAircraftComponent();
	if (!AircraftComponent)
	{
		TreeView->RequestTreeRefresh();
		return;
	}

	const AActor* const SceneActor = AircraftComponent->GetOwner();

	// 根部：场景 Actor
	TSharedRef<FOutlinerItem> ActorItem = MakeShared<FOutlinerItem>();
	ActorItem->Label = SceneActor ? SceneActor->GetName() : FString(TEXT("Scene Actor"));
	ActorItem->TypeName = SceneActor ? SceneActor->GetClass()->GetName() : FString(TEXT("Actor"));

	// 子项：AircraftComponent 及其挂接组件/动画实例
	TSharedRef<FOutlinerItem> AircraftItem = MakeShared<FOutlinerItem>();
	AircraftItem->Label = AircraftComponent->GetName();
	AircraftItem->TypeName = AircraftComponent->GetClass()->GetName();

	if (const USkeletalMeshComponent* const SkelMesh = Cast<USkeletalMeshComponent>(AircraftComponent))
	{
		TSharedRef<FOutlinerItem> SkeletalItem = MakeShared<FOutlinerItem>();
		SkeletalItem->Label = SkelMesh->GetSkeletalMeshAsset() ? SkelMesh->GetSkeletalMeshAsset()->GetName() : FString(TEXT("<no mesh>"));
		SkeletalItem->TypeName = SkelMesh->GetClass()->GetName();
		AircraftItem->Children.Add(SkeletalItem);

		if (UAnimInstance* const AnimInstance = SkelMesh->GetAnimInstance())
		{
			TSharedRef<FOutlinerItem> AnimItem = MakeShared<FOutlinerItem>();
			AnimItem->Label = AnimInstance->GetClass()->GetName();
			AnimItem->TypeName = TEXT("AnimInstance");
			SkeletalItem->Children.Add(AnimItem);
		}
	}

	ActorItem->Children.Add(AircraftItem);
	RootItem->Children.Add(ActorItem);

	TreeView->RequestTreeRefresh();
}

TSharedRef<ITableRow> SAircraftSceneOutliner::OnGenerateRow(TSharedRef<FOutlinerItem> Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	return SNew(STableRow<TSharedRef<FOutlinerItem>>, OwnerTable)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot()
			.AutoWidth()
			.Padding(4.0f, 0.0f)
			[
				SNew(STextBlock)
				.Text(FText::FromString(Item->Label))
			]
			+ SHorizontalBox::Slot()
			.AutoWidth()
			.Padding(4.0f, 0.0f)
			[
				SNew(STextBlock)
				.Text(FText::FromString(Item->TypeName))
				.ColorAndOpacity(FSlateColor::UseSubduedForeground())
			]
		];
}

void SAircraftSceneOutliner::OnGetChildren(TSharedRef<FOutlinerItem> Item, TArray<TSharedRef<FOutlinerItem>>& OutChildren)
{
	OutChildren = Item->Children;
}

void SAircraftSceneOutliner::OnSelectionChanged(TSharedPtr<FOutlinerItem> Item, ESelectInfo::Type SelectInfo)
{
	SelectedItem = Item;
	if (!Item.IsValid())
	{
		return;
	}

	// 选中机体组件行时请求视口聚焦（由 Toolkit 绑定到视口的 Focus 回调）
	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = PreviewSceneWeak.Pin())
	{
		if (UAircraftComponent* const AircraftComponent = PreviewScene->GetAircraftComponent())
		{
			if (Item->Label == AircraftComponent->GetName())
			{
				FocusRequestedDelegate.ExecuteIfBound();
			}
		}
	}
}

#undef LOCTEXT_NAMESPACE
