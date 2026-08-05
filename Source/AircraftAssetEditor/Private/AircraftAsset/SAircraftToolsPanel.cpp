// 侧边工具面板：两个空 Group（General / Aircraft），供后续工具开发填充。

#include "AircraftAsset/SAircraftToolsPanel.h"

#include "Styling/AppStyle.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "SAircraftToolsPanel"

void SAircraftToolsPanel::Construct(const FArguments& InArgs)
{
	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(4.0f)
		[
			BuildGroup(LOCTEXT("GeneralGroup", "General"))
		]
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(4.0f)
		[
			BuildGroup(LOCTEXT("AircraftGroup", "Aircraft"))
		]
	];
}

TSharedRef<SWidget> SAircraftToolsPanel::BuildGroup(const FText& GroupName)
{
	return SNew(SBorder)
		.BorderImage(FAppStyle::Get().GetBrush("ToolPanel.GroupBorder"))
		.Padding(4.0f)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot()
			.AutoHeight()
			[
				SNew(STextBlock)
				.Text(GroupName)
				.Font(FAppStyle::GetFontStyle(TEXT("DetailsView.CategoryFontStyle")))
			]
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(0.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("EmptyGroupHint", "No tools registered."))
				.ColorAndOpacity(FSlateColor::UseSubduedForeground())
			]
		];
}

#undef LOCTEXT_NAMESPACE
