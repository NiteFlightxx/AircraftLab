// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/SClothCollectionOutliner.h：
// FManagedArrayCollection 数据查看器——按 Group 提取属性名与值形成表格。

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Input/SComboBox.h"

struct FManagedArrayCollection;

struct FAircraftCollectionHeaderData
{
	TArray<FName> AttributeNames;

	// 第一列显示元素数字索引
	static const FName ColumnZeroName;
};

struct FAircraftCollectionItem
{
	TArray<FString> AttributeValues;
};

class SAircraftCollectionOutlinerRow : public SMultiColumnTableRow<TSharedPtr<const FAircraftCollectionItem>>
{
public:
	SLATE_BEGIN_ARGS(SAircraftCollectionOutlinerRow) {}
		SLATE_ARGUMENT(TSharedPtr<const FAircraftCollectionHeaderData>, HeaderData)
		SLATE_ARGUMENT(TSharedPtr<const FAircraftCollectionItem>, Item)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, TSharedRef<STableViewBase> OwnerTableView, const TSharedPtr<const FAircraftCollectionHeaderData>& InHeaderData, const TSharedPtr<const FAircraftCollectionItem>& InItemToEdit);

	// SMultiColumnTableRow
	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& ColumnName) override;

private:

	TSharedPtr<const FAircraftCollectionHeaderData> HeaderData;
	TSharedPtr<const FAircraftCollectionItem> Item;
};

/// FManagedArrayCollection 数据查看器：按 Group 提取所有属性名与值形成表格。
class SAircraftCollectionOutliner : public SCompoundWidget
{
public:

	SLATE_BEGIN_ARGS(SAircraftCollectionOutliner) {}
		SLATE_ARGUMENT(TSharedPtr<FManagedArrayCollection>, AircraftCollection)
		SLATE_ARGUMENT(FName, SelectedGroupName)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	void SetAircraftCollection(TSharedPtr<FManagedArrayCollection> AircraftCollection);

	/// 切换选中 Group 会同时重建表头与所有行
	void SetSelectedGroupName(const FName& SelectedGroupName);

	const FName& GetSelectedGroupName() const;

	TSharedRef<ITableRow> GenerateRow(TSharedPtr<const FAircraftCollectionItem> InItem, const TSharedRef<STableViewBase>& OwnerTable);

private:

	TSharedPtr<FManagedArrayCollection> AircraftCollection;

	TSharedPtr<SComboBox<FName>> SelectedGroupNameComboBox;
	TArray<FName> AircraftCollectionGroupNames;		// SelectedGroupNameComboBox 的数据源

	FName SelectedGroupName;
	FName SavedLastValidGroupName;

	TSharedPtr<SListView<TSharedPtr<const FAircraftCollectionItem>>> ListView;
	TArray<TSharedPtr<const FAircraftCollectionItem>> ListItems;

	TSharedPtr<FAircraftCollectionHeaderData> HeaderData;
	TSharedPtr<SHeaderRow> HeaderRowWidget;

	void RegenerateHeader();
	void RepopulateListView();
};
