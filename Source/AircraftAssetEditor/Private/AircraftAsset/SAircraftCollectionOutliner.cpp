// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/SClothCollectionOutliner.cpp。

#include "AircraftAsset/SAircraftCollectionOutliner.h"

#include "GeometryCollection/ManagedArrayCollection.h"
#include "Styling/AppStyle.h"
#include "Widgets/Views/SHeaderRow.h"

#define LOCTEXT_NAMESPACE "AircraftCollectionOutliner"

const FName FAircraftCollectionHeaderData::ColumnZeroName = FName("ID");

void SAircraftCollectionOutlinerRow::Construct(const FArguments& InArgs,
	TSharedRef<STableViewBase> OwnerTableView,
	const TSharedPtr<const FAircraftCollectionHeaderData>& InHeaderData,
	const TSharedPtr<const FAircraftCollectionItem>& InItemToEdit)
{
	HeaderData = InHeaderData;
	Item = InItemToEdit;

	SMultiColumnTableRow<TSharedPtr<const FAircraftCollectionItem>>::Construct(
		FSuperRowType::FArguments()
		.Style(&FAppStyle().GetWidgetStyle<FTableRowStyle>("TableView.AlternatingRow"))
		, OwnerTableView);
}

TSharedRef<SWidget> SAircraftCollectionOutlinerRow::GenerateWidgetForColumn(const FName& ColumnName)
{
	int32 FoundIndex;
	if (HeaderData->AttributeNames.Find(ColumnName, FoundIndex))
	{
		const FString& AttrValue = Item->AttributeValues[FoundIndex];

		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot()
			.AutoWidth()
			[
				SNew(STextBlock)
				.Text(FText::FromString(AttrValue))
			];
	}

	return SNullWidget::NullWidget;
}

void SAircraftCollectionOutliner::Construct(const FArguments& InArgs)
{
	AircraftCollection = InArgs._AircraftCollection;
	SelectedGroupName = InArgs._SelectedGroupName;

	HeaderRowWidget =
		SNew(SHeaderRow)
		.Visibility(EVisibility::Visible);

	if (AircraftCollection.IsValid())
	{
		RegenerateHeader();
		RepopulateListView();
	}

	ChildSlot
	[
		SNew(SVerticalBox)

		+ SVerticalBox::Slot()
		.AutoHeight()
		[
			SAssignNew(SelectedGroupNameComboBox, SComboBox<FName>)
			.OptionsSource(&AircraftCollectionGroupNames)
			.OnSelectionChanged(SComboBox<FName>::FOnSelectionChanged::CreateLambda(
				[this](FName SelectedName, ESelectInfo::Type)
				{
					SetSelectedGroupName(SelectedName);
				}))
			.OnGenerateWidget(SComboBox<FName>::FOnGenerateWidget::CreateLambda(
				[](FName Item)
				{
					return SNew(STextBlock)
						.Text(FText::FromName(Item));
				}))
			[
				SNew(STextBlock)
				.Text_Lambda([this]()
				{
					return FText::FromName(GetSelectedGroupName());
				})
			]
		]

		+ SVerticalBox::Slot()
		.Padding(FMargin(0.0f, 3.f))
		[
			SNew(SOverlay)
			+ SOverlay::Slot()
			[
				SAssignNew(ListView, SListView<TSharedPtr<const FAircraftCollectionItem>>)
				.SelectionMode(ESelectionMode::None)
				.ListItemsSource(&ListItems)
				.OnGenerateRow(this, &SAircraftCollectionOutliner::GenerateRow)
				.HeaderRow(HeaderRowWidget)
			]
		]
	];
}

void SAircraftCollectionOutliner::SetAircraftCollection(TSharedPtr<FManagedArrayCollection> InAircraftCollection)
{
	if (!InAircraftCollection.IsValid() && AircraftCollection.IsValid())
	{
		// 从"有选中节点"切换到"无选中节点"
		SavedLastValidGroupName = SelectedGroupName;
	}
	else if (InAircraftCollection.IsValid() && !AircraftCollection.IsValid())
	{
		// 从"无选中节点"切换到"有选中节点"
		SelectedGroupName = SavedLastValidGroupName;
	}

	AircraftCollection = InAircraftCollection;

	if (AircraftCollection.IsValid())
	{
		AircraftCollectionGroupNames = AircraftCollection->GroupNames();
		RegenerateHeader();
		RepopulateListView();
	}
	else
	{
		AircraftCollectionGroupNames.Reset();
		HeaderRowWidget->ClearColumns();
		if (HeaderData)
		{
			HeaderData->AttributeNames.Empty();
		}
		ListItems.Empty();
		SelectedGroupName = FName();
	}
	// Group 名集合变化后刷新下拉框
	SelectedGroupNameComboBox->RefreshOptions();
}

void SAircraftCollectionOutliner::SetSelectedGroupName(const FName& InSelectedGroupName)
{
	SelectedGroupName = InSelectedGroupName;

	RegenerateHeader();
	RepopulateListView();
}

const FName& SAircraftCollectionOutliner::GetSelectedGroupName() const
{
	return SelectedGroupName;
}

void SAircraftCollectionOutliner::RegenerateHeader()
{
	constexpr float CustomFillWidth = 2.0f;

	if (!AircraftCollection.IsValid())
	{
		return;
	}

	HeaderRowWidget->ClearColumns();

	HeaderData = MakeShared<FAircraftCollectionHeaderData>();
	HeaderData->AttributeNames.Add(FAircraftCollectionHeaderData::ColumnZeroName);
	HeaderData->AttributeNames.Append(AircraftCollection->AttributeNames(SelectedGroupName));

	int32 UnnamedCount = 0;
	for (int32 AttributeNameIndex = 0; AttributeNameIndex < HeaderData->AttributeNames.Num(); ++AttributeNameIndex)
	{
		FName AttrName = HeaderData->AttributeNames[AttributeNameIndex];
		if (AttrName == NAME_None)  // AddColumn 需要名字，否则会崩溃
		{
			AttrName = FName(FString::Format(TEXT("Unnamed{0}"), { UnnamedCount++ }));
		}

		HeaderRowWidget->AddColumn(
			SHeaderRow::Column(AttrName)
			.DefaultLabel(FText::FromName(AttrName))
			.FillWidth(CustomFillWidth)
		);
	}
}

namespace AircraftCollectionOutlinerHelpers
{
	template<typename T>
	FString AttributeValueToString(const T& Value)
	{
		return Value.ToString();
	}

	FString AttributeValueToString(float Value)
	{
		return FString::SanitizeFloat(Value);
	}

	FString AttributeValueToString(int32 Value)
	{
		return FString::FromInt(Value);
	}

	FString AttributeValueToString(uint8 Value)
	{
		return FString::FromInt(Value);
	}

	FString AttributeValueToString(const FString& Value)
	{
		return Value;
	}

	FString AttributeValueToString(const FName& Value)
	{
		return Value.ToString();
	}

	FString AttributeValueToString(bool Value)
	{
		return Value ? TEXT("true") : TEXT("false");
	}

	FString AttributeValueToString(const FSoftObjectPath& Value)
	{
		return Value.ToString();
	}

	FString AttributeValueToString(const FIntVector2& Value)
	{
		return FString::Printf(TEXT("X=%d Y=%d"), Value.X, Value.Y);
	}

	template<typename T>
	FString AttributeValueToString(const TArray<T>& Array)
	{
		FString Out;
		for (int32 ArrayIndex = 0; ArrayIndex < Array.Num(); ++ArrayIndex)
		{
			Out += AttributeValueToString(Array[ArrayIndex]);

			if (ArrayIndex != Array.Num() - 1)
			{
				Out += "; ";
			}
		}
		return Out;
	}

	template<typename T>
	FString AttributeValueToString(const TSet<T>& Set)
	{
		FString Out;
		typename TSet<T>::TConstIterator Iter = Set.CreateConstIterator();
		while (Iter)
		{
			Out += AttributeValueToString(*Iter);

			if (++Iter)
			{
				Out += "; ";
			}
		}
		return Out;
	}

	template<typename T>
	FString AttributeValueToString(const FManagedArrayCollection& InCollection, const FName& AttributeName, const FName& GroupName, int32 AttributeArrayIndex)
	{
		const TManagedArray<T>* const Array = InCollection.FindAttributeTyped<T>(AttributeName, GroupName);
		if (Array == nullptr)
		{
			return FString("(Unknown Attribute)");
		}

		return AttributeValueToString((*Array)[AttributeArrayIndex]);
	}

	FString AttributeValueToString(const FManagedArrayCollection& InCollection, const FName& AttributeName, const FName& GroupName, int32 AttributeArrayIndex)
	{
		const FManagedArrayCollection::EArrayType ArrayType = InCollection.GetAttributeType(AttributeName, GroupName);

		FString ValueAsString;

		switch (ArrayType)
		{
		case FManagedArrayCollection::EArrayType::FVectorType:
			ValueAsString = AttributeValueToString<FVector3f>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FVector2DType:
			ValueAsString = AttributeValueToString<FVector2f>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FFloatType:
			ValueAsString = AttributeValueToString<float>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FIntVectorType:
			ValueAsString = AttributeValueToString<FIntVector3>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FVector2DArrayType:
			ValueAsString = AttributeValueToString<TArray<FVector2f>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FLinearColorType:
			ValueAsString = AttributeValueToString<FLinearColor>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FInt32Type:
			ValueAsString = AttributeValueToString<int32>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FInt32ArrayType:
			ValueAsString = AttributeValueToString<TArray<int32>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FFloatArrayType:
			ValueAsString = AttributeValueToString<TArray<float>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FStringType:
			ValueAsString = AttributeValueToString<FString>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FIntVector2Type:
			ValueAsString = AttributeValueToString<FIntVector2>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FIntVector2ArrayType:
			ValueAsString = AttributeValueToString<TArray<FIntVector2>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FIntArrayType:
			ValueAsString = AttributeValueToString<TSet<int32>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FUInt8Type:
			ValueAsString = AttributeValueToString<uint8>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FIntVector3ArrayType:
			ValueAsString = AttributeValueToString<TArray<FIntVector3>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		case FManagedArrayCollection::EArrayType::FVector4fArrayType:
			ValueAsString = AttributeValueToString<TArray<FVector4f>>(InCollection, AttributeName, GroupName, AttributeArrayIndex);
			break;
		default:
			ensure(false);
			ValueAsString = "(Unknown Data Type)";
		}

		return ValueAsString;
	}
}

void SAircraftCollectionOutliner::RepopulateListView()
{
	ListItems.Empty();

	if (!AircraftCollection.IsValid())
	{
		return;
	}

	const int32 NumElements = AircraftCollection->NumElements(SelectedGroupName);

	for (int32 ElementIndex = 0; ElementIndex < NumElements; ++ElementIndex)
	{
		const TSharedPtr<FAircraftCollectionItem> NewItem = MakeShared<FAircraftCollectionItem>();
		NewItem->AttributeValues.SetNum(HeaderData->AttributeNames.Num());

		for (int32 AttributeNameIndex = 0; AttributeNameIndex < HeaderData->AttributeNames.Num(); ++AttributeNameIndex)
		{
			const FName& AttributeName = HeaderData->AttributeNames[AttributeNameIndex];
			if (AttributeName == FAircraftCollectionHeaderData::ColumnZeroName)
			{
				NewItem->AttributeValues[AttributeNameIndex] = FString::FromInt(ElementIndex);
			}
			else
			{
				NewItem->AttributeValues[AttributeNameIndex] = AircraftCollectionOutlinerHelpers::AttributeValueToString(*AircraftCollection, AttributeName, SelectedGroupName, ElementIndex);
			}
		}

		ListItems.Add(NewItem);
	}

	ListView->RequestListRefresh();
}

TSharedRef<ITableRow> SAircraftCollectionOutliner::GenerateRow(TSharedPtr<const FAircraftCollectionItem> InItem, const TSharedRef<STableViewBase>& OwnerTable)
{
	return SNew(SAircraftCollectionOutlinerRow, OwnerTable, this->HeaderData, InItem);
}

#undef LOCTEXT_NAMESPACE
