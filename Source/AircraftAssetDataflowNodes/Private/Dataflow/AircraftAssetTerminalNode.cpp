#include "Dataflow/AircraftAssetTerminalNode.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAssetTerminalNode)

namespace UE::AircraftLab::AircraftAsset::Private
{
	static void ResetDisconnectedInputs(FAircraftAssetTerminalNode& Node)
	{
		if (!Node.IsConnected(&Node.Collection))
		{
			Node.Collection = FManagedArrayCollection();
		}

		if (!Node.IsConnected(&Node.AircraftAsset))
		{
			Node.AircraftAsset = nullptr;
		}
	}
}

FAircraftAssetTerminalNode::FAircraftAssetTerminalNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowTerminalNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterInputConnection(&AircraftAsset);
}


void FAircraftAssetTerminalNode::SetAssetValue(TObjectPtr<UObject> Asset, UE::Dataflow::FContext& Context) const
{
	UAircraftAsset* AircraftAssetObject = Cast<UAircraftAsset>(Asset.Get());
	if (!AircraftAssetObject)
	{
		const TObjectPtr<UAircraftAssetBase> AssetInput = GetValue(Context, &AircraftAsset);
		AircraftAssetObject = Cast<UAircraftAsset>(AssetInput.Get());
	}

	if (AircraftAssetObject)
	{
		FManagedArrayCollection AircraftCollection = GetValue(Context, &Collection);
		TArray<TSharedRef<const FManagedArrayCollection>> Collections;
		Collections.Add(MakeShared<FManagedArrayCollection>(MoveTemp(AircraftCollection)));

		FText ErrorText;
		FText VerboseText;
		AircraftAssetObject->Build(Collections, &ErrorText, &VerboseText);

		if (!ErrorText.IsEmpty())
		{
			//FAircraftDataflowTools::LogAndToastWarning(*this, ErrorText, VerboseText);
		}

		// Asset must be resaved
		AircraftAssetObject->MarkPackageDirty();
	}
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::AddPins()
{
	return FDataflowTerminalNode::AddPins();
}

TArray<UE::Dataflow::FPin> FAircraftAssetTerminalNode::GetPinsToRemove() const
{
	return FDataflowTerminalNode::GetPinsToRemove();
}

void FAircraftAssetTerminalNode::OnPinRemoved(const UE::Dataflow::FPin& Pin)
{
	FDataflowTerminalNode::OnPinRemoved(Pin);
}

void FAircraftAssetTerminalNode::OnInvalidate()
{
	UE::AircraftLab::AircraftAsset::Private::ResetDisconnectedInputs(*this);
}

void FAircraftAssetTerminalNode::PostSerialize(const FArchive& Ar)
{
	FDataflowTerminalNode::PostSerialize(Ar);

	if (Ar.IsLoading() || Ar.IsTransacting())
	{
		UE::AircraftLab::AircraftAsset::Private::ResetDisconnectedInputs(*this);
	}
}

