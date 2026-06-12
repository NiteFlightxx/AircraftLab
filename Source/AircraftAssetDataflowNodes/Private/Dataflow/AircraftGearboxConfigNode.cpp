#include "Dataflow/AircraftGearboxConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"


#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftGearboxConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static FString SerializeFloatArray(const TArray<float>& Values)
	{
		TArray<FString> Tokens;
		Tokens.Reserve(Values.Num());

		for (const float Value : Values)
		{
			Tokens.Add(LexToString(Value));
		}

		return FString::Join(Tokens, TEXT(","));
	}
}

FAircraftGearboxConfigNode::FAircraftGearboxConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	//RegisterInputConnection(&GearboxType);
	RegisterInputConnection(&ForwardRatios);
	RegisterInputConnection(&ReverseRatios);
	RegisterInputConnection(&FinalDriveRatio);
	RegisterInputConnection(&ShiftUpRPM);
	RegisterInputConnection(&ShiftDownRPM);
	RegisterInputConnection(&bAutoReverse);
}

void FAircraftGearboxConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftGearboxConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Powertrain) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Powertrain);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Powertrain) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Powertrain);
	}

	InFacade.FindOrAddAttribute<FString>(
		AircraftCollectionAttribute::PowertrainGearboxForwardRatios,
		AircraftCollectionGroup::Powertrain)[0] =
		UE::AircraftLab::AircraftAssetDataflowNodes::Private::SerializeFloatArray(ForwardRatios);
	InFacade.FindOrAddAttribute<FString>(
		AircraftCollectionAttribute::PowertrainGearboxReverseRatios,
		AircraftCollectionGroup::Powertrain)[0] =
		UE::AircraftLab::AircraftAssetDataflowNodes::Private::SerializeFloatArray(ReverseRatios);
	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainGearboxFinalDriveRatio,
		AircraftCollectionGroup::Powertrain)[0] = FinalDriveRatio;
	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainGearboxShiftUpRPM,
		AircraftCollectionGroup::Powertrain)[0] = ShiftUpRPM;
	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainGearboxShiftDownRPM,
		AircraftCollectionGroup::Powertrain)[0] = ShiftDownRPM;
	InFacade.FindOrAddAttribute<bool>(
		AircraftCollectionAttribute::PowertrainGearboxAutoReverse,
		AircraftCollectionGroup::Powertrain)[0] = bAutoReverse;
}
