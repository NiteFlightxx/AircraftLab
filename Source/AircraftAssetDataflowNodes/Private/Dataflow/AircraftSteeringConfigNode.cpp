#include "Dataflow/AircraftSteeringConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSteeringConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static int32 FindSteeringIndex(
		const TManagedArray<FName>& SteeringNames,
		const FName SteeringName)
	{
		for (int32 Index = 0; Index < SteeringNames.Num(); ++Index)
		{
			if (SteeringNames[Index] == SteeringName)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static int32 FindSteeringConfigWheelIndex(
		const TManagedArray<FName>& WheelNames,
		const FName WheelName)
	{
		for (int32 Index = 0; Index < WheelNames.Num(); ++Index)
		{
			if (WheelNames[Index] == WheelName)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}
}

FAircraftSteeringConfigNode::FAircraftSteeringConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllSteeringSystems);
	RegisterInputConnection(&SteeringName);
	RegisterInputConnection(&WheelNames);
	RegisterInputConnection(&MaxSteerAngleDeg);
	RegisterInputConnection(&AckermannRatio);
}

void FAircraftSteeringConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftSteeringConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (bReplaceAllSteeringSystems)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Steering);

		if (TManagedArray<FName>* const WheelSteeringNames =
			InFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelSteeringName, AircraftCollectionGroup::Wheels))
		{
			for (FName& WheelSteeringName : *WheelSteeringNames)
			{
				WheelSteeringName = NAME_None;
			}
		}
	}

	if (SteeringName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& SteeringNames =
		InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::SteeringName, AircraftCollectionGroup::Steering);
	TManagedArray<float>& MaxSteerAngles =
		InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::SteeringMaxSteerAngleDeg, AircraftCollectionGroup::Steering);
	TManagedArray<float>& AckermannRatios =
		InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::SteeringAckermannRatio, AircraftCollectionGroup::Steering);

	int32 SteeringIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindSteeringIndex(SteeringNames, SteeringName);
	if (SteeringIndex == INDEX_NONE)
	{
		SteeringIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Steering);
	}

	SteeringNames[SteeringIndex] = SteeringName;
	MaxSteerAngles[SteeringIndex] = MaxSteerAngleDeg;
	AckermannRatios[SteeringIndex] = AckermannRatio;

	if (TManagedArray<FName>* const WheelSteeringNames =
		InFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelSteeringName, AircraftCollectionGroup::Wheels))
	{
		for (FName& WheelSteeringName : *WheelSteeringNames)
		{
			if (WheelSteeringName == SteeringName)
			{
				WheelSteeringName = NAME_None;
			}
		}

		if (const TManagedArray<FName>* const ExistingWheelNames =
			InFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelName, AircraftCollectionGroup::Wheels))
		{
			for (const FName ConfiguredWheelName : WheelNames)
			{
				const int32 WheelIndex =
					UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindSteeringConfigWheelIndex(*ExistingWheelNames, ConfiguredWheelName);
				if (WheelIndex != INDEX_NONE)
				{
					(*WheelSteeringNames)[WheelIndex] = SteeringName;
				}
			}
		}
	}
}

