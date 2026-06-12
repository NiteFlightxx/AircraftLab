#include "Dataflow/AircraftWheelConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftWheelConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static int32 FindWheelIndex(
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

FAircraftWheelConfigNode::FAircraftWheelConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllWheels);
	RegisterInputConnection(&WheelName);
	RegisterInputConnection(&BoneName);
	RegisterInputConnection(&SuspensionName);
	RegisterInputConnection(&AxleName);
	RegisterInputConnection(&SteeringName);
	RegisterInputConnection(&BrakeName);
	RegisterInputConnection(&TireName);
	RegisterInputConnection(&RadiusCm);
	RegisterInputConnection(&WidthCm);
	RegisterInputConnection(&MassKg);
}

void FAircraftWheelConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftWheelConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (bReplaceAllWheels)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Wheels);
	}

	if (WheelName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& WheelNames = InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelName, AircraftCollectionGroup::Wheels);

	int32 WheelIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindWheelIndex(WheelNames, WheelName);
	if (WheelIndex == INDEX_NONE)
	{
		WheelIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Wheels);
	}

	WheelNames[WheelIndex] = WheelName;
	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelBoneName, AircraftCollectionGroup::Wheels)[WheelIndex] = BoneName;
	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelSuspensionName, AircraftCollectionGroup::Wheels)[WheelIndex] = SuspensionName;
	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelAxleName, AircraftCollectionGroup::Wheels)[WheelIndex] = AxleName;
	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelSteeringName, AircraftCollectionGroup::Wheels)[WheelIndex] = SteeringName;
	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelBrakeName, AircraftCollectionGroup::Wheels)[WheelIndex] = BrakeName;
	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::WheelTireName, AircraftCollectionGroup::Wheels)[WheelIndex] = TireName;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::WheelRadiusCm, AircraftCollectionGroup::Wheels)[WheelIndex] = RadiusCm;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::WheelWidthCm, AircraftCollectionGroup::Wheels)[WheelIndex] = WidthCm;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::WheelMassKg, AircraftCollectionGroup::Wheels)[WheelIndex] = MassKg;
}
