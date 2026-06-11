#include "Dataflow/AircraftAxleConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAxleConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static int32 FindAxleIndex(
		const TManagedArray<FName>& AxleNames,
		const FName AxleName)
	{
		for (int32 Index = 0; Index < AxleNames.Num(); ++Index)
		{
			if (AxleNames[Index] == AxleName)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static int32 FindAxleConfigWheelIndex(
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

FAircraftAxleConfigNode::FAircraftAxleConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllAxles);
	RegisterInputConnection(&AxleName);
	RegisterInputConnection(&WheelNames);
	RegisterInputConnection(&bIsSteeringAxle);
	RegisterInputConnection(&bIsDrivenAxle);
}

void FAircraftAxleConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftAxleConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	if (bReplaceAllAxles)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Axles);

		if (TManagedArray<FName>* const WheelAxleNames =
			AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelAxleName, AircraftCollectionGroup::Wheels))
		{
			for (FName& WheelAxleName : *WheelAxleNames)
			{
				WheelAxleName = NAME_None;
			}
		}
	}

	if (AxleName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& AxleNames =
		AircraftFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::AxleName, AircraftCollectionGroup::Axles);
	TManagedArray<bool>& AxleSteeringFlags =
		AircraftFacade.FindOrAddAttribute<bool>(AircraftCollectionAttribute::AxleIsSteeringAxle, AircraftCollectionGroup::Axles);
	TManagedArray<bool>& AxleDrivenFlags =
		AircraftFacade.FindOrAddAttribute<bool>(AircraftCollectionAttribute::AxleIsDrivenAxle, AircraftCollectionGroup::Axles);

	int32 AxleIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindAxleIndex(AxleNames, AxleName);
	if (AxleIndex == INDEX_NONE)
	{
		AxleIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Axles);
	}

	AxleNames[AxleIndex] = AxleName;
	AxleSteeringFlags[AxleIndex] = bIsSteeringAxle;
	AxleDrivenFlags[AxleIndex] = bIsDrivenAxle;

	if (TManagedArray<FName>* const WheelAxleNames =
		AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelAxleName, AircraftCollectionGroup::Wheels))
	{
		for (FName& WheelAxleName : *WheelAxleNames)
		{
			if (WheelAxleName == AxleName)
			{
				WheelAxleName = NAME_None;
			}
		}

		if (const TManagedArray<FName>* const ExistingWheelNames =
			AircraftFacade.FindAttribute<FName>(AircraftCollectionAttribute::WheelName, AircraftCollectionGroup::Wheels))
		{
			for (const FName ConfiguredWheelName : WheelNames)
			{
				const int32 WheelIndex =
					UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindAxleConfigWheelIndex(*ExistingWheelNames, ConfiguredWheelName);
				if (WheelIndex != INDEX_NONE)
				{
					(*WheelAxleNames)[WheelIndex] = AxleName;
				}
			}
		}
	}
}
