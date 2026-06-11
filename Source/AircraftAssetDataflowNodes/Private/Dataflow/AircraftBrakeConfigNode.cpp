#include "Dataflow/AircraftBrakeConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftBrakeConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static int32 FindBrakeIndex(const TManagedArray<FName>& BrakeNames, const FName BrakeName)
	{
		for (int32 Index = 0; Index < BrakeNames.Num(); ++Index)
		{
			if (BrakeNames[Index] == BrakeName)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}

	static FString SerializeWheelNames(const TArray<FName>& WheelNames)
	{
		TArray<FString> SerializedWheelNames;
		SerializedWheelNames.Reserve(WheelNames.Num());

		for (const FName WheelName : WheelNames)
		{
			if (!WheelName.IsNone())
			{
				SerializedWheelNames.Add(WheelName.ToString());
			}
		}

		return FString::Join(SerializedWheelNames, TEXT(","));
	}
}

FAircraftBrakeConfigNode::FAircraftBrakeConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllBrakes);
	RegisterInputConnection(&BrakeName);
	RegisterInputConnection(&WheelNames);
	RegisterInputConnection(&MaxBrakeTorqueNm);
	RegisterInputConnection(&bHandbrake);
}

void FAircraftBrakeConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftBrakeConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	if (bReplaceAllBrakes)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Brakes);
	}

	if (BrakeName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& BrakeNames =
		AircraftFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::BrakeName, AircraftCollectionGroup::Brakes);
	TManagedArray<FString>& BrakeWheelNames =
		AircraftFacade.FindOrAddAttribute<FString>(AircraftCollectionAttribute::BrakeWheelNames, AircraftCollectionGroup::Brakes);
	TManagedArray<float>& BrakeMaxTorques =
		AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::BrakeMaxTorqueNm, AircraftCollectionGroup::Brakes);
	TManagedArray<bool>& BrakeHandbrakeFlags =
		AircraftFacade.FindOrAddAttribute<bool>(AircraftCollectionAttribute::BrakeIsHandbrake, AircraftCollectionGroup::Brakes);

	int32 BrakeIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindBrakeIndex(BrakeNames, BrakeName);
	if (BrakeIndex == INDEX_NONE)
	{
		BrakeIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Brakes);
	}

	BrakeNames[BrakeIndex] = BrakeName;
	BrakeWheelNames[BrakeIndex] = UE::AircraftLab::AircraftAssetDataflowNodes::Private::SerializeWheelNames(WheelNames);
	BrakeMaxTorques[BrakeIndex] = FMath::Max(0.0f, MaxBrakeTorqueNm);
	BrakeHandbrakeFlags[BrakeIndex] = bHandbrake;
}
