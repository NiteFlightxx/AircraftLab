#include "Dataflow/AircraftDifferentialConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftDifferentialConfigNode)

FAircraftDifferentialConfigNode::FAircraftDifferentialConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	//RegisterInputConnection(&DifferentialType);
	RegisterInputConnection(&FrontRearSplit);
	RegisterInputConnection(&bDriveFrontAxle);
	RegisterInputConnection(&bDriveRearAxle);
}

void FAircraftDifferentialConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftDifferentialConfigNode::EvaluateAircraftCollection(
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

	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainDifferentialFrontRearSplit,
		AircraftCollectionGroup::Powertrain)[0] = FrontRearSplit;
	InFacade.FindOrAddAttribute<bool>(
		AircraftCollectionAttribute::PowertrainDifferentialDriveFrontAxle,
		AircraftCollectionGroup::Powertrain)[0] = bDriveFrontAxle;
	InFacade.FindOrAddAttribute<bool>(
		AircraftCollectionAttribute::PowertrainDifferentialDriveRearAxle,
		AircraftCollectionGroup::Powertrain)[0] = bDriveRearAxle;
}
