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
	const TSharedRef<FManagedArrayCollection>& AircraftCollection) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Powertrain) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Powertrain);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Powertrain) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Powertrain);
	}

	AircraftFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainDifferentialFrontRearSplit,
		AircraftCollectionGroup::Powertrain)[0] = FrontRearSplit;
	AircraftFacade.FindOrAddAttribute<bool>(
		AircraftCollectionAttribute::PowertrainDifferentialDriveFrontAxle,
		AircraftCollectionGroup::Powertrain)[0] = bDriveFrontAxle;
	AircraftFacade.FindOrAddAttribute<bool>(
		AircraftCollectionAttribute::PowertrainDifferentialDriveRearAxle,
		AircraftCollectionGroup::Powertrain)[0] = bDriveRearAxle;
}
