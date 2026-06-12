#include "Dataflow/AircraftChassisConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftChassisConfigNode)

FAircraftChassisConfigNode::FAircraftChassisConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&RootBone);
	RegisterInputConnection(&MassKg);
	RegisterInputConnection(&DragCoefficient);
	RegisterInputConnection(&CenterOfMassOffset);
	RegisterInputConnection(&InertiaTensorScale);
}

void FAircraftChassisConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftChassisConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Chassis) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Chassis);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Chassis) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Chassis);
	}

	InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::ChassisRootBone, AircraftCollectionGroup::Chassis)[0] = RootBone;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::ChassisMassKg, AircraftCollectionGroup::Chassis)[0] = MassKg;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::ChassisDragCoefficient, AircraftCollectionGroup::Chassis)[0] = DragCoefficient;
	InFacade.FindOrAddAttribute<FVector3f>(AircraftCollectionAttribute::ChassisCenterOfMassOffset, AircraftCollectionGroup::Chassis)[0] = FVector3f(CenterOfMassOffset);
	InFacade.FindOrAddAttribute<FVector3f>(AircraftCollectionAttribute::ChassisInertiaTensorScale, AircraftCollectionGroup::Chassis)[0] = FVector3f(InertiaTensorScale);
}
