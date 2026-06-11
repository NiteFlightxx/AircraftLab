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
	const TSharedRef<FManagedArrayCollection>& AircraftCollection) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Chassis) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Chassis);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Chassis) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Chassis);
	}

	AircraftFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::ChassisRootBone, AircraftCollectionGroup::Chassis)[0] = RootBone;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::ChassisMassKg, AircraftCollectionGroup::Chassis)[0] = MassKg;
	AircraftFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::ChassisDragCoefficient, AircraftCollectionGroup::Chassis)[0] = DragCoefficient;
	AircraftFacade.FindOrAddAttribute<FVector3f>(AircraftCollectionAttribute::ChassisCenterOfMassOffset, AircraftCollectionGroup::Chassis)[0] = FVector3f(CenterOfMassOffset);
	AircraftFacade.FindOrAddAttribute<FVector3f>(AircraftCollectionAttribute::ChassisInertiaTensorScale, AircraftCollectionGroup::Chassis)[0] = FVector3f(InertiaTensorScale);
}
