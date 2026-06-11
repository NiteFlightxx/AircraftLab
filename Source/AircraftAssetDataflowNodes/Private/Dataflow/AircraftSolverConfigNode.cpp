#include "Dataflow/AircraftSolverConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSolverConfigNode)

FAircraftSolverConfigNode::FAircraftSolverConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&MaxSolverSubsteps);

}

void FAircraftSolverConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftSolverConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& Context,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	FCollectionAircraftFacade AircraftFacade(AircraftCollection);
	AircraftFacade.DefineSchema();

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Solver) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Solver);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Solver) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Solver);
	}

	AircraftFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::SolverMaxSolverSubsteps, AircraftCollectionGroup::Solver)[0] =
		FMath::Max(1, GetValue(Context, &MaxSolverSubsteps));
}
