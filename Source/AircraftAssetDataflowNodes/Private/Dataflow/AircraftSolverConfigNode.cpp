#include "Dataflow/AircraftSolverConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSolverConfigNode)

FAircraftSolverConfigNode::FAircraftSolverConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&AsyncFixedTimeStepSize);
	RegisterInputConnection(&bOverrideIterationCounts);
	RegisterInputConnection(&PositionSolverIterationCount);
	RegisterInputConnection(&VelocitySolverIterationCount);
	RegisterInputConnection(&ProjectionSolverIterationCount);
}

void FAircraftSolverConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftSolverConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& Context,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Solver) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Solver);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Solver) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Solver);
	}

	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::AsyncFixedTimeStepSize, AircraftCollectionGroup::Solver)[0] =
		FMath::Clamp(GetValue(Context, &AsyncFixedTimeStepSize), 0.001f, 0.066667f);
	InFacade.FindOrAddAttribute<uint8>(AircraftCollectionAttribute::OverrideIterationCounts, AircraftCollectionGroup::Solver)[0] =
		GetValue(Context, &bOverrideIterationCounts) ? uint8(1) : uint8(0);
	InFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::PositionSolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		FMath::Clamp(GetValue(Context, &PositionSolverIterationCount), 0, 255);
	InFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::VelocitySolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		FMath::Clamp(GetValue(Context, &VelocitySolverIterationCount), 0, 255);
	InFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::ProjectionSolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		FMath::Clamp(GetValue(Context, &ProjectionSolverIterationCount), 0, 255);
}
