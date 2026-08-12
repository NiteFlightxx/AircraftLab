#include "Dataflow/AircraftSolverConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSolverConfigNode)

FAircraftSolverConfigNode::FAircraftSolverConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
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
	if (!FMath::IsFinite(AsyncFixedTimeStepSize)
		|| AsyncFixedTimeStepSize < 0.001f || AsyncFixedTimeStepSize > 0.066667f
		|| PositionSolverIterationCount < 0 || PositionSolverIterationCount > 255
		|| VelocitySolverIterationCount < 0 || VelocitySolverIterationCount > 255
		|| ProjectionSolverIterationCount < 0 || ProjectionSolverIterationCount > 255)
	{
		Context.Error(FText::FromString(TEXT("Solver time step or iteration count is outside its valid range.")), this);
		return;
	}

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Solver) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Solver);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Solver) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Solver);
	}

	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::AsyncFixedTimeStepSize, AircraftCollectionGroup::Solver)[0] =
		AsyncFixedTimeStepSize;
	InFacade.FindOrAddAttribute<uint8>(AircraftCollectionAttribute::OverrideIterationCounts, AircraftCollectionGroup::Solver)[0] =
		bOverrideIterationCounts ? uint8(1) : uint8(0);
	InFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::PositionSolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		PositionSolverIterationCount;
	InFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::VelocitySolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		VelocitySolverIterationCount;
	InFacade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::ProjectionSolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		ProjectionSolverIterationCount;
}
