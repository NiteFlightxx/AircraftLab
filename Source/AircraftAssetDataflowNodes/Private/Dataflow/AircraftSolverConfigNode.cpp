#include "Dataflow/AircraftSolverConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSolverConfigNode)

FAircraftSolverConfigNode::FAircraftSolverConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftSolverConfigNode::ApplyToAircraftCollection(
	FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	if (PositionSolverIterationCount < 0 || PositionSolverIterationCount > 255
		|| VelocitySolverIterationCount < 0 || VelocitySolverIterationCount > 255
		|| ProjectionSolverIterationCount < 0 || ProjectionSolverIterationCount > 255)
	{
		return Context.Error(TEXT("Solver iteration count is outside its valid range."));
	}
	auto& AircraftCollection = Context.GetCollection();
	auto& Facade = Context.GetAircraft();

	if (AircraftCollection.NumElements(AircraftCollectionGroup::Solver) == 0)
	{
		AircraftCollection.AddElements(1, AircraftCollectionGroup::Solver);
	}
	else if (AircraftCollection.NumElements(AircraftCollectionGroup::Solver) > 1)
	{
		AircraftCollection.Resize(1, AircraftCollectionGroup::Solver);
	}

	Facade.FindOrAddAttribute<uint8>(AircraftCollectionAttribute::OverrideIterationCounts, AircraftCollectionGroup::Solver)[0] =
		bOverrideIterationCounts ? uint8(1) : uint8(0);
	Facade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::PositionSolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		PositionSolverIterationCount;
	Facade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::VelocitySolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		VelocitySolverIterationCount;
	Facade.FindOrAddAttribute<int32>(AircraftCollectionAttribute::ProjectionSolverIterationCount, AircraftCollectionGroup::Solver)[0] =
		ProjectionSolverIterationCount;
	return true;
}
