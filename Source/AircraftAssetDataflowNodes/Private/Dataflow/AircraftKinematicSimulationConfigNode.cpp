#include "Dataflow/AircraftKinematicSimulationConfigNode.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftKinematicSimulationConfigNode)

FAircraftKinematicSimulationConfigNode::FAircraftKinematicSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftKinematicSimulationConfigNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.SweepMovement"), Config.bSweepMovement);
	return true;
}
