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
	if (!Config.AttitudeReference.IsValid())
	{
		return Context.Error(TEXT("Kinematic attitude-reference limits must be finite and non-negative."));
	}
	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.MaxTiltAngleDegrees"), Config.AttitudeReference.MaxTiltAngleDegrees);
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.NaturalFrequencyHz"), Config.AttitudeReference.NaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.DampingRatio"), Config.AttitudeReference.DampingRatio);
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.MaxAngularRateDegPerSec"), FVector3f(Config.AttitudeReference.MaxAngularRateDegPerSec));
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.MaxAngularAccelerationDegPerSecSq"), FVector3f(Config.AttitudeReference.MaxAngularAccelerationDegPerSecSq));
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.MaxAngularJerkDegPerSecCubed"), FVector3f(Config.AttitudeReference.MaxAngularJerkDegPerSecCubed));
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.Attitude.Reference.DynamicsFeedForwardScale"), Config.AttitudeReference.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("Simulation.Kinematic.SweepMovement"), Config.bSweepMovement);
	return true;
}
