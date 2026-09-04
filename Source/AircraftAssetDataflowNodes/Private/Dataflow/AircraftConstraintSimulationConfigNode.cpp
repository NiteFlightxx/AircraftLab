#include "Dataflow/AircraftConstraintSimulationConfigNode.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftConstraintSimulationConfigNode)

FAircraftConstraintSimulationConfigNode::FAircraftConstraintSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftConstraintSimulationConfigNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	const float Values[] = { Config.LinearNaturalFrequencyHz, Config.LinearDampingRatio,
		Config.LinearExtraDampingPerSecond, Config.LinearForceLimitN,
		Config.GravityFeedForwardScale, Config.DynamicsFeedForwardScale,
		Config.AttitudeNaturalFrequencyHz, Config.AttitudeDampingRatio,
		Config.AttitudeExtraDampingPerSecond, Config.AttitudeTorqueLimitNm };
	for (const float Value : Values)
	{
		if (!FMath::IsFinite(Value) || Value < 0.0f)
		{
			return Context.Error(TEXT("Constraint strength, damping, limits, and feed-forward scales must be finite and non-negative."));
		}
	}
	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.NaturalFrequencyHz"), Config.LinearNaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.DampingRatio"), Config.LinearDampingRatio);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.ExtraDampingPerSecond"), Config.LinearExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.ForceLimitN"), Config.LinearForceLimitN);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.GravityFeedForwardScale"), Config.GravityFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.DynamicsFeedForwardScale"), Config.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.NaturalFrequencyHz"), Config.AttitudeNaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.DampingRatio"), Config.AttitudeDampingRatio);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.ExtraDampingPerSecond"), Config.AttitudeExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.TorqueLimitNm"), Config.AttitudeTorqueLimitNm);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.AccelerationMode"), Config.bLinearAccelerationMode);
	return true;
}
