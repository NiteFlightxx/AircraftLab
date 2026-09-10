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
		Config.AttitudeExtraDampingPerSecond, Config.AttitudeTorqueLimitNm };
	for (const float Value : Values)
	{
		if (!FMath::IsFinite(Value) || Value < 0.0f)
		{
			return Context.Error(TEXT("Constraint strength, damping, limits, and feed-forward scales must be finite and non-negative."));
		}
	}
	if (!Config.Attitude.IsValid())
	{
		return Context.Error(TEXT("Constraint attitude-reference limits must be finite and non-negative."));
	}
	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.NaturalFrequencyHz"), Config.LinearNaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.DampingRatio"), Config.LinearDampingRatio);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.ExtraDampingPerSecond"), Config.LinearExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.ForceLimitN"), Config.LinearForceLimitN);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.GravityFeedForwardScale"), Config.GravityFeedForwardScale);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.DynamicsFeedForwardScale"), Config.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Linear.AccelerationMode"), Config.bLinearAccelerationMode);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.MaxTiltAngleDegrees"), Config.Attitude.MaxTiltAngleDegrees);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.NaturalFrequencyHz"), Config.Attitude.NaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.DampingRatio"), Config.Attitude.DampingRatio);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.MaxAngularRateDegPerSec"), FVector3f(Config.Attitude.MaxAngularRateDegPerSec));
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.MaxAngularAccelerationDegPerSecSq"), FVector3f(Config.Attitude.MaxAngularAccelerationDegPerSecSq));
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.MaxAngularJerkDegPerSecCubed"), FVector3f(Config.Attitude.MaxAngularJerkDegPerSecCubed));
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.DynamicsFeedForwardScale"), Config.Attitude.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.ExtraDampingPerSecond"), Config.AttitudeExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.TorqueLimitNm"), Config.AttitudeTorqueLimitNm);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.AccelerationMode"), Config.bAngularAccelerationMode);
	return true;
}
