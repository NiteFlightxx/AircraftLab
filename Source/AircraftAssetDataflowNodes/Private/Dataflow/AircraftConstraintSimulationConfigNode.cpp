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
		Config.AttitudeServoNaturalFrequencyHz, Config.AttitudeServoDampingRatio,
		Config.AttitudeServoExtraDampingPerSecond, Config.AttitudeTorqueLimitNm };
	for (const float Value : Values)
	{
		if (!FMath::IsFinite(Value) || Value < 0.0f)
		{
			return Context.Error(TEXT("Constraint strength, damping, limits, and feed-forward scales must be finite and non-negative."));
		}
	}
	if (!Config.AttitudeReference.IsValid())
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
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.MaxTiltAngleDegrees"), Config.AttitudeReference.MaxTiltAngleDegrees);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.NaturalFrequencyHz"), Config.AttitudeReference.NaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.DampingRatio"), Config.AttitudeReference.DampingRatio);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.MaxAngularRateDegPerSec"), FVector3f(Config.AttitudeReference.MaxAngularRateDegPerSec));
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.MaxAngularAccelerationDegPerSecSq"), FVector3f(Config.AttitudeReference.MaxAngularAccelerationDegPerSecSq));
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.MaxAngularJerkDegPerSecCubed"), FVector3f(Config.AttitudeReference.MaxAngularJerkDegPerSecCubed));
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Reference.DynamicsFeedForwardScale"), Config.AttitudeReference.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Servo.NaturalFrequencyHz"), Config.AttitudeServoNaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Servo.DampingRatio"), Config.AttitudeServoDampingRatio);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Servo.ExtraDampingPerSecond"), Config.AttitudeServoExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("Simulation.Constraint.Attitude.Servo.TorqueLimitNm"), Config.AttitudeTorqueLimitNm);
	return true;
}
