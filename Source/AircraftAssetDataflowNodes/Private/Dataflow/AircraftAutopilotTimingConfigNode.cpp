#include "Dataflow/AircraftAutopilotTimingConfigNode.h"

#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAutopilotTimingConfigNode)

FAircraftAutopilotTimingConfigNode::FAircraftAutopilotTimingConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftAutopilotTimingConfigNode::ApplyToAircraftCollection(
	FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	auto IsReserveValid = [](float Value) { return FMath::IsFinite(Value) && Value >= 0.0f && Value < 1.0f; };
	const bool bInvalid = !FMath::IsFinite(Config.SampleSpacingCm) || Config.SampleSpacingCm <= 0.0f
		|| !IsReserveValid(Config.ThrustReserveFraction)
		|| !IsReserveValid(Config.CurvatureAccelerationReserveFraction)
		|| !IsReserveValid(Config.BrakingReserveFraction)
		|| Config.MaxIterations <= 0
		|| !FMath::IsFinite(Config.SpeedConvergenceToleranceCmPerSec)
		|| Config.SpeedConvergenceToleranceCmPerSec <= 0.0f;
	if (bInvalid)
	{
		return Context.Error(TEXT("Dynamic trajectory timing configuration is invalid."));
	}

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.SampleSpacingCm"), Config.SampleSpacingCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.ThrustReserveFraction"), Config.ThrustReserveFraction);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.CurvatureAccelerationReserveFraction"), Config.CurvatureAccelerationReserveFraction);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.BrakingReserveFraction"), Config.BrakingReserveFraction);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.MaxIterations"), Config.MaxIterations);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.SpeedConvergenceToleranceCmPerSec"), Config.SpeedConvergenceToleranceCmPerSec);
	return true;
}
