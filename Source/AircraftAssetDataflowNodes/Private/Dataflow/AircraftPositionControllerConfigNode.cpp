#include "Dataflow/AircraftPositionControllerConfigNode.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPositionControllerConfigNode)

FAircraftPositionControllerConfigNode::FAircraftPositionControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftPositionControllerConfigNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	auto IsValidPid = [](const FAircraftPidChannelConfig& Pid)
	{
		return FMath::IsFinite(Pid.Kp) && FMath::IsFinite(Pid.Ki) && FMath::IsFinite(Pid.Kd)
			&& FMath::IsFinite(Pid.Kff) && FMath::IsFinite(Pid.IntegralLimit)
			&& FMath::IsFinite(Pid.OutputLimit) && FMath::IsFinite(Pid.DerivativeCutoffHz)
			&& Pid.IntegralLimit >= 0.0f && Pid.OutputLimit >= 0.0f && Pid.DerivativeCutoffHz >= 0.0f;
	};
	if (!IsValidPid(Config.PositionX) || !IsValidPid(Config.PositionY)
		|| !IsValidPid(Config.VelocityX) || !IsValidPid(Config.VelocityY)
		|| !FMath::IsFinite(Config.LinearDampingFeedForwardScale)
		|| !FMath::IsFinite(Config.DampingAccelerationReserveFraction)
		|| Config.LinearDampingFeedForwardScale < 0.0f
		|| Config.DampingAccelerationReserveFraction < 0.0f
		|| Config.DampingAccelerationReserveFraction > 0.9f)
	{
		return Context.Error(TEXT("Position-controller PID or damping values are outside their valid range."));
	}

	auto& Facade = Context.GetAircraft();
#define UE_AIRCRAFT_WRITE_POSITION(GetterName, Value) if (TArrayView<FVector3f> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKp, FVector3f(Config.PositionX.Kp, Config.PositionY.Kp, 0.0f))
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKi, FVector3f(Config.PositionX.Ki, Config.PositionY.Ki, 0.0f))
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKd, FVector3f(Config.PositionX.Kd, Config.PositionY.Kd, 0.0f))
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKp, FVector3f(Config.VelocityX.Kp, Config.VelocityY.Kp, 0.0f))
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKi, FVector3f(Config.VelocityX.Ki, Config.VelocityY.Ki, 0.0f))
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKd, FVector3f(Config.VelocityX.Kd, Config.VelocityY.Kd, 0.0f))
#undef UE_AIRCRAFT_WRITE_POSITION

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("FlightController.Position.PositionKff"), FVector3f(Config.PositionX.Kff, Config.PositionY.Kff, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.PositionIntegralLimit"), FVector3f(Config.PositionX.IntegralLimit, Config.PositionY.IntegralLimit, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.PositionOutputLimit"), FVector3f(Config.PositionX.OutputLimit, Config.PositionY.OutputLimit, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.PositionDerivativeCutoffHz"), FVector3f(Config.PositionX.DerivativeCutoffHz, Config.PositionY.DerivativeCutoffHz, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.PositionXFreezeIntegralWhenSaturated"), Config.PositionX.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Position.PositionYFreezeIntegralWhenSaturated"), Config.PositionY.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityKff"), FVector3f(Config.VelocityX.Kff, Config.VelocityY.Kff, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityIntegralLimit"), FVector3f(Config.VelocityX.IntegralLimit, Config.VelocityY.IntegralLimit, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityOutputLimit"), FVector3f(Config.VelocityX.OutputLimit, Config.VelocityY.OutputLimit, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), FVector3f(Config.VelocityX.DerivativeCutoffHz, Config.VelocityY.DerivativeCutoffHz, 0.0f));
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityXFreezeIntegralWhenSaturated"), Config.VelocityX.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityYFreezeIntegralWhenSaturated"), Config.VelocityY.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Position.LinearDampingFeedForwardScale"), Config.LinearDampingFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Position.DampingAccelerationReserveFraction"), Config.DampingAccelerationReserveFraction);
	return true;
}
