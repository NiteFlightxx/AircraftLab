#include "Dataflow/AircraftControllerInputConfigNode.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftControllerInputConfigNode)

FAircraftControllerInputConfigNode::FAircraftControllerInputConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftControllerInputConfigNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!FMath::IsFinite(Config.HorizontalHoldStickDeadband)
		|| !FMath::IsFinite(Config.VerticalHoldStickDeadband)
		|| !FMath::IsFinite(Config.YawHoldStickDeadband)
		|| !FMath::IsFinite(Config.HorizontalBrakeToHoldSpeedCmPerSec)
		|| !FMath::IsFinite(Config.VerticalBrakeToHoldSpeedCmPerSec)
		|| Config.HorizontalBrakeToHoldSpeedCmPerSec < 0.0f
		|| Config.VerticalBrakeToHoldSpeedCmPerSec < 0.0f
		|| Config.HorizontalHoldStickDeadband < 0.0f || Config.HorizontalHoldStickDeadband > 1.0f
		|| Config.VerticalHoldStickDeadband < 0.0f || Config.VerticalHoldStickDeadband > 1.0f
		|| Config.YawHoldStickDeadband < 0.0f || Config.YawHoldStickDeadband > 1.0f)
	{
		return Context.Error(TEXT("Controller-input deadbands or brake-to-hold speed are outside their valid range."));
	}

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalHoldStickDeadband"), Config.HorizontalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.VerticalHoldStickDeadband"), Config.VerticalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.YawHoldStickDeadband"), Config.YawHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalBrakeToHoldSpeedCmPerSec"), Config.HorizontalBrakeToHoldSpeedCmPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.Input.VerticalBrakeToHoldSpeedCmPerSec"), Config.VerticalBrakeToHoldSpeedCmPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.Execution.ControllerEnabledByDefault"), Config.bControllerEnabledByDefault);
	SetConfigProperty(Properties, TEXT("Aircraft.Initial.StartArmed"), Config.bStartArmed);
	SetConfigProperty(Properties, TEXT("Aircraft.Initial.FlightMode"), static_cast<int32>(Config.InitialFlightMode));
	return true;
}
