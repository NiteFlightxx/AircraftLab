#include "Dataflow/AircraftFlightControlLimitsConfigNode.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftFlightControlLimitsConfigNode)

FAircraftFlightControlLimitsConfigNode::FAircraftFlightControlLimitsConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftFlightControlLimitsConfigNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	const float LimitValues[] = {
		Config.MaxTiltAngleDegrees, Config.MaxYawRateDegreesPerSec, Config.MaxRollRateDegreesPerSec,
		Config.MaxPitchRateDegreesPerSec, Config.MaxClimbRateCmPerSec, Config.MaxDescentRateCmPerSec,
		Config.MaxHorizontalSpeedCmPerSec, Config.MaxHorizontalAccelerationCmPerSecSq,
		Config.MaxHorizontalDecelerationCmPerSecSq, Config.MaxHorizontalJerkCmPerSecCubed,
		Config.MaxVerticalAccelerationCmPerSecSq, Config.MaxVerticalJerkCmPerSecCubed,
		Config.MaxYawAccelerationDegPerSecSq, Config.MaxYawJerkDegPerSecCubed,
		Config.MinCollectiveCommand,
		Config.HoverCollectiveCommand, Config.MaxCollectiveCommand };
	bool bInvalid = false;
	for (const float Value : LimitValues)
	{
		bInvalid |= !FMath::IsFinite(Value) || Value < 0.0f;
	}
	bInvalid |= Config.MinCollectiveCommand > Config.HoverCollectiveCommand
		|| Config.HoverCollectiveCommand > Config.MaxCollectiveCommand
		|| Config.MaxCollectiveCommand > 1.0f;
	const float EstimatorValues[] = {
		Config.HoverThrustInitialStateVariance, Config.HoverThrustProcessNoiseVariance,
		Config.HoverThrustAccelNoiseVariance, Config.HoverThrustGateSize,
		Config.HoverThrustMin, Config.HoverThrustMax };
	for (const float Value : EstimatorValues)
	{
		bInvalid |= !FMath::IsFinite(Value) || Value < 0.0f;
	}
	bInvalid |= Config.HoverThrustMin >= Config.HoverThrustMax;
	if (bInvalid)
	{
		return Context.Error(TEXT("Flight-control limits must be finite, non-negative, and satisfy 0 <= Min <= Hover <= Max <= 1."));
	}

	auto& Facade = Context.GetAircraft();

#define UE_AIRCRAFT_WRITE_LIMIT(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxTiltAngleDegrees, Config.MaxTiltAngleDegrees)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxYawRateDegreesPerSec, Config.MaxYawRateDegreesPerSec)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxClimbRateCmPerSec, Config.MaxClimbRateCmPerSec)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxDescentRateCmPerSec, Config.MaxDescentRateCmPerSec)
	UE_AIRCRAFT_WRITE_LIMIT(FcMaxHorizontalSpeedCmPerSec, Config.MaxHorizontalSpeedCmPerSec)
#undef UE_AIRCRAFT_WRITE_LIMIT

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("FlightController.MaxRollRateDegreesPerSec"), Config.MaxRollRateDegreesPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.MaxPitchRateDegreesPerSec"), Config.MaxPitchRateDegreesPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.MaxHorizontalAccelerationCmPerSecSq"), Config.MaxHorizontalAccelerationCmPerSecSq);
	SetConfigProperty(Properties, TEXT("FlightController.MaxHorizontalDecelerationCmPerSecSq"), Config.MaxHorizontalDecelerationCmPerSecSq);
	SetConfigProperty(Properties, TEXT("FlightController.MaxHorizontalJerkCmPerSecCubed"), Config.MaxHorizontalJerkCmPerSecCubed);
	SetConfigProperty(Properties, TEXT("FlightController.MaxVerticalAccelerationCmPerSecSq"), Config.MaxVerticalAccelerationCmPerSecSq);
	SetConfigProperty(Properties, TEXT("FlightController.MaxVerticalJerkCmPerSecCubed"), Config.MaxVerticalJerkCmPerSecCubed);
	SetConfigProperty(Properties, TEXT("FlightController.MaxYawAccelerationDegPerSecSq"), Config.MaxYawAccelerationDegPerSecSq);
	SetConfigProperty(Properties, TEXT("FlightController.MaxYawJerkDegPerSecCubed"), Config.MaxYawJerkDegPerSecCubed);
	SetConfigProperty(Properties, TEXT("FlightController.MinCollectiveCommand"), Config.MinCollectiveCommand);
	SetConfigProperty(Properties, TEXT("FlightController.HoverCollectiveCommand"), Config.HoverCollectiveCommand);
	SetConfigProperty(Properties, TEXT("FlightController.MaxCollectiveCommand"), Config.MaxCollectiveCommand);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.Enabled"), Config.bEnableHoverThrustEstimator);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.InitialStateVariance"), Config.HoverThrustInitialStateVariance);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.ProcessNoiseVariance"), Config.HoverThrustProcessNoiseVariance);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.AccelNoiseVariance"), Config.HoverThrustAccelNoiseVariance);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.GateSize"), Config.HoverThrustGateSize);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.MinHoverThrust"), Config.HoverThrustMin);
	SetConfigProperty(Properties, TEXT("FlightController.HoverThrustEstimator.MaxHoverThrust"), Config.HoverThrustMax);

	return true;
}
