#include "Dataflow/AircraftAltitudeControllerConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAltitudeControllerConfigNode)

FAircraftAltitudeControllerConfigNode::FAircraftAltitudeControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAltitudeControllerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	auto IsValidPid = [](const auto& Pid)
	{
		return FMath::IsFinite(Pid.Kp) && FMath::IsFinite(Pid.Ki) && FMath::IsFinite(Pid.Kd)
			&& FMath::IsFinite(Pid.IntegralLimit) && FMath::IsFinite(Pid.OutputLimit)
			&& FMath::IsFinite(Pid.DerivativeCutoffHz) && Pid.IntegralLimit >= 0.0f
			&& Pid.OutputLimit >= 0.0f && Pid.DerivativeCutoffHz >= 0.0f;
	};
	if (!IsValidPid(Config.Altitude) || !FMath::IsFinite(Config.Altitude.Kff)
		|| !IsValidPid(Config.VerticalVelocity)
		|| !FMath::IsFinite(Config.VerticalDampingFeedForwardScale)
		|| Config.VerticalDampingFeedForwardScale < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Altitude-controller PID or damping values are outside their valid range.")), this);
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_ALTITUDE(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKp, Config.Altitude.Kp)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKi, Config.Altitude.Ki)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKd, Config.Altitude.Kd)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKp, Config.VerticalVelocity.Kp)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKi, Config.VerticalVelocity.Ki)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKd, Config.VerticalVelocity.Kd)
#undef UE_AIRCRAFT_WRITE_ALTITUDE

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.AltitudeKff"), Config.Altitude.Kff);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.AltitudeIntegralLimit"), Config.Altitude.IntegralLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.AltitudeOutputLimit"), Config.Altitude.OutputLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.AltitudeDerivativeCutoffHz"), Config.Altitude.DerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.AltitudeFreezeIntegralWhenSaturated"), Config.Altitude.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityIntegralLimit"), Config.VerticalVelocity.IntegralLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityOutputLimit"), Config.VerticalVelocity.OutputLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityDerivativeCutoffHz"), Config.VerticalVelocity.DerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityFreezeIntegralWhenSaturated"), Config.VerticalVelocity.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalDampingFeedForwardScale"), Config.VerticalDampingFeedForwardScale);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
