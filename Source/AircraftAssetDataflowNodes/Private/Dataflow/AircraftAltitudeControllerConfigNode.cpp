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
	if (Config.VerticalDampingFeedForwardScale < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Altitude-controller vertical damping must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_ALTITUDE(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKp, Config.AltitudeKp)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKi, Config.AltitudeKi)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcAltitudeKd, Config.AltitudeKd)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKp, Config.VerticalVelocityKp)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKi, Config.VerticalVelocityKi)
	UE_AIRCRAFT_WRITE_ALTITUDE(FcVerticalVelocityKd, Config.VerticalVelocityKd)
#undef UE_AIRCRAFT_WRITE_ALTITUDE

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalVelocityDerivativeCutoffHz"), Config.VerticalVelocityDerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Altitude.VerticalDampingFeedForwardScale"), Config.VerticalDampingFeedForwardScale);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
