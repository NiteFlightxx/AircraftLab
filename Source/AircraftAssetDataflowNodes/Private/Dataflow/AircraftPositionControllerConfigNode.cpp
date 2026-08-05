#include "Dataflow/AircraftPositionControllerConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftPositionControllerConfigNode)

FAircraftPositionControllerConfigNode::FAircraftPositionControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftPositionControllerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.LinearDampingFeedForwardScale < 0.0f
		|| Config.DampingAccelerationReserveFraction < 0.0f
		|| Config.DampingAccelerationReserveFraction > 0.9f)
	{
		Context.Error(FText::FromString(TEXT("Position-controller damping values are outside their valid range.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_POSITION(GetterName, Value) if (TArrayView<FVector3f> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKp, Config.PositionKp)
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKi, Config.PositionKi)
	UE_AIRCRAFT_WRITE_POSITION(FcPositionKd, Config.PositionKd)
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKp, Config.VelocityKp)
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKi, Config.VelocityKi)
	UE_AIRCRAFT_WRITE_POSITION(FcVelocityKd, Config.VelocityKd)
#undef UE_AIRCRAFT_WRITE_POSITION

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), Config.VelocityDerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Position.LinearDampingFeedForwardScale"), Config.LinearDampingFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Position.DampingAccelerationReserveFraction"), Config.DampingAccelerationReserveFraction);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
