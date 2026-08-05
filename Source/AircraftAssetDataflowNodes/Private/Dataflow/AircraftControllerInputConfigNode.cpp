#include "Dataflow/AircraftControllerInputConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftControllerInputConfigNode)

FAircraftControllerInputConfigNode::FAircraftControllerInputConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftControllerInputConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.HorizontalBrakeToHoldSpeedCmPerSec < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Controller-input brake-to-hold speed must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_INPUT(GetterName, Value) if (TArrayView<float> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoRoll, Config.RcExpoRollPitchYaw.X)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoPitch, Config.RcExpoRollPitchYaw.Y)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoYaw, Config.RcExpoRollPitchYaw.Z)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelRcExpoThrottle, Config.RcExpoThrottle)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelInputDeadzone, Config.InputDeadzone)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelStickResponseTimeSeconds, Config.StickResponseTimeSeconds)
	UE_AIRCRAFT_WRITE_INPUT(GameFeelCameraShakeScale, Config.CameraShakeScale)
#undef UE_AIRCRAFT_WRITE_INPUT

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalHoldStickDeadband"), Config.HorizontalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.VerticalHoldStickDeadband"), Config.VerticalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.YawHoldStickDeadband"), Config.YawHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalBrakeToHoldSpeedCmPerSec"), Config.HorizontalBrakeToHoldSpeedCmPerSec);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
