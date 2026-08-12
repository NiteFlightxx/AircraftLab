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
	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	if (!FMath::IsFinite(Config.HorizontalHoldStickDeadband)
		|| !FMath::IsFinite(Config.VerticalHoldStickDeadband)
		|| !FMath::IsFinite(Config.YawHoldStickDeadband)
		|| !FMath::IsFinite(Config.HorizontalBrakeToHoldSpeedCmPerSec)
		|| Config.HorizontalBrakeToHoldSpeedCmPerSec < 0.0f
		|| Config.HorizontalHoldStickDeadband < 0.0f || Config.HorizontalHoldStickDeadband > 1.0f
		|| Config.VerticalHoldStickDeadband < 0.0f || Config.VerticalHoldStickDeadband > 1.0f
		|| Config.YawHoldStickDeadband < 0.0f || Config.YawHoldStickDeadband > 1.0f)
	{
		Context.Error(FText::FromString(TEXT("Controller-input deadbands or brake-to-hold speed are outside their valid range.")), this);
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalHoldStickDeadband"), Config.HorizontalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.VerticalHoldStickDeadband"), Config.VerticalHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.YawHoldStickDeadband"), Config.YawHoldStickDeadband);
	SetConfigProperty(Properties, TEXT("FlightController.Input.HorizontalBrakeToHoldSpeedCmPerSec"), Config.HorizontalBrakeToHoldSpeedCmPerSec);
	SetConfigProperty(Properties, TEXT("FlightController.Execution.ControllerEnabledByDefault"), Config.bControllerEnabledByDefault);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
