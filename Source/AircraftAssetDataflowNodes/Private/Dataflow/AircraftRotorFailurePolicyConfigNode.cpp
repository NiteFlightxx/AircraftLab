#include "Dataflow/AircraftRotorFailurePolicyConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftRotorFailurePolicyConfigNode)

FAircraftRotorFailurePolicyConfigNode::FAircraftRotorFailurePolicyConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftRotorFailurePolicyConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	if (Config.MinimumHealthyRotorCount < 0)
	{
		Context.Error(FText::FromString(TEXT("Failure-policy minimum healthy rotor count must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();

	SetConfigProperty(Properties, TEXT("FlightController.Failure.Enabled"), Config.bEnabled);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.EvaluateOnlyWhenArmed"), Config.bEvaluateOnlyWhenArmed);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.MinimumHealthyRotorCount"), Config.MinimumHealthyRotorCount);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.MinimumCollectiveAuthority"), Config.MinimumCollectiveAuthority);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.MinimumRollAuthority"), Config.MinimumRollAuthority);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.MinimumPitchAuthority"), Config.MinimumPitchAuthority);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.MinimumYawAuthority"), Config.MinimumYawAuthority);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.ConfirmationTimeSeconds"), Config.ConfirmationTimeSeconds);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.RecoveryConfirmationTimeSeconds"), Config.RecoveryConfirmationTimeSeconds);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.LatchTriggeredAction"), Config.bLatchTriggeredAction);
	SetConfigProperty(Properties, TEXT("FlightController.Failure.Action"), static_cast<int32>(Config.Action));
	SetConfigProperty(Properties, TEXT("FlightController.Failure.DegradedFlightMode"), static_cast<int32>(Config.DegradedFlightMode));

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
