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

	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	auto InvalidAuthority = [](float Value) { return !FMath::IsFinite(Value) || Value < 0.0f || Value > 1.0f; };
	if (Config.MinimumHealthyRotorCount < 0
		|| InvalidAuthority(Config.MinimumCollectiveAuthority)
		|| InvalidAuthority(Config.MinimumRollAuthority)
		|| InvalidAuthority(Config.MinimumPitchAuthority)
		|| InvalidAuthority(Config.MinimumYawAuthority)
		|| !FMath::IsFinite(Config.ConfirmationTimeSeconds) || Config.ConfirmationTimeSeconds < 0.0f
		|| !FMath::IsFinite(Config.RecoveryConfirmationTimeSeconds) || Config.RecoveryConfirmationTimeSeconds < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Failure-policy rotor count, authority thresholds, and confirmation times are invalid.")), this);
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
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
