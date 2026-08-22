#include "Dataflow/AircraftAutopilotTimingConfigNode.h"

#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAutopilotTimingConfigNode)

FAircraftAutopilotTimingConfigNode::FAircraftAutopilotTimingConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAutopilotTimingConfigNode::Evaluate(
	UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	const FManagedArrayCollection Input = GetValue<FManagedArrayCollection>(Context, &Collection);
	auto IsReserveValid = [](float Value) { return FMath::IsFinite(Value) && Value >= 0.0f && Value < 1.0f; };
	const bool bInvalid = Config.SampleSpacingCm <= 0.0f
		|| !IsReserveValid(Config.ThrustReserveFraction)
		|| !IsReserveValid(Config.TorqueReserveFraction)
		|| !IsReserveValid(Config.BrakingReserveFraction)
		|| Config.MaxIterations <= 0
		|| Config.FeasibilityTolerance <= 0.0f;
	if (bInvalid)
	{
		Context.Error(FText::FromString(TEXT("Dynamic trajectory timing configuration is invalid.")), this);
		SetValue(Context, Input, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> Output = MakeShared<FManagedArrayCollection>(Input);
	FCollectionAircraftPropertyMutableFacade Properties(Output);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.SampleSpacingCm"), Config.SampleSpacingCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.ThrustReserveFraction"), Config.ThrustReserveFraction);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.TorqueReserveFraction"), Config.TorqueReserveFraction);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.BrakingReserveFraction"), Config.BrakingReserveFraction);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.MaxIterations"), Config.MaxIterations);
	SetConfigProperty(Properties, TEXT("Autopilot.Timing.FeasibilityTolerance"), Config.FeasibilityTolerance);
	SetValue(Context, MoveTemp(*Output), &Collection);
}
