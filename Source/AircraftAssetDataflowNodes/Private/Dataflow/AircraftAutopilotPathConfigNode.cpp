#include "Dataflow/AircraftAutopilotPathConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAutopilotPathConfigNode)

FAircraftAutopilotPathConfigNode::FAircraftAutopilotPathConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAutopilotPathConfigNode::Evaluate(
	UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	const FManagedArrayCollection Input = GetValue<FManagedArrayCollection>(Context, &Collection);
	const bool bInvalid = Config.ResampleSpacingCm <= 0.0f
		|| Config.MinimumSegmentLengthCm <= 0.0f
		|| Config.CorridorSafetyMarginCm < 0.0f
		|| Config.CenterlineWeight < 0.0f
		|| Config.CurvatureWeight < 0.0f
		|| Config.SnapWeight < 0.0f
		|| Config.MaxIterations <= 0
		|| Config.ConvergenceToleranceCm <= 0.0f;
	if (bInvalid)
	{
		Context.Error(FText::FromString(TEXT("Spatial path optimization configuration is invalid.")), this);
		SetValue(Context, Input, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> Output = MakeShared<FManagedArrayCollection>(Input);
	FCollectionAircraftPropertyMutableFacade Properties(Output);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("Autopilot.Path.ResampleSpacingCm"), Config.ResampleSpacingCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.MinimumSegmentLengthCm"), Config.MinimumSegmentLengthCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.CorridorSafetyMarginCm"), Config.CorridorSafetyMarginCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.CenterlineWeight"), Config.CenterlineWeight);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.CurvatureWeight"), Config.CurvatureWeight);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.SnapWeight"), Config.SnapWeight);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.MaxIterations"), Config.MaxIterations);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.ConvergenceToleranceCm"), Config.ConvergenceToleranceCm);
	SetValue(Context, MoveTemp(*Output), &Collection);
}
