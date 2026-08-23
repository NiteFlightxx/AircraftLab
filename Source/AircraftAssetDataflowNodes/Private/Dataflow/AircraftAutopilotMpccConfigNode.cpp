#include "Dataflow/AircraftAutopilotMpccConfigNode.h"

#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAutopilotMpccConfigNode)

FAircraftAutopilotMpccConfigNode::FAircraftAutopilotMpccConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAutopilotMpccConfigNode::Evaluate(
	UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	const FManagedArrayCollection Input = GetValue<FManagedArrayCollection>(Context, &Collection);
	const float Weights[] = { Config.ContourErrorWeight, Config.LagErrorWeight,
		Config.SpeedTrackingWeight, Config.AccelerationWeight, Config.JerkWeight,
		Config.TerminalPositionWeight, Config.TerminalVelocityWeight };
	bool bInvalid = !FMath::IsFinite(Config.UpdateRateHz) || Config.UpdateRateHz <= 0.0f
		|| !FMath::IsFinite(Config.HorizonSeconds) || Config.HorizonSeconds <= 0.0f
		|| Config.HorizonSteps < 2 || Config.MaxOptimizationIterations <= 0
		|| !FMath::IsFinite(Config.SolveTimeBudgetMilliseconds) || Config.SolveTimeBudgetMilliseconds <= 0.0f
		|| !FMath::IsFinite(Config.Regularization) || Config.Regularization <= 0.0f
		|| !FMath::IsFinite(Config.YawResponseTimeSeconds) || Config.YawResponseTimeSeconds <= 0.0f
		|| !FMath::IsFinite(Config.ContourErrorGovernorScaleCm) || Config.ContourErrorGovernorScaleCm <= 0.0f
		|| Config.MaxConsecutiveFailures <= 0
		|| !FMath::IsFinite(Config.MaximumReferenceAgeSeconds) || Config.MaximumReferenceAgeSeconds <= 0.0f;
	for (const float Weight : Weights)
	{
		bInvalid |= !FMath::IsFinite(Weight) || Weight < 0.0f;
	}
	if (bInvalid)
	{
		Context.Error(FText::FromString(TEXT("MPCC configuration is invalid.")), this);
		SetValue(Context, Input, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> Output = MakeShared<FManagedArrayCollection>(Input);
	FCollectionAircraftPropertyMutableFacade Properties(Output);
	Properties.DefineSchema();
#define SET_MPCC(Name) SetConfigProperty(Properties, TEXT("Autopilot.Mpcc." #Name), Config.Name)
	SET_MPCC(UpdateRateHz);
	SET_MPCC(HorizonSeconds);
	SET_MPCC(HorizonSteps);
	SET_MPCC(MaxOptimizationIterations);
	SET_MPCC(SolveTimeBudgetMilliseconds);
	SET_MPCC(ContourErrorWeight);
	SET_MPCC(LagErrorWeight);
	SET_MPCC(SpeedTrackingWeight);
	SET_MPCC(AccelerationWeight);
	SET_MPCC(JerkWeight);
	SET_MPCC(YawResponseTimeSeconds);
	SET_MPCC(ContourErrorGovernorScaleCm);
	SET_MPCC(TerminalPositionWeight);
	SET_MPCC(TerminalVelocityWeight);
	SET_MPCC(Regularization);
	SET_MPCC(MaxConsecutiveFailures);
	SET_MPCC(MaximumReferenceAgeSeconds);
#undef SET_MPCC
	SetValue(Context, MoveTemp(*Output), &Collection);
}
