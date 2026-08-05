#include "Dataflow/AircraftAutopilotConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAutopilotConfigNode)

FAircraftAutopilotConfigNode::FAircraftAutopilotConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAutopilotConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	if (Config.PurePursuitMinLookAheadCm > Config.PurePursuitMaxLookAheadCm)
	{
		Context.Error(FText::FromString(TEXT("Autopilot pure-pursuit look-ahead must satisfy Min <= Max.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();

	SetConfigProperty(Properties, TEXT("Autopilot.EnableCoordinatedTurns"), Config.bEnableCoordinatedTurns);
	SetConfigProperty(Properties, TEXT("Autopilot.Turn.CoordinatedTurnSpeedThresholdCmPerSec"), Config.CoordinatedTurnSpeedThresholdCmPerSec);
	SetConfigProperty(Properties, TEXT("Autopilot.Turn.MaxBankAngleDegrees"), Config.MaxBankAngleDegrees);
	SetConfigProperty(Properties, TEXT("Autopilot.Turn.MaxLateralAccelCmPerSecSq"), Config.MaxLateralAccelCmPerSecSq);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.GuidanceStrategy"), static_cast<int32>(Config.GuidanceStrategy));
	SetConfigProperty(Properties, TEXT("Autopilot.Path.PurePursuitLookAheadGain"), Config.PurePursuitLookAheadGain);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.PurePursuitMinLookAheadCm"), Config.PurePursuitMinLookAheadCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.PurePursuitMaxLookAheadCm"), Config.PurePursuitMaxLookAheadCm);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.VectorFieldCrossTrackGain"), Config.VectorFieldCrossTrackGain);
	SetConfigProperty(Properties, TEXT("Autopilot.Path.VectorFieldMaxCrossTrackCorrectionCm"), Config.VectorFieldMaxCrossTrackCorrectionCm);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.EnableEstimator"), Config.bEnableHoverThrustEstimator);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.InitialStateVariance"), Config.InitialStateVariance);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.ProcessNoiseVariance"), Config.ProcessNoiseVariance);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.AccelNoiseVariance"), Config.AccelNoiseVariance);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.GateSize"), Config.GateSize);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.MinHoverThrust"), Config.MinHoverThrust);
	SetConfigProperty(Properties, TEXT("Autopilot.HoverThrust.MaxHoverThrust"), Config.MaxHoverThrust);

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
