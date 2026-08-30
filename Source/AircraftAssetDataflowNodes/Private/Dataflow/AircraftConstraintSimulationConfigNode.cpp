#include "Dataflow/AircraftConstraintSimulationConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftConstraintSimulationConfigNode)

FAircraftConstraintSimulationConfigNode::FAircraftConstraintSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftConstraintSimulationConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const float Values[] = { Config.LinearNaturalFrequencyHz, Config.LinearDampingRatio,
		Config.LinearExtraDampingPerSecond, Config.LinearForceLimitN,
		Config.GravityFeedForwardScale, Config.DynamicsFeedForwardScale,
		Config.AttitudeNaturalFrequencyHz, Config.AttitudeDampingRatio,
		Config.AttitudeExtraDampingPerSecond, Config.AttitudeTorqueLimitNm };
	for (const float Value : Values)
	{
		if (!FMath::IsFinite(Value) || Value < 0.0f)
		{
			Context.Error(FText::FromString(TEXT("Constraint strength, damping, limits, and feed-forward scales must be finite and non-negative.")), this);
			SetValue(Context, InputCollection, &Collection);
			return;
		}
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.NaturalFrequencyHz"), Config.LinearNaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.DampingRatio"), Config.LinearDampingRatio);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.ExtraDampingPerSecond"), Config.LinearExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.ForceLimitN"), Config.LinearForceLimitN);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.GravityFeedForwardScale"), Config.GravityFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.DynamicsFeedForwardScale"), Config.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.NaturalFrequencyHz"), Config.AttitudeNaturalFrequencyHz);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.DampingRatio"), Config.AttitudeDampingRatio);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.ExtraDampingPerSecond"), Config.AttitudeExtraDampingPerSecond);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Attitude.TorqueLimitNm"), Config.AttitudeTorqueLimitNm);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.Linear.AccelerationMode"), Config.bLinearAccelerationMode);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
