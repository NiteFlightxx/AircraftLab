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
	const float Values[] = { Config.LinearStrength, Config.LinearDampingRatio,
		Config.LinearExtraDamping, Config.LinearForceLimit,
		Config.GravityFeedForwardScale, Config.DynamicsFeedForwardScale, Config.AngularStrength,
		Config.AngularDampingRatio, Config.AngularExtraDamping, Config.AngularTorqueLimit };
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
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearStrength"), Config.LinearStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearDampingRatio"), Config.LinearDampingRatio);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearExtraDamping"), Config.LinearExtraDamping);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearForceLimit"), Config.LinearForceLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.GravityFeedForwardScale"), Config.GravityFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.DynamicsFeedForwardScale"), Config.DynamicsFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularStrength"), Config.AngularStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularDampingRatio"), Config.AngularDampingRatio);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularExtraDamping"), Config.AngularExtraDamping);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularTorqueLimit"), Config.AngularTorqueLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AccelerationMode"), Config.bAccelerationMode);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
