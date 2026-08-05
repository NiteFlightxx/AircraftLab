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
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearPositionStrength"), Config.LinearPositionStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearVelocityStrength"), Config.LinearVelocityStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.LinearForceLimit"), Config.LinearForceLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularPositionStrength"), Config.AngularPositionStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularVelocityStrength"), Config.AngularVelocityStrength);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AngularTorqueLimit"), Config.AngularTorqueLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Constraint.AccelerationMode"), Config.bAccelerationMode);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
