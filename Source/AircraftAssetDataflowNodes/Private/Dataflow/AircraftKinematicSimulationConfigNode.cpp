#include "Dataflow/AircraftKinematicSimulationConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftKinematicSimulationConfigNode)

FAircraftKinematicSimulationConfigNode::FAircraftKinematicSimulationConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftKinematicSimulationConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	if (!FMath::IsFinite(Config.PositionCorrectionRate) || Config.PositionCorrectionRate < 0.0f
		|| !FMath::IsFinite(Config.RotationInterpSpeed) || Config.RotationInterpSpeed < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Kinematic correction rates must be finite and non-negative.")), this);
		SetValue(Context, InputCollection, &Collection);
		return;
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.SweepMovement"), Config.bSweepMovement);
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.PositionCorrectionRate"), Config.PositionCorrectionRate);
	SetConfigProperty(Properties, TEXT("FlightController.Kinematic.RotationInterpSpeed"), Config.RotationInterpSpeed);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
