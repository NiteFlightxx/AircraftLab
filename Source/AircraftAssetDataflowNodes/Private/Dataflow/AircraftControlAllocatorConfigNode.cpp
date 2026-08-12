#include "Dataflow/AircraftControlAllocatorConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftControlAllocatorConfigNode)

FAircraftControlAllocatorConfigNode::FAircraftControlAllocatorConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftControlAllocatorConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	if (!FMath::IsFinite(Config.DampedPseudoInverseLambda) || !FMath::IsFinite(Config.MinimumCosTilt)
		|| Config.DampedPseudoInverseLambda < 0.0f || Config.MinimumCosTilt < 0.05f || Config.MinimumCosTilt > 1.0f)
	{
		Context.Error(FText::FromString(TEXT("Control-allocator values are outside their valid range.")), this);
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
	if (TArrayView<float> Values = Facade.GetFcAllocationDamping(); !Values.IsEmpty())
	{
		Values[0] = Config.DampedPseudoInverseLambda;
	}

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Allocator.EnableTiltCompensation"), Config.bEnableTiltCompensation);
	SetConfigProperty(Properties, TEXT("FlightController.Allocator.MinimumCosTilt"), Config.MinimumCosTilt);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
