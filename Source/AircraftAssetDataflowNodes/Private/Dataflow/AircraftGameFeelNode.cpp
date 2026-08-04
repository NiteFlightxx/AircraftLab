#include "Dataflow/AircraftGameFeelNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftGameFeelNode)

FAircraftGameFeelNode::FAircraftGameFeelNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);

	RegisterInputConnection(&RcExpoRoll);
	RegisterInputConnection(&RcExpoPitch);
	RegisterInputConnection(&RcExpoYaw);
	RegisterInputConnection(&RcExpoThrottle);
	RegisterInputConnection(&InputDeadzone);
	RegisterInputConnection(&StickResponseTimeSeconds);
	RegisterInputConnection(&CameraShakeScale);
}

void FAircraftGameFeelNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));

	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

	if (TArrayView<float> A = Facade.GetGameFeelRcExpoRoll(); A.Num() > 0)              { A[0] = GetValue(Context, &RcExpoRoll); }
	if (TArrayView<float> A = Facade.GetGameFeelRcExpoPitch(); A.Num() > 0)             { A[0] = GetValue(Context, &RcExpoPitch); }
	if (TArrayView<float> A = Facade.GetGameFeelRcExpoYaw(); A.Num() > 0)               { A[0] = GetValue(Context, &RcExpoYaw); }
	if (TArrayView<float> A = Facade.GetGameFeelRcExpoThrottle(); A.Num() > 0)          { A[0] = GetValue(Context, &RcExpoThrottle); }
	if (TArrayView<float> A = Facade.GetGameFeelInputDeadzone(); A.Num() > 0)           { A[0] = GetValue(Context, &InputDeadzone); }
	if (TArrayView<float> A = Facade.GetGameFeelStickResponseTimeSeconds(); A.Num() > 0){ A[0] = GetValue(Context, &StickResponseTimeSeconds); }
	if (TArrayView<float> A = Facade.GetGameFeelCameraShakeScale(); A.Num() > 0)        { A[0] = GetValue(Context, &CameraShakeScale); }

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
