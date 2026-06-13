#include "Dataflow/AircraftBatteryConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftBatteryConfigNode)

FAircraftBatteryConfigNode::FAircraftBatteryConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);

	RegisterInputConnection(&CapacityMilliAmpHour);
	RegisterInputConnection(&NominalVoltageV);
	RegisterInputConnection(&MinVoltageV);
	RegisterInputConnection(&MaxDischargeC);
	RegisterInputConnection(&InternalResistanceOhm);
}

void FAircraftBatteryConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
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

	if (TArrayView<float> A = Facade.GetBatteryCapacityMilliAmpHour(); A.Num() > 0)
	{
		A[0] = GetValue(Context, &CapacityMilliAmpHour);
	}
	if (TArrayView<float> A = Facade.GetBatteryNominalVoltageV(); A.Num() > 0)
	{
		A[0] = GetValue(Context, &NominalVoltageV);
	}
	if (TArrayView<float> A = Facade.GetBatteryMinVoltageV(); A.Num() > 0)
	{
		A[0] = GetValue(Context, &MinVoltageV);
	}
	if (TArrayView<float> A = Facade.GetBatteryMaxDischargeC(); A.Num() > 0)
	{
		A[0] = GetValue(Context, &MaxDischargeC);
	}
	if (TArrayView<float> A = Facade.GetBatteryInternalResistanceOhm(); A.Num() > 0)
	{
		A[0] = GetValue(Context, &InternalResistanceOhm);
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
