#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSimulationLODProfileNode)

namespace
{
	using namespace UE::AircraftLab::AircraftAsset;

	template<typename T>
	void SetLODProperty(FCollectionAircraftPropertyMutableFacade& Properties, const FName Key, const T& Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		}
		Properties.SetValue(Index, Value);
	}

	void SetLODStringProperty(FCollectionAircraftPropertyMutableFacade& Properties, const FName Key, const FString& Value)
	{
		int32 Index = Properties.GetKeyNameIndex(Key);
		if (Index == INDEX_NONE)
		{
			Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		}
		Properties.SetStringValue(Index, Value);
	}
}

FAircraftSimulationLODProfileNode::FAircraftSimulationLODProfileNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftSimulationLODProfileNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	if (Profile.Name.IsNone())
	{
		Context.Error(FText::FromString(TEXT("Simulation LOD requires a name.")), this);
		SetValue(Context, MoveTemp(InCollection), &Collection);
		return;
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();

	SetLODStringProperty(Properties, TEXT("SimulationLOD.Name"), Profile.Name.ToString());
	SetLODProperty(Properties, TEXT("SimulationLOD.DriveMode"), static_cast<int32>(Profile.DriveMode));
	SetLODProperty(Properties, TEXT("SimulationLOD.CollisionMode"), static_cast<int32>(Profile.CollisionMode));
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
