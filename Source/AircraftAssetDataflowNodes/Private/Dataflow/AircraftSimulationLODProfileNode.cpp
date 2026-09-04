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
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftSimulationLODProfileNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	if (Profile.Name.IsNone())
	{
		return Context.Error(TEXT("Simulation LOD requires a name."));
	}
	auto& Properties = Context.GetProperties();

	SetLODStringProperty(Properties, TEXT("SimulationLOD.Name"), Profile.Name.ToString());
	SetLODProperty(Properties, TEXT("SimulationLOD.DriveMode"), static_cast<int32>(Profile.DriveMode));
	SetLODProperty(Properties, TEXT("SimulationLOD.CollisionMode"), static_cast<int32>(Profile.CollisionMode));
	return true;
}
