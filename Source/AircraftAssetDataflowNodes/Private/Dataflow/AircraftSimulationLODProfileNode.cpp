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

FAircraftSimulationLODProfileData::FAircraftSimulationLODProfileData()
{
	LODs.SetNum(4);
	LODs[0].Name = TEXT("LOD0");
	LODs[0].DriveMode = EAircraftProfileDriveMode::FlightController;
	LODs[0].MaxDistanceCm = 6000.0f;
	LODs[0].CollisionMode = EAircraftProfileCollisionMode::QueryAndPhysics;
	LODs[0].SuggestedNetUpdateFrequency = 30.0f;

	LODs[1].Name = TEXT("LOD1");
	LODs[1].DriveMode = EAircraftProfileDriveMode::PhysicsConstraint;
	LODs[1].MaxDistanceCm = 15000.0f;
	LODs[1].SlowLogicIntervalSeconds = 0.05f;
	LODs[1].CollisionMode = EAircraftProfileCollisionMode::QueryAndPhysics;
	LODs[1].SuggestedNetUpdateFrequency = 15.0f;

	LODs[2].Name = TEXT("LOD2");
	LODs[2].DriveMode = EAircraftProfileDriveMode::Kinematic;
	LODs[2].MaxDistanceCm = 50000.0f;
	LODs[2].SlowLogicIntervalSeconds = 0.10f;
	LODs[2].CollisionMode = EAircraftProfileCollisionMode::QueryOnly;
	LODs[2].SuggestedNetUpdateFrequency = 8.0f;

	LODs[3].Name = TEXT("LOD3");
	LODs[3].DriveMode = EAircraftProfileDriveMode::None;
	LODs[3].MaxDistanceCm = 0.0f;
	LODs[3].bRunSlowLogic = false;
	LODs[3].SlowLogicIntervalSeconds = 0.0f;
	LODs[3].CollisionMode = EAircraftProfileCollisionMode::Disabled;
	LODs[3].SuggestedNetUpdateFrequency = 2.0f;
	LODs[3].bEnableNetworkDormancy = true;
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
	if (Profile.LODs.IsEmpty())
	{
		Context.Error(FText::FromString(TEXT("Simulation LOD Profile must contain at least one LOD.")), this);
	}
	float PreviousDistance = -1.0f;
	TSet<FName> LODNames;
	for (int32 Index = 0; Index < Profile.LODs.Num(); ++Index)
	{
		const FAircraftSimulationLODProfileEntry& LOD = Profile.LODs[Index];
		if (LOD.Name.IsNone() || LODNames.Contains(LOD.Name) || LOD.MaxDistanceCm < 0.0f
			|| LOD.SlowLogicIntervalSeconds < 0.0f || LOD.SuggestedNetUpdateFrequency < 1.0f
			|| (Index + 1 < Profile.LODs.Num() && LOD.MaxDistanceCm <= PreviousDistance))
		{
			Context.Error(FText::FromString(FString::Printf(TEXT("Simulation LOD %d contains invalid or unordered settings."), Index)), this);
		}
		LODNames.Add(LOD.Name);
		PreviousDistance = LOD.MaxDistanceCm;
	}
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();

	SetLODProperty(Properties, TEXT("SimulationLOD.EvaluationIntervalSeconds"), Profile.EvaluationIntervalSeconds);
	SetLODProperty(Properties, TEXT("SimulationLOD.MaxEvaluationsPerFrame"), Profile.MaxEvaluationsPerFrame);
	SetLODProperty(Properties, TEXT("SimulationLOD.DistanceHysteresisCm"), Profile.DistanceHysteresisCm);
	SetLODProperty(Properties, TEXT("SimulationLOD.MinimumResidenceSeconds"), Profile.MinimumLODResidenceSeconds);
	SetLODProperty(Properties, TEXT("SimulationLOD.CombatKeepAliveSeconds"), Profile.CombatKeepAliveSeconds);
	SetLODProperty(Properties, TEXT("SimulationLOD.AuthoritySimulationOnly"), Profile.bAuthoritySimulationOnly);
	SetLODProperty(Properties, TEXT("SimulationLOD.ClientProxyUsesPhysicsReplication"), Profile.bClientProxyUsesDefaultPhysicsReplication);
	SetLODProperty(Properties, TEXT("SimulationLOD.Count"), Profile.LODs.Num());

	for (int32 Index = 0; Index < Profile.LODs.Num(); ++Index)
	{
		const FAircraftSimulationLODProfileEntry& LOD = Profile.LODs[Index];
		const FString Prefix = FString::Printf(TEXT("SimulationLOD.%d."), Index);
		SetLODStringProperty(Properties, *(Prefix + TEXT("Name")), LOD.Name.ToString());
		SetLODProperty(Properties, *(Prefix + TEXT("DriveMode")), static_cast<int32>(LOD.DriveMode));
		SetLODProperty(Properties, *(Prefix + TEXT("MaxDistanceCm")), LOD.MaxDistanceCm);
		SetLODProperty(Properties, *(Prefix + TEXT("RunSlowLogic")), LOD.bRunSlowLogic);
		SetLODProperty(Properties, *(Prefix + TEXT("SlowLogicIntervalSeconds")), LOD.SlowLogicIntervalSeconds);
		SetLODProperty(Properties, *(Prefix + TEXT("CollisionMode")), static_cast<int32>(LOD.CollisionMode));
		SetLODProperty(Properties, *(Prefix + TEXT("SuggestedNetUpdateFrequency")), LOD.SuggestedNetUpdateFrequency);
		SetLODProperty(Properties, *(Prefix + TEXT("AllowDebugDraw")), LOD.bAllowDebugDraw);
		SetLODProperty(Properties, *(Prefix + TEXT("EnableNetworkDormancy")), LOD.bEnableNetworkDormancy);
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}
