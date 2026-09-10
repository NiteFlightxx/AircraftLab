#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "UObject/UnrealType.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Actor.h"

#include "AircraftRuntimeCommon/AircraftPawn.h"
#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCollisionBudgetStateTest,
	"AircraftLab.SimulationLOD.CollisionBudgetPreservesOriginalState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCollisionBudgetStateTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	AActor* const Owner = NewObject<AActor>();
	UAircraftSimulationLODComponent* const LODComponent =
		NewObject<UAircraftSimulationLODComponent>(Owner);
	UStaticMeshComponent* const PhysicalPrimitive = NewObject<UStaticMeshComponent>(Owner);
	UStaticMeshComponent* const DisabledPrimitive = NewObject<UStaticMeshComponent>(Owner);
	Owner->AddInstanceComponent(LODComponent);
	Owner->AddInstanceComponent(PhysicalPrimitive);
	Owner->AddInstanceComponent(DisabledPrimitive);
	PhysicalPrimitive->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	DisabledPrimitive->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	LODComponent->RefreshCollisionComponents();
	FAircraftSimulationBudget DisabledBudget;
	DisabledBudget.CollisionMode = EAircraftSimulationCollisionMode::Disabled;
	LODComponent->ApplyCollisionBudget(DisabledBudget);
	TestEqual(TEXT("Disabled LOD turns the physical primitive collision off"),
		PhysicalPrimitive->GetCollisionEnabled(), ECollisionEnabled::NoCollision);

	// 模拟 OnRep 先应用 LOD3、随后 BeginPlay 再次刷新缓存。
	LODComponent->RefreshCollisionComponents();
	FAircraftSimulationBudget PhysicalBudget;
	PhysicalBudget.CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	LODComponent->ApplyCollisionBudget(PhysicalBudget);
	TestEqual(TEXT("Physical LOD restores the initially physical primitive"),
		PhysicalPrimitive->GetCollisionEnabled(), ECollisionEnabled::QueryAndPhysics);
	TestEqual(TEXT("Physical LOD preserves a primitive that was originally disabled"),
		DisabledPrimitive->GetCollisionEnabled(), ECollisionEnabled::NoCollision);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNetworkReplicationContractTest,
	"AircraftLab.SimulationLOD.NetworkReplicationContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNetworkReplicationContractTest::RunTest(const FString& Parameters)
{
	const AAircraftPawn* const Pawn = GetDefault<AAircraftPawn>();
	TestTrue(TEXT("Aircraft pawn replicates"), Pawn->GetIsReplicated());
	TestTrue(TEXT("Aircraft pawn replicates movement"), Pawn->IsReplicatingMovement());

	const UAircraftSimulationLODComponent* const Component =
		GetDefault<UAircraftSimulationLODComponent>();
	TestTrue(TEXT("Simulation LOD component replicates by default"),
		Component->GetIsReplicated());
	TestTrue(TEXT("Authority-only simulation is owned by the component"),
		Component->bAuthoritySimulationOnly);
	TestTrue(TEXT("Client physics replication is owned by the component"),
		Component->bClientProxyUsesDefaultPhysicsReplication);
	TestEqual(TEXT("Component supplies one network policy for each default LOD"),
		Component->NetworkSettingsPerLOD.Num(), 4);
	if (Component->NetworkSettingsPerLOD.Num() == 4)
	{
		TestEqual(TEXT("LOD0 network frequency defaults to 30 Hz"),
			Component->NetworkSettingsPerLOD[0].NetUpdateFrequency, 30.0f);
		TestEqual(TEXT("LOD3 network frequency defaults to 2 Hz"),
			Component->NetworkSettingsPerLOD[3].NetUpdateFrequency, 2.0f);
		TestTrue(TEXT("LOD3 enables dormancy by default"),
			Component->NetworkSettingsPerLOD[3].bEnableDormancy);
	}
	const FProperty* const LODProperty = FindFProperty<FProperty>(
		UAircraftSimulationLODComponent::StaticClass(), TEXT("CurrentLODIndex"));
	TestNotNull(TEXT("Current LOD property exists"), LODProperty);
	if (LODProperty)
	{
		TestTrue(TEXT("Current LOD is a replicated property"),
			LODProperty->HasAnyPropertyFlags(CPF_Net));
		TestEqual(TEXT("Current LOD uses the network-proxy apply callback"),
			LODProperty->RepNotifyFunc, FName(TEXT("OnRep_CurrentLODIndex")));
	}
	TestNotNull(TEXT("Gameplay owns an explicit simulation LOD selection entry point"),
		UAircraftSimulationLODComponent::StaticClass()->FindFunctionByName(TEXT("SetSimulationLOD")));
	return true;
}
#if WITH_METADATA
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLodPreservationPolicyContractTest,
	"AircraftLab.SimulationLOD.PreservationPolicyContract",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLodPreservationPolicyContractTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const UFunction* const SetLodFunction =
		UAircraftSimulationLODComponent::StaticClass()->FindFunctionByName(TEXT("SetSimulationLOD"));
	TestNotNull(TEXT("SetSimulationLOD exists"), SetLodFunction);
	if (SetLodFunction)
	{
		TestNotNull(TEXT("SetSimulationLOD exposes the preservation policy"),
			FindFProperty<FBoolProperty>(SetLodFunction, TEXT("bPreserveSimulationState")));
		TestEqual(TEXT("Blueprint preservation policy defaults to false"),
			SetLodFunction->GetMetaData(TEXT("CPP_Default_bPreserveSimulationState")),
			FString(TEXT("false")));
	}

	TestNotNull(TEXT("Simulation budget carries the preservation policy"),
		FindFProperty<FBoolProperty>(
			FAircraftSimulationBudget::StaticStruct(), TEXT("bPreserveSimulationState")));
	const FAircraftSimulationBudget DefaultBudget;
	TestFalse(TEXT("Simulation state preservation is disabled by default"),
		DefaultBudget.bPreserveSimulationState);
	return true;
}
#endif
#endif
