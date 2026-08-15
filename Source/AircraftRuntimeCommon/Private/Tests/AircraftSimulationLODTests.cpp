#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "UObject/UnrealType.h"

#include "AircraftRuntimeCommon/AircraftPawn.h"
#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftRecentDamageWindowTest,
	"AircraftLab.SimulationLOD.RecentDamageExpires",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftRecentDamageWindowTest::RunTest(const FString& Parameters)
{
	UAircraftSimulationLODComponent* const Component = NewObject<UAircraftSimulationLODComponent>();
	Component->DamageKeepAliveSeconds = 5.0f;
	Component->NotifyRecentlyDamaged();

	TestTrue(TEXT("Damage raises simulation importance inside the keep-alive window"),
		Component->BuildSnapshot(TNumericLimits<float>::Max(), 4.0f).Importance.bRecentlyDamaged);
	TestFalse(TEXT("Damage importance expires after the keep-alive window"),
		Component->BuildSnapshot(TNumericLimits<float>::Max(), 5.01f).Importance.bRecentlyDamaged);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationEvaluationScheduleTest,
	"AircraftLab.SimulationLOD.EvaluationSchedule",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationEvaluationScheduleTest::RunTest(const FString& Parameters)
{
	UAircraftSimulationLODComponent* const Component = NewObject<UAircraftSimulationLODComponent>();
	Component->EvaluationIntervalSeconds = 0.25f;

	TestTrue(TEXT("A new component is immediately due"), Component->IsEvaluationDue(10.0f));
	Component->MarkEvaluated(10.0f);
	TestFalse(TEXT("The component is not due before its interval"), Component->IsEvaluationDue(10.24f));
	TestTrue(TEXT("The component is due at its interval"), Component->IsEvaluationDue(10.25f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMotionTargetModeTest,
	"AircraftLab.SimulationLOD.MotionTargetModeIsExplicit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMotionTargetModeTest::RunTest(const FString& Parameters)
{
	FAircraftMotionTarget Target;
	TestEqual(TEXT("Streamed targets track their moving setpoint by default"),
		Target.Mode, EAircraftMotionTargetMode::Tracked);
	Target.Mode = EAircraftMotionTargetMode::ConstrainedTrajectory;
	TestEqual(TEXT("Fully constrained trajectories have a distinct target semantic"),
		Target.Mode, EAircraftMotionTargetMode::ConstrainedTrajectory);
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
	return true;
}

#endif
