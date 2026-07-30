#include "AircraftSimulationLODPolicy.h"
#include "AircraftSimulationLODProfileAsset.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODDistanceSelectionTest,
	"AircraftLab.SimulationLOD.ArrayDistanceSelection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODDistanceSelectionTest::RunTest(
	const FString& Parameters)
{
	const UAircraftSimulationLODProfileAsset* Profile =
		GetDefault<UAircraftSimulationLODProfileAsset>();
	TestEqual(TEXT("The default profile contains four authored entries"),
		Profile->LODs.Num(), 4);
	TestEqual(TEXT("Near player selects array entry zero"),
		AircraftSimulationLODPolicy::SelectNominalLOD(*Profile, 3000.0f), 0);
	TestEqual(TEXT("Second distance band selects entry one"),
		AircraftSimulationLODPolicy::SelectNominalLOD(*Profile, 10000.0f), 1);
	TestEqual(TEXT("Third distance band selects entry two"),
		AircraftSimulationLODPolicy::SelectNominalLOD(*Profile, 30000.0f), 2);
	TestEqual(TEXT("The final entry is the infinite-distance fallback"),
		AircraftSimulationLODPolicy::SelectNominalLOD(*Profile, 80000.0f), 3);

	UAircraftSimulationLODProfileAsset* TwoLevelProfile =
		NewObject<UAircraftSimulationLODProfileAsset>();
	TwoLevelProfile->LODs.SetNum(2);
	TwoLevelProfile->LODs[0].MaxDistanceCm = 1000.0f;
	TestEqual(TEXT("Array length is not fixed to four"),
		AircraftSimulationLODPolicy::SelectNominalLOD(
			*TwoLevelProfile, 5000.0f),
		1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODDataDrivenDriveTest,
	"AircraftLab.SimulationLOD.DriveMappingIsAuthoredData",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODDataDrivenDriveTest::RunTest(
	const FString& Parameters)
{
	UAircraftSimulationLODProfileAsset* Profile =
		NewObject<UAircraftSimulationLODProfileAsset>();
	Profile->LODs[0].DriveMode = EAircraftSimulationDriveMode::Kinematic;
	Profile->LODs[1].DriveMode =
		EAircraftSimulationDriveMode::FlightController;
	Profile->LODs[2].DriveMode =
		EAircraftSimulationDriveMode::PhysicsConstraint;
	Profile->LODs[3].DriveMode = EAircraftSimulationDriveMode::None;

	TestEqual(TEXT("LOD0 uses its authored kinematic backend"),
		Profile->BuildBudget(0).DriveMode,
		EAircraftSimulationDriveMode::Kinematic);
	TestEqual(TEXT("LOD1 uses its authored flight-controller backend"),
		Profile->BuildBudget(1).DriveMode,
		EAircraftSimulationDriveMode::FlightController);
	TestEqual(TEXT("Drive lookup follows the array instead of a fixed map"),
		Profile->FindLODForDriveMode(
			EAircraftSimulationDriveMode::PhysicsConstraint),
		2);

	FAircraftSimulationSnapshot Snapshot;
	Snapshot.DriveOverride.bValid = true;
	Snapshot.DriveOverride.DriveMode =
		EAircraftSimulationDriveMode::PhysicsConstraint;
	TestEqual(TEXT("A temporary drive request resolves to the configured entry"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 0, 0.0f, Snapshot),
		2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODImportanceOverrideTest,
	"AircraftLab.SimulationLOD.ImportanceForcesHighestPriorityEntry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODImportanceOverrideTest::RunTest(
	const FString& Parameters)
{
	UAircraftSimulationLODProfileAsset* Profile =
		NewObject<UAircraftSimulationLODProfileAsset>();
	Profile->MinimumLODResidenceSeconds = 0.0f;
	FAircraftSimulationSnapshot Snapshot;
	Snapshot.NearestPlayerDistanceCm = 100000.0f;
	Snapshot.Importance.bInCombat = true;
	TestEqual(TEXT("Combat selects the first array entry"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 3, 0.0f, Snapshot),
		0);
	Snapshot.Importance = FAircraftSimulationImportance();
	Snapshot.Importance.bPlayerControlled = true;
	TestEqual(TEXT("Player control selects the first array entry"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 2, 10.0f, Snapshot),
		0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODHysteresisTest,
	"AircraftLab.SimulationLOD.ArrayHysteresisAndResidence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODHysteresisTest::RunTest(
	const FString& Parameters)
{
	UAircraftSimulationLODProfileAsset* Profile =
		NewObject<UAircraftSimulationLODProfileAsset>();
	Profile->DistanceHysteresisCm = 1000.0f;
	Profile->MinimumLODResidenceSeconds = 1.0f;
	FAircraftSimulationSnapshot Snapshot;
	Snapshot.NearestPlayerDistanceCm = 8000.0f;
	TestEqual(TEXT("Minimum residence prevents an early downgrade"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 0, 0.5f, Snapshot),
		0);

	Profile->MinimumLODResidenceSeconds = 0.0f;
	Snapshot.NearestPlayerDistanceCm = 6500.0f;
	TestEqual(TEXT("Outer hysteresis keeps the current entry"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 0, 2.0f, Snapshot),
		0);
	Snapshot.NearestPlayerDistanceCm = 7100.0f;
	TestEqual(TEXT("Crossing outer hysteresis downgrades"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 0, 2.0f, Snapshot),
		1);
	Snapshot.NearestPlayerDistanceCm = 5500.0f;
	TestEqual(TEXT("Inner hysteresis keeps the current entry"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 1, 2.0f, Snapshot),
		1);
	Snapshot.NearestPlayerDistanceCm = 4900.0f;
	TestEqual(TEXT("Crossing inner hysteresis upgrades"),
		AircraftSimulationLODPolicy::ResolveLOD(*Profile, 1, 2.0f, Snapshot),
		0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODBudgetTest,
	"AircraftLab.SimulationLOD.ArrayBudgetMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODBudgetTest::RunTest(const FString& Parameters)
{
	const UAircraftSimulationLODProfileAsset* Profile =
		GetDefault<UAircraftSimulationLODProfileAsset>();
	const FAircraftSimulationBudget Constraint = Profile->BuildBudget(1);
	TestEqual(TEXT("The default second entry selects physics constraint"),
		Constraint.DriveMode,
		EAircraftSimulationDriveMode::PhysicsConstraint);
	TestTrue(TEXT("A physical backend keeps Chaos enabled"),
		Constraint.bEnablePhysics);

	const FAircraftSimulationBudget Kinematic = Profile->BuildBudget(2);
	TestEqual(TEXT("The default third entry selects kinematic"),
		Kinematic.DriveMode, EAircraftSimulationDriveMode::Kinematic);
	TestFalse(TEXT("Kinematic disables Chaos"), Kinematic.bEnablePhysics);
	TestTrue(TEXT("Kinematic keeps slow guidance"), Kinematic.bRunSlowLogic);

	const FAircraftSimulationBudget Proxy = Profile->BuildBudget(2, true);
	TestTrue(TEXT("Remote client is marked as proxy"), Proxy.bIsNetworkProxy);
	TestEqual(TEXT("Remote client runs no local drive backend"),
		Proxy.DriveMode, EAircraftSimulationDriveMode::None);
	TestFalse(TEXT("Remote client does not run slow logic"),
		Proxy.bRunSlowLogic);
	TestFalse(TEXT("Kinematic proxy disables physics"), Proxy.bEnablePhysics);

	const FAircraftSimulationBudget Final = Profile->BuildBudget(3);
	TestTrue(TEXT("Network dormancy is authored on the final default entry"),
		Final.bEnableNetworkDormancy);
	return true;
}

#endif
