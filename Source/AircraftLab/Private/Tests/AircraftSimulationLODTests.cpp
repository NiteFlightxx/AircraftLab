#include "AircraftSimulationLODPolicy.h"
#include "AircraftSimulationLODProfileAsset.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODDistanceSelectionTest,
	"AircraftLab.SimulationLOD.DistanceSelection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODDistanceSelectionTest::RunTest(const FString& Parameters)
{
	const UAircraftSimulationLODProfileAsset* Profile = GetDefault<UAircraftSimulationLODProfileAsset>();
	TestEqual(TEXT("Near player uses full physics"),
		AircraftSimulationLODPolicy::SelectNominalTier(*Profile, 3000.0f),
		EAircraftSimulationTier::FullPhysics);
	TestEqual(TEXT("Nearby patrol uses reduced physics"),
		AircraftSimulationLODPolicy::SelectNominalTier(*Profile, 10000.0f),
		EAircraftSimulationTier::ReducedPhysics);
	TestEqual(TEXT("Distant patrol uses kinematic simulation"),
		AircraftSimulationLODPolicy::SelectNominalTier(*Profile, 30000.0f),
		EAircraftSimulationTier::Kinematic);
	TestEqual(TEXT("Irrelevant aircraft becomes dormant"),
		AircraftSimulationLODPolicy::SelectNominalTier(*Profile, 80000.0f),
		EAircraftSimulationTier::Dormant);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODImportanceOverrideTest,
	"AircraftLab.SimulationLOD.ImportanceForcesFullPhysics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODImportanceOverrideTest::RunTest(const FString& Parameters)
{
	UAircraftSimulationLODProfileAsset* Profile = NewObject<UAircraftSimulationLODProfileAsset>();
	Profile->MinimumTierResidenceSeconds = 0.0f;
	FAircraftSimulationSnapshot Snapshot;
	Snapshot.NearestPlayerDistanceCm = 100000.0f;
	Snapshot.Importance.bInCombat = true;
	TestEqual(TEXT("Combat overrides distance and residence"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::Dormant, 0.0f, Snapshot),
		EAircraftSimulationTier::FullPhysics);
	Snapshot.Importance = FAircraftSimulationImportance();
	Snapshot.Importance.bPlayerControlled = true;
	TestEqual(TEXT("Player control always uses full physics"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::Kinematic, 10.0f, Snapshot),
		EAircraftSimulationTier::FullPhysics);
	Snapshot.Importance = FAircraftSimulationImportance();
	Snapshot.Importance.bHasExternalPhysicsConstraint = true;
	TestEqual(TEXT("External payload/world constraint forces full physics"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::Kinematic, 10.0f, Snapshot),
		EAircraftSimulationTier::FullPhysics);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODHysteresisTest,
	"AircraftLab.SimulationLOD.HysteresisAndResidence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODHysteresisTest::RunTest(const FString& Parameters)
{
	UAircraftSimulationLODProfileAsset* Profile = NewObject<UAircraftSimulationLODProfileAsset>();
	Profile->DistanceHysteresisCm = 1000.0f;
	Profile->MinimumTierResidenceSeconds = 1.0f;
	FAircraftSimulationSnapshot Snapshot;
	Snapshot.NearestPlayerDistanceCm = 8000.0f;
	TestEqual(TEXT("Minimum residence prevents an early downgrade"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::FullPhysics, 0.5f, Snapshot),
		EAircraftSimulationTier::FullPhysics);

	Profile->MinimumTierResidenceSeconds = 0.0f;
	Snapshot.NearestPlayerDistanceCm = 6500.0f;
	TestEqual(TEXT("Full/reduced boundary holds inside outer hysteresis"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::FullPhysics, 2.0f, Snapshot),
		EAircraftSimulationTier::FullPhysics);
	Snapshot.NearestPlayerDistanceCm = 7100.0f;
	TestEqual(TEXT("Crossing outer hysteresis downgrades"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::FullPhysics, 2.0f, Snapshot),
		EAircraftSimulationTier::ReducedPhysics);
	Snapshot.NearestPlayerDistanceCm = 5500.0f;
	TestEqual(TEXT("Reduced/full boundary holds inside inner hysteresis"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::ReducedPhysics, 2.0f, Snapshot),
		EAircraftSimulationTier::ReducedPhysics);
	Snapshot.NearestPlayerDistanceCm = 4900.0f;
	TestEqual(TEXT("Crossing inner hysteresis upgrades"),
		AircraftSimulationLODPolicy::ResolveTier(
			*Profile, EAircraftSimulationTier::ReducedPhysics, 2.0f, Snapshot),
		EAircraftSimulationTier::FullPhysics);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationLODBudgetTest,
	"AircraftLab.SimulationLOD.BudgetMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationLODBudgetTest::RunTest(const FString& Parameters)
{
	const UAircraftSimulationLODProfileAsset* Profile = GetDefault<UAircraftSimulationLODProfileAsset>();
	const FAircraftSimulationBudget Kinematic = Profile->BuildBudget(EAircraftSimulationTier::Kinematic);
	TestFalse(TEXT("Kinematic tier disables Chaos control"), Kinematic.bRunFlightController);
	TestTrue(TEXT("Kinematic tier keeps slow guidance"), Kinematic.bRunSlowLogic);
	TestTrue(TEXT("Kinematic tier enables manager movement"), Kinematic.bEnableKinematicMovement);
	TestEqual(TEXT("Kinematic collision remains queryable"), Kinematic.CollisionMode,
		EAircraftSimulationCollisionMode::QueryOnly);

	const FAircraftSimulationBudget Proxy = Profile->BuildBudget(EAircraftSimulationTier::Kinematic, true);
	TestTrue(TEXT("Remote client is marked as proxy"), Proxy.bIsNetworkProxy);
	TestFalse(TEXT("Remote client does not run flight controller"), Proxy.bRunFlightController);
	TestFalse(TEXT("Remote client does not run autopilot"), Proxy.bRunSlowLogic);
	TestFalse(TEXT("Remote client follows replication instead of kinematic guidance"),
		Proxy.bEnableKinematicMovement);
	TestFalse(TEXT("Kinematic server tier also disables client proxy physics"), Proxy.bEnablePhysics);

	const FAircraftSimulationBudget PhysicalProxy = Profile->BuildBudget(
		EAircraftSimulationTier::FullPhysics, true);
	TestTrue(TEXT("Physical client proxy keeps Chaos for UE default physics interpolation"),
		PhysicalProxy.bEnablePhysics);
	TestFalse(TEXT("Physical client proxy still does not run the flight controller"),
		PhysicalProxy.bRunFlightController);
	return true;
}

#endif
