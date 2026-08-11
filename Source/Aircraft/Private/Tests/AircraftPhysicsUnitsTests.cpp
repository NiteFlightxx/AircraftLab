
#include "Aircraft/AircraftPhysicsUnits.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPhysicsUnitsConversionTest,
	"AircraftLab.Physics.Units.SIToChaosBoundary",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPhysicsUnitsConversionTest::RunTest(const FString& Parameters)
{
	const FVector ForceChaos = AircraftPhysicsUnits::NewtonsToChaosForce(FVector(1.0, -2.0, 3.0));
	TestEqual(TEXT("1 N equals 100 Chaos force units"), ForceChaos, FVector(100.0, -200.0, 300.0));

	const FVector TorqueChaos = AircraftPhysicsUnits::NewtonMetersToChaosTorque(FVector(1.0, -2.0, 3.0));
	TestEqual(TEXT("1 Nm equals 10000 Chaos torque units"), TorqueChaos, FVector(10000.0, -20000.0, 30000.0));
	return true;
}

#endif
