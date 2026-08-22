#include "Aircraft/AircraftAerodynamics.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAerodynamicsUnitsTest,
	"AircraftLab.Aerodynamics.SIUnitsAndBodyAxes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAerodynamicsUnitsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAerodynamicsRuntimeConfig Config;
	Config.AirDensityKgPerM3 = 1.225f;
	Config.LinearDragNsPerM = FVector(2.0, 3.0, 4.0);
	Config.DragAreaCoefficientM2 = FVector(1.0, 0.5, 0.25);
	Config.AngularDragNmPerRadPerSec = FVector(1.0, 2.0, 3.0);
	Config.QuadraticAngularDragNmPerRadPerSecSq = FVector(0.5, 0.5, 0.5);
	Config.MaxRelativeAirspeedCmPerSec = 10000.0f;

	const FAircraftAerodynamicWrench Wrench = AircraftAerodynamics::ComputeWrench(
		Config, FQuat::Identity, FVector(1000.0, 0.0, 0.0), FVector::ZeroVector,
		FVector(2.0, 0.0, 0.0));
	const float ExpectedForceXN = -(2.0f * 10.0f + 0.5f * 1.225f * 1.0f * 100.0f);
	const float ExpectedTorqueXNm = -(1.0f * 2.0f + 0.5f * 4.0f);
	TestTrue(TEXT("UE cm/s is converted to SI m/s exactly once"),
		FMath::IsNearlyEqual(static_cast<float>(Wrench.ForceWorldN.X), ExpectedForceXN, 0.01f));
	TestTrue(TEXT("No force leaks into orthogonal body axes"),
		FMath::IsNearlyZero(static_cast<float>(Wrench.ForceWorldN.Y), 0.001f)
		&& FMath::IsNearlyZero(static_cast<float>(Wrench.ForceWorldN.Z), 0.001f));
	TestTrue(TEXT("Angular drag remains in N m and opposes angular velocity"),
		FMath::IsNearlyEqual(static_cast<float>(Wrench.TorqueBodyNm.X), ExpectedTorqueXNm, 0.01f));
	return true;
}

#endif
