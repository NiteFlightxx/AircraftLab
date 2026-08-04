#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftFlightControllerProfileNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDataflowProfileDefaultsTest,
	"AircraftLab.Dataflow.Profiles.AuthoritativeDefaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDataflowProfileDefaultsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	const FAircraftFlightControllerProfileData FlightController;
	TestEqual(TEXT("Flight controller profile defaults to +Y forward"), FlightController.ForwardAxis, EAircraftProfileForwardAxis::PositiveY);
	TestTrue(TEXT("Collective limits are ordered"),
		FlightController.Limits.MinCollectiveCommand <= FlightController.Limits.HoverCollectiveCommand
		&& FlightController.Limits.HoverCollectiveCommand <= FlightController.Limits.MaxCollectiveCommand);

	const FAircraftAirscrewProfileData Airscrew;
	TestFalse(TEXT("A single airscrew profile has an identity"), Airscrew.Name.IsNone());
	TestTrue(TEXT("A single airscrew profile is enabled by default"), Airscrew.bEnabled);
	TestTrue(TEXT("A single airscrew profile has a positive radius"), Airscrew.RadiusCm > 0.0f);
	TestTrue(TEXT("Airscrew profile has positive maximum RPM"), Airscrew.Motor.MaxRpm > Airscrew.Motor.IdleRpm);
	TestTrue(TEXT("Airscrew profile has a non-zero thrust axis"), !Airscrew.ThrustAxisLocal.IsNearlyZero());

	const FAircraftSimulationLODProfileData SimulationLOD;
	TestFalse(TEXT("A simulation LOD node has a name"), SimulationLOD.Name.IsNone());
	TestEqual(TEXT("A standalone simulation LOD node defaults to flight-controller drive"),
		SimulationLOD.DriveMode, EAircraftProfileDriveMode::FlightController);
	TestEqual(TEXT("A standalone simulation LOD node defaults to full collision"),
		SimulationLOD.CollisionMode, EAircraftProfileCollisionMode::QueryAndPhysics);
	return true;
}

#endif
