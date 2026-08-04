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
	TestTrue(TEXT("Airscrew profile has positive maximum RPM"), Airscrew.Motor.MaxRpm > Airscrew.Motor.IdleRpm);
	TestTrue(TEXT("Airscrew profile has a non-zero thrust axis"), !Airscrew.ThrustAxisLocal.IsNearlyZero());

	const FAircraftSimulationLODProfileData SimulationLOD;
	TestEqual(TEXT("Default simulation LOD profile contains four configurable entries"), SimulationLOD.LODs.Num(), 4);
	TestEqual(TEXT("LOD0 uses the flight controller"), SimulationLOD.LODs[0].DriveMode, EAircraftProfileDriveMode::FlightController);
	TestEqual(TEXT("LOD1 uses physics constraints"), SimulationLOD.LODs[1].DriveMode, EAircraftProfileDriveMode::PhysicsConstraint);
	TestEqual(TEXT("LOD2 is kinematic"), SimulationLOD.LODs[2].DriveMode, EAircraftProfileDriveMode::Kinematic);
	TestEqual(TEXT("LOD3 disables simulation"), SimulationLOD.LODs[3].DriveMode, EAircraftProfileDriveMode::None);
	return true;
}

#endif
