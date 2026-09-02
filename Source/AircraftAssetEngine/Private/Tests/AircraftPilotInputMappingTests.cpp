#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

#include "AircraftAsset/AircraftPilotInputMapping.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSharedPilotInputMappingTest,
	"AircraftLab.Dataflow.Runtime.SharedPilotInputMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSharedPilotInputMappingTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.MaxHorizontalSpeedCmPerSec = 800.0f;
	Config.MaxClimbRateCmPerSec = 300.0f;
	Config.MaxDescentRateCmPerSec = 200.0f;
	Config.MaxYawRateDegreesPerSec = 90.0f;

	FAircraftPilotInput Pilot;
	Pilot.Pitch = 1.0f;
	FAircraftManualCommand Command = UE::AircraftLab::PilotInputMapping::BuildManualCommand(
		Pilot, FQuat::Identity, Config);
	TestTrue(TEXT("W follows configured +Y forward"),
		Command.DesiredVelocityCmPerSec.Equals(FVector(0.0, 800.0, 0.0), 1.e-3));

	Pilot.ResetAxes();
	Pilot.Roll = 1.0f;
	Command = UE::AircraftLab::PilotInputMapping::BuildManualCommand(Pilot, FQuat::Identity, Config);
	TestTrue(TEXT("D follows configured right axis"),
		Command.DesiredVelocityCmPerSec.Equals(FVector(-800.0, 0.0, 0.0), 1.e-3));

	Pilot.ResetAxes();
	Pilot.Throttle = 1.0f;
	Pilot.Yaw = 1.0f;
	Command = UE::AircraftLab::PilotInputMapping::BuildManualCommand(Pilot, FQuat::Identity, Config);
	TestEqual(TEXT("Positive throttle requests climb speed"), Command.DesiredVelocityCmPerSec.Z, 300.0);
	TestEqual(TEXT("Positive yaw requests configured yaw rate"), Command.DesiredYawRateDegPerSec, 90.0f);

	Pilot.ResetAxes();
	Pilot.Pitch = Config.HorizontalHoldStickDeadband * 0.5f;
	Pilot.Throttle = Config.VerticalHoldStickDeadband * 0.5f;
	Command = UE::AircraftLab::PilotInputMapping::BuildManualCommand(Pilot, FQuat::Identity, Config);
	TestTrue(TEXT("Inputs inside hold deadbands produce zero velocity"),
		Command.DesiredVelocityCmPerSec.IsNearlyZero());
	return true;
}

#endif
