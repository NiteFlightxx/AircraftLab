#include "RotorHitRecoveryPolicyAsset.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FRotorHitRecoveryCurveTest,
	"AircraftLab.FlightController.GameplayPolicy.RotorHitRecovery.Curve",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRotorHitRecoveryCurveTest::RunTest(const FString& Parameters)
{
	TestEqual(TEXT("Unstable window produces zero recovery"),
		RotorHitRecoveryPolicy::ComputeRecoveryAlpha(1.0f, 1.25f, 2.0f), 0.0f);
	TestEqual(TEXT("Ramp midpoint"),
		RotorHitRecoveryPolicy::ComputeRecoveryAlpha(2.25f, 1.25f, 2.0f), 0.5f);
	TestEqual(TEXT("Ramp clamps at completion"),
		RotorHitRecoveryPolicy::ComputeRecoveryAlpha(10.0f, 1.25f, 2.0f), 1.0f);
	TestEqual(TEXT("Zero ramp recovers immediately"),
		RotorHitRecoveryPolicy::ComputeRecoveryAlpha(1.25f, 1.25f, 0.0f), 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FRotorHitRecoveryStabilityTest,
	"AircraftLab.FlightController.GameplayPolicy.RotorHitRecovery.Stability",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRotorHitRecoveryStabilityTest::RunTest(const FString& Parameters)
{
	FDroneKinematicState State;
	State.AttitudeDegrees = FRotator(5.0f, 0.0f, 6.0f);
	State.AngularVelocityBodyDegreesPerSec = FVector(5.0f, 4.0f, 3.0f);
	State.VelocityCmPerSec.Z = 40.0f;
	TestTrue(TEXT("State inside every limit is stable"),
		RotorHitRecoveryPolicy::IsKinematicStateStable(State, 8.0f, 20.0f, 80.0f));

	State.AttitudeDegrees.Roll = 12.0f;
	TestFalse(TEXT("Excessive tilt is unstable"),
		RotorHitRecoveryPolicy::IsKinematicStateStable(State, 8.0f, 20.0f, 80.0f));
	State.AttitudeDegrees.Roll = 0.0f;
	State.AngularVelocityBodyDegreesPerSec = FVector(30.0f, 0.0f, 0.0f);
	TestFalse(TEXT("Excessive angular rate is unstable"),
		RotorHitRecoveryPolicy::IsKinematicStateStable(State, 8.0f, 20.0f, 80.0f));
	State.AngularVelocityBodyDegreesPerSec = FVector::ZeroVector;
	State.VelocityCmPerSec.Z = 120.0f;
	TestFalse(TEXT("Excessive vertical speed is unstable"),
		RotorHitRecoveryPolicy::IsKinematicStateStable(State, 8.0f, 20.0f, 80.0f));
	return true;
}

#endif
