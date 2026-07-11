#include "FlightControllerRuntimeObjects.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFlightFailurePolicyDisabledTest,
	"AircraftLab.FlightController.FailurePolicy.DisabledByDefault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFlightFailurePolicyDisabledTest::RunTest(const FString& Parameters)
{
	FRotorFailureManager Manager;
	Manager.PolicyStatus.bHasAuthoritySample = true;
	Manager.AuthorityInfo.HealthyRotorCount = 0;
	FFlightControllerFailurePolicyConfig Policy;
	EFlightFailurePolicyAction Action = EFlightFailurePolicyAction::EmergencyStop;

	TestFalse(TEXT("Disabled policy never triggers"), Manager.EvaluatePolicy(Policy, 10.0f, Action));
	TestFalse(TEXT("Disabled policy remains unlatched"), Manager.PolicyStatus.bTriggered);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFlightFailurePolicyConfirmationAndLatchTest,
	"AircraftLab.FlightController.FailurePolicy.ConfirmationAndLatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFlightFailurePolicyConfirmationAndLatchTest::RunTest(const FString& Parameters)
{
	FRotorFailureManager Manager;
	Manager.PolicyStatus.bHasAuthoritySample = true;
	Manager.AuthorityInfo.HealthyRotorCount = 3;

	FFlightControllerFailurePolicyConfig Policy;
	Policy.bEnabled = true;
	Policy.MinimumHealthyRotorCount = 4;
	Policy.MinimumCollectiveAuthority = 0.0f;
	Policy.MinimumRollAuthority = 0.0f;
	Policy.MinimumPitchAuthority = 0.0f;
	Policy.MinimumYawAuthority = 0.0f;
	Policy.ConfirmationTimeSeconds = 0.10f;
	Policy.Action = EFlightFailurePolicyAction::Failsafe;

	EFlightFailurePolicyAction Action = EFlightFailurePolicyAction::WarningOnly;
	TestFalse(TEXT("Violation waits for confirmation"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestTrue(TEXT("Violation triggers at confirmation"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestEqual(TEXT("Configured action is returned"), Action, EFlightFailurePolicyAction::Failsafe);
	TestTrue(TEXT("Trigger is latched"), Manager.PolicyStatus.bTriggered);
	TestFalse(TEXT("Latched policy does not repeat every tick"), Manager.EvaluatePolicy(Policy, 1.0f, Action));

	Manager.ResetPolicyLatch();
	TestFalse(TEXT("Explicit reset clears latch"), Manager.PolicyStatus.bTriggered);
	TestTrue(TEXT("Authority sample survives explicit latch reset"), Manager.PolicyStatus.bHasAuthoritySample);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FFlightFailurePolicyAutomaticRecoveryTest,
	"AircraftLab.FlightController.FailurePolicy.AutomaticRecoveryWhenNotLatched",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFlightFailurePolicyAutomaticRecoveryTest::RunTest(const FString& Parameters)
{
	FRotorFailureManager Manager;
	Manager.PolicyStatus.bHasAuthoritySample = true;
	Manager.AuthorityInfo.HealthyRotorCount = 3;

	FFlightControllerFailurePolicyConfig Policy;
	Policy.bEnabled = true;
	Policy.bLatchTriggeredAction = false;
	Policy.MinimumHealthyRotorCount = 4;
	Policy.MinimumCollectiveAuthority = 0.0f;
	Policy.MinimumRollAuthority = 0.0f;
	Policy.MinimumPitchAuthority = 0.0f;
	Policy.MinimumYawAuthority = 0.0f;
	Policy.ConfirmationTimeSeconds = 0.0f;
	Policy.RecoveryConfirmationTimeSeconds = 0.10f;

	EFlightFailurePolicyAction Action = EFlightFailurePolicyAction::WarningOnly;
	TestTrue(TEXT("Zero confirmation triggers immediately"), Manager.EvaluatePolicy(Policy, 0.0f, Action));
	Manager.AuthorityInfo.HealthyRotorCount = 4;
	TestFalse(TEXT("Recovery does not emit an action"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestTrue(TEXT("Latch remains during recovery confirmation"), Manager.PolicyStatus.bTriggered);
	TestFalse(TEXT("Recovery completion still emits no action"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestFalse(TEXT("Non-latched policy resets after healthy confirmation"), Manager.PolicyStatus.bTriggered);
	return true;
}

#endif
