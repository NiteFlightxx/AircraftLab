// 移植自 NxGame AircraftLab/Private/Tests/FlightFailurePolicyTests.cpp。

#include "Aircraft/RotorFailureManager.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftFailurePolicyDisabledTest,
	"AircraftLab.FlightController.FailurePolicy.DisabledByDefault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftFailurePolicyDisabledTest::RunTest(const FString& Parameters)
{
	FAircraftRotorFailureManager Manager;
	Manager.PolicyStatus.bHasAuthoritySample = true;
	Manager.AuthorityInfo.HealthyRotorCount = 0;
	FAircraftFailurePolicyConfig Policy;
	EAircraftFailurePolicyAction Action = EAircraftFailurePolicyAction::EmergencyStop;

	TestFalse(TEXT("Disabled policy never triggers"), Manager.EvaluatePolicy(Policy, 10.0f, Action));
	TestFalse(TEXT("Disabled policy remains unlatched"), Manager.PolicyStatus.bTriggered);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftFailurePolicyConfirmationAndLatchTest,
	"AircraftLab.FlightController.FailurePolicy.ConfirmationAndLatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftFailurePolicyConfirmationAndLatchTest::RunTest(const FString& Parameters)
{
	FAircraftRotorFailureManager Manager;
	Manager.PolicyStatus.bHasAuthoritySample = true;
	Manager.AuthorityInfo.HealthyRotorCount = 3;

	FAircraftFailurePolicyConfig Policy;
	Policy.bEnabled = true;
	Policy.MinimumHealthyRotorCount = 4;
	Policy.MinimumCollectiveAuthority = 0.0f;
	Policy.MinimumRollAuthority = 0.0f;
	Policy.MinimumPitchAuthority = 0.0f;
	Policy.MinimumYawAuthority = 0.0f;
	Policy.ConfirmationTimeSeconds = 0.10f;
	Policy.Action = EAircraftFailurePolicyAction::Failsafe;

	EAircraftFailurePolicyAction Action = EAircraftFailurePolicyAction::WarningOnly;
	TestFalse(TEXT("Violation waits for confirmation"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestTrue(TEXT("Violation triggers at confirmation"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestEqual(TEXT("Configured action is returned"), Action, EAircraftFailurePolicyAction::Failsafe);
	TestTrue(TEXT("Trigger is latched"), Manager.PolicyStatus.bTriggered);
	TestFalse(TEXT("Latched policy does not repeat every tick"), Manager.EvaluatePolicy(Policy, 1.0f, Action));

	Manager.ResetPolicyLatch();
	TestFalse(TEXT("Explicit reset clears latch"), Manager.PolicyStatus.bTriggered);
	TestTrue(TEXT("Authority sample survives explicit latch reset"), Manager.PolicyStatus.bHasAuthoritySample);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftFailurePolicyAutomaticRecoveryTest,
	"AircraftLab.FlightController.FailurePolicy.AutomaticRecoveryWhenNotLatched",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftFailurePolicyAutomaticRecoveryTest::RunTest(const FString& Parameters)
{
	FAircraftRotorFailureManager Manager;
	Manager.PolicyStatus.bHasAuthoritySample = true;
	Manager.AuthorityInfo.HealthyRotorCount = 3;

	FAircraftFailurePolicyConfig Policy;
	Policy.bEnabled = true;
	Policy.bLatchTriggeredAction = false;
	Policy.MinimumHealthyRotorCount = 4;
	Policy.MinimumCollectiveAuthority = 0.0f;
	Policy.MinimumRollAuthority = 0.0f;
	Policy.MinimumPitchAuthority = 0.0f;
	Policy.MinimumYawAuthority = 0.0f;
	Policy.ConfirmationTimeSeconds = 0.0f;
	Policy.RecoveryConfirmationTimeSeconds = 0.10f;

	EAircraftFailurePolicyAction Action = EAircraftFailurePolicyAction::WarningOnly;
	TestTrue(TEXT("Zero confirmation triggers immediately"), Manager.EvaluatePolicy(Policy, 0.0f, Action));
	Manager.AuthorityInfo.HealthyRotorCount = 4;
	TestFalse(TEXT("Recovery does not emit an action"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestTrue(TEXT("Latch remains during recovery confirmation"), Manager.PolicyStatus.bTriggered);
	TestFalse(TEXT("Recovery completion still emits no action"), Manager.EvaluatePolicy(Policy, 0.05f, Action));
	TestFalse(TEXT("Non-latched policy resets after healthy confirmation"), Manager.PolicyStatus.bTriggered);
	return true;
}

#endif
