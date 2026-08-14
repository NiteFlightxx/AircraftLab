#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

#include "AircraftRuntimeCommon/LOD/AircraftSimulationLODComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftRecentDamageWindowTest,
	"AircraftLab.SimulationLOD.RecentDamageExpires",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftRecentDamageWindowTest::RunTest(const FString& Parameters)
{
	UAircraftSimulationLODComponent* const Component = NewObject<UAircraftSimulationLODComponent>();
	Component->DamageKeepAliveSeconds = 5.0f;
	Component->NotifyRecentlyDamaged();

	TestTrue(TEXT("Damage raises simulation importance inside the keep-alive window"),
		Component->BuildSnapshot(TNumericLimits<float>::Max(), 4.0f).Importance.bRecentlyDamaged);
	TestFalse(TEXT("Damage importance expires after the keep-alive window"),
		Component->BuildSnapshot(TNumericLimits<float>::Max(), 5.01f).Importance.bRecentlyDamaged);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSimulationEvaluationScheduleTest,
	"AircraftLab.SimulationLOD.EvaluationSchedule",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSimulationEvaluationScheduleTest::RunTest(const FString& Parameters)
{
	UAircraftSimulationLODComponent* const Component = NewObject<UAircraftSimulationLODComponent>();
	Component->EvaluationIntervalSeconds = 0.25f;

	TestTrue(TEXT("A new component is immediately due"), Component->IsEvaluationDue(10.0f));
	Component->MarkEvaluated(10.0f);
	TestFalse(TEXT("The component is not due before its interval"), Component->IsEvaluationDue(10.24f));
	TestTrue(TEXT("The component is due at its interval"), Component->IsEvaluationDue(10.25f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMotionTargetModeTest,
	"AircraftLab.SimulationLOD.MotionTargetModeIsExplicit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMotionTargetModeTest::RunTest(const FString& Parameters)
{
	FAircraftMotionTarget Target;
	TestEqual(TEXT("Streamed targets track their moving setpoint by default"),
		Target.Mode, EAircraftMotionTargetMode::Tracked);
	Target.Mode = EAircraftMotionTargetMode::DirectPose;
	TestEqual(TEXT("Direct position drivers have a distinct target semantic"),
		Target.Mode, EAircraftMotionTargetMode::DirectPose);
	return true;
}

#endif
