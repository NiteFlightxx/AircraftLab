// 对齐 NxGame AircraftAutopilot/Private/Tests/MovementExecutionTests.cpp 的行为规范
// （按新执行器 API 重写核心用例：提交/完成/取消/超时/替换/事件）。

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementExecutor.h"

namespace
{
	FAircraftAutopilotVehicleSnapshot MakeSnapshot(const FVector& Position)
	{
		FAircraftAutopilotVehicleSnapshot Snapshot;
		Snapshot.PositionCm = Position;
		return Snapshot;
	}

	FAutopilotMovementIntent MakeMoveToIntent(const FVector& Target)
	{
		FAutopilotMovementIntent Intent;
		Intent.Type = EAutopilotMovementIntentType::MoveToPosition;
		Intent.TargetPositionCm = Target;
		Intent.ArrivalCriteria.HorizontalToleranceCm = 50.0f;
		Intent.ArrivalCriteria.VerticalToleranceCm = 100.0f;
		Intent.ArrivalCriteria.SpeedToleranceCmPerSec = 50.0f;
		Intent.ArrivalCriteria.StableTimeSeconds = 0.0f;
		return Intent;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorSubmitAndCompleteTest,
	"AircraftAutopilot.Movement.SubmitAndCompleteLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorSubmitAndCompleteTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	Executor.Initialize();

	const FVector Target(500.0f, 0.0f, 0.0f);
	const FAutopilotIntentHandle Handle = Executor.Submit(
		MakeMoveToIntent(Target), MakeSnapshot(FVector::ZeroVector), EAutopilotIntentFailureReason::None);
	TestTrue(TEXT("MoveTo intent returns a valid handle"), Handle.IsValid());
	TestEqual(TEXT("Fresh intent starts accepted"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Accepted);

	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Executor produces a setpoint"),
		Executor.BuildSetpoint(MakeSnapshot(FVector::ZeroVector), 0.02f, FProfiledSetpoint(), Setpoint));
	TestEqual(TEXT("First setpoint transitions to executing"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Executing);

	// 到达目标并保持 → 完成
	FProfiledSetpoint Profiled;
	Profiled.PositionCm = Target;
	Profiled.YawDegrees = 0.0f;
	Profiled.bValid = true;
	Executor.UpdateCompletion(MakeSnapshot(Target), 0.02f, Profiled);
	TestEqual(TEXT("Reaching the target within tolerances succeeds"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Succeeded);

	TArray<FAutopilotIntentResult> Started;
	TArray<FAutopilotIntentResult> Finished;
	Executor.DrainEvents(Started, Finished);
	TestTrue(TEXT("Start event was queued"), Started.Num() >= 1);
	TestTrue(TEXT("Finish event was queued"), Finished.Num() >= 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorCancelAndReplaceTest,
	"AircraftAutopilot.Movement.CancelAndReplace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorCancelAndReplaceTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	Executor.Initialize();

	const FAutopilotIntentHandle First = Executor.Submit(
		MakeMoveToIntent(FVector(500.0f, 0.0f, 0.0f)), MakeSnapshot(FVector::ZeroVector),
		EAutopilotIntentFailureReason::None);
	const FAutopilotIntentHandle Second = Executor.Submit(
		MakeMoveToIntent(FVector(-500.0f, 0.0f, 0.0f)), MakeSnapshot(FVector::ZeroVector),
		EAutopilotIntentFailureReason::None);

	TestTrue(TEXT("Second intent is accepted"), Second.IsValid());
	TestEqual(TEXT("First intent reports replaced interruption"),
		Executor.GetResult(First).Status, EAutopilotIntentStatus::Interrupted);
	TestEqual(TEXT("Replacement reason is recorded"),
		Executor.GetResult(First).FailureReason, EAutopilotIntentFailureReason::Replaced);

	TestTrue(TEXT("Cancel succeeds for the active intent"),
		Executor.Cancel(Second, MakeSnapshot(FVector::ZeroVector)));
	TestEqual(TEXT("Cancelled intent reports cancellation"),
		Executor.GetResult(Second).Status, EAutopilotIntentStatus::Cancelled);
	TestFalse(TEXT("Cancelling a terminal intent fails"),
		Executor.Cancel(Second, MakeSnapshot(FVector::ZeroVector)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorTimeoutTest,
	"AircraftAutopilot.Movement.TimeoutFailsIntent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorTimeoutTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	Executor.Initialize();

	FAutopilotMovementIntent Intent = MakeMoveToIntent(FVector(100000.0f, 0.0f, 0.0f));
	Intent.TimeoutSeconds = 0.05f;
	const FAutopilotIntentHandle Handle = Executor.Submit(
		Intent, MakeSnapshot(FVector::ZeroVector), EAutopilotIntentFailureReason::None);
	TestTrue(TEXT("Timeout intent returns a valid handle"), Handle.IsValid());

	FTrajectoryPoint Setpoint;
	Executor.BuildSetpoint(MakeSnapshot(FVector::ZeroVector), 0.03f, FProfiledSetpoint(), Setpoint);
	Executor.BuildSetpoint(MakeSnapshot(FVector::ZeroVector), 0.03f, FProfiledSetpoint(), Setpoint);

	TestEqual(TEXT("Exceeding the timeout fails the intent"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Failed);
	TestEqual(TEXT("Timeout reason is recorded"),
		Executor.GetResult(Handle).FailureReason, EAutopilotIntentFailureReason::Timeout);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorInvalidIntentTest,
	"AircraftAutopilot.Movement.InvalidIntentRejected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorInvalidIntentTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	Executor.Initialize();

	// 路径点不足 → 拒绝
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathPointsCm = { FVector::ZeroVector };
	const FAutopilotIntentHandle Handle = Executor.Submit(
		Intent, MakeSnapshot(FVector::ZeroVector), EAutopilotIntentFailureReason::None);
	TestFalse(TEXT("Under-specified path intent returns no handle"), Handle.IsValid());

	// 未激活/飞控不可用时提交 → 拒绝原因透传
	const FAutopilotIntentHandle RejectedHandle = Executor.Submit(
		MakeMoveToIntent(FVector(100.0f, 0.0f, 0.0f)), MakeSnapshot(FVector::ZeroVector),
		EAutopilotIntentFailureReason::AutopilotInactive);
	TestFalse(TEXT("Inactive autopilot submission returns no handle"), RejectedHandle.IsValid());
	return true;
}

#endif
