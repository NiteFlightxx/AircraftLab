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

	// 路径点不足 → 拒绝
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathPointsCm = { FVector::ZeroVector };
	const FAutopilotIntentHandle Handle = Executor.Submit(
		Intent, MakeSnapshot(FVector::ZeroVector), EAutopilotIntentFailureReason::None);
	TestTrue(TEXT("Under-specified path intent receives a queryable handle"), Handle.IsValid());
	TestEqual(TEXT("Under-specified path intent is rejected"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Rejected);

	// 未激活/飞控不可用时提交 → 拒绝原因透传
	const FAutopilotIntentHandle RejectedHandle = Executor.Submit(
		MakeMoveToIntent(FVector(100.0f, 0.0f, 0.0f)), MakeSnapshot(FVector::ZeroVector),
		EAutopilotIntentFailureReason::AutopilotInactive);
	TestTrue(TEXT("Inactive autopilot submission receives a queryable handle"), RejectedHandle.IsValid());
	TestEqual(TEXT("Inactive rejection reason is retained"),
		Executor.GetResult(RejectedHandle).FailureReason,
		EAutopilotIntentFailureReason::AutopilotInactive);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorRootMotionLifecycleTest,
	"AircraftAutopilot.Movement.RootMotionUsesExternalLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorRootMotionLifecycleTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	const FAircraftAutopilotVehicleSnapshot Snapshot = MakeSnapshot(FVector(100.0f, 200.0f, 300.0f));
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::RootMotion;
	const FAutopilotIntentHandle Handle = Executor.Submit(
		Intent, Snapshot, EAutopilotIntentFailureReason::None);

	TestTrue(TEXT("External tick starts Root Motion execution"),
		Executor.TickExternalIntent(Handle, Snapshot, 0.25f, 0.4f));
	TestEqual(TEXT("Root Motion is executing"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Executing);
	TestTrue(TEXT("Root Motion progress is retained"),
		FMath::IsNearlyEqual(Executor.GetResult(Handle).Progress, 0.4f));
	TestTrue(TEXT("External completion succeeds"),
		Executor.FinishExternalIntent(
			Handle, Snapshot, EAutopilotIntentStatus::Succeeded,
			EAutopilotIntentFailureReason::None));
	TestEqual(TEXT("Completed Root Motion reports success"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Succeeded);
	TestEqual(TEXT("Root Motion completion returns to Hold"),
		Executor.GetActiveIntent().Type, EAutopilotMovementIntentType::Hold);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorVelocityContinuityTest,
	"AircraftAutopilot.Movement.VelocityCommandPreservesProfilePosition",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorVelocityContinuityTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::MoveWithVelocity;
	Intent.DesiredVelocityCmPerSec = FVector(300.0f, 0.0f, 0.0f);
	const FAircraftAutopilotVehicleSnapshot Snapshot = MakeSnapshot(FVector(1000.0f, 0.0f, 0.0f));
	Executor.Submit(Intent, Snapshot, EAutopilotIntentFailureReason::None);
	FProfiledSetpoint Previous;
	Previous.PositionCm = FVector(250.0f, 0.0f, 0.0f);
	Previous.bValid = true;
	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Velocity command produces a setpoint"),
		Executor.BuildSetpoint(Snapshot, 0.02f, Previous, Setpoint));
	TestTrue(TEXT("Velocity command continues from the previous profiled position"),
		Setpoint.PositionCm.Equals(Previous.PositionCm));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorHeadingSemanticsTest,
	"AircraftAutopilot.Movement.HeadingUsesHeldYawAndBrakingRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorHeadingSemanticsTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	FAircraftAutopilotVehicleSnapshot Snapshot = MakeSnapshot(FVector::ZeroVector);
	Snapshot.YawDegrees = 15.0f;
	Executor.EnterHold(Snapshot);
	Snapshot.YawDegrees = 40.0f;
	FTrajectoryPoint Setpoint;
	Setpoint.bValid = true;
	Executor.ApplyHeading(Snapshot, Setpoint);
	TestTrue(TEXT("KeepCurrent retains the captured yaw"),
		FMath::IsNearlyEqual(Setpoint.YawDegrees, 15.0f));

	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::Hold;
	Intent.HeadingMode = EAutopilotHeadingMode::FixedYaw;
	Intent.FixedYawDegrees = 41.0f;
	Intent.DesiredYawRateDegPerSec = 25.0f;
	Intent.MotionConstraints.MaxYawRateDegPerSec = 25.0f;
	Intent.MotionConstraints.MaxYawAccelerationDegPerSecSq = 10.0f;
	Executor.Submit(Intent, Snapshot, EAutopilotIntentFailureReason::None);
	Executor.ApplyHeading(Snapshot, Setpoint);
	TestTrue(TEXT("Fixed yaw rate is reduced to the braking-limited rate"),
		FMath::IsNearlyEqual(Setpoint.YawRateDegreesPerSec, FMath::Sqrt(20.0f), 0.01f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftExecutorCircleArcCompletionTest,
	"AircraftAutopilot.Movement.FullCircleDoesNotCompleteAtStart",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftExecutorCircleArcCompletionTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	FAircraftAutopilotVehicleSnapshot Snapshot = MakeSnapshot(FVector(100.0f, 0.0f, 0.0f));
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::CircleArc;
	Intent.OrbitRadiusCm = 100.0f;
	Intent.ArcStartAngleDegrees = 0.0f;
	Intent.ArcEndAngleDegrees = 360.0f;
	Intent.ArrivalMode = EAutopilotArrivalMode::StopAndComplete;
	const FAutopilotIntentHandle Handle = Executor.Submit(
		Intent, Snapshot, EAutopilotIntentFailureReason::None);
	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Full-circle trajectory produces a setpoint"),
		Executor.BuildSetpoint(Snapshot, 0.02f, FProfiledSetpoint(), Setpoint));
	FProfiledSetpoint Profiled;
	Profiled.bValid = true;
	Executor.UpdateCompletion(Snapshot, 1.0f, Profiled);
	TestEqual(TEXT("Coincident endpoint does not complete before the sweep"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Executing);
	return true;
}

#endif
