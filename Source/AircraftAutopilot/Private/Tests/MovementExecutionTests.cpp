#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "AutopilotMovementExecutor.h"
#include "MotionProfile/MotionProfile.h"
#include "PathFollowing/PurePursuitGuidance.h"
#include "Trajectory/TrajectoryGenerator.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotMotionProfileContinuityTest,
	"AircraftAutopilot.Movement.MotionProfilePreservesInitialState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotMotionProfileContinuityTest::RunTest(const FString& Parameters)
{
	UMotionProfile* Profile = NewObject<UMotionProfile>();
	FMotionProfileLimits Limits;
	Limits.MaxHorizontalSpeedCmPerSec = 2000.0f;
	Limits.MaxHorizontalAccelCmPerSecSq = 100.0f;
	Limits.MaxHorizontalJerkCmPerSecCubed = 0.0f;
	Profile->SetLimits(Limits);
	Profile->Initialize(
		FVector(100.0f, 0.0f, 0.0f),
		FVector(200.0f, 0.0f, 0.0f),
		FVector::ZeroVector,
		0.0f,
		0.0f);

	FTrajectoryPoint Nominal;
	Nominal.PositionCm = FVector(120.0f, 0.0f, 0.0f);
	Nominal.VelocityCmPerSec = FVector(1000.0f, 0.0f, 0.0f);
	Nominal.bValid = true;
	const FProfiledSetpoint Result = Profile->Update(Nominal, 0.1f);

	TestTrue(TEXT("Setpoint remains valid"), Result.bValid);
	TestTrue(TEXT("Existing velocity is preserved instead of resetting to zero"),
		Result.VelocityCmPerSec.X >= 200.0f);
	TestTrue(TEXT("Intent transition obeys the acceleration limit"),
		Result.VelocityCmPerSec.X <= 210.01f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotIntentLifecycleTest,
	"AircraftAutopilot.Movement.IntentLifecycleIsExplicit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotIntentLifecycleTest::RunTest(const FString& Parameters)
{
	UAutopilotMovementExecutor* Executor = NewObject<UAutopilotMovementExecutor>();
	Executor->Initialize();
	FAutopilotVehicleSnapshot Snapshot;
	Snapshot.PositionCm = FVector(10.0f, 20.0f, 30.0f);

	FAutopilotMovementIntent First;
	First.Type = EAutopilotMovementIntentType::MoveToPosition;
	First.TargetPositionCm = FVector(1000.0f, 0.0f, 0.0f);
	const FAutopilotIntentHandle FirstHandle = Executor->Submit(
		First, Snapshot, EAutopilotIntentFailureReason::None);
	TestTrue(TEXT("Valid intent receives a handle"), FirstHandle.IsValid());
	TestEqual(TEXT("Valid intent is accepted"), Executor->GetResult(FirstHandle).Status,
		EAutopilotIntentStatus::Accepted);

	FAutopilotMovementIntent Second = First;
	Second.TargetPositionCm = FVector(2000.0f, 0.0f, 0.0f);
	const FAutopilotIntentHandle SecondHandle = Executor->Submit(
		Second, Snapshot, EAutopilotIntentFailureReason::None);
	TestEqual(TEXT("Replacing an intent explicitly interrupts the previous handle"),
		Executor->GetResult(FirstHandle).Status, EAutopilotIntentStatus::Interrupted);
	TestEqual(TEXT("The replacement becomes current"),
		Executor->GetResult(SecondHandle).Status, EAutopilotIntentStatus::Accepted);
	TestTrue(TEXT("Current intent can be cancelled by its handle"), Executor->Cancel(SecondHandle, Snapshot));
	TestEqual(TEXT("Cancellation is retained as a terminal result"),
		Executor->GetResult(SecondHandle).Status, EAutopilotIntentStatus::Cancelled);

	FAutopilotMovementIntent InvalidPath;
	InvalidPath.Type = EAutopilotMovementIntentType::FollowPath;
	const FAutopilotIntentHandle InvalidHandle = Executor->Submit(
		InvalidPath, Snapshot, EAutopilotIntentFailureReason::None);
	TestEqual(TEXT("Invalid paths are rejected before trajectory generation"),
		Executor->GetResult(InvalidHandle).Status, EAutopilotIntentStatus::Rejected);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotYawDynamicsLimitTest,
	"AircraftAutopilot.Movement.YawRateUsesAccelerationLimit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotYawDynamicsLimitTest::RunTest(const FString& Parameters)
{
	UMotionProfile* Profile = NewObject<UMotionProfile>();
	FMotionProfileLimits Limits;
	Limits.MaxYawRateDegPerSec = 30.0f;
	Limits.MaxYawAccelDegPerSecSq = 10.0f;
	Limits.MaxYawJerkDegPerSecCubed = 0.0f;
	Profile->SetLimits(Limits);
	Profile->Initialize(FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector, 0.0f, 0.0f);

	FTrajectoryPoint Nominal;
	Nominal.PositionCm = FVector::ZeroVector;
	Nominal.YawRateDegreesPerSec = 90.0f;
	Nominal.bValid = true;
	const FProfiledSetpoint Result = Profile->Update(Nominal, 0.1f);

	TestTrue(TEXT("Yaw rate target is rate-clamped and acceleration-shaped"),
		FMath::IsNearlyEqual(Result.YawRateDegreesPerSec, 1.0f, 0.01f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotOrbitGuidanceWrapTest,
	"AircraftAutopilot.Movement.PurePursuitContinuesAcrossOrbitLaps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotOrbitGuidanceWrapTest::RunTest(const FString& Parameters)
{
	UTrajectoryGenerator* Generator = NewObject<UTrajectoryGenerator>();
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Orbit;
	Request.StartPositionCm = FVector(100.0f, 0.0f, 0.0f);
	Request.OrbitCenterCm = FVector::ZeroVector;
	Request.OrbitRadiusCm = 100.0f;
	Request.CruiseSpeedCmPerSec = 100.0f;
	Request.OrbitAngularRateDegPerSec = 45.0f;
	TestTrue(TEXT("Orbit request builds"), Generator->SetRequest(Request));

	FTrajectoryPoint Current;
	const float TravelArc = 2.5f * PI * Request.OrbitRadiusCm;
	TestTrue(TEXT("Orbit advances beyond one lap"), Generator->UpdateSetpoint(
		TravelArc / Request.CruiseSpeedCmPerSec,
		Request.StartPositionCm,
		FVector::ZeroVector,
		Current));

	UPurePursuitGuidance* Guidance = NewObject<UPurePursuitGuidance>();
	Guidance->SetTrajectory(Generator);
	FGuidanceCommand Command;
	TestTrue(TEXT("Guidance remains valid after the first lap"), Guidance->Update(
		Current.PositionCm, Current.VelocityCmPerSec, 0.01f, Command));

	const float ExpectedLookAheadArc = Generator->GetCurrentArcLength() + 150.0f;
	const FTrajectoryPoint Expected = Generator->SampleAtGlobalArc(ExpectedLookAheadArc, 100.0f);
	TestTrue(TEXT("Look-ahead samples the continuing lap instead of clamping to the first lap"),
		Command.LookAheadPointCm.Equals(Expected.PositionCm, 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotGuidanceSpeedTest,
	"AircraftAutopilot.Movement.GuidancePreservesConfiguredCruiseSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotGuidanceSpeedTest::RunTest(const FString& Parameters)
{
	UTrajectoryGenerator* Generator = NewObject<UTrajectoryGenerator>();
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Line;
	Request.StartPositionCm = FVector::ZeroVector;
	Request.TargetPositionCm = FVector(10000.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 1500.0f;
	Request.PlanningAccelerationCmPerSecSq = 10000.0f;
	Request.PlanningDecelerationCmPerSecSq = 10000.0f;
	TestTrue(TEXT("High-speed path builds"), Generator->SetRequest(Request));

	FTrajectoryPoint Current;
	TestTrue(TEXT("High-speed path advances"), Generator->UpdateSetpoint(
		1.0f, FVector::ZeroVector, FVector::ZeroVector, Current));
	UPurePursuitGuidance* Guidance = NewObject<UPurePursuitGuidance>();
	Guidance->SetTrajectory(Generator);
	FGuidanceCommand Command;
	TestTrue(TEXT("Guidance produces a command"), Guidance->Update(
		Current.PositionCm, Current.VelocityCmPerSec, 0.01f, Command));
	TestTrue(TEXT("Guidance does not impose the removed 800 cm/s duplicate cap"),
		Command.DesiredVelocityCmPerSec.Size() > 1400.0f);
	return true;
}

#endif
