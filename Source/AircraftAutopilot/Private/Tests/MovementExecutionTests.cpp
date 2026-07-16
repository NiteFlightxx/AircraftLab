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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotBezierIntentIntegrationTest,
	"AircraftAutopilot.Movement.BezierPathIntentIsConnected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotBezierIntentIntegrationTest::RunTest(const FString& Parameters)
{
	UAutopilotMovementExecutor* Executor = NewObject<UAutopilotMovementExecutor>();
	Executor->Initialize();
	FAutopilotVehicleSnapshot Snapshot;

	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathTrajectoryMode = EAutopilotPathTrajectoryMode::Bezier;
	Intent.PathPointsCm = {
		FVector(0.0f, 0.0f, 0.0f),
		FVector(100.0f, 100.0f, 0.0f),
		FVector(200.0f, 0.0f, 0.0f)
	};
	Intent.MotionConstraints.CruiseSpeedCmPerSec = 100.0f;
	Intent.MotionConstraints.MaxAccelerationCmPerSecSq = 1000.0f;
	Intent.MotionConstraints.MaxDecelerationCmPerSecSq = 1000.0f;

	const FAutopilotIntentHandle Handle = Executor->Submit(
		Intent, Snapshot, EAutopilotIntentFailureReason::None);
	TestEqual(TEXT("Bezier intent is accepted"), Executor->GetResult(Handle).Status,
		EAutopilotIntentStatus::Accepted);

	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Executor builds the Bezier trajectory"), Executor->BuildSetpoint(
		Snapshot, 0.02f, FProfiledSetpoint(), Setpoint));
	UTrajectoryGenerator* Generator = Executor->GetTrajectoryGenerator();
	TestTrue(TEXT("Bezier trajectory is valid"), Generator && Generator->IsValid());
	if (!Generator || !Generator->IsValid()) return false;

	const FTrajectoryPoint Midpoint = Generator->SampleAtGlobalArc(
		Generator->GetTotalArcLength() * 0.5f, 100.0f);
	const FTrajectoryPoint Endpoint = Generator->SampleAtGlobalArc(
		Generator->GetTotalArcLength(), 0.0f);
	TestTrue(TEXT("Bezier midpoint follows the curved control polygon"), Midpoint.PositionCm.Y > 40.0f);
	TestTrue(TEXT("Right-hand Bezier bend accelerates toward the curve"),
		Midpoint.AccelerationCmPerSecSq.Y < 0.0f && Midpoint.YawRateDegreesPerSec < 0.0f);
	TestTrue(TEXT("Bezier endpoint matches the last control point"),
		Endpoint.PositionCm.Equals(Intent.PathPointsCm.Last(), 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotCircleArcIntentIntegrationTest,
	"AircraftAutopilot.Movement.CircleArcIntentIsConnected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotCircleArcIntentIntegrationTest::RunTest(const FString& Parameters)
{
	UAutopilotMovementExecutor* Executor = NewObject<UAutopilotMovementExecutor>();
	Executor->Initialize();
	FAutopilotVehicleSnapshot Snapshot;
	Snapshot.PositionCm = FVector(100.0f, 0.0f, 0.0f);

	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::CircleArc;
	Intent.TargetPositionCm = FVector::ZeroVector;
	Intent.OrbitRadiusCm = 100.0f;
	Intent.ArcStartAngleDegrees = 0.0f;
	Intent.ArcEndAngleDegrees = 90.0f;
	Intent.MotionConstraints.CruiseSpeedCmPerSec = 100.0f;
	Intent.MotionConstraints.MaxAccelerationCmPerSecSq = 1000.0f;
	Intent.MotionConstraints.MaxDecelerationCmPerSecSq = 1000.0f;

	const FAutopilotIntentHandle Handle = Executor->Submit(
		Intent, Snapshot, EAutopilotIntentFailureReason::None);
	TestEqual(TEXT("Circle arc intent is accepted"), Executor->GetResult(Handle).Status,
		EAutopilotIntentStatus::Accepted);

	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Executor builds the circle trajectory"), Executor->BuildSetpoint(
		Snapshot, 0.02f, FProfiledSetpoint(), Setpoint));
	UTrajectoryGenerator* Generator = Executor->GetTrajectoryGenerator();
	TestTrue(TEXT("Circle trajectory is valid"), Generator && Generator->IsValid());
	if (!Generator || !Generator->IsValid()) return false;

	const FTrajectoryPoint Endpoint = Generator->SampleAtGlobalArc(
		Generator->GetTotalArcLength(), 0.0f);
	TestTrue(TEXT("Circle endpoint is derived from center, radius and end angle"),
		Endpoint.PositionCm.Equals(FVector(0.0f, 100.0f, 0.0f), 0.1f));

	FAutopilotMovementIntent FullCircle = Intent;
	FullCircle.ArcEndAngleDegrees = 360.0f;
	const FAutopilotIntentHandle FullCircleHandle = Executor->Submit(
		FullCircle, Snapshot, EAutopilotIntentFailureReason::None);
	TestTrue(TEXT("Executor builds a full circle"), Executor->BuildSetpoint(
		Snapshot, 0.02f, FProfiledSetpoint(), Setpoint));
	Executor->UpdateCompletion(Snapshot, 1.0f, FProfiledSetpoint());
	TestEqual(TEXT("A full circle does not complete immediately at its coincident endpoint"),
		Executor->GetResult(FullCircleHandle).Status, EAutopilotIntentStatus::Executing);

	FAutopilotMovementIntent Invalid = Intent;
	Invalid.ArcEndAngleDegrees = Invalid.ArcStartAngleDegrees;
	const FAutopilotIntentHandle InvalidHandle = Executor->Submit(
		Invalid, Snapshot, EAutopilotIntentFailureReason::None);
	TestEqual(TEXT("Zero-sweep circle arc is rejected"), Executor->GetResult(InvalidHandle).Status,
		EAutopilotIntentStatus::Rejected);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotJerkLimitedStopDoesNotReverseTest,
	"AircraftAutopilot.MotionProfile.JerkLimitedStopDoesNotReverse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotJerkLimitedStopDoesNotReverseTest::RunTest(const FString& Parameters)
{
	FSlewLimiter Limiter;
	Limiter.Reset(800.0f);
	Limiter.bInitialized = true;
	float MinimumValue = Limiter.Value;
	for (int32 Step = 0; Step < 1000; ++Step)
	{
		MinimumValue = FMath::Min(MinimumValue,
			Limiter.Update(0.0f, 0.01f, 400.0f, 2000.0f));
	}
	TestTrue(TEXT("Jerk-limited velocity never reverses through the stop target"), MinimumValue >= -UE_SMALL_NUMBER);
	TestTrue(TEXT("Jerk-limited velocity settles at zero"), FMath::IsNearlyZero(Limiter.Value, 0.01f));
	TestTrue(TEXT("Acceleration also settles at zero"), FMath::IsNearlyZero(Limiter.Rate, 0.01f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotMoveToProfileBrakesBeforeTargetTest,
	"AircraftAutopilot.Movement.MoveToProfileBrakesBeforeTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotMoveToProfileBrakesBeforeTargetTest::RunTest(const FString& Parameters)
{
	UTrajectoryGenerator* Generator = NewObject<UTrajectoryGenerator>();
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Waypoint;
	Request.StartPositionCm = FVector::ZeroVector;
	Request.TargetPositionCm = FVector(2000.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 800.0f;
	Request.PlanningAccelerationCmPerSecSq = 400.0f;
	Request.PlanningDecelerationCmPerSecSq = 400.0f;
	Request.PlanningJerkCmPerSecCubed = 2000.0f;
	Request.AcceptanceRadiusCm = 1.0f;
	TestTrue(TEXT("Move-to trajectory builds"), Generator->SetRequest(Request));

	UMotionProfile* Profile = NewObject<UMotionProfile>();
	FMotionProfileLimits Limits;
	Limits.MaxHorizontalSpeedCmPerSec = 800.0f;
	Limits.MaxHorizontalAccelCmPerSecSq = 400.0f;
	Limits.MaxHorizontalJerkCmPerSecCubed = 2000.0f;
	Profile->SetLimits(Limits);
	Profile->Initialize(FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector, 0.0f, 0.0f);

	constexpr float DeltaSeconds = 0.01f;
	FProfiledSetpoint Profiled = Profile->GetCurrentSetpoint();
	float MaximumProfiledX = Profiled.PositionCm.X;
	for (int32 Step = 0; Step < 3000; ++Step)
	{
		FTrajectoryPoint Nominal;
		TestTrue(TEXT("Move-to trajectory keeps producing setpoints"), Generator->UpdateSetpoint(
			DeltaSeconds, Profiled.PositionCm, Profiled.VelocityCmPerSec, Nominal));
		Profiled = Profile->Update(Nominal, DeltaSeconds);
		MaximumProfiledX = FMath::Max(MaximumProfiledX, Profiled.PositionCm.X);
		if (Generator->IsComplete()
			&& Profiled.PositionCm.Equals(Request.TargetPositionCm, 0.1f)
			&& Profiled.VelocityCmPerSec.IsNearlyZero(0.1f))
		{
			break;
		}
	}

	TestTrue(TEXT("Profiled setpoint does not pass the move-to target"),
		MaximumProfiledX <= Request.TargetPositionCm.X + 1.0f);
	TestTrue(TEXT("Profiled setpoint settles on the target"),
		Profiled.PositionCm.Equals(Request.TargetPositionCm, 0.2f));
	TestTrue(TEXT("Profiled setpoint is stopped at the target"),
		Profiled.VelocityCmPerSec.IsNearlyZero(0.2f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotIndependentHeadingTargetTest,
	"AircraftAutopilot.Movement.IndependentHeadingDoesNotReplaceDestination",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotIndependentHeadingTargetTest::RunTest(const FString& Parameters)
{
	UAutopilotMovementExecutor* Executor = NewObject<UAutopilotMovementExecutor>();
	Executor->Initialize();
	FAutopilotVehicleSnapshot Snapshot;

	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::MoveToPosition;
	Intent.TargetPositionCm = FVector(2000.0f, 0.0f, 0.0f);
	Intent.HeadingMode = EAutopilotHeadingMode::FaceTarget;
	Intent.bUseIndependentHeadingTarget = true;
	Intent.HeadingTargetPositionCm = FVector(0.0f, 2000.0f, 0.0f);
	const FAutopilotIntentHandle Handle = Executor->Submit(
		Intent, Snapshot, EAutopilotIntentFailureReason::None);

	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Move-to with an independent heading target builds"), Executor->BuildSetpoint(
		Snapshot, 0.02f, FProfiledSetpoint(), Setpoint));
	Executor->ApplyHeading(Snapshot, Setpoint);
	TestTrue(TEXT("Vehicle faces the look-at point instead of its destination"),
		FMath::IsNearlyEqual(Setpoint.YawDegrees, 90.0f, 0.1f));

	FAutopilotMovementIntent Updated = Executor->GetActiveIntent();
	Updated.HeadingTargetPositionCm = FVector(0.0f, -2000.0f, 0.0f);
	TestTrue(TEXT("Heading can be updated under the same movement handle"),
		Executor->Update(Handle, Updated));
	Executor->ApplyHeading(Snapshot, Setpoint);
	TestTrue(TEXT("Updated heading is applied while the move-to destination remains unchanged"),
		FMath::IsNearlyEqual(Setpoint.YawDegrees, -90.0f, 0.1f)
		&& Executor->GetActiveIntent().TargetPositionCm.Equals(Intent.TargetPositionCm));
	TestEqual(TEXT("Heading update keeps the original intent handle"),
		Executor->GetCurrentResult().Handle.Id, Handle.Id);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAutopilotFixedYawHeadingRateTest,
	"AircraftAutopilot.Movement.FixedYawHeadingConsumesTurnRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAutopilotFixedYawHeadingRateTest::RunTest(const FString& Parameters)
{
	UAutopilotMovementExecutor* Executor = NewObject<UAutopilotMovementExecutor>();
	Executor->Initialize();
	FAutopilotVehicleSnapshot Snapshot;
	Snapshot.YawDegrees = 10.0f;

	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::Hold;
	Intent.HeadingMode = EAutopilotHeadingMode::FixedYaw;
	Intent.FixedYawDegrees = 100.0f;
	Intent.DesiredYawRateDegPerSec = 25.0f;
	Intent.MotionConstraints.MaxYawRateDegPerSec = 25.0f;
	Executor->Submit(Intent, Snapshot, EAutopilotIntentFailureReason::None);

	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Target-yaw hold setpoint builds"), Executor->BuildSetpoint(
		Snapshot, 1.0f, FProfiledSetpoint(), Setpoint));
	Executor->ApplyHeading(Snapshot, Setpoint);
	TestTrue(TEXT("Target yaw remains the final heading"),
		FMath::IsNearlyEqual(Setpoint.YawDegrees, 100.0f, 0.01f));
	TestTrue(TEXT("Scan rate is emitted toward the target"),
		FMath::IsNearlyEqual(Setpoint.YawRateDegreesPerSec, 25.0f, 0.01f));
	return true;
}

#endif
