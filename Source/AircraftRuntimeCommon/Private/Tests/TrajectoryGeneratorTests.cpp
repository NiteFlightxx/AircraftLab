
#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "AircraftRuntimeCommon/Autopilot/MotionProfile.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"

namespace
{
	FAircraftTrajectoryPlan MakePolylinePlan(const FVector& StartCm, const FVector& TargetCm)
	{
		FAircraftTrajectoryPlan Plan;
		Plan.Path.Geometry = EAircraftPathGeometry::Polyline;
		Plan.Path.PointsCm = { StartCm, TargetCm };
		return Plan;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotOrbitWrapTest,
	"AircraftAutopilot.Trajectory.OrbitContinuesAfterOneLap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotOrbitWrapTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan;
	Plan.Path.Geometry = EAircraftPathGeometry::Circle;
	Plan.Path.CircleCenterCm = FVector::ZeroVector;
	Plan.Path.CircleRadiusCm = 100.0f;
	Plan.Path.CircleStartAngleDegrees = 0.0f;
	Plan.Path.CircleSweepAngleDegrees = 360.0f;
	Plan.Traversal = EAircraftPathTraversal::Loop;
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 100.0f;

	TestTrue(TEXT("Looping circle plan builds"), Generator.SetPlan(Plan));

	FVector SimulatedPosition(100.0f, 0.0f, 0.0f);
	FTrajectoryPoint Setpoint;
	const float TargetArc = 2.5f * PI * Plan.Path.CircleRadiusCm;
	for (int32 Step = 0; Step < 2000 && Generator.GetCurrentArcLength() < TargetArc; ++Step)
	{
		TestTrue(TEXT("Looping circle produces a setpoint"),
			Generator.UpdateSetpoint(0.01f, SimulatedPosition, Setpoint));
		SimulatedPosition = Setpoint.PositionCm;
	}

	TestTrue(TEXT("Setpoint remains valid after one lap"), Setpoint.bValid);
	TestTrue(TEXT("Arc length is not clamped to one lap"),
		Generator.GetCurrentArcLength() > Generator.GetTotalArcLength());
	TestTrue(TEXT("Looping path remains on the configured circle"),
		FMath::IsNearlyEqual(Setpoint.PositionCm.Size2D(), Plan.Path.CircleRadiusCm, 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotFiniteTrajectoryCompletionTest,
	"AircraftAutopilot.Trajectory.FiniteTrajectoryCompletesOnAdvanceFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotFiniteTrajectoryCompletionTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan = MakePolylinePlan(
		FVector::ZeroVector, FVector(100.0f, 0.0f, 0.0f));
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 1000.0f;
	Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 10000.0f;
	Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 10000.0f;
	Plan.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Line plan builds"), Generator.SetPlan(Plan));
	FTrajectoryPoint Setpoint;
	FVector SimulatedPosition = FVector::ZeroVector;
	TestTrue(TEXT("Line produces a setpoint"), Generator.UpdateSetpoint(
		10.0f, SimulatedPosition, Setpoint));
	SimulatedPosition = Setpoint.PositionCm;
	TestTrue(TEXT("Line produces the terminal braking setpoint"), Generator.UpdateSetpoint(
		10.0f, SimulatedPosition, Setpoint));
	TestTrue(TEXT("Trajectory completes in the frame that reaches its end"), Generator.IsComplete());
	TestTrue(TEXT("Completion frame outputs the endpoint"),
		Setpoint.PositionCm.Equals(Plan.Path.PointsCm.Last(), 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotAsymmetricCruiseProfileTest,
	"AircraftAutopilot.Trajectory.AsymmetricAccelerationCruiseAndBraking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotAsymmetricCruiseProfileTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan = MakePolylinePlan(
		FVector::ZeroVector, FVector(3000.0f, 0.0f, 0.0f));
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 800.0f;
	Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 400.0f;
	Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 800.0f;
	Plan.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Long line plan builds"), Generator.SetPlan(Plan));

	constexpr float Dt = 0.01f;
	float MaxSpeed = 0.0f;
	float MaxObservedAcceleration = 0.0f;
	float MaxObservedDeceleration = 0.0f;
	float PreviousSpeed = 0.0f;
	FVector SimulatedPosition = FVector::ZeroVector;
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 2000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(Dt, SimulatedPosition, Setpoint);
		SimulatedPosition = Setpoint.PositionCm;
		const float Speed = Setpoint.VelocityCmPerSec.Size();
		const float Rate = (Speed - PreviousSpeed) / Dt;
		MaxObservedAcceleration = FMath::Max(MaxObservedAcceleration, Rate);
		MaxObservedDeceleration = FMath::Max(MaxObservedDeceleration, -Rate);
		MaxSpeed = FMath::Max(MaxSpeed, Speed);
		PreviousSpeed = Speed;
	}

	TestTrue(TEXT("Profile reaches configured cruise speed"), FMath::IsNearlyEqual(MaxSpeed, 800.0f, 1.0f));
	TestTrue(TEXT("Acceleration respects configured limit"), MaxObservedAcceleration <= 401.0f);
	TestTrue(TEXT("Deceleration respects configured limit"), MaxObservedDeceleration <= 801.0f);
	TestTrue(TEXT("Finite profile reaches the target"), Generator.IsComplete());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotShortPathPeakSpeedTest,
	"AircraftAutopilot.Trajectory.ShortPathUsesFeasiblePeakSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotShortPathPeakSpeedTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan = MakePolylinePlan(
		FVector::ZeroVector, FVector(600.0f, 0.0f, 0.0f));
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 800.0f;
	Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 300.0f;
	Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 600.0f;
	Plan.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Short line plan builds"), Generator.SetPlan(Plan));
	constexpr float Dt = 0.005f;
	float MaxSpeed = 0.0f;
	FVector SimulatedPosition = FVector::ZeroVector;
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 2000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(Dt, SimulatedPosition, Setpoint);
		SimulatedPosition = Setpoint.PositionCm;
		MaxSpeed = FMath::Max(MaxSpeed, Setpoint.VelocityCmPerSec.Size());
	}

	const float ExpectedPeak = FMath::Sqrt(240000.0f);
	TestTrue(TEXT("Short path does not claim unreachable cruise speed"),
		MaxSpeed < Plan.MotionConstraints.CruiseSpeedCmPerSec);
	TestTrue(TEXT("Short path peak matches asymmetric physical envelope"),
		FMath::IsNearlyEqual(MaxSpeed, ExpectedPeak, 2.0f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotTerminalSpeedTest,
	"AircraftAutopilot.Trajectory.PreservesConfiguredTerminalSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotTerminalSpeedTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan = MakePolylinePlan(
		FVector::ZeroVector, FVector(2000.0f, 0.0f, 0.0f));
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 600.0f;
	Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 300.0f;
	Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 500.0f;
	Plan.TerminalVelocityCmPerSec = FVector(200.0f, 0.0f, 0.0f);
	Plan.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Fly-through line plan builds"), Generator.SetPlan(Plan));
	FTrajectoryPoint Setpoint;
	FVector SimulatedPosition = FVector::ZeroVector;
	for (int32 Step = 0; Step < 3000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(0.005f, SimulatedPosition, Setpoint);
		SimulatedPosition = Setpoint.PositionCm;
	}

	TestTrue(TEXT("Fly-through profile reaches target"), Generator.IsComplete());
	TestTrue(TEXT("Configured terminal speed is preserved"),
		FMath::IsNearlyEqual(Setpoint.VelocityCmPerSec.Size(), 200.0f, 1.0f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotJerkLimitedMoveToTest,
	"AircraftAutopilot.Trajectory.MoveToPublishesJerkLimitedAcceleration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotJerkLimitedMoveToTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan = MakePolylinePlan(
		FVector::ZeroVector, FVector(6000.0f, 0.0f, 0.0f));
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 800.0f;
	Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 400.0f;
	Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 500.0f;
	Plan.MotionConstraints.MaxJerkCmPerSecCubed = 1000.0f;
	Plan.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Jerk-limited MoveTo plan builds"), Generator.SetPlan(Plan));
	constexpr float DeltaSeconds = 0.01f;
	float PreviousAcceleration = 0.0f;
	float MaxAccelerationChange = 0.0f;
	bool bSawAccelerationFeedForward = false;
	FVector SimulatedPosition = FVector::ZeroVector;
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 4000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(DeltaSeconds, SimulatedPosition, Setpoint);
		const float Acceleration = Setpoint.AccelerationCmPerSecSq.X;
		MaxAccelerationChange = FMath::Max(
			MaxAccelerationChange, FMath::Abs(Acceleration - PreviousAcceleration));
		bSawAccelerationFeedForward |= FMath::Abs(Acceleration) > 1.0f;
		PreviousAcceleration = Acceleration;
		SimulatedPosition = Setpoint.PositionCm;
	}

	TestTrue(TEXT("MoveTo publishes non-zero acceleration feed-forward"),
		bSawAccelerationFeedForward);
	TestTrue(TEXT("MoveTo acceleration changes respect the jerk limit"),
		MaxAccelerationChange <= Plan.MotionConstraints.MaxJerkCmPerSecCubed * DeltaSeconds + 0.1f);
	TestTrue(TEXT("Jerk-limited MoveTo reaches the target"), Generator.IsComplete());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotMoveToVehicleProgressTest,
	"AircraftAutopilot.Trajectory.MoveToReferenceFollowsVehicleProgress",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotMoveToVehicleProgressTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FAircraftTrajectoryPlan Plan = MakePolylinePlan(
		FVector::ZeroVector, FVector(5000.0f, 0.0f, 0.0f));
	Plan.MotionConstraints.CruiseSpeedCmPerSec = 800.0f;
	Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 400.0f;
	Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 400.0f;
	Plan.MotionConstraints.MaxJerkCmPerSecCubed = 2000.0f;
	Plan.AcceptanceRadiusCm = 1.0f;
	TestTrue(TEXT("Vehicle-synchronized MoveTo plan builds"), Generator.SetPlan(Plan));

	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 200; ++Step)
	{
		Generator.UpdateSetpoint(0.01f, FVector::ZeroVector, Setpoint);
	}
	TestTrue(TEXT("A stalled vehicle cannot leave an unbounded position reference ahead"),
		Setpoint.PositionCm.X <= Plan.MotionConstraints.CruiseSpeedCmPerSec * 0.01f + 0.1f);
	TestFalse(TEXT("A stalled vehicle does not complete its reference trajectory"),
		Generator.IsComplete());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotConstrainedTrajectoryOwnershipTest,
	"AircraftAutopilot.MotionProfile.MoveToIsNotProfiledTwice",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotConstrainedTrajectoryOwnershipTest::RunTest(const FString& Parameters)
{
	FAircraftMotionProfile Profile;
	Profile.Initialize(
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector,
		0.0f,
		0.0f);

	FTrajectoryPoint Constrained;
	Constrained.PositionCm = FVector(123.0f, 45.0f, 67.0f);
	Constrained.VelocityCmPerSec = FVector(500.0f, 25.0f, -10.0f);
	Constrained.AccelerationCmPerSecSq = FVector(-200.0f, 10.0f, 5.0f);
	Constrained.YawDegrees = 30.0f;
	Constrained.bValid = true;

	const FProfiledSetpoint Result = Profile.FollowConstrainedTrajectory(Constrained, 0.02f);
	TestTrue(TEXT("Constrained position is accepted without reintegration"),
		Result.PositionCm.Equals(Constrained.PositionCm));
	TestTrue(TEXT("Constrained velocity is accepted without a second brake"),
		Result.VelocityCmPerSec.Equals(Constrained.VelocityCmPerSec));
	TestTrue(TEXT("Constrained acceleration remains the feed-forward acceleration"),
		Result.AccelerationCmPerSecSq.Equals(Constrained.AccelerationCmPerSecSq));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotYawConstraintTest,
	"AircraftAutopilot.MotionProfile.YawUsesFullMotionConstraints",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotYawConstraintTest::RunTest(const FString& Parameters)
{
	FAircraftMotionProfile Profile;
	FMotionProfileLimits Limits = Profile.GetLimits();
	Limits.MaxYawRateDegPerSec = 30.0f;
	Limits.MaxYawAccelDegPerSecSq = 40.0f;
	Limits.MaxYawJerkDegPerSecCubed = 200.0f;
	Profile.SetLimits(Limits);
	Profile.Initialize(
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector,
		0.0f,
		0.0f);

	FTrajectoryPoint Constrained;
	Constrained.YawDegrees = 90.0f;
	Constrained.bValid = true;
	constexpr float DeltaSeconds = 0.01f;
	float PreviousYawRate = 0.0f;
	float PreviousYawAcceleration = 0.0f;
	float MaxYawRate = 0.0f;
	float MaxYawAcceleration = 0.0f;
	float MaxYawJerk = 0.0f;
	FProfiledSetpoint Result;
	for (int32 Step = 0; Step < 1000; ++Step)
	{
		Result = Profile.FollowConstrainedTrajectory(Constrained, DeltaSeconds);
		const float YawAcceleration =
			(Result.YawRateDegreesPerSec - PreviousYawRate) / DeltaSeconds;
		const float YawJerk =
			(YawAcceleration - PreviousYawAcceleration) / DeltaSeconds;
		MaxYawRate = FMath::Max(MaxYawRate, FMath::Abs(Result.YawRateDegreesPerSec));
		MaxYawAcceleration = FMath::Max(MaxYawAcceleration, FMath::Abs(YawAcceleration));
		MaxYawJerk = FMath::Max(MaxYawJerk, FMath::Abs(YawJerk));
		PreviousYawRate = Result.YawRateDegreesPerSec;
		PreviousYawAcceleration = YawAcceleration;
	}

	TestTrue(TEXT("Yaw reaches the constrained target"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(Result.YawDegrees, Constrained.YawDegrees)) <= 1.0f);
	TestTrue(TEXT("Yaw rate respects its limit"), MaxYawRate <= 30.1f);
	TestTrue(TEXT("Yaw acceleration respects its limit"), MaxYawAcceleration <= 40.1f);
	TestTrue(TEXT("Yaw jerk respects its limit"), MaxYawJerk <= 200.1f);
	return true;
}

#endif
