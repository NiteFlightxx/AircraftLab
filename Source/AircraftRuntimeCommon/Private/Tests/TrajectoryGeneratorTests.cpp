
#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "AircraftRuntimeCommon/Autopilot/MotionProfile.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotOrbitWrapTest,
	"AircraftAutopilot.Trajectory.OrbitContinuesAfterOneLap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotOrbitWrapTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Orbit;
	Request.OrbitCenterCm = FVector::ZeroVector;
	Request.OrbitRadiusCm = 100.0f;
	Request.StartPositionCm = FVector(100.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 100.0f;
	Request.OrbitAngularRateDegPerSec = 45.0f;

	TestTrue(TEXT("Orbit request builds"), Generator.SetRequest(Request));

	const float ArcAfterOneAndQuarterLaps = 2.5f * PI * Request.OrbitRadiusCm;
	const float DeltaSeconds = ArcAfterOneAndQuarterLaps / Request.CruiseSpeedCmPerSec;
	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Orbit produces a setpoint"), Generator.UpdateSetpoint(
		DeltaSeconds, Request.StartPositionCm, FVector::ZeroVector, Setpoint));

	TestTrue(TEXT("Setpoint remains valid after one lap"), Setpoint.bValid);
	TestTrue(TEXT("Arc length is not clamped to one lap"),
		Generator.GetCurrentArcLength() > Generator.GetTotalArcLength());
	TestTrue(TEXT("Position wraps continuously to the quarter-lap point"),
		Setpoint.PositionCm.Equals(FVector(0.0f, 100.0f, 0.0f), 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotFiniteTrajectoryCompletionTest,
	"AircraftAutopilot.Trajectory.FiniteTrajectoryCompletesOnAdvanceFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotFiniteTrajectoryCompletionTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Line;
	Request.StartPositionCm = FVector::ZeroVector;
	Request.TargetPositionCm = FVector(100.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 1000.0f;
	Request.PlanningAccelerationCmPerSecSq = 10000.0f;
	Request.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Line request builds"), Generator.SetRequest(Request));
	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Line produces a setpoint"), Generator.UpdateSetpoint(
		10.0f, FVector::ZeroVector, FVector::ZeroVector, Setpoint));
	TestTrue(TEXT("Line produces the terminal braking setpoint"), Generator.UpdateSetpoint(
		10.0f, FVector::ZeroVector, FVector::ZeroVector, Setpoint));
	TestTrue(TEXT("Trajectory completes in the frame that reaches its end"), Generator.IsComplete());
	TestTrue(TEXT("Completion frame outputs the endpoint"),
		Setpoint.PositionCm.Equals(Request.TargetPositionCm, 0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotAsymmetricCruiseProfileTest,
	"AircraftAutopilot.Trajectory.AsymmetricAccelerationCruiseAndBraking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotAsymmetricCruiseProfileTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Line;
	Request.StartPositionCm = FVector::ZeroVector;
	Request.TargetPositionCm = FVector(3000.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 800.0f;
	Request.PlanningAccelerationCmPerSecSq = 400.0f;
	Request.PlanningDecelerationCmPerSecSq = 800.0f;
	Request.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Long line request builds"), Generator.SetRequest(Request));

	constexpr float Dt = 0.01f;
	float MaxSpeed = 0.0f;
	float MaxObservedAcceleration = 0.0f;
	float MaxObservedDeceleration = 0.0f;
	float PreviousSpeed = 0.0f;
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 2000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(Dt, FVector::ZeroVector, FVector::ZeroVector, Setpoint);
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
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Line;
	Request.StartPositionCm = FVector::ZeroVector;
	Request.TargetPositionCm = FVector(600.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 800.0f;
	Request.PlanningAccelerationCmPerSecSq = 300.0f;
	Request.PlanningDecelerationCmPerSecSq = 600.0f;
	Request.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Short line request builds"), Generator.SetRequest(Request));
	constexpr float Dt = 0.005f;
	float MaxSpeed = 0.0f;
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 2000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(Dt, FVector::ZeroVector, FVector::ZeroVector, Setpoint);
		MaxSpeed = FMath::Max(MaxSpeed, Setpoint.VelocityCmPerSec.Size());
	}

	const float ExpectedPeak = FMath::Sqrt(240000.0f);
	TestTrue(TEXT("Short path does not claim unreachable cruise speed"), MaxSpeed < Request.CruiseSpeedCmPerSec);
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
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Line;
	Request.StartPositionCm = FVector::ZeroVector;
	Request.TargetPositionCm = FVector(2000.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 600.0f;
	Request.PlanningAccelerationCmPerSecSq = 300.0f;
	Request.PlanningDecelerationCmPerSecSq = 500.0f;
	Request.TargetVelocityCmPerSec = FVector(200.0f, 0.0f, 0.0f);
	Request.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Fly-through line request builds"), Generator.SetRequest(Request));
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 3000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(0.005f, FVector::ZeroVector, FVector::ZeroVector, Setpoint);
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
	FTrajectoryRequest Request;
	Request.Type = ETrajectoryType::Waypoint;
	Request.TargetPositionCm = FVector(6000.0f, 0.0f, 0.0f);
	Request.CruiseSpeedCmPerSec = 800.0f;
	Request.PlanningAccelerationCmPerSecSq = 400.0f;
	Request.PlanningDecelerationCmPerSecSq = 500.0f;
	Request.PlanningJerkCmPerSecCubed = 1000.0f;
	Request.AcceptanceRadiusCm = 1.0f;

	TestTrue(TEXT("Jerk-limited MoveTo request builds"), Generator.SetRequest(Request));
	constexpr float DeltaSeconds = 0.01f;
	float PreviousAcceleration = 0.0f;
	float MaxAccelerationChange = 0.0f;
	bool bSawAccelerationFeedForward = false;
	FTrajectoryPoint Setpoint;
	for (int32 Step = 0; Step < 4000 && !Generator.IsComplete(); ++Step)
	{
		Generator.UpdateSetpoint(
			DeltaSeconds, FVector::ZeroVector, FVector::ZeroVector, Setpoint);
		const float Acceleration = Setpoint.AccelerationCmPerSecSq.X;
		MaxAccelerationChange = FMath::Max(
			MaxAccelerationChange, FMath::Abs(Acceleration - PreviousAcceleration));
		bSawAccelerationFeedForward |= FMath::Abs(Acceleration) > 1.0f;
		PreviousAcceleration = Acceleration;
	}

	TestTrue(TEXT("MoveTo publishes non-zero acceleration feed-forward"),
		bSawAccelerationFeedForward);
	TestTrue(TEXT("MoveTo acceleration changes respect the jerk limit"),
		MaxAccelerationChange <= Request.PlanningJerkCmPerSecCubed * DeltaSeconds + 0.1f);
	TestTrue(TEXT("Jerk-limited MoveTo reaches the target"), Generator.IsComplete());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotPlannedTrajectoryOwnershipTest,
	"AircraftAutopilot.MotionProfile.MoveToIsNotProfiledTwice",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotPlannedTrajectoryOwnershipTest::RunTest(const FString& Parameters)
{
	FAircraftMotionProfile Profile;
	Profile.Initialize(
		FVector::ZeroVector,
		FVector::ZeroVector,
		FVector::ZeroVector,
		0.0f,
		0.0f);

	FTrajectoryPoint Planned;
	Planned.PositionCm = FVector(123.0f, 45.0f, 67.0f);
	Planned.VelocityCmPerSec = FVector(500.0f, 25.0f, -10.0f);
	Planned.AccelerationCmPerSecSq = FVector(-200.0f, 10.0f, 5.0f);
	Planned.YawDegrees = 30.0f;
	Planned.bValid = true;

	const FProfiledSetpoint Result = Profile.FollowPlannedTrajectory(Planned, 0.02f);
	TestTrue(TEXT("Planned position is accepted without reintegration"),
		Result.PositionCm.Equals(Planned.PositionCm));
	TestTrue(TEXT("Planned velocity is accepted without a second brake"),
		Result.VelocityCmPerSec.Equals(Planned.VelocityCmPerSec));
	TestTrue(TEXT("Planned acceleration remains the feed-forward acceleration"),
		Result.AccelerationCmPerSecSq.Equals(Planned.AccelerationCmPerSecSq));
	return true;
}

#endif
