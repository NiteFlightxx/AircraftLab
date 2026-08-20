
#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementExecutor.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectorySegments.h"

namespace
{
	FAircraftTrajectoryPlan MakeMinimumSnapPlan()
	{
		FAircraftTrajectoryPlan Plan;
		Plan.Path.Geometry = EAircraftPathGeometry::MinimumSnap;
		Plan.Path.PointsCm = {
			FVector::ZeroVector,
			FVector(1000.0f, 700.0f, 300.0f),
			FVector(2200.0f, -400.0f, 800.0f),
			FVector(3500.0f, 200.0f, 1000.0f)
		};
		Plan.MotionConstraints.CruiseSpeedCmPerSec = 800.0f;
		Plan.MotionConstraints.MaxAccelerationCmPerSecSq = 600.0f;
		Plan.MotionConstraints.MaxDecelerationCmPerSecSq = 600.0f;
		Plan.MotionConstraints.MaxJerkCmPerSecCubed = 2000.0f;
		Plan.AcceptanceRadiusCm = 10.0f;
		return Plan;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMinimumSnapWaypointAndContinuityTest,
	"AircraftAutopilot.Trajectory.MinimumSnap.WaypointsAndContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMinimumSnapWaypointAndContinuityTest::RunTest(const FString& Parameters)
{
	FAircraftMinimumSnapPathGeometry Geometry;
	const FAircraftTrajectoryPlan Plan = MakeMinimumSnapPlan();
	FString Error;
	const bool bBuilt = Geometry.BuildPath(Plan, Error);
	TestTrue(FString::Printf(TEXT("Minimum-snap solve succeeds: %s"), *Error), bBuilt);
	if (!bBuilt) return false;
	if (Geometry.GetWaypointCount() != Plan.Path.PointsCm.Num()) return false;

	for (int32 Index = 0; Index < Plan.Path.PointsCm.Num(); ++Index)
	{
		const FVector Position = Geometry.EvaluateDerivativeAtTime(
			Geometry.GetWaypointTimeSeconds(Index), 0);
		TestTrue(FString::Printf(TEXT("Trajectory passes waypoint %d"), Index),
			Position.Equals(Plan.Path.PointsCm[Index], 0.5f));
	}

	for (int32 Index = 1; Index < Plan.Path.PointsCm.Num() - 1; ++Index)
	{
		const float Time = Geometry.GetWaypointTimeSeconds(Index);
		constexpr float Epsilon = 1.0e-4f;
		TestTrue(TEXT("Velocity is continuous at internal waypoint"),
			Geometry.EvaluateDerivativeAtTime(Time - Epsilon, 1).Equals(
				Geometry.EvaluateDerivativeAtTime(Time + Epsilon, 1), 1.0f));
		TestTrue(TEXT("Acceleration is continuous at internal waypoint"),
			Geometry.EvaluateDerivativeAtTime(Time - Epsilon, 2).Equals(
				Geometry.EvaluateDerivativeAtTime(Time + Epsilon, 2), 5.0f));
		TestTrue(TEXT("Jerk is continuous at internal waypoint"),
			Geometry.EvaluateDerivativeAtTime(Time - Epsilon, 3).Equals(
				Geometry.EvaluateDerivativeAtTime(Time + Epsilon, 3), 50.0f));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMinimumSnapMotionLimitsTest,
	"AircraftAutopilot.Trajectory.MinimumSnap.RespectsMotionLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMinimumSnapMotionLimitsTest::RunTest(const FString& Parameters)
{
	FAircraftMinimumSnapPathGeometry Geometry;
	const FAircraftTrajectoryPlan Plan = MakeMinimumSnapPlan();
	FString Error;
	if (!Geometry.BuildPath(Plan, Error))
	{
		AddError(Error);
		return false;
	}
	const float Duration = Geometry.GetTotalDurationSeconds();
	for (int32 Sample = 0; Sample <= 500; ++Sample)
	{
		const float Time = Duration * static_cast<float>(Sample) / 500.0f;
		TestTrue(TEXT("Velocity remains within configured limit"),
			Geometry.EvaluateDerivativeAtTime(Time, 1).Size() <= Plan.MotionConstraints.CruiseSpeedCmPerSec * 1.03f);
		TestTrue(TEXT("Acceleration remains within configured limit"),
			Geometry.EvaluateDerivativeAtTime(Time, 2).Size() <= Plan.MotionConstraints.MaxAccelerationCmPerSecSq * 1.05f);
		TestTrue(TEXT("Jerk remains within configured limit"),
			Geometry.EvaluateDerivativeAtTime(Time, 3).Size() <= Plan.MotionConstraints.MaxJerkCmPerSecCubed * 1.05f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMinimumSnapGeneratorIntegrationTest,
	"AircraftAutopilot.Trajectory.MinimumSnap.NativeTimeGeneratorIntegration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMinimumSnapGeneratorIntegrationTest::RunTest(const FString& Parameters)
{
	FAircraftTrajectoryGenerator Generator;
	const FAircraftTrajectoryPlan Plan = MakeMinimumSnapPlan();
	TestTrue(TEXT("TrajectoryGenerator accepts MinimumSnap"), Generator.SetPlan(Plan));
	if (!Generator.IsValid()) return false;

	FTrajectoryPoint Setpoint;
	float Elapsed = 0.0f;
	while (!Generator.IsComplete() && Elapsed < 60.0f)
	{
		TestTrue(TEXT("Native-time trajectory produces valid samples"),
			Generator.UpdateSetpoint(0.02f, Setpoint.PositionCm, Setpoint));
		Elapsed += 0.02f;
	}
	TestTrue(TEXT("MinimumSnap completes on its native clock"), Generator.IsComplete());
	TestTrue(TEXT("MinimumSnap generator ends at the requested endpoint"),
		Setpoint.PositionCm.Equals(Plan.Path.PointsCm.Last(), 0.5f));
	TestTrue(TEXT("Progress reaches one"), FMath::IsNearlyEqual(Generator.GetProgress(), 1.0f, 0.001f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMinimumSnapMovementIntentIntegrationTest,
	"AircraftAutopilot.Movement.MinimumSnapPathIntent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMinimumSnapMovementIntentIntegrationTest::RunTest(const FString& Parameters)
{
	FAircraftAutopilotMovementExecutor Executor;
	FAircraftAutopilotVehicleSnapshot Snapshot;
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathTrajectoryMode = EAutopilotPathTrajectoryMode::MinimumSnap;
	Intent.PathPointsCm = MakeMinimumSnapPlan().Path.PointsCm;
	Intent.MotionConstraints.CruiseSpeedCmPerSec = 800.0f;
	Intent.MotionConstraints.MaxAccelerationCmPerSecSq = 600.0f;
	Intent.MotionConstraints.MaxDecelerationCmPerSecSq = 600.0f;
	Intent.MotionConstraints.MaxJerkCmPerSecCubed = 2000.0f;
	const FAutopilotIntentHandle Handle = Executor.Submit(
		Intent, Snapshot, EAutopilotIntentFailureReason::None);
	TestEqual(TEXT("Minimum-snap path intent is accepted"),
		Executor.GetResult(Handle).Status, EAutopilotIntentStatus::Accepted);

	FTrajectoryPoint Setpoint;
	TestTrue(TEXT("Movement executor builds and samples the minimum-snap trajectory"),
		Executor.BuildSetpoint(Snapshot, 0.02f, FProfiledSetpoint(), Setpoint));
	TestTrue(TEXT("Minimum-snap movement setpoint is valid"), Setpoint.bValid);
	return true;
}

#endif
