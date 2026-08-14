
#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "AircraftRuntimeCommon/Autopilot/AutopilotMovementExecutor.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectoryGenerator.h"
#include "AircraftRuntimeCommon/Autopilot/TrajectorySegments.h"

namespace
{
	FTrajectoryRequest MakeMinimumSnapRequest()
	{
		FTrajectoryRequest Request;
		Request.Type = ETrajectoryType::MinimumSnap;
		Request.StartPositionCm = FVector::ZeroVector;
		Request.PathPointsCm = {
			FVector::ZeroVector,
			FVector(1000.0f, 700.0f, 300.0f),
			FVector(2200.0f, -400.0f, 800.0f),
			FVector(3500.0f, 200.0f, 1000.0f)
		};
		Request.TargetPositionCm = Request.PathPointsCm.Last();
		Request.CruiseSpeedCmPerSec = 800.0f;
		Request.PlanningAccelerationCmPerSecSq = 600.0f;
		Request.PlanningDecelerationCmPerSecSq = 600.0f;
		Request.PlanningJerkCmPerSecCubed = 2000.0f;
		Request.AcceptanceRadiusCm = 10.0f;
		return Request;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMinimumSnapWaypointAndContinuityTest,
	"AircraftAutopilot.Trajectory.MinimumSnap.WaypointsAndContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMinimumSnapWaypointAndContinuityTest::RunTest(const FString& Parameters)
{
	FAircraftMinSnapTrajectorySegment Segment;
	const FTrajectoryRequest Request = MakeMinimumSnapRequest();
	FString Error;
	const bool bBuilt = Segment.BuildSegment(Request, Error);
	TestTrue(FString::Printf(TEXT("Minimum-snap solve succeeds: %s"), *Error), bBuilt);
	if (!bBuilt) return false;
	if (Segment.GetWaypointCount() != Request.PathPointsCm.Num()) return false;

	for (int32 Index = 0; Index < Request.PathPointsCm.Num(); ++Index)
	{
		const FVector Position = Segment.EvaluateDerivativeAtTime(
			Segment.GetWaypointTimeSeconds(Index), 0);
		TestTrue(FString::Printf(TEXT("Trajectory passes waypoint %d"), Index),
			Position.Equals(Request.PathPointsCm[Index], 0.5f));
	}

	for (int32 Index = 1; Index < Request.PathPointsCm.Num() - 1; ++Index)
	{
		const float Time = Segment.GetWaypointTimeSeconds(Index);
		constexpr float Epsilon = 1.0e-4f;
		TestTrue(TEXT("Velocity is continuous at internal waypoint"),
			Segment.EvaluateDerivativeAtTime(Time - Epsilon, 1).Equals(
				Segment.EvaluateDerivativeAtTime(Time + Epsilon, 1), 1.0f));
		TestTrue(TEXT("Acceleration is continuous at internal waypoint"),
			Segment.EvaluateDerivativeAtTime(Time - Epsilon, 2).Equals(
				Segment.EvaluateDerivativeAtTime(Time + Epsilon, 2), 5.0f));
		TestTrue(TEXT("Jerk is continuous at internal waypoint"),
			Segment.EvaluateDerivativeAtTime(Time - Epsilon, 3).Equals(
				Segment.EvaluateDerivativeAtTime(Time + Epsilon, 3), 50.0f));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMinimumSnapMotionLimitsTest,
	"AircraftAutopilot.Trajectory.MinimumSnap.RespectsMotionLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMinimumSnapMotionLimitsTest::RunTest(const FString& Parameters)
{
	FAircraftMinSnapTrajectorySegment Segment;
	const FTrajectoryRequest Request = MakeMinimumSnapRequest();
	FString Error;
	if (!Segment.BuildSegment(Request, Error))
	{
		AddError(Error);
		return false;
	}
	const float Duration = Segment.GetTotalDurationSeconds();
	for (int32 Sample = 0; Sample <= 500; ++Sample)
	{
		const float Time = Duration * static_cast<float>(Sample) / 500.0f;
		TestTrue(TEXT("Velocity remains within configured limit"),
			Segment.EvaluateDerivativeAtTime(Time, 1).Size() <= Request.CruiseSpeedCmPerSec * 1.03f);
		TestTrue(TEXT("Acceleration remains within configured limit"),
			Segment.EvaluateDerivativeAtTime(Time, 2).Size() <= Request.PlanningAccelerationCmPerSecSq * 1.05f);
		TestTrue(TEXT("Jerk remains within configured limit"),
			Segment.EvaluateDerivativeAtTime(Time, 3).Size() <= Request.PlanningJerkCmPerSecCubed * 1.05f);
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
	const FTrajectoryRequest Request = MakeMinimumSnapRequest();
	TestTrue(TEXT("TrajectoryGenerator accepts MinimumSnap"), Generator.SetRequest(Request));
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
		Setpoint.PositionCm.Equals(Request.TargetPositionCm, 0.5f));
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
	Intent.PathPointsCm = MakeMinimumSnapRequest().PathPointsCm;
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
