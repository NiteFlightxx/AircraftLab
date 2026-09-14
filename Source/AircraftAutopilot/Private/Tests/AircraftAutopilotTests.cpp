#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "AircraftAutopilot/AircraftMpccController.h"
#include "AircraftAutopilot/AircraftTrajectoryRuntime.h"
#include "AircraftAutopilot/AircraftSpatialPath.h"
#include "AircraftRuntimeInterface/AircraftNavigationGuidance.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FAircraftDynamicCapabilitySnapshot MakeCapability()
	{
		FAircraftDynamicCapabilitySnapshot Result;
		Result.MassKg = 100.0f;
		Result.GravityCmPerSecSq = 980.0f;
		Result.MaxHorizontalSpeedCmPerSec = 2000.0f;
		Result.MaxHorizontalAccelerationCmPerSecSq = 1000.0f;
		Result.MaxHorizontalDecelerationCmPerSecSq = 1000.0f;
		Result.MaxHorizontalJerkCmPerSecCubed = 4000.0f;
		Result.MaxVerticalAccelerationCmPerSecSq = 800.0f;
		Result.MaxVerticalJerkCmPerSecCubed = 3000.0f;
		Result.MaxClimbRateCmPerSec = 600.0f;
		Result.MaxDescentRateCmPerSec = 500.0f;
		Result.MaxTiltRadians = FMath::DegreesToRadians(25.0f);
		Result.bHasTiltLimit = true;
		Result.CollectiveAuthorityN = 2500.0f;
		Result.MaxBodyRateRadPerSec = FVector(
			FMath::DegreesToRadians(180.0f),
			FMath::DegreesToRadians(180.0f),
			FMath::DegreesToRadians(90.0f));
		Result.MaxBodyAngularAccelerationRadPerSecSq = FVector(FMath::DegreesToRadians(180.0f));
		Result.MaxBodyAngularJerkRadPerSecCubed = FVector(FMath::DegreesToRadians(600.0f));
		Result.bCanControlRoll = true;
		Result.bCanControlPitch = true;
		Result.bCanControlYaw = true;
		Result.PositiveTorqueAuthorityNm = FVector(1000.0f);
		Result.NegativeTorqueAuthorityNm = FVector(1000.0f);
		Result.bValid = true;
		return Result;
	}

	FAircraftMovementIntent MakeRouteIntent(float DistanceCm)
	{
		FAircraftMovementIntent Intent;
		Intent.Type = EAircraftMovementIntentType::Route;
		Intent.Route.PointsCm = { FVector::ZeroVector, FVector(DistanceCm, 0.0, 0.0) };
		Intent.Limits.CruiseSpeedCmPerSec = 800.0f;
		Intent.Limits.MaxAccelerationCmPerSecSq = 400.0f;
		Intent.Limits.MaxDecelerationCmPerSecSq = 400.0f;
		Intent.bHasRequestedMotionLimits = true;
		Intent.Completion.ArrivalMode = EAircraftArrivalMode::Stop;
		return Intent;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotTypedIntentApiTest,
	"AircraftAutopilot.API.TypedIntentMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotTypedIntentApiTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAutopilotComponent* Autopilot = NewObject<UAutopilotComponent>();
	TestNotNull(TEXT("Autopilot component is created"), Autopilot);
	Autopilot->SetAutopilotActive(true);

	FAircraftMovementIntentSettings Settings;
	Settings.bOverrideMotionLimits = true;
	Settings.Limits.CruiseSpeedCmPerSec = 725.0f;
	Settings.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	Settings.Heading.FixedYawDegrees = 37.0f;
	Settings.TimeoutSeconds = 12.0f;
	FAircraftCompletionPolicy Completion;
	Completion.HorizontalToleranceCm = 25.0f;

	FAircraftHoldIntent Hold;
	Hold.bCaptureCurrentPosition = false;
	Hold.PositionCm = FVector(100.0, 200.0, 300.0);
	const FAircraftMovementIntentHandle Handle = Autopilot->SubmitHoldIntent(
		Hold, Settings, Completion);
	TestTrue(TEXT("Typed hold submission returns a handle"), Handle.IsValid());

	FAircraftMovementIntent Internal;
	FAircraftMovementIntentHandle InternalHandle;
	uint64 Revision = 0;
	TestTrue(TEXT("Submitted hold is available to the flight controller"),
		Autopilot->GetAircraftMovementIntent(Internal, InternalHandle, Revision));
	TestEqual(TEXT("Hold type is mapped"), Internal.Type, EAircraftMovementIntentType::Hold);
	TestTrue(TEXT("Hold payload is mapped"), Internal.Hold.PositionCm.Equals(Hold.PositionCm));
	TestEqual(TEXT("Shared speed setting is mapped"),
		Internal.Limits.CruiseSpeedCmPerSec, Settings.Limits.CruiseSpeedCmPerSec);
	TestEqual(TEXT("Completion policy is mapped"),
		Internal.Completion.HorizontalToleranceCm, Completion.HorizontalToleranceCm);

	FAircraftVelocityIntent Velocity;
	Velocity.VelocityCmPerSec = FVector(500.0, 0.0, 0.0);
	TestTrue(TEXT("Typed velocity update succeeds"),
		Autopilot->UpdateVelocityIntent(Handle, Velocity, Settings));
	Autopilot->GetAircraftMovementIntent(Internal, InternalHandle, Revision);
	TestEqual(TEXT("Velocity type is mapped"), Internal.Type, EAircraftMovementIntentType::Velocity);
	TestTrue(TEXT("Velocity payload is mapped"),
		Internal.Velocity.VelocityCmPerSec.Equals(Velocity.VelocityCmPerSec));

	FAircraftRouteIntent Route;
	Route.PointsCm = { FVector::ZeroVector, FVector(1000.0, 0.0, 0.0) };
	TestTrue(TEXT("Typed route update succeeds"),
		Autopilot->UpdateRouteIntent(Handle, Route, Settings, Completion));
	Autopilot->GetAircraftMovementIntent(Internal, InternalHandle, Revision);
	TestEqual(TEXT("Route type is mapped"), Internal.Type, EAircraftMovementIntentType::Route);
	TestEqual(TEXT("Route payload is mapped"), Internal.Route.PointsCm.Num(), 2);

	FAircraftOrbitIntent Orbit;
	Orbit.CenterCm = FVector(50.0, 75.0, 100.0);
	TestTrue(TEXT("Typed orbit update succeeds"),
		Autopilot->UpdateOrbitIntent(Handle, Orbit, Settings));
	Autopilot->GetAircraftMovementIntent(Internal, InternalHandle, Revision);
	TestEqual(TEXT("Orbit type is mapped"), Internal.Type, EAircraftMovementIntentType::Orbit);
	TestTrue(TEXT("Orbit payload is mapped"), Internal.Orbit.CenterCm.Equals(Orbit.CenterCm));

	FAircraftTimedTrajectoryIntent TimedTrajectory;
	TimedTrajectory.Samples.AddDefaulted();
	FAircraftTimedTrajectorySample& LastSample = TimedTrajectory.Samples.AddDefaulted_GetRef();
	LastSample.TimeSeconds = 1.0f;
	LastSample.PositionCm = FVector(100.0, 0.0, 0.0);
	TestTrue(TEXT("Typed timed trajectory update succeeds"),
		Autopilot->UpdateTimedTrajectoryIntent(
			Handle, TimedTrajectory, Settings, Completion));
	Autopilot->GetAircraftMovementIntent(Internal, InternalHandle, Revision);
	TestEqual(TEXT("Timed trajectory type is mapped"),
		Internal.Type, EAircraftMovementIntentType::TimedTrajectory);
	TestEqual(TEXT("Timed trajectory payload is mapped"),
		Internal.TimedTrajectory.Samples.Num(), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftTrajectoryPauseClockRebaseTest,
	"AircraftAutopilot.Trajectory.PauseClockRebase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftTrajectoryPauseClockRebaseTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftTrajectoryRuntime Runtime;
	FAircraftMovementIntent Intent = MakeRouteIntent(2000.0f);
	FAircraftAutopilotRuntimeConfig Config;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	TestTrue(TEXT("Route intent is accepted"),
		Runtime.SetIntent(Intent, 1, 1, Config, State, Capability));

	State.TimeSeconds = 1.05;
	FAircraftTrajectoryReference BeforePause;
	TestTrue(TEXT("Reference advances before pause"),
		Runtime.UpdateKinematic(State, Capability, BeforePause));

	State.TimeSeconds = 101.05;
	Runtime.RebaseTime(State.TimeSeconds);
	FAircraftTrajectoryReference AfterResume;
	TestTrue(TEXT("Reference remains valid after resume"),
		Runtime.UpdateKinematic(State, Capability, AfterResume));
	TestTrue(TEXT("Pause duration does not advance the trajectory cursor"),
		AfterResume.PositionCm.Equals(BeforePause.PositionCm, 0.01));
	TestTrue(TEXT("Pause duration does not change the planned velocity"),
		AfterResume.VelocityCmPerSec.Equals(BeforePause.VelocityCmPerSec, 0.01));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMpccPauseClockRebaseTest,
	"AircraftAutopilot.MPCC.PauseClockRebase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMpccPauseClockRebaseTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent = MakeRouteIntent(2000.0f);
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.Sequence = 1;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Flight-controller route intent is accepted"),
		Controller.SetIntent(Intent, 2, 1, Config, State, Capability));

	State.TimeSeconds = 1.001;
	State.Sequence = 2;
	FAircraftTrajectoryReference BeforePause;
	TestTrue(TEXT("MPCC reference is solved before pause"),
		Controller.Update(State, Capability, BeforePause));

	State.TimeSeconds = 101.001;
	State.Sequence = 3;
	Controller.RebaseTime(State.TimeSeconds);
	FAircraftTrajectoryReference AfterResume;
	TestTrue(TEXT("MPCC reference remains valid after resume"),
		Controller.Update(State, Capability, AfterResume));
	TestEqual(TEXT("Resume reuses the scheduled reference instead of solving early"),
		AfterResume.StateSequence, BeforePause.StateSequence);
	TestTrue(TEXT("Resume preserves the MPCC position reference"),
		AfterResume.PositionCm.Equals(BeforePause.PositionCm, 0.01));
	TestTrue(TEXT("Resume preserves the MPCC velocity reference"),
		AfterResume.VelocityCmPerSec.Equals(BeforePause.VelocityCmPerSec, 0.01));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSpatialPathContinuityTest,
	"AircraftAutopilot.Path.C2Continuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSpatialPathContinuityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftRouteIntent Route;
	Route.PointsCm = {
		FVector(0.0, 0.0, 0.0), FVector(1000.0, 200.0, 100.0),
		FVector(2000.0, -300.0, 250.0), FVector(3000.0, 0.0, 0.0)
	};
	FAircraftPathOptimizationRuntimeConfig Config;
	Config.CenterlineWeight = 1.0f;
	Config.CurvatureWeight = 0.0f;
	Config.SnapWeight = 0.0f;
	Config.MaxIterations = 1;
	FAircraftSpatialPath Path;
	TestTrue(TEXT("C2 path builds"), Path.Build(Route, Config));

	FAircraftSpatialPathState Knot;
	TestTrue(TEXT("Middle knot projects"), Path.Project(Route.PointsCm[1], 1000.0f, false, Knot));
	FAircraftSpatialPathState Before;
	FAircraftSpatialPathState After;
	Path.Evaluate(Knot.DistanceCm - 0.1f, Before);
	Path.Evaluate(Knot.DistanceCm + 0.1f, After);
	TestTrue(TEXT("Position remains continuous"),
		FVector::Distance(Before.PositionCm, After.PositionCm) < 1.0);
	TestTrue(TEXT("First derivative direction remains continuous"),
		FVector::Distance(Before.Tangent, After.Tangent) < 0.01);
	TestTrue(TEXT("Curvature remains continuous"),
		FVector::Distance(Before.CurvaturePerCm, After.CurvaturePerCm) < 1.0e-3);

	FAircraftRouteIntent CorridorRoute;
	CorridorRoute.PointsCm = {
		FVector(0.0, 0.0, 0.0), FVector(500.0, 40.0, 0.0), FVector(1000.0, 0.0, 0.0)
	};
	FAircraftSafeCorridorSegment& Corridor = CorridorRoute.Corridor.AddDefaulted_GetRef();
	Corridor.AxisStartCm = CorridorRoute.PointsCm[0];
	Corridor.AxisEndCm = CorridorRoute.PointsCm.Last();
	Corridor.RadiusCm = 100.0f;
	Corridor.StartDistanceCm = 0.0f;
	Corridor.EndDistanceCm = FVector::Distance(
		CorridorRoute.PointsCm[0], CorridorRoute.PointsCm[1])
		+ FVector::Distance(CorridorRoute.PointsCm[1], CorridorRoute.PointsCm[2]);
	FAircraftSpatialPath CorridorPath;
	TestTrue(TEXT("Path satisfying a capsule safe corridor builds"),
		CorridorPath.Build(CorridorRoute, Config));
	for (float Distance = 0.0f; Distance <= CorridorPath.GetLengthCm(); Distance += 10.0f)
	{
		FAircraftSpatialPathState Sample;
		CorridorPath.Evaluate(Distance, Sample);
		TestTrue(TEXT("Every sampled path point respects the corridor margin"),
			Corridor.ComputeCorrectionCm(
				Sample.PositionCm, Config.CorridorSafetyMarginCm).IsNearlyZero(
					Config.ConvergenceToleranceCm));
	}
	TestTrue(TEXT("Point outside the safety corridor is diagnosed"),
		CorridorPath.ComputeCorridorViolationCm(FVector(500.0, 150.0, 0.0), 500.0f) > 0.0f);
	TestTrue(TEXT("Corridor correction points back into the feasible region"),
		CorridorPath.ComputeCorridorCorrectionCm(
			FVector(500.0, 150.0, 0.0), 500.0f).Y < 0.0f);

	FAircraftRouteIntent NearbyBranchesRoute;
	NearbyBranchesRoute.PointsCm = {
		FVector(0.0, 0.0, 0.0), FVector(1000.0, 0.0, 0.0),
		FVector(1000.0, 100.0, 0.0), FVector(0.0, 100.0, 0.0)
	};
	FAircraftPathOptimizationRuntimeConfig ProjectionConfig = Config;
	ProjectionConfig.ProjectionBacktrackToleranceCm = 25.0f;
	ProjectionConfig.ProjectionSearchDistanceCm = 250.0f;
	FAircraftSpatialPath NearbyBranchesPath;
	TestTrue(TEXT("Nearby-branch path builds"),
		NearbyBranchesPath.Build(NearbyBranchesRoute, ProjectionConfig));
	FAircraftSpatialPathState LocalProjection;
	FAircraftSpatialPathState GlobalProjection;
	TestTrue(TEXT("Local projection succeeds"), NearbyBranchesPath.Project(
		FVector(500.0, 100.0, 0.0), 400.0f, false, LocalProjection));
	TestTrue(TEXT("Initial global projection succeeds"), NearbyBranchesPath.Project(
		FVector(500.0, 100.0, 0.0), 0.0f, true, GlobalProjection));
	TestTrue(TEXT("Continuous projection cannot jump to a nearby later branch"),
		LocalProjection.DistanceCm < 800.0f && GlobalProjection.DistanceCm > 1200.0f);

	FAircraftRouteIntent LongRoute;
	LongRoute.PointsCm = { FVector::ZeroVector, FVector(10000.0, 0.0, 0.0) };
	FAircraftSpatialPath LongPath;
	TestTrue(TEXT("Long open path builds"), LongPath.Build(LongRoute, ProjectionConfig));
	FAircraftSpatialPathState RecoveredOpenProjection;
	TestTrue(TEXT("A local projection recovers after an open-path teleport"),
		LongPath.Project(FVector(8000.0, 0.0, 0.0), 100.0f, false,
			RecoveredOpenProjection));
	TestTrue(TEXT("Open-path recovery selects the global nearest point"),
		FMath::IsNearlyEqual(RecoveredOpenProjection.DistanceCm, 8000.0f, 5.0f));

	FAircraftRouteIntent ClosedRoute;
	ClosedRoute.bClosed = true;
	ClosedRoute.PointsCm = {
		FVector(0.0, 0.0, 0.0), FVector(1000.0, 0.0, 0.0),
		FVector(1000.0, 1000.0, 0.0), FVector(0.0, 1000.0, 0.0)
	};
	FAircraftSpatialPath ClosedPath;
	TestTrue(TEXT("Closed path builds"), ClosedPath.Build(ClosedRoute, ProjectionConfig));
	FAircraftSpatialPathState RecoveredClosedProjection;
	TestTrue(TEXT("A local projection recovers after a closed-loop jump"),
		ClosedPath.Project(FVector(1000.0, 900.0, 0.0), 100.0f, false,
			RecoveredClosedProjection));
	TestTrue(TEXT("Closed-loop recovery reaches the opposite side"),
		FVector::Distance(RecoveredClosedProjection.PositionCm,
			FVector(1000.0, 900.0, 0.0)) < 20.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftTrajectoryRetimingTest,
	"AircraftAutopilot.Timing.DynamicRetiming",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftTrajectoryRetimingTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const FAircraftAutopilotRuntimeConfig Config;
	const FAircraftVehicleStateSnapshot State;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();

	FAircraftMotionPlan LongPlan;
	const FAircraftMovementIntent LongIntent = MakeRouteIntent(10000.0f);
	TestTrue(TEXT("Long route builds"), LongPlan.Build(LongIntent, Config, State, Capability));
	float LongPeakSpeed = 0.0f;
	for (const FAircraftMotionPlanSample& Sample : LongPlan.GetSamples())
	{
		LongPeakSpeed = FMath::Max(LongPeakSpeed,
			static_cast<float>(Sample.VelocityCmPerSec.Size()));
	}
	TestTrue(TEXT("Long route reaches cruise speed"), FMath::IsNearlyEqual(LongPeakSpeed, 800.0f, 1.0f));
	TestTrue(TEXT("Stop route has zero terminal speed"),
		LongPlan.GetSamples().Last().VelocityCmPerSec.IsNearlyZero(0.1f));

	FAircraftMotionPlan ShortPlan;
	const FAircraftMovementIntent ShortIntent = MakeRouteIntent(300.0f);
	TestTrue(TEXT("Two-point short route builds"), ShortPlan.Build(ShortIntent, Config, State, Capability));
	float ShortPeakSpeed = 0.0f;
	for (const FAircraftMotionPlanSample& Sample : ShortPlan.GetSamples())
	{
		ShortPeakSpeed = FMath::Max(ShortPeakSpeed,
			static_cast<float>(Sample.VelocityCmPerSec.Size()));
	}
	TestTrue(TEXT("Short route does not claim unreachable cruise speed"), ShortPeakSpeed < 800.0f);

	FAircraftMovementIntent AsymmetricIntent = MakeRouteIntent(6000.0f);
	AsymmetricIntent.Limits.MaxAccelerationCmPerSecSq = 200.0f;
	AsymmetricIntent.Limits.MaxDecelerationCmPerSecSq = 600.0f;
	AsymmetricIntent.bHasRequestedMotionLimits = true;
	FAircraftMotionPlan AsymmetricPlan;
	TestTrue(TEXT("Asymmetric route builds"),
		AsymmetricPlan.Build(AsymmetricIntent, Config, State, Capability));
	float PeakAcceleration = 0.0f;
	float PeakDeceleration = 0.0f;
	for (const FAircraftMotionPlanSample& Sample : AsymmetricPlan.GetSamples())
	{
		const float AlongTrack = static_cast<float>(Sample.AccelerationCmPerSecSq.X);
		PeakAcceleration = FMath::Max(PeakAcceleration, AlongTrack);
		PeakDeceleration = FMath::Max(PeakDeceleration, -AlongTrack);
	}
	TestTrue(TEXT("Deceleration profile is independently stronger"),
		PeakDeceleration > PeakAcceleration * 2.0f);

	FAircraftMovementIntent JerkLimitedIntent = MakeRouteIntent(6000.0f);
	JerkLimitedIntent.Limits.MaxAccelerationCmPerSecSq = 600.0f;
	JerkLimitedIntent.Limits.MaxDecelerationCmPerSecSq = 600.0f;
	JerkLimitedIntent.Limits.MaxJerkCmPerSecCubed = 100.0f;
	JerkLimitedIntent.bHasRequestedMotionLimits = true;
	FAircraftMotionPlan JerkLimitedPlan;
	TestTrue(TEXT("Jerk-limited route builds"),
		JerkLimitedPlan.Build(JerkLimitedIntent, Config, State, Capability));
	float PeakJerk = 0.0f;
	int32 PeakJerkIndex = INDEX_NONE;
	const TArray<FAircraftMotionPlanSample>& JerkSamples = JerkLimitedPlan.GetSamples();
	for (int32 Index = 1; Index < JerkSamples.Num(); ++Index)
	{
		const float Dt = JerkSamples[Index].TimeSeconds - JerkSamples[Index - 1].TimeSeconds;
		if (Dt > UE_SMALL_NUMBER)
		{
			const float Jerk = FMath::Abs(
				static_cast<float>(JerkSamples[Index].AccelerationCmPerSecSq.X
					- JerkSamples[Index - 1].AccelerationCmPerSecSq.X)) / Dt;
			if (Jerk > PeakJerk)
			{
				PeakJerk = Jerk;
				PeakJerkIndex = Index;
			}
		}
	}
	const float PeakJerkDt = PeakJerkIndex > 0
		? JerkSamples[PeakJerkIndex].TimeSeconds - JerkSamples[PeakJerkIndex - 1].TimeSeconds
		: 0.0f;
	TestTrue(*FString::Printf(TEXT("Time parameterization respects requested horizontal jerk (peak %.3f at %d, accel %.3f -> %.3f, dt %.3f)"),
		PeakJerk, PeakJerkIndex,
		PeakJerkIndex > 0 ? JerkSamples[PeakJerkIndex - 1].AccelerationCmPerSecSq.X : 0.0,
		PeakJerkIndex > 0 ? JerkSamples[PeakJerkIndex].AccelerationCmPerSecSq.X : 0.0,
		PeakJerkDt),
		PeakJerk <= JerkLimitedIntent.Limits.MaxJerkCmPerSecCubed + 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOrbitContinuityTest,
	"AircraftAutopilot.Timing.OrbitSeamContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOrbitContinuityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Orbit;
	Intent.Orbit.CenterCm = FVector::ZeroVector;
	Intent.Orbit.RadiusCm = 1000.0f;
	Intent.Limits.CruiseSpeedCmPerSec = 500.0f;
	Intent.bHasRequestedMotionLimits = true;
	FAircraftVehicleStateSnapshot State;
	State.PositionCm = FVector(1000.0, 0.0, 0.0);
	FAircraftMotionPlan Plan;
	TestTrue(TEXT("Orbit builds"), Plan.Build(Intent,
		FAircraftAutopilotRuntimeConfig(), State, MakeCapability()));
	FAircraftMotionPlanSample Before;
	FAircraftMotionPlanSample After;
	Plan.Evaluate(Plan.GetDurationSeconds() - 0.001f, Before);
	Plan.Evaluate(Plan.GetDurationSeconds() + 0.001f, After);
	TestTrue(TEXT("Orbit position is continuous across seam"),
		FVector::Distance(Before.PositionCm, After.PositionCm) < 2.0);
	TestTrue(TEXT("Orbit velocity is continuous across seam"),
		FVector::Distance(Before.VelocityCmPerSec, After.VelocityCmPerSec) < 2.0);
	TestTrue(TEXT("Orbit yaw is continuous across seam"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(Before.YawDegrees, After.YawDegrees)) < 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPredictiveReferenceTest,
	"AircraftAutopilot.MPCC.ReferenceConstraintsAndFreshness",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPredictiveReferenceTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Velocity;
	Intent.Velocity.VelocityCmPerSec = FVector(800.0, 0.0, 0.0);
	Intent.Limits.MaxAccelerationCmPerSecSq = 400.0f;
	Intent.Limits.MaxJerkCmPerSecCubed = 2000.0f;
	Intent.bHasRequestedMotionLimits = true;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.Sequence = 10;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.bHasExplicitAerodynamics = true;
	Capability.AirDensityKgPerM3 = 1.225f;
	Capability.MaxRelativeAirspeedCmPerSec = 10000.0f;
	Capability.LinearDragAircraftNsPerM = FVector(10.0, 10.0, 10.0);
	Capability.DragAreaCoefficientAircraftM2 = FVector(0.2, 0.2, 0.2);

	FAircraftMpccController Controller;
	TestTrue(TEXT("Velocity intent is accepted"),
		Controller.SetIntent(Intent, 7, 3, Config, State, Capability));
	FAircraftTrajectoryReference First;
	TestTrue(TEXT("A fresh reference is solved"), Controller.Update(State, Capability, First));
	TestTrue(TEXT("Reference carries identity"),
		First.bValid && First.IntentId == 7 && First.IntentRevision == 3);
	TestTrue(TEXT("Velocity reference ramps instead of stepping to cruise speed"),
		First.VelocityCmPerSec.X > 0.0f
		&& First.VelocityCmPerSec.X < Intent.Velocity.VelocityCmPerSec.X);
	TestTrue(TEXT("Acceleration remains inside requested horizontal authority"),
		FVector2D(First.AccelerationCmPerSecSq.X, First.AccelerationCmPerSecSq.Y).Size() <= 400.1f);
	TestTrue(TEXT("Aerodynamic feed-forward acts in the commanded direction"),
		First.DynamicsFeedForwardAccelerationCmPerSecSq.X > 0.0f);
	TestFalse(TEXT("Velocity intent does not invent a position target"),
		First.bPositionTrackingEnabled);

	State.TimeSeconds += 0.001;
	State.Sequence = 11;
	FAircraftTrajectoryReference Reused;
	TestTrue(TEXT("Fresh reference is reusable between solver updates"),
		Controller.Update(State, Capability, Reused));
	TestEqual(TEXT("Reused reference retains originating state sequence"),
		Reused.StateSequence, First.StateSequence);
	TestTrue(TEXT("Reference expires at its declared deadline"),
		!First.IsFresh(First.ValidUntilSeconds + 0.001));

	FAircraftMovementIntent BrakeIntent = Intent;
	BrakeIntent.Velocity.VelocityCmPerSec = FVector::ZeroVector;
	State.TimeSeconds = 2.0;
	State.Sequence = 12;
	State.VelocityCmPerSec = FVector(800.0, 0.0, 0.0);
	State.AccelerationCmPerSecSq = FVector::ZeroVector;
	TestTrue(TEXT("Zero velocity braking intent is accepted"),
		Controller.SetIntent(BrakeIntent, 7, 4, Config, State, Capability));
	FAircraftTrajectoryReference Braking;
	TestTrue(TEXT("A braking reference is solved"),
		Controller.Update(State, Capability, Braking));
	TestTrue(TEXT("Release command decelerates without an instantaneous stop"),
		Braking.VelocityCmPerSec.X > 0.0f
		&& Braking.VelocityCmPerSec.X < State.VelocityCmPerSec.X);
	TestTrue(TEXT("Braking feed-forward remains inside requested authority"),
		FMath::Abs(Braking.AccelerationCmPerSecSq.X)
		<= FMath::Max(Intent.Limits.MaxAccelerationCmPerSecSq,
			Intent.Limits.MaxDecelerationCmPerSecSq) + 0.1f);
	State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
	++State.Sequence;
	FAircraftTrajectoryReference ContinuedBraking;
	TestTrue(TEXT("The next braking reference is solved"),
		Controller.Update(State, Capability, ContinuedBraking));
	TestTrue(TEXT("Drag feed-forward does not reverse the braking profile"),
		ContinuedBraking.VelocityCmPerSec.X < Braking.VelocityCmPerSec.X);

	FAircraftTrajectoryReference TerminalReference = ContinuedBraking;
	for (int32 Step = 0; Step < 100 && TerminalReference.VelocityCmPerSec.X > UE_SMALL_NUMBER; ++Step)
	{
		State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz;
		++State.Sequence;
		TestTrue(TEXT("Terminal braking reference remains solvable"),
			Controller.Update(State, Capability, TerminalReference));
		TestTrue(TEXT("Terminal braking never commands reverse velocity"),
			TerminalReference.VelocityCmPerSec.X >= -UE_SMALL_NUMBER);
	}
	TestTrue(TEXT("Terminal braking reaches zero reference speed"),
		FMath::IsNearlyZero(TerminalReference.VelocityCmPerSec.X, UE_SMALL_NUMBER));

	FAircraftAutopilotRuntimeConfig BudgetConfig = Config;
	BudgetConfig.Mpcc.SolveTimeBudgetMilliseconds = 1.0e-9f;
	FAircraftMpccController BudgetController;
	const FAircraftMovementIntent BudgetIntent = MakeRouteIntent(5000.0f);
	TestTrue(TEXT("Budget-limited route intent is accepted"),
		BudgetController.SetIntent(BudgetIntent, 8, 1, BudgetConfig, State, Capability));
	FAircraftTrajectoryReference BudgetReference;
	TestTrue(TEXT("A cooperative deadline publishes a feasible partial solution"),
		BudgetController.Update(State, Capability, BudgetReference));
	TestTrue(TEXT("The deadline truncates optimization iterations"),
		BudgetController.GetDiagnostics().SolverIterations
		< BudgetConfig.Mpcc.MaxOptimizationIterations);
	TestFalse(TEXT("Exhausting an optimization budget is not a solver failure"),
		BudgetController.GetDiagnostics().bSolverFailed);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCapabilityHardLimitsTest,
	"AircraftAutopilot.MPCC.CapabilityHardLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCapabilityHardLimitsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Velocity;
	Intent.Velocity.VelocityCmPerSec = FVector(800.0f, 0.0f, 300.0f);
	Intent.Limits.CruiseSpeedCmPerSec = 800.0f;
	Intent.Limits.MaxAccelerationCmPerSecSq = 600.0f;
	Intent.Limits.MaxDecelerationCmPerSecSq = 700.0f;
	Intent.Limits.MaxVerticalAccelerationCmPerSecSq = 500.0f;
	Intent.Limits.MaxClimbRateCmPerSec = 300.0f;
	Intent.Limits.MaxDescentRateCmPerSec = 250.0f;
	Intent.Limits.MaxYawRateDegPerSec = 120.0f;
	Intent.bHasRequestedMotionLimits = true;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.MaxHorizontalSpeedCmPerSec = 350.0f;
	Capability.MaxHorizontalAccelerationCmPerSecSq = 220.0f;
	Capability.MaxHorizontalDecelerationCmPerSecSq = 220.0f;
	Capability.MaxVerticalAccelerationCmPerSecSq = 180.0f;
	Capability.MaxClimbRateCmPerSec = 140.0f;
	Capability.MaxDescentRateCmPerSec = 110.0f;
	Capability.MaxBodyRateRadPerSec.Z = FMath::DegreesToRadians(45.0f);
	FAircraftVehicleStateSnapshot State;
	FAircraftMpccController Controller;
	const FAircraftAutopilotRuntimeConfig Config;
	TestTrue(TEXT("Capability-limited intent is accepted"),
		Controller.SetIntent(Intent, 9, 1, Config, State, Capability));
	const FAircraftRequestedMotionLimits& Limits = Controller.GetPlan().GetIntent().Limits;
	TestEqual(TEXT("Vehicle speed capability is authoritative"), Limits.CruiseSpeedCmPerSec, 350.0f);
	TestEqual(TEXT("Vehicle acceleration capability is authoritative"), Limits.MaxAccelerationCmPerSecSq, 220.0f);
	TestEqual(TEXT("Vehicle braking capability is authoritative"), Limits.MaxDecelerationCmPerSecSq, 220.0f);
	TestEqual(TEXT("Vehicle vertical acceleration capability is authoritative"), Limits.MaxVerticalAccelerationCmPerSecSq, 180.0f);
	TestEqual(TEXT("Vehicle climb capability is authoritative"), Limits.MaxClimbRateCmPerSec, 140.0f);
	TestEqual(TEXT("Vehicle descent capability is authoritative"), Limits.MaxDescentRateCmPerSec, 110.0f);
	TestEqual(TEXT("Vehicle yaw capability is authoritative"), Limits.MaxYawRateDegPerSec, 45.0f);
	Capability.MaxHorizontalSpeedCmPerSec = 200.0f;
	State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("A changed capability rebuilds the plan"),
		Controller.Update(State, Capability, Reference));
	TestEqual(TEXT("The rebuilt plan consumes the current hard speed limit"),
		Controller.GetPlan().GetIntent().Limits.CruiseSpeedCmPerSec, 200.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawReferenceUsesControlFrameTest,
	"AircraftAutopilot.MPCC.YawReferenceUsesControlFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawReferenceUsesControlFrameTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Velocity;
	Intent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	Intent.Heading.FixedYawDegrees = 25.0f;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.Sequence = 1;
	// 原始刚体轴可与 Dataflow 定义的控制机头不同；航向参考只能使用后者。
	State.BodyRotation = FRotator(0.0f, -65.0f, 0.0f).Quaternion();
	State.ControlRotation = FRotator(0.0f, 25.0f, 0.0f).Quaternion();
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Control-frame yaw intent is accepted"),
		Controller.SetIntent(Intent, 20, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Control-frame yaw reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestEqual(TEXT("Matching control heading produces no artificial yaw error"),
		Reference.YawDegrees, 25.0f, 1.e-4f);
	TestEqual(TEXT("Matching control heading produces no artificial yaw-rate feed-forward"),
		Reference.YawRateDegPerSec, 0.0f, 1.e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawReferenceRequiresPhysicalAuthorityTest,
	"AircraftAutopilot.MPCC.YawReferenceRequiresPhysicalAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawReferenceRequiresPhysicalAuthorityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Hold;
	Intent.Hold.bCaptureCurrentPosition = false;
	Intent.Hold.PositionCm = FVector::ZeroVector;
	Intent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	Intent.Heading.FixedYawDegrees = 90.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;
	State.BodyRotation = FQuat::Identity;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.PositiveTorqueAuthorityNm.Z = 0.0f;
	Capability.NegativeTorqueAuthorityNm.Z = 0.0f;
	Capability.bCanControlYaw = false;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftTrajectoryRuntime TrajectoryRuntime;

	TestTrue(TEXT("Intent remains valid without physical yaw authority"),
		TrajectoryRuntime.SetIntent(Intent, 21, 1, Config, State, Capability));
	TestEqual(TEXT("Motion plan publishes zero effective yaw-rate limit"),
		TrajectoryRuntime.GetPlan().GetIntent().Limits.MaxYawRateDegPerSec, 0.0f);
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Translation reference remains solvable without yaw authority"),
		TrajectoryRuntime.UpdateFlightController(State, Capability, Reference));
	TestEqual(TEXT("Unavailable yaw axis stays on measured heading"),
		Reference.YawDegrees, 0.0f, 1.e-4f);
	TestEqual(TEXT("Unavailable yaw axis publishes no yaw-rate command"),
		Reference.YawRateDegPerSec, 0.0f, 1.e-4f);
	TestEqual(TEXT("Unavailable yaw axis is explicit in the published reference"),
		Reference.YawRateLimitDegPerSec, 0.0f, 1.e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPathProgressTracksVehicleProjectionTest,
	"AircraftAutopilot.MPCC.PathProgressTracksVehicleProjection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPathProgressTracksVehicleProjectionTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Route intent is accepted"),
		Controller.SetIntent(Intent, 30, 1, Config, State, Capability));

	FAircraftTrajectoryReference Reference;
	for (int32 Index = 0; Index < 20; ++Index)
	{
		State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
		++State.Sequence;
		TestTrue(TEXT("Stationary route reference remains solvable"),
			Controller.Update(State, Capability, Reference));
	}
	TestTrue(TEXT("Reference progress does not run ahead of a stationary aircraft"),
		Reference.PathProgress < 0.01f);
	TestTrue(TEXT("Position reference remains anchored to the projected path point"),
		Reference.PositionCm.IsNearlyZero(1.0f));
	TestTrue(TEXT("A stationary route publishes forward control acceleration for backend cold start"),
		Reference.ControlAccelerationCmPerSecSq.X > 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVelocityReferenceRespectsDragAndTiltAuthorityTest,
	"AircraftAutopilot.MPCC.VelocityReferenceRespectsDragAndTiltAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVelocityReferenceRespectsDragAndTiltAuthorityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Velocity;
	Intent.Velocity.VelocityCmPerSec = FVector(800.0f, 0.0f, 0.0f);
	Intent.Limits.CruiseSpeedCmPerSec = 800.0f;
	Intent.bHasRequestedMotionLimits = true;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.LinearDampingPerSecond = FVector(2.0f);
	FAircraftMpccController Controller;
	TestTrue(TEXT("Drag-limited velocity intent is accepted"),
		Controller.SetIntent(Intent, 31, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Drag-limited velocity reference is solved"),
		Controller.Update(State, Capability, Reference));

	const FVector TotalControlAcceleration = Reference.ControlAccelerationCmPerSecSq
		+ Reference.DynamicsFeedForwardAccelerationCmPerSecSq;
	const float TiltAuthority = Capability.GravityCmPerSecSq
		* FMath::Tan(Capability.MaxTiltRadians);
	TestTrue(TEXT("Sustainable speed is reduced by the configured damping"),
		Reference.VelocityCmPerSec.X < 800.0f);
	TestTrue(TEXT("Control plus dynamics feed-forward stays inside tilt authority"),
		FVector2D(TotalControlAcceleration.X, TotalControlAcceleration.Y).Size()
			<= TiltAuthority + 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftTimedTrajectoryUsesExplicitClockTest,
	"AircraftAutopilot.MPCC.TimedTrajectoryUsesExplicitClock",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftTimedTrajectoryUsesExplicitClockTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::TimedTrajectory;
	FAircraftTimedTrajectorySample Start;
	Start.TimeSeconds = 0.0f;
	Start.PositionCm = FVector::ZeroVector;
	Start.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	FAircraftTimedTrajectorySample Finish;
	Finish.TimeSeconds = 2.0f;
	Finish.PositionCm = FVector(200.0f, 0.0f, 0.0f);
	Finish.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	Intent.TimedTrajectory.Samples = { Start, Finish };
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 10.0;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Timed trajectory intent is accepted"),
		Controller.SetIntent(Intent, 32, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Timed trajectory initial reference is solved"),
		Controller.Update(State, Capability, Reference));
	State.TimeSeconds = 11.0;
	++State.Sequence;
	State.PositionCm = FVector(100.0f, 0.0f, 0.0f);
	State.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	TestTrue(TEXT("Timed trajectory midpoint reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestEqual(TEXT("Explicit trajectory clock reaches the midpoint"),
		Reference.PositionCm.X, 100.0, 1.0);
	TestEqual(TEXT("Timed trajectory reports half progress"),
		Reference.PathProgress, 0.5f, 0.01f);
	FAircraftMovementIntent UnreachableIntent = Intent;
	UnreachableIntent.TimedTrajectory.Samples.Last().VelocityCmPerSec.X =
		Capability.MaxHorizontalSpeedCmPerSec + 1.0f;
	FAircraftMpccController RejectingController;
	TestFalse(TEXT("Timed trajectory cannot exceed backend hard speed"),
		RejectingController.SetIntent(
			UnreachableIntent, 33, 1, Config, State, Capability));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPathProgressIsMonotonicTest,
	"AircraftAutopilot.MPCC.PathProgressIsMonotonic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPathProgressIsMonotonicTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Monotonic-progress route is accepted"),
		Controller.SetIntent(Intent, 40, 1, Config, State, Capability));

	FAircraftTrajectoryReference Reference;
	float PreviousProgress = 0.0f;
	for (int32 Index = 0; Index < 30; ++Index)
	{
		State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
		++State.Sequence;
		State.PositionCm.X += 8.0f;
		State.VelocityCmPerSec.X = 400.0f;
		TestTrue(TEXT("Forward route reference is solved"),
			Controller.Update(State, Capability, Reference));
		TestTrue(TEXT("Forward progress never regresses"),
			Reference.PathProgress + UE_KINDA_SMALL_NUMBER >= PreviousProgress);
		PreviousProgress = Reference.PathProgress;
	}
	TestTrue(TEXT("Same-handle route update is accepted"),
		Controller.SetIntent(Intent, 40, 2, Config, State, Capability));
	State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
	++State.Sequence;
	TestTrue(TEXT("Updated route reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("Same-handle route update preserves monotonic progress"),
		Reference.PathProgress + UE_KINDA_SMALL_NUMBER >= PreviousProgress);
	PreviousProgress = Reference.PathProgress;
	Capability.MaxHorizontalAccelerationCmPerSecSq *= 0.5f;
	State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
	++State.Sequence;
	TestTrue(TEXT("Capability retiming reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("Capability retiming preserves monotonic progress"),
		Reference.PathProgress + UE_KINDA_SMALL_NUMBER >= PreviousProgress);
	PreviousProgress = Reference.PathProgress;
	State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
	++State.Sequence;
	State.PositionCm.X -= 100.0f;
	State.VelocityCmPerSec.X = -400.0f;
	TestTrue(TEXT("Off-course recovery reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("Closest-point movement cannot reverse route progress"),
		Reference.PathProgress + UE_KINDA_SMALL_NUMBER >= PreviousProgress);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPathReferenceGovernorDoesNotSelfThrottleTest,
	"AircraftAutopilot.MPCC.PathReferenceGovernorDoesNotSelfThrottle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPathReferenceGovernorDoesNotSelfThrottleTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Slow-tracking route is accepted"),
		Controller.SetIntent(Intent, 43, 1, Config, State, Capability));

	FAircraftTrajectoryReference Reference;
	const float DeltaTime = 1.0f / Config.Mpcc.UpdateRateHz + 0.001f;
	for (int32 Index = 0; Index < 80; ++Index)
	{
		State.TimeSeconds += DeltaTime;
		++State.Sequence;
		State.VelocityCmPerSec = FVector(50.0f, 0.0f, 0.0f);
		State.PositionCm += State.VelocityCmPerSec * DeltaTime;
		TestTrue(TEXT("Slow but on-path reference remains solvable"),
			Controller.Update(State, Capability, Reference));
	}
	TestTrue(TEXT("Low measured speed cannot recursively throttle an on-path reference"),
		Controller.GetDiagnostics().ProgressScale > 0.95f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNominalBrakingSurvivesVelocityErrorTest,
	"AircraftAutopilot.MPCC.NominalBrakingSurvivesVelocityError",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNominalBrakingSurvivesVelocityErrorTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::TimedTrajectory;
	FAircraftTimedTrajectorySample Start;
	Start.TimeSeconds = 0.0f;
	Start.PositionCm = FVector::ZeroVector;
	Start.VelocityCmPerSec = FVector(500.0f, 0.0f, 0.0f);
	Start.AccelerationCmPerSecSq = FVector(-300.0f, 0.0f, 0.0f);
	FAircraftTimedTrajectorySample Finish = Start;
	Finish.TimeSeconds = 2.0f;
	Finish.PositionCm = FVector(700.0f, 0.0f, 0.0f);
	Intent.TimedTrajectory.Samples = { Start, Finish };
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.VelocityCmPerSec = FVector(300.0f, 0.0f, 0.0f);
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	TestTrue(TEXT("Braking trajectory is accepted"),
		Controller.SetIntent(Intent, 41, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Braking trajectory reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("Positive velocity error cannot replace nominal braking with acceleration"),
		Reference.ControlAccelerationCmPerSecSq.X < 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawReferenceRespectsDynamicLimitsTest,
	"AircraftAutopilot.MPCC.YawReferenceRespectsDynamicLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawReferenceRespectsDynamicLimitsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Velocity;
	Intent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	Intent.Heading.FixedYawDegrees = 90.0f;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;
	State.BodyRotation = FQuat::Identity;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftTrajectoryRuntime TrajectoryRuntime;
	TestTrue(TEXT("Yaw-governed intent is accepted"),
		TrajectoryRuntime.SetIntent(Intent, 42, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	for (int32 Index = 0; Index < 20; ++Index)
	{
		State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
		++State.Sequence;
		TestTrue(TEXT("Yaw-governed reference is solved"),
			TrajectoryRuntime.UpdateFlightController(State, Capability, Reference));
	}
	TestTrue(TEXT("Yaw reference progresses toward the requested heading"),
		Reference.YawDegrees > 0.0f && Reference.YawDegrees <= 90.0f);
	TestTrue(TEXT("Yaw reference respects the rate limit"),
		FMath::Abs(Reference.YawRateDegPerSec)
			<= Intent.Limits.MaxYawRateDegPerSec + UE_KINDA_SMALL_NUMBER);
	TestTrue(TEXT("Yaw reference respects the acceleration limit"),
		FMath::Abs(Reference.YawAccelerationDegPerSecSq)
			<= Intent.Limits.MaxYawAccelerationDegPerSecSq + UE_KINDA_SMALL_NUMBER);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCorridorPredictionPriorityTest,
	"AircraftAutopilot.MPCC.CorridorPredictionPriority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCorridorPredictionPriorityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent = MakeRouteIntent(2000.0f);
	FAircraftSafeCorridorSegment& Corridor = Intent.Route.Corridor.AddDefaulted_GetRef();
	Corridor.AxisStartCm = Intent.Route.PointsCm[0];
	Corridor.AxisEndCm = Intent.Route.PointsCm.Last();
	Corridor.RadiusCm = 100.0f;
	Corridor.StartDistanceCm = 0.0f;
	Corridor.EndDistanceCm = 2000.0f;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Path.CorridorSafetyMarginCm = 0.0f;
	Config.Mpcc.MaxOptimizationIterations = 4;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.PositionCm = FVector(0.0f, 70.0f, 0.0f);
	State.VelocityCmPerSec = FVector(200.0f, 400.0f, 0.0f);
	FAircraftMpccController Controller;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	TestTrue(TEXT("Corridor route is accepted"),
		Controller.SetIntent(Intent, 34, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Corridor-constrained reference is solved"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("Predictive controller commands toward the corridor interior"),
		Reference.ControlAccelerationCmPerSecSq.Y < 0.0f);
	TestTrue(TEXT("Predicted corridor conflict is reported"),
		Controller.GetDiagnostics().PathTrackingState
			== EAircraftPathTrackingState::CorridorConstrained
		|| Controller.GetDiagnostics().PathTrackingState
			== EAircraftPathTrackingState::CorridorRecovery);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftManualYawRateReleaseCapturesMeasuredHoldTest,
	"AircraftAutopilot.MPCC.ManualYawRateReleaseCapturesMeasuredHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftManualYawRateReleaseCapturesMeasuredHoldTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Hold;
	Intent.Hold.bCaptureCurrentPosition = false;
	Intent.Hold.PositionCm = FVector::ZeroVector;
	Intent.Heading.Mode = EAircraftHeadingMode::YawRate;
	Intent.Heading.YawRateDegPerSec = 90.0f;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;
	State.BodyRotation = FQuat::Identity;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftTrajectoryRuntime TrajectoryRuntime;
	FAircraftTrajectoryReference Reference;
	const float DeltaTime = 1.0f / Config.Mpcc.UpdateRateHz;

	TestTrue(TEXT("Manual yaw-rate intent is accepted"),
		TrajectoryRuntime.SetIntent(Intent, 50, 1, Config, State, Capability));
	for (int32 Step = 0; Step < 30; ++Step)
	{
		TestTrue(TEXT("Manual yaw-rate reference remains solvable"),
			TrajectoryRuntime.UpdateFlightController(State, Capability, Reference));
		State.ControlRotation = FRotator(0.0f, Reference.YawDegrees, 0.0f).Quaternion();
		State.BodyRotation = State.ControlRotation;
		State.AngularVelocityBodyRadPerSec = FVector(
			0.0f, 0.0f, FMath::DegreesToRadians(Reference.YawRateDegPerSec));
		State.TimeSeconds += DeltaTime;
		++State.Sequence;
	}

	TestTrue(TEXT("Manual yaw-rate reference accelerates toward the command"),
		Reference.YawRateDegPerSec > 20.0f);
	const float ReleaseRateDegPerSec = Reference.YawRateDegPerSec;
	Intent.Heading.YawRateDegPerSec = 0.0f;
	TestTrue(TEXT("Released yaw-rate intent revision is accepted"),
		TrajectoryRuntime.SetIntent(Intent, 50, 2, Config, State, Capability));
	TestTrue(TEXT("First released yaw-rate reference is solvable"),
		TrajectoryRuntime.UpdateFlightController(State, Capability, Reference));
	TestTrue(TEXT("Yaw angle feedback remains aligned to the measured heading while rate braking starts"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(
			Reference.YawDegrees, State.ControlRotation.Rotator().Yaw)) < 0.1f);
	State.TimeSeconds += DeltaTime;
	++State.Sequence;
	for (int32 Step = 1; Step < 180; ++Step)
	{
		State.ControlRotation = FRotator(0.0f, Reference.YawDegrees, 0.0f).Quaternion();
		State.BodyRotation = State.ControlRotation;
		State.AngularVelocityBodyRadPerSec = FVector(
			0.0f, 0.0f, FMath::DegreesToRadians(Reference.YawRateDegPerSec));
		TestTrue(TEXT("Released yaw-rate reference remains solvable"),
			TrajectoryRuntime.UpdateFlightController(State, Capability, Reference));
		State.TimeSeconds += DeltaTime;
		++State.Sequence;
	}

	TestTrue(TEXT("Release decelerates the yaw-rate reference"),
		FMath::Abs(Reference.YawRateDegPerSec) < FMath::Abs(ReleaseRateDegPerSec));
	TestTrue(TEXT("Released reference settles without reversing"),
		Reference.YawRateDegPerSec >= -UE_KINDA_SMALL_NUMBER);
	TestTrue(*FString::Printf(
		TEXT("Released reference converges to a stationary hold (rate=%.6f acceleration=%.6f)"),
		Reference.YawRateDegPerSec, Reference.YawAccelerationDegPerSecSq),
		FMath::Abs(Reference.YawRateDegPerSec) < 0.25f
			&& FMath::Abs(Reference.YawAccelerationDegPerSecSq) < 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVelocityReferenceTerminalJerkContinuityTest,
	"AircraftAutopilot.Runtime.VelocityReferenceTerminalJerkContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVelocityReferenceTerminalJerkContinuityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Velocity;
	Intent.Velocity.VelocityCmPerSec = FVector(50.0f, 0.0f, 0.0f);
	Intent.Limits.CruiseSpeedCmPerSec = 100.0f;
	Intent.Limits.MaxAccelerationCmPerSecSq = 120.0f;
	Intent.Limits.MaxDecelerationCmPerSecSq = 120.0f;
	Intent.Limits.MaxJerkCmPerSecCubed = 60.0f;
	Intent.bHasRequestedMotionLimits = true;

	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.MaxHorizontalAccelerationCmPerSecSq = 120.0f;
	Capability.MaxHorizontalDecelerationCmPerSecSq = 120.0f;
	Capability.MaxHorizontalJerkCmPerSecCubed = 60.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("Velocity intent is accepted"),
		Runtime.SetIntent(Intent, 51, 1, Config, State, Capability));

	constexpr float DeltaSeconds = 0.05f;
	FVector PreviousAcceleration = FVector::ZeroVector;
	FAircraftTrajectoryReference Reference;
	for (int32 Step = 0; Step < 200; ++Step)
	{
		State.TimeSeconds += DeltaSeconds;
		++State.Sequence;
		TestTrue(TEXT("Velocity reference remains valid"),
			Runtime.UpdatePhysicsConstraint(State, Capability, Reference));
		const float AccelerationStep = FVector2D(
			Reference.AccelerationCmPerSecSq.X - PreviousAcceleration.X,
			Reference.AccelerationCmPerSecSq.Y - PreviousAcceleration.Y).Size();
		TestTrue(TEXT("Reaching target velocity never clears acceleration faster than the jerk limit"),
			AccelerationStep
				<= Intent.Limits.MaxJerkCmPerSecCubed * DeltaSeconds + 1.e-3f);
		PreviousAcceleration = Reference.AccelerationCmPerSecSq;
	}

	TestTrue(TEXT("Velocity reference converges to the requested speed"),
		FMath::Abs(Reference.VelocityCmPerSec.X
			- Intent.Velocity.VelocityCmPerSec.X) < 0.1f);
	TestTrue(TEXT("Terminal acceleration converges to zero"),
		Reference.AccelerationCmPerSecSq.IsNearlyZero(0.1f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDeterministicBackendTrajectoryTest,
	"AircraftAutopilot.Runtime.DeterministicBackends",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDeterministicBackendTrajectoryTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.bHasExplicitAerodynamics = true;
	Capability.AirDensityKgPerM3 = 1.225f;
	Capability.LinearDragAircraftNsPerM = FVector(10.0f);
	Capability.DragAreaCoefficientAircraftM2 = FVector(0.2f);
	Capability.MaxRelativeAirspeedCmPerSec = 10000.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.BodyRotation = FQuat::Identity;
	State.ControlRotation = FQuat::Identity;

	FAircraftTrajectoryRuntime Runtime;
	const FAircraftMovementIntent RouteIntent = MakeRouteIntent(5000.0f);
	TestTrue(TEXT("The shared runtime builds the route"),
		Runtime.SetIntent(RouteIntent, 60, 1, Config, State, Capability));
	State.TimeSeconds += 0.1;
	++State.Sequence;
	FAircraftTrajectoryReference KinematicReference;
	TestTrue(TEXT("Kinematic sampling succeeds without MPCC"),
		Runtime.UpdateKinematic(State, Capability, KinematicReference));
	FAircraftMotionPlanSample ExpectedSample;
	TestTrue(TEXT("The immutable plan evaluates at the kinematic cursor"),
		Runtime.GetPlan().Evaluate(0.1f, ExpectedSample));
	TestTrue(TEXT("Kinematic position is the exact plan sample"),
		KinematicReference.PositionCm.Equals(ExpectedSample.PositionCm, 0.01f));
	TestTrue(TEXT("Kinematic sampling never consumes aerodynamics"),
		KinematicReference.DynamicsFeedForwardAccelerationCmPerSecSq.IsNearlyZero());

	FAircraftTrajectoryRuntime GovernedRuntime;
	State.PositionCm = FVector(0.0f, 300.0f, 0.0f);
	TestTrue(TEXT("The constraint runtime builds the route"),
		GovernedRuntime.SetIntent(RouteIntent, 61, 1, Config, State, Capability));
	State.TimeSeconds += 0.1;
	++State.Sequence;
	FAircraftTrajectoryReference ConstraintReference;
	TestTrue(TEXT("Constraint deterministic sampling succeeds"),
		GovernedRuntime.UpdatePhysicsConstraint(State, Capability, ConstraintReference));
	TestTrue(TEXT("Constraint progress governor reacts to contour error"),
		GovernedRuntime.GetDiagnostics().ProgressScale < 1.0f);
	TestTrue(TEXT("Constraint sampling retains dynamics feed-forward"),
		!ConstraintReference.DynamicsFeedForwardAccelerationCmPerSecSq.IsNearlyZero());

	FAircraftMovementIntent VelocityIntent;
	VelocityIntent.Type = EAircraftMovementIntentType::Velocity;
	VelocityIntent.Velocity.VelocityCmPerSec = FVector(800.0f, 0.0f, 0.0f);
	VelocityIntent.Limits = RouteIntent.Limits;
	VelocityIntent.bHasRequestedMotionLimits = true;
	FAircraftTrajectoryRuntime VelocityRuntime;
	State.PositionCm = FVector::ZeroVector;
	State.VelocityCmPerSec = FVector::ZeroVector;
	TestTrue(TEXT("Velocity intent builds"), VelocityRuntime.SetIntent(
		VelocityIntent, 62, 1, Config, State, Capability));
	State.TimeSeconds += 0.1;
	++State.Sequence;
	FAircraftTrajectoryReference AcceleratingReference;
	TestTrue(TEXT("Velocity reference accelerates deterministically"),
		VelocityRuntime.UpdateKinematic(State, Capability, AcceleratingReference));
	VelocityIntent.Velocity.VelocityCmPerSec = FVector::ZeroVector;
	TestTrue(TEXT("Same-handle release preserves the velocity profile"),
		VelocityRuntime.SetIntent(VelocityIntent, 62, 2, Config, State, Capability));
	State.TimeSeconds += 0.1;
	++State.Sequence;
	FAircraftTrajectoryReference BrakingReference;
	TestTrue(TEXT("Velocity reference brakes deterministically"),
		VelocityRuntime.UpdateKinematic(State, Capability, BrakingReference));
	State.TimeSeconds += 0.1;
	++State.Sequence;
	FAircraftTrajectoryReference ContinuedBrakingReference;
	TestTrue(TEXT("Jerk-limited braking continues deterministically"),
		VelocityRuntime.UpdateKinematic(State, Capability, ContinuedBrakingReference));
	TestTrue(TEXT("Release preserves velocity continuity instead of stopping instantly"),
		BrakingReference.VelocityCmPerSec.X > 0.0f);
	TestTrue(TEXT("Release immediately reduces the positive acceleration within the jerk limit"),
		BrakingReference.AccelerationCmPerSecSq.X
			< AcceleratingReference.AccelerationCmPerSecSq.X);
	TestTrue(TEXT("Braking never reverses the deterministic reference"),
		ContinuedBrakingReference.VelocityCmPerSec.X >= 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotLodInterruptionTest,
	"AircraftLab.Autopilot.Intent.LodInterruption",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotLodInterruptionTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAutopilotComponent* const Autopilot = NewObject<UAutopilotComponent>();
	Autopilot->SetAutopilotActive(true);

	FAircraftHoldIntent Hold;
	Hold.PositionCm = FVector(100.0, 200.0, 300.0);
	const FAircraftMovementIntentHandle Handle = Autopilot->SubmitHoldIntent(
		Hold, FAircraftMovementIntentSettings(), FAircraftCompletionPolicy());
	TestTrue(TEXT("The source intent is active before the LOD transition"),
		Autopilot->IsAircraftMovementIntentActive());

	Autopilot->OnAircraftMovementIntentInterrupted(
		Handle, EAircraftMovementFailureReason::SimulationLODChanged);

	const FAircraftMovementIntentResult Result = Autopilot->GetCurrentIntentResult();
	TestEqual(TEXT("LOD transition interrupts the source intent"),
		Result.Status, EAircraftMovementIntentStatus::Interrupted);
	TestEqual(TEXT("The interruption reports the LOD reason"),
		Result.FailureReason, EAircraftMovementFailureReason::SimulationLODChanged);
	TestFalse(TEXT("The interrupted source intent cannot be pushed again"),
		Autopilot->IsAircraftMovementIntentActive());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidancePreservesIntentTest,
	"AircraftLab.Autopilot.NavigationGuidance.PreservesPrimaryIntent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidancePreservesIntentTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The primary route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 71, 3, Config, State, Capability));

	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 71;
	Guidance.SourceIntentRevision = 3;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector(0.0f, 200.0f, 0.0f), FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 600.0f, 0.0f), FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector }
	};
	TestTrue(TEXT("The guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 9));

	State.TimeSeconds = 1.25;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The guided reference is produced"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestEqual(TEXT("Guidance does not replace the primary intent id"), Reference.IntentId, int64(71));
	TestEqual(TEXT("Guidance does not replace the primary intent revision"), Reference.IntentRevision, int64(3));
	TestTrue(TEXT("Guidance replaces the short-term position reference"),
		Reference.PositionCm.Equals(FVector(0.0f, 300.0f, 0.0f), 0.01f));
	TestTrue(TEXT("The nominal route reference remains separately available"),
		Runtime.GetNominalReference().PositionCm.Y == 0.0f);
	TestEqual(TEXT("The applied guidance revision is published"),
		Runtime.GetNavigationGuidanceStatus().Revision, uint64(9));
	TestEqual(TEXT("Guidance reports the applied state"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Applied);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceExpiryTest,
	"AircraftLab.Autopilot.NavigationGuidance.ExpiredGuidanceBrakes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceExpiryTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;
	State.VelocityCmPerSec = FVector(400.0f, 0.0f, 0.0f);

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The primary route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 72, 1, Config, State, Capability));

	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 72;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 1.2;
	Guidance.Samples = {
		{ 0.0f, FVector::ZeroVector, FVector(400.0f, 0.0f, 0.0f), FVector::ZeroVector },
		{ 0.2f, FVector(80.0f, 0.0f, 0.0f), FVector(400.0f, 0.0f, 0.0f), FVector::ZeroVector }
	};
	TestTrue(TEXT("The guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 10));

	State.TimeSeconds = 1.3;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("An expired guidance still produces a fail-safe reference"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestEqual(TEXT("Expired guidance enters braking"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Braking);
	TestEqual(TEXT("Expiry is reported explicitly"),
		Runtime.GetNavigationGuidanceStatus().FailureReason,
		EAircraftNavigationGuidanceFailureReason::Expired);
	TestTrue(TEXT("Braking targets a point ahead of the current COM"),
		Reference.PositionCm.X > State.PositionCm.X);
	TestTrue(TEXT("Kinematic braking reduces speed without teleporting to the stop point"),
		Reference.VelocityCmPerSec.X >= 0.0f
		&& Reference.VelocityCmPerSec.X < State.VelocityCmPerSec.X
		&& Reference.PositionCm.X < 80.0f);
	TestEqual(TEXT("Braking keeps the primary task identity"), Reference.IntentId, int64(72));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceClearTest,
	"AircraftLab.Autopilot.NavigationGuidance.ClearRestoresNominalPlan",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceClearTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The primary route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 73, 1, Config, State, Capability));
	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 73;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector(0.0f, 200.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 200.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector }
	};
	TestTrue(TEXT("The guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 11));
	Runtime.ClearNavigationGuidance(12);

	State.TimeSeconds = 1.1;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The nominal route reference is produced after clearing guidance"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestTrue(TEXT("The route reference is no longer laterally overridden"),
		FMath::IsNearlyZero(Reference.PositionCm.Y));
	TestEqual(TEXT("Clearing guidance publishes inactive state"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Inactive);
	TestEqual(TEXT("The clear revision is published"),
		Runtime.GetNavigationGuidanceStatus().Revision, uint64(12));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceRampInTest,
	"AircraftLab.Autopilot.NavigationGuidance.BlendRampsInFromNominal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceRampInTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The primary route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 81, 1, Config, State, Capability));

	// 引导在 t=1.0 发布，与 nominal 明显分离（Y 方向 400cm 偏移、恒定 0 速度样本）。
	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 81;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector }
	};
	TestTrue(TEXT("The guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 20));

	// t=1.1：渐入一半（0.1/0.2）——参考应处于 nominal 与样本的中间。
	State.TimeSeconds = 1.1;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The blended reference is produced"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestTrue(TEXT("Ramp-in blends toward the guidance sample"),
		Reference.PositionCm.Y > 0.0f && Reference.PositionCm.Y < 400.0f);
	TestEqual(TEXT("Ramp-in still reports the applied state"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Applied);

	// 记录中间参考用于验证"介于两端"。
	const float MidBlendY = Reference.PositionCm.Y;

	// t=1.21：渐变完成（0.21/0.2 ≥ 1）——Steady 全量覆写。
	State.TimeSeconds = 1.21;
	State.Sequence = 2;
	FAircraftTrajectoryReference SteadyReference;
	TestTrue(TEXT("The steady reference is produced"),
		Runtime.UpdateKinematic(State, Capability, SteadyReference));
	TestTrue(TEXT("Steady state applies the full guidance sample"),
		SteadyReference.PositionCm.Y > MidBlendY);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftUnifiedYawReferenceAcrossDriveModesTest,
	"AircraftAutopilot.Runtime.UnifiedYawReferenceAcrossDriveModes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftUnifiedYawReferenceAcrossDriveModesTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Hold;
	Intent.Hold.bCaptureCurrentPosition = false;
	Intent.Hold.PositionCm = FVector::ZeroVector;
	Intent.Heading.Mode = EAircraftHeadingMode::YawRate;
	Intent.Heading.YawRateDegPerSec = 60.0f;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;
	State.BodyRotation = FQuat::Identity;
	FAircraftTrajectoryRuntime FlightRuntime;
	FAircraftTrajectoryRuntime ConstraintRuntime;
	FAircraftTrajectoryRuntime KinematicRuntime;
	TestTrue(TEXT("Flight runtime accepts the shared yaw intent"),
		FlightRuntime.SetIntent(Intent, 70, 1, Config, State, Capability));
	TestTrue(TEXT("Constraint runtime accepts the shared yaw intent"),
		ConstraintRuntime.SetIntent(Intent, 70, 1, Config, State, Capability));
	TestTrue(TEXT("Kinematic runtime accepts the shared yaw intent"),
		KinematicRuntime.SetIntent(Intent, 70, 1, Config, State, Capability));

	State.TimeSeconds += 0.05;
	++State.Sequence;
	FAircraftTrajectoryReference FlightReference;
	FAircraftTrajectoryReference ConstraintReference;
	FAircraftTrajectoryReference KinematicReference;
	TestTrue(TEXT("Flight runtime solves the shared yaw reference"),
		FlightRuntime.UpdateFlightController(State, Capability, FlightReference));
	TestTrue(TEXT("Constraint runtime solves the shared yaw reference"),
		ConstraintRuntime.UpdatePhysicsConstraint(State, Capability, ConstraintReference));
	TestTrue(TEXT("Kinematic runtime solves the shared yaw reference"),
		KinematicRuntime.UpdateKinematic(State, Capability, KinematicReference));
	TestEqual(TEXT("All drives publish the same yaw angle"),
		FlightReference.YawDegrees, ConstraintReference.YawDegrees, 1.e-4f);
	TestEqual(TEXT("Constraint and Kinematic publish the same yaw angle"),
		ConstraintReference.YawDegrees, KinematicReference.YawDegrees, 1.e-4f);
	TestEqual(TEXT("All drives publish the same yaw rate"),
		FlightReference.YawRateDegPerSec, ConstraintReference.YawRateDegPerSec, 1.e-4f);
	TestEqual(TEXT("Constraint and Kinematic publish the same yaw rate"),
		ConstraintReference.YawRateDegPerSec, KinematicReference.YawRateDegPerSec, 1.e-4f);
	TestEqual(TEXT("All drives publish the same yaw acceleration"),
		FlightReference.YawAccelerationDegPerSecSq,
		ConstraintReference.YawAccelerationDegPerSecSq, 1.e-4f);
	TestEqual(TEXT("Constraint and Kinematic publish the same yaw acceleration"),
		ConstraintReference.YawAccelerationDegPerSecSq,
		KinematicReference.YawAccelerationDegPerSecSq, 1.e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicObstructionFreezesPlanTest,
	"AircraftAutopilot.Trajectory.KinematicObstructionFreezesPlan",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicObstructionFreezesPlanTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftTrajectoryRuntime Runtime;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftAutopilotRuntimeConfig Config;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.VelocityCmPerSec = FVector(300.0f, 0.0f, 0.0f);
	TestTrue(TEXT("Route intent is accepted"), Runtime.SetIntent(
		MakeRouteIntent(3000.0f), 101, 1, Config, State, Capability));

	State.TimeSeconds = 1.1;
	State.Sequence = 1;
	FAircraftTrajectoryReference MovingReference;
	TestTrue(TEXT("The route advances before impact"),
		Runtime.UpdateKinematic(State, Capability, MovingReference));
	const float ProgressAtImpact = MovingReference.PathProgress;

	State.TimeSeconds = 1.2;
	State.Sequence = 2;
	State.PositionCm = MovingReference.PositionCm * 0.25f;
	FAircraftTrajectoryReference FirstBlockedReference;
	TestTrue(TEXT("A blocking sweep produces a fail-safe braking reference"),
		Runtime.UpdateKinematicObstructed(State, Capability, FirstBlockedReference));
	TestEqual(TEXT("The blocked reference keeps the active movement intent"),
		FirstBlockedReference.IntentId, int64(101));
	TestTrue(TEXT("The plan cursor rebases to the actual impact position"),
		FirstBlockedReference.PathProgress < ProgressAtImpact);
	TestTrue(TEXT("The blocked reference decelerates from actual velocity"),
		FirstBlockedReference.VelocityCmPerSec.X >= 0.0f
		&& FirstBlockedReference.VelocityCmPerSec.X < State.VelocityCmPerSec.X);

	State.TimeSeconds = 2.2;
	State.Sequence = 3;
	FAircraftTrajectoryReference PersistentBlockedReference;
	TestTrue(TEXT("Persistent obstruction continues publishing a reference"),
		Runtime.UpdateKinematicObstructed(State, Capability, PersistentBlockedReference));
	TestTrue(TEXT("Elapsed blocked time cannot advance the rebased route cursor"),
		FMath::IsNearlyEqual(PersistentBlockedReference.PathProgress,
			FirstBlockedReference.PathProgress, 1.e-5f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceCorridorValidationTest,
	"AircraftLab.Autopilot.NavigationGuidance.BlendedReferenceRemainsInCorridor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceCorridorValidationTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Path.CorridorSafetyMarginCm = 0.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	FAircraftSafeCorridorSegment& Corridor = Intent.Route.Corridor.AddDefaulted_GetRef();
	Corridor.AxisStartCm = FVector::ZeroVector;
	Corridor.AxisEndCm = FVector(5000.0f, 0.0f, 0.0f);
	Corridor.RadiusCm = 100.0f;
	Corridor.StartDistanceCm = 0.0f;
	Corridor.EndDistanceCm = 5000.0f;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The corridor route is accepted"), Runtime.SetIntent(
		Intent, 91, 1, Config, State, Capability));

	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 91;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector }
	};
	TestTrue(TEXT("The external guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 31));

	State.TimeSeconds = 1.1;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The blended reference is produced"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestTrue(TEXT("The final blended reference remains inside the route corridor"),
		FMath::Abs(Reference.PositionCm.Y) <= Corridor.RadiusCm + 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidancePredictedCorridorValidationTest,
	"AircraftLab.Autopilot.NavigationGuidance.PredictedReferenceRemainsInCorridor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidancePredictedCorridorValidationTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Path.CorridorSafetyMarginCm = 0.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	FAircraftSafeCorridorSegment& Corridor = Intent.Route.Corridor.AddDefaulted_GetRef();
	Corridor.AxisStartCm = FVector::ZeroVector;
	Corridor.AxisEndCm = FVector(5000.0f, 0.0f, 0.0f);
	Corridor.RadiusCm = 100.0f;
	Corridor.StartDistanceCm = 0.0f;
	Corridor.EndDistanceCm = 5000.0f;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The corridor route is accepted"), Runtime.SetIntent(
		Intent, 92, 1, Config, State, Capability));

	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 92;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector::ZeroVector, FVector(0.0f, 500.0f, 0.0f), FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 500.0f, 0.0f), FVector(0.0f, 500.0f, 0.0f), FVector::ZeroVector }
	};
	TestTrue(TEXT("The moving guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 32));

	State.TimeSeconds = 1.2;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The guided reference is produced"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	const FVector PredictedPositionCm = Reference.PositionCm
		+ Reference.VelocityCmPerSec * 0.25f;
	TestTrue(TEXT("Non-zero guided velocity cannot leave the corridor within its validity window"),
		FMath::Abs(PredictedPositionCm.Y) <= Corridor.RadiusCm + 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceQuadraticCornerTest,
	"AircraftLab.Autopilot.NavigationGuidance.QuadraticReferenceRemainsInCornerCorridor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceQuadraticCornerTest::RunTest(
	const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Path.CorridorSafetyMarginCm = 0.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftMovementIntent Intent = MakeRouteIntent(4000.0f);
	Intent.Route.PointsCm = {
		FVector::ZeroVector,
		FVector(1000.0, 0.0, 0.0),
		FVector(1000.0, 1000.0, 0.0),
		FVector(4000.0, 1000.0, 0.0)
	};
	float RouteDistanceCm = 0.0f;
	for (int32 Index = 1; Index < Intent.Route.PointsCm.Num(); ++Index)
	{
		FAircraftSafeCorridorSegment& Segment =
			Intent.Route.Corridor.AddDefaulted_GetRef();
		Segment.AxisStartCm = Intent.Route.PointsCm[Index - 1];
		Segment.AxisEndCm = Intent.Route.PointsCm[Index];
		Segment.RadiusCm = 175.0f;
		Segment.StartDistanceCm = RouteDistanceCm;
		RouteDistanceCm += static_cast<float>(FVector::Distance(
			Segment.AxisStartCm, Segment.AxisEndCm));
		Segment.EndDistanceCm = RouteDistanceCm;
	}

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The formally partitioned corner corridor route is accepted"), Runtime.SetIntent(
		Intent, 93, 1, Config, State, Capability));

	// This quadratic attempts to leave the first capsule before the route reaches
	// the corner. The fixture itself remains a legal contiguous route partition;
	// the runtime must constrain the dynamic reference rather than relying on an
	// invalid synthetic corridor layout.
	const FVector BowVelocityCmPerSec(500.0f, 1200.0f, 0.0f);
	const FVector BowAccelerationCmPerSecSq(0.0f, -800.0f, 0.0f);
	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 93;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector::ZeroVector, BowVelocityCmPerSec, BowAccelerationCmPerSecSq },
		{ 1.0f, FVector(500.0f, 800.0f, 0.0f), FVector(500.0f, 400.0f, 0.0f),
			BowAccelerationCmPerSecSq }
	};
	TestTrue(TEXT("The curved guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 33));

	State.TimeSeconds = 1.2;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("A corridor-safe reference is produced"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	const float UnsampledTimeSeconds = 0.25f;
	const FVector PredictedUnsampledPositionCm = Reference.PositionCm
		+ Reference.VelocityCmPerSec * UnsampledTimeSeconds
		+ 0.5f * Reference.AccelerationCmPerSecSq
			* FMath::Square(UnsampledTimeSeconds);
	bool bFinalPointInsideUnion = false;
	for (const FAircraftSafeCorridorSegment& Segment : Intent.Route.Corridor)
	{
		bFinalPointInsideUnion |= Segment.ComputeCorrectionCm(
			PredictedUnsampledPositionCm, 0.0f).IsNearlyZero();
	}
	TestTrue(TEXT("Adaptive validation keeps the quadratic in the capsule union"),
		bFinalPointInsideUnion);
	TestTrue(TEXT("The unsafe full-strength quadratic guidance is constrained or braking"),
		FMath::Abs(Reference.VelocityCmPerSec.Y) < BowVelocityCmPerSec.Y - 0.01f
		|| Runtime.GetNavigationGuidanceStatus().State
			== EAircraftNavigationGuidanceState::Braking);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceStaleBrakeTest,
	"AircraftLab.Autopilot.NavigationGuidance.StaleGuidanceBrakesInsideCorridor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceStaleBrakeTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Path.CorridorSafetyMarginCm = 0.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	FAircraftSafeCorridorSegment& Corridor = Intent.Route.Corridor.AddDefaulted_GetRef();
	Corridor.AxisStartCm = FVector::ZeroVector;
	Corridor.AxisEndCm = FVector(5000.0f, 0.0f, 0.0f);
	Corridor.RadiusCm = 5.0f;
	Corridor.StartDistanceCm = 0.0f;
	Corridor.EndDistanceCm = 5000.0f;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The primary route is accepted"), Runtime.SetIntent(
		Intent, 82, 1, Config, State, Capability));

	// The raw guidance points across the narrow corridor. While fresh it is constrained
	// before publication; once stale it must not be reused by an unvalidated hold path.
	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 82;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 1.3;
	Guidance.Samples = {
		{ 0.0f, FVector::ZeroVector, FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 400.0f, 0.0f), FVector(0.0f, 400.0f, 0.0f), FVector::ZeroVector }
	};
	TestTrue(TEXT("The guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 21));

	State.TimeSeconds = 1.25;
	State.Sequence = 1;
	State.VelocityCmPerSec = FVector(400.0f, 0.0f, 0.0f);
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The steady guided reference is produced"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestTrue(TEXT("Fresh guidance is constrained to the corridor"),
		Runtime.GetPlan().ComputeCorridorViolationCm(Reference.PositionCm, 0.0f) <= 0.1f);

	State.TimeSeconds = 1.35;
	State.Sequence = 2;
	FAircraftTrajectoryReference BrakeReference;
	TestTrue(TEXT("A braking reference is produced immediately after guidance expiry"),
		Runtime.UpdateKinematic(State, Capability, BrakeReference));
	TestEqual(TEXT("Stale guidance reports braking state"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Braking);
	TestEqual(TEXT("Stale guidance reports the expired reason"),
		Runtime.GetNavigationGuidanceStatus().FailureReason,
		EAircraftNavigationGuidanceFailureReason::Expired);
	TestTrue(TEXT("Stale lateral guidance velocity is discarded"),
		FMath::IsNearlyZero(BrakeReference.VelocityCmPerSec.Y, 0.01f));
	TestTrue(TEXT("Immediate braking reference remains inside the corridor"),
		Runtime.GetPlan().ComputeCorridorViolationCm(BrakeReference.PositionCm, 0.0f) <= 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceRebaseTest,
	"AircraftLab.Autopilot.NavigationGuidance.RebaseKeepsGuidanceFreshAcrossPause",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceRebaseTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The primary route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 83, 1, Config, State, Capability));

	// 引导在 t=1.0 发布，有效期到 1.25。
	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 83;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 1.25;
	Guidance.Samples = {
		{ 0.0f, FVector(0.0f, 300.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 300.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector }
	};
	TestTrue(TEXT("The guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 30));

	// 暂停 2 秒后恢复：仿真时间跳到 3.0。无重基时引导必然 Expired；
	// RebaseTime 补偿后应仍按 Applied 消费。
	Runtime.RebaseTime(3.0);
	State.TimeSeconds = 3.0;
	State.Sequence = 1;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The guided reference is produced after rebase"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestEqual(TEXT("Rebased guidance stays applied across the pause"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Applied);
	TestEqual(TEXT("No failure reason is reported after rebase"),
		Runtime.GetNavigationGuidanceStatus().FailureReason,
		EAircraftNavigationGuidanceFailureReason::None);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftNavigationGuidanceIntentMismatchTest,
	"AircraftLab.Autopilot.NavigationGuidance.IntentMismatchCannotSteerNewTask",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftNavigationGuidanceIntentMismatchTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("The new primary route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 74, 2, Config, State, Capability));
	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 73;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 2.0;
	Guidance.Samples = {
		{ 0.0f, FVector(0.0f, 500.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector },
		{ 1.0f, FVector(0.0f, 500.0f, 0.0f), FVector::ZeroVector, FVector::ZeroVector }
	};
	TestTrue(TEXT("Structurally valid stale guidance is accepted for evaluation"),
		Runtime.SetNavigationGuidance(
			MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 13));

	State.TimeSeconds = 1.1;
	State.Sequence = 1;
	State.PositionCm = FVector(1200.0f, 0.0f, 100.0f);
	State.VelocityCmPerSec = FVector(400.0f, 0.0f, 0.0f);
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The mismatched guidance produces a fail-safe reference"),
		Runtime.UpdateKinematic(State, Capability, Reference));
	TestTrue(TEXT("Guidance generated for the old task cannot steer the new task"),
		FMath::IsNearlyZero(Reference.PositionCm.Y));
	TestEqual(TEXT("Intent mismatch enters the braking state"),
		Runtime.GetNavigationGuidanceStatus().State,
		EAircraftNavigationGuidanceState::Braking);
	TestEqual(TEXT("The mismatch is reported"),
		Runtime.GetNavigationGuidanceStatus().FailureReason,
		EAircraftNavigationGuidanceFailureReason::IntentMismatch);
	TestTrue(TEXT("The fail-safe decelerates from the actual velocity"),
		Reference.VelocityCmPerSec.X > 0.0f
		&& Reference.VelocityCmPerSec.X < State.VelocityCmPerSec.X
		&& Reference.AccelerationCmPerSecSq.X < 0.0f);
	TestTrue(TEXT("The braking reference is integrated from the actual position"),
		Reference.PositionCm.X > State.PositionCm.X
		&& Reference.PositionCm.X < State.PositionCm.X + 50.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftMpccProjectionRecoveryTest,
	"AircraftLab.Autopilot.MPCC.TeleportRebasesProjectionHorizon",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftMpccProjectionRecoveryTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.Sequence = 1;
	FAircraftMpccController Controller;
	TestTrue(TEXT("Long route is accepted"), Controller.SetIntent(
		MakeRouteIntent(10000.0f), 204, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	State.TimeSeconds = 1.01;
	State.Sequence = 2;
	TestTrue(TEXT("Initial horizon is solved"),
		Controller.Update(State, Capability, Reference));

	State.PositionCm = FVector(8000.0, 0.0, 0.0);
	State.TimeSeconds = 1.05;
	State.Sequence = 3;
	TestTrue(TEXT("A teleported state still produces a reference"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("The stale local horizon is rebased near the recovered path point"),
		Reference.PathProgress > 0.7f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotActorTargetReplanHysteresisTest,
	"AircraftLab.Autopilot.Replan.ActorTargetMoveHysteresis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotActorTargetReplanHysteresisTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	// 滞回：位移在容差内不 bump。
	TestFalse(TEXT("Movement within the hysteresis distance does not bump"),
		UAutopilotComponent::ShouldBumpRevisionForActorTargetMove(
			true, 24.0, 10.0, 25.0, 0.25));
	// 首次解析只建立已解析 Actor 位置基线，不产生额外 revision。
	TestFalse(TEXT("The first actor target resolution only establishes its baseline"),
		UAutopilotComponent::ShouldBumpRevisionForActorTargetMove(
			false, 200.0, 10.0, 25.0, 0.25));
	// 限频：超容差但间隔未到，不 bump。
	TestFalse(TEXT("Movement beyond the distance but within the interval does not bump"),
		UAutopilotComponent::ShouldBumpRevisionForActorTargetMove(
			true, 30.0, 0.1, 25.0, 0.25));
	// 常规更新：超容差且间隔已到，bump。
	TestTrue(TEXT("Movement beyond the distance after the interval bumps"),
		UAutopilotComponent::ShouldBumpRevisionForActorTargetMove(
			true, 30.0, 0.3, 25.0, 0.25));
	// 快速目标/瞬移：超 4× 容差距离，无视限频立即 bump。
	TestTrue(TEXT("A teleport-scale jump bumps immediately regardless of the interval"),
		UAutopilotComponent::ShouldBumpRevisionForActorTargetMove(
			true, 200.0, 0.01, 25.0, 0.25));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotCapabilityToleranceTest,
	"AircraftLab.Autopilot.MPCC.CapabilityTolerancePreventsRebuild",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotCapabilityToleranceTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;

	FAircraftMpccController Controller;
	TestTrue(TEXT("The route intent builds an initial plan"), Controller.SetIntent(
		MakeRouteIntent(5000.0f), 7, 1, Config, State, Capability));
	const uint64 InitialPlanRevision = Controller.GetDiagnostics().PlanRevision;
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("The first update produces a reference"),
		Controller.Update(State, Capability, Reference));
	const uint64 RevisionAfterFirstUpdate = Controller.GetDiagnostics().PlanRevision;

	// ULP 级抖动（远小于容差）：不得触发计划重建（PlanRevision 不变）。
	FAircraftDynamicCapabilitySnapshot Jittered = Capability;
	Jittered.MassKg = Capability.MassKg * (1.0f + 1.0e-6f);
	Jittered.MaxHorizontalAccelerationCmPerSecSq =
		Capability.MaxHorizontalAccelerationCmPerSecSq * (1.0f - 1.0e-6f);
	State.TimeSeconds += 10.0;
	TestTrue(TEXT("The jittered update still produces a reference"),
		Controller.Update(State, Jittered, Reference));
	TestEqual(TEXT("ULP-scale capability jitter does not rebuild the plan"),
		Controller.GetDiagnostics().PlanRevision, RevisionAfterFirstUpdate);
	(void)InitialPlanRevision;

	// 真实能力变化（超容差）：必须触发重建（PlanRevision 递增）。
	FAircraftDynamicCapabilitySnapshot Changed = Capability;
	Changed.MaxHorizontalAccelerationCmPerSecSq =
		Capability.MaxHorizontalAccelerationCmPerSecSq * 0.5f;
	State.TimeSeconds += 10.0;
	TestTrue(TEXT("The changed capability update still produces a reference"),
		Controller.Update(State, Changed, Reference));
	TestTrue(TEXT("A real capability change rebuilds the plan"),
		Controller.GetDiagnostics().PlanRevision > RevisionAfterFirstUpdate);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotIntentHashingTest,
	"AircraftLab.Autopilot.MPCC.IntentGeometryAndMetadataHashes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotIntentHashingTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftMpccController Controller;
	FAircraftMovementIntent Intent = MakeRouteIntent(5000.0f);
	TestTrue(TEXT("Initial route builds"), Controller.SetIntent(
		Intent, 90, 1, Config, State, Capability));
	const uint64 InitialPlanRevision = Controller.GetDiagnostics().PlanRevision;

	Intent.Completion.HorizontalToleranceCm = 125.0f;
	Intent.Completion.HorizontalSpeedToleranceCmPerSec = 15.0f;
	Intent.Completion.VerticalSpeedToleranceCmPerSec = 5.0f;
	Intent.TimeoutSeconds = 8.0f;
	TestTrue(TEXT("Metadata-only update is accepted"), Controller.SetIntent(
		Intent, 90, 2, Config, State, Capability));
	TestEqual(TEXT("Tolerances, stability policy, and timeout do not rebuild the plan"),
		Controller.GetDiagnostics().PlanRevision, InitialPlanRevision);
	TestEqual(TEXT("Metadata update still publishes the new intent revision"),
		Controller.GetDiagnostics().IntentRevision, uint64(2));

	Intent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	Intent.Heading.FixedYawDegrees = 75.0f;
	TestTrue(TEXT("Heading update is accepted"), Controller.SetIntent(
		Intent, 90, 3, Config, State, Capability));
	const uint64 HeadingRevision = Controller.GetDiagnostics().PlanRevision;
	TestTrue(TEXT("Heading changes rebuild route timing and yaw samples"),
		HeadingRevision > InitialPlanRevision);

	Intent.Completion.ArrivalMode = EAircraftArrivalMode::PassThrough;
	TestTrue(TEXT("Arrival-mode update is accepted"), Controller.SetIntent(
		Intent, 90, 4, Config, State, Capability));
	const uint64 ArrivalModeRevision = Controller.GetDiagnostics().PlanRevision;
	TestTrue(TEXT("Arrival mode rebuilds the terminal speed envelope"),
		ArrivalModeRevision > HeadingRevision);

	Intent.Completion.TerminalHorizontalSpeedCmPerSec = 80.0f;
	Intent.Completion.TerminalVerticalSpeedCmPerSec = 25.0f;
	TestTrue(TEXT("Terminal-speed update is accepted"), Controller.SetIntent(
		Intent, 90, 5, Config, State, Capability));
	const uint64 TerminalSpeedRevision = Controller.GetDiagnostics().PlanRevision;
	TestTrue(TEXT("Terminal speeds rebuild the speed envelope"),
		TerminalSpeedRevision > ArrivalModeRevision);

	Intent.Route.PointsCm.Last().Y = 250.0f;
	TestTrue(TEXT("Geometry update is accepted"), Controller.SetIntent(
		Intent, 90, 6, Config, State, Capability));
	const uint64 GeometryRevision = Controller.GetDiagnostics().PlanRevision;
	TestTrue(TEXT("Route geometry rebuilds the plan"), GeometryRevision > TerminalSpeedRevision);

	Intent.Limits.MaxAccelerationCmPerSecSq *= 0.5f;
	TestTrue(TEXT("Motion-limit update is accepted"), Controller.SetIntent(
		Intent, 90, 7, Config, State, Capability));
	TestTrue(TEXT("Requested motion limits rebuild the plan"),
		Controller.GetDiagnostics().PlanRevision > GeometryRevision);

	FAircraftMovementIntent UnboundedIntent = Intent;
	UnboundedIntent.bHasRequestedMotionLimits = false;
	TestTrue(TEXT("Removing requested limits rebuilds the plan"), Controller.SetIntent(
		UnboundedIntent, 90, 8, Config, State, Capability));
	const uint64 UnboundedRevision = Controller.GetDiagnostics().PlanRevision;
	UnboundedIntent.Limits.MaxAccelerationCmPerSecSq *= 0.5f;
	TestTrue(TEXT("Unused limit payload update is accepted"), Controller.SetIntent(
		UnboundedIntent, 90, 9, Config, State, Capability));
	TestEqual(TEXT("Unused limit payload does not rebuild the plan"),
		Controller.GetDiagnostics().PlanRevision, UnboundedRevision);

	FAircraftMovementIntent InvalidMetadataIntent = UnboundedIntent;
	InvalidMetadataIntent.Completion.VerticalToleranceCm = -1.0f;
	TestFalse(TEXT("Invalid metadata cannot bypass full plan validation"),
		Controller.SetIntent(InvalidMetadataIntent, 90, 10, Config, State, Capability));

	FAircraftMovementIntent TimedIntent;
	TimedIntent.Type = EAircraftMovementIntentType::TimedTrajectory;
	FAircraftTimedTrajectorySample TimedStart;
	TimedStart.YawDegrees = 15.0f;
	TimedStart.YawRateDegPerSec = 20.0f;
	FAircraftTimedTrajectorySample TimedEnd;
	TimedEnd.TimeSeconds = 1.0f;
	TimedEnd.PositionCm = FVector(100.0, 0.0, 0.0);
	TimedEnd.YawDegrees = 80.0f;
	TimedEnd.YawRateDegPerSec = 30.0f;
	TimedIntent.TimedTrajectory.Samples = { TimedStart, TimedEnd };
	FAircraftMotionPlan TimedPlan;
	TestTrue(TEXT("Authored timed trajectory builds"),
		TimedPlan.Build(TimedIntent, Config, State, Capability));
	TimedIntent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	TimedIntent.Heading.FixedYawDegrees = -120.0f;
	TimedIntent.Completion.HorizontalToleranceCm = 40.0f;
	TimedIntent.TimeoutSeconds = 5.0f;
	TimedPlan.UpdateMetadata(TimedIntent);
	FAircraftMotionPlanSample AuthoredSample;
	TestTrue(TEXT("Timed trajectory remains evaluable after metadata update"),
		TimedPlan.Evaluate(1.0f, AuthoredSample));
	TestTrue(TEXT("Metadata updates preserve authored timed-trajectory yaw"),
		FMath::IsNearlyEqual(AuthoredSample.YawDegrees, 80.0f));
	TestTrue(TEXT("Metadata updates preserve authored timed-trajectory yaw rate"),
		FMath::IsNearlyEqual(AuthoredSample.YawRateDegPerSec, 30.0f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftSteadyGuidanceSkipsMpccTest,
	"AircraftLab.Autopilot.NavigationGuidance.SteadySkipsMpccOptimization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftSteadyGuidanceSkipsMpccTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	const FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	FAircraftTrajectoryRuntime Runtime;
	TestTrue(TEXT("Route is accepted"), Runtime.SetIntent(
		MakeRouteIntent(5000.0f), 91, 1, Config, State, Capability));

	FAircraftNavigationGuidance Guidance;
	Guidance.SourceIntentId = 91;
	Guidance.SourceIntentRevision = 1;
	Guidance.GeneratedAtSeconds = 1.0;
	Guidance.ValidUntilSeconds = 3.0;
	Guidance.Samples = {
		{ 0.0f, FVector::ZeroVector, FVector(300.0f, 0.0f, 0.0f), FVector::ZeroVector },
		{ 2.0f, FVector(600.0f, 0.0f, 0.0f), FVector(300.0f, 0.0f, 0.0f), FVector::ZeroVector }
	};
	TestTrue(TEXT("Guidance is accepted"), Runtime.SetNavigationGuidance(
		MakeShared<FAircraftNavigationGuidance, ESPMode::ThreadSafe>(Guidance), 30));
	FAircraftTrajectoryReference Reference;
	State.TimeSeconds = 1.0;
	State.Sequence = 1;
	TestTrue(TEXT("Guidance begins ramping"),
		Runtime.UpdateFlightController(State, Capability, Reference));
	State.TimeSeconds = 1.21;
	State.Sequence = 2;
	TestTrue(TEXT("Guidance reaches steady state"),
		Runtime.UpdateFlightController(State, Capability, Reference));
	const double LastSolveMilliseconds = Runtime.GetDiagnostics().LastSolveMilliseconds;
	State.TimeSeconds = 1.31;
	State.Sequence = 3;
	State.PositionCm += FVector(30.0f, 0.0f, 0.0f);
	TestTrue(TEXT("Steady guidance keeps publishing references"),
		Runtime.UpdateFlightController(State, Capability, Reference));
	TestEqual(TEXT("Steady guidance does not run a discarded MPCC solve"),
		Runtime.GetDiagnostics().LastSolveMilliseconds, LastSolveMilliseconds);
	TestTrue(TEXT("Steady guidance still advances path progress"), Reference.PathProgress > 0.0f);
	return true;
}

#endif
