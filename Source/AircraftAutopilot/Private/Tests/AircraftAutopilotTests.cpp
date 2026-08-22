#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "AircraftAutopilot/AircraftPredictiveController.h"
#include "AircraftAutopilot/AircraftSpatialPath.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FAircraftDynamicCapabilitySnapshot MakeCapability()
	{
		FAircraftDynamicCapabilitySnapshot Result;
		Result.MassKg = 100.0f;
		Result.MaxHorizontalSpeedCmPerSec = 2000.0f;
		Result.MaxHorizontalAccelerationCmPerSecSq = 1000.0f;
		Result.MaxVerticalAccelerationCmPerSecSq = 800.0f;
		Result.MaxClimbRateCmPerSec = 600.0f;
		Result.MaxDescentRateCmPerSec = 500.0f;
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

	FAircraftMovementIntentSettings Settings;
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
	TestTrue(TEXT("Middle knot projects"), Path.Project(Route.PointsCm[1], 1000.0f, Knot));
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
	Corridor.StartDistanceCm = 0.0f;
	Corridor.EndDistanceCm = 1200.0f;
	Corridor.BoundaryPlanes = {
		FPlane(FVector(0.0, 100.0, 0.0), FVector::RightVector),
		FPlane(FVector(0.0, -100.0, 0.0), -FVector::RightVector),
		FPlane(FVector(0.0, 0.0, 100.0), FVector::UpVector),
		FPlane(FVector(0.0, 0.0, -100.0), -FVector::UpVector)
	};
	FAircraftSpatialPath CorridorPath;
	TestTrue(TEXT("Path satisfying a convex safe corridor builds"),
		CorridorPath.Build(CorridorRoute, Config));
	for (float Distance = 0.0f; Distance <= CorridorPath.GetLengthCm(); Distance += 10.0f)
	{
		FAircraftSpatialPathState Sample;
		CorridorPath.Evaluate(Distance, Sample);
		for (const FPlane& Plane : Corridor.BoundaryPlanes)
		{
			TestTrue(TEXT("Every sampled path point respects the corridor margin"),
				Plane.PlaneDot(Sample.PositionCm) + Config.CorridorSafetyMarginCm
				<= Config.ConvergenceToleranceCm);
		}
	}
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
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.Sequence = 10;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	Capability.bHasExplicitAerodynamics = true;
	Capability.AirDensityKgPerM3 = 1.225f;
	Capability.LinearDragBodyNsPerM = FVector(10.0, 10.0, 10.0);
	Capability.DragAreaCoefficientBodyM2 = FVector(0.2, 0.2, 0.2);

	FAircraftPredictiveController Controller;
	TestTrue(TEXT("Velocity intent is accepted"),
		Controller.SetIntent(Intent, 7, 3, Config, State, Capability));
	FAircraftTrajectoryReference First;
	TestTrue(TEXT("A fresh reference is solved"), Controller.Update(State, Capability, First));
	TestTrue(TEXT("Reference carries identity"),
		First.bValid && First.IntentId == 7 && First.IntentRevision == 3);
	TestTrue(TEXT("Velocity command is preserved"),
		First.VelocityCmPerSec.Equals(Intent.Velocity.VelocityCmPerSec, 0.1f));
	TestTrue(TEXT("Acceleration remains inside requested horizontal authority"),
		FVector2D(First.AccelerationCmPerSecSq.X, First.AccelerationCmPerSecSq.Y).Size() <= 400.1f);
	TestTrue(TEXT("Aerodynamic feed-forward acts in the commanded direction"),
		First.AccelerationCmPerSecSq.X > 0.0f);

	State.TimeSeconds += 0.001;
	State.Sequence = 11;
	FAircraftTrajectoryReference Reused;
	TestTrue(TEXT("Fresh reference is reusable between solver updates"),
		Controller.Update(State, Capability, Reused));
	TestEqual(TEXT("Reused reference retains originating state sequence"),
		Reused.StateSequence, First.StateSequence);
	TestTrue(TEXT("Reference expires at its declared deadline"),
		!First.IsFresh(First.ValidUntilSeconds + 0.001));
	return true;
}

#endif
