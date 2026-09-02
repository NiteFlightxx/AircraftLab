#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "AircraftAutopilot/AircraftMpccController.h"
#include "AircraftAutopilot/AircraftTrajectoryRuntime.h"
#include "AircraftAutopilot/AircraftSpatialPath.h"
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
	TestTrue(TEXT("Path satisfying a convex safe corridor builds"),
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
	FAircraftMpccController Controller;

	TestTrue(TEXT("Intent remains valid without physical yaw authority"),
		Controller.SetIntent(Intent, 21, 1, Config, State, Capability));
	TestEqual(TEXT("Motion plan publishes zero effective yaw-rate limit"),
		Controller.GetPlan().GetIntent().Limits.MaxYawRateDegPerSec, 0.0f);
	FAircraftTrajectoryReference Reference;
	TestTrue(TEXT("Translation reference remains solvable without yaw authority"),
		Controller.Update(State, Capability, Reference));
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
	FAircraftYawReferenceRemainsAnchoredToMeasuredHeadingTest,
	"AircraftAutopilot.MPCC.YawReferenceRemainsAnchoredToMeasuredHeading",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawReferenceRemainsAnchoredToMeasuredHeadingTest::RunTest(const FString& Parameters)
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
	FAircraftMpccController Controller;
	TestTrue(TEXT("Yaw-governed intent is accepted"),
		Controller.SetIntent(Intent, 42, 1, Config, State, Capability));
	FAircraftTrajectoryReference Reference;
	for (int32 Index = 0; Index < 20; ++Index)
	{
		State.TimeSeconds += 1.0 / Config.Mpcc.UpdateRateHz + 0.001;
		++State.Sequence;
		TestTrue(TEXT("Yaw-governed reference is solved"),
			Controller.Update(State, Capability, Reference));
	}
	TestTrue(TEXT("Yaw reference cannot run away from a stationary measured heading"),
		FMath::Abs(Reference.YawDegrees) < 5.0f);
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
	FAircraftYawReferenceSurvivesSameIntentUpdatesTest,
	"AircraftAutopilot.MPCC.YawReferenceSurvivesSameIntentUpdates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawReferenceSurvivesSameIntentUpdatesTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftMovementIntent Intent;
	Intent.Type = EAircraftMovementIntentType::Hold;
	Intent.Hold.bCaptureCurrentPosition = false;
	Intent.Hold.PositionCm = FVector::ZeroVector;
	Intent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	FAircraftAutopilotRuntimeConfig Config;
	Config.Mpcc.SolveTimeBudgetMilliseconds = 100.0f;
	FAircraftVehicleStateSnapshot State;
	State.TimeSeconds = 1.0;
	State.ControlRotation = FQuat::Identity;
	State.BodyRotation = FQuat::Identity;
	FAircraftDynamicCapabilitySnapshot Capability = MakeCapability();
	FAircraftMpccController Controller;
	FAircraftTrajectoryReference Reference;
	const float DeltaTime = 1.0f / Config.Mpcc.UpdateRateHz;

	for (uint64 Revision = 1; Revision <= 30; ++Revision)
	{
		Intent.Heading.FixedYawDegrees = FRotator::NormalizeAxis(
			90.0f * DeltaTime * static_cast<float>(Revision));
		TestTrue(TEXT("Updated manual yaw intent remains accepted"),
			Controller.SetIntent(Intent, 50, Revision, Config, State, Capability));
		TestTrue(TEXT("Updated manual yaw reference remains solvable"),
			Controller.Update(State, Capability, Reference));
		State.TimeSeconds += DeltaTime;
		++State.Sequence;
	}

	TestTrue(TEXT("Same-handle revisions retain yaw acceleration continuity"),
		Reference.YawAccelerationDegPerSecSq > 100.0f);
	TestTrue(TEXT("Yaw-rate reference can build before the rigid body responds"),
		Reference.YawRateDegPerSec > 20.0f);
	TestTrue(TEXT("Yaw reference lead remains bounded while the rigid body is stationary"),
		FMath::Abs(Reference.YawDegrees)
			<= Intent.Limits.MaxYawRateDegPerSec * Config.Mpcc.YawResponseTimeSeconds
				+ UE_KINDA_SMALL_NUMBER);

	State.TimeSeconds += DeltaTime;
	++State.Sequence;
	Intent.Heading.FixedYawDegrees = -90.0f;
	TestTrue(TEXT("A new yaw command handle is accepted"),
		Controller.SetIntent(Intent, 51, 1, Config, State, Capability));
	TestTrue(TEXT("A new yaw command handle is solvable"),
		Controller.Update(State, Capability, Reference));
	TestTrue(TEXT("A new handle starts a fresh yaw jerk profile"),
		FMath::Abs(Reference.YawAccelerationDegPerSecSq)
			<= Intent.Limits.MaxYawJerkDegPerSecCubed * DeltaTime
				+ UE_KINDA_SMALL_NUMBER);
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
	TestTrue(TEXT("Release decelerates instead of stopping instantly"),
		BrakingReference.VelocityCmPerSec.X > 0.0f
		&& BrakingReference.VelocityCmPerSec.X
			<= AcceleratingReference.VelocityCmPerSec.X);
	TestTrue(TEXT("Braking never reverses the deterministic reference"),
		ContinuedBrakingReference.VelocityCmPerSec.X >= 0.0f);
	return true;
}

#endif
