#include "FlightControllerRuntimeObjects.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftLinearDampingFeedForwardTest,
	"AircraftLab.Control.Velocity.LinearDampingFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftLinearDampingFeedForwardTest::RunTest(const FString& Parameters)
{
	const FVector DesiredVelocity(800.0, -400.0, 125.0);
	const FVector FeedForward = FlightControlDynamics::ComputeLinearDampingFeedForward(
		DesiredVelocity, 0.3f, 1.0f);

	TestTrue(TEXT("Feedforward opposes modeled drag at the desired horizontal velocity"),
		FeedForward.Equals(FVector(240.0, -120.0, 0.0), 1.e-4));
	TestTrue(TEXT("Zero damping produces zero feedforward"),
		FlightControlDynamics::ComputeLinearDampingFeedForward(DesiredVelocity, 0.0f, 1.0f).IsNearlyZero());
	TestTrue(TEXT("Negative scale is safely clamped to zero"),
		FlightControlDynamics::ComputeLinearDampingFeedForward(DesiredVelocity, 0.3f, -1.0f).IsNearlyZero());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVelocityPidMaintainsTargetSpeedTest,
	"AircraftLab.Control.Velocity.ZeroErrorUsesDynamicsFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVelocityPidMaintainsTargetSpeedTest::RunTest(const FString& Parameters)
{
	FControllerRuntimeState Runtime;
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector(800.0, 0.0, 0.0);
	FPhysicsCache PhysicsCache;
	PhysicsCache.LinearDampingPerSecond = 0.3f;
	FModeCapabilities Capabilities;
	FFlightControllerRuntimeConfig Config;
	FlightControllerConfig::InitializeDefaults(Config.Controller);
	FAutopilotMovementIntent MovementIntent;
	FAutopilotInjection Injection;
	FControlAllocator Allocator;
	FFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, MovementIntent, Injection, Allocator, false };
	FFlightControlSolver Solver;

	const FVector Acceleration = Solver.ComputeVelocityPidAcceleration(
		Context, FVector(800.0, 0.0, 0.0), FVector::ZeroVector, 0.004f);
	TestEqual(TEXT("At zero speed error the controller still compensates linear damping"),
		Acceleration.X, 240.0, 1.e-3);
	TestEqual(TEXT("No lateral target produces no lateral acceleration"),
		Acceleration.Y, 0.0, 1.e-3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVerticalDampingFeedForwardTest,
	"AircraftLab.Control.Damping.VerticalCollectiveFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVerticalDampingFeedForwardTest::RunTest(const FString& Parameters)
{
	const float FeedForward = FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
		200.0f, 0.3f, 1000.0f, 0.5f, 1.0f);
	TestEqual(TEXT("Climb damping is converted from acceleration to collective"),
		FeedForward, 0.03f, 1.e-5f);
	TestEqual(TEXT("Descent damping reduces collective with the opposite sign"),
		FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
			-200.0f, 0.3f, 1000.0f, 0.5f, 1.0f), -0.03f, 1.e-5f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVerticalVelocitySetpointSlewTest,
	"AircraftLab.Control.Altitude.VerticalVelocitySetpointSlewAccumulates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVerticalVelocitySetpointSlewTest::RunTest(const FString& Parameters)
{
	FControllerRuntimeState Runtime;
	Runtime.EstimatedState.State.PositionCm.Z = 0.0f;
	Runtime.EstimatedState.State.VelocityCmPerSec.Z = -100.0f;
	Runtime.HoldTargets.HeldAltitudeCm = 1000.0f;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	FPhysicsCache PhysicsCache;
	PhysicsCache.GravityMagnitudeCmPerSecSq = 980.0f;
	FModeCapabilities Capabilities;
	Capabilities.CanHoldAltitude = true;
	FFlightControllerRuntimeConfig Config;
	FlightControllerConfig::InitializeDefaults(Config.Controller);
	Config.Controller.Limits.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
	FAutopilotMovementIntent MovementIntent;
	FAutopilotInjection Injection;
	FControlAllocator Allocator;
	FFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, MovementIntent, Injection, Allocator, false };
	FFlightControlSolver Solver;

	float FirstDesiredVelocity = 0.0f;
	Solver.ComputeVerticalControl(Context, 0.004f, FirstDesiredVelocity);
	float SecondDesiredVelocity = 0.0f;
	Solver.ComputeVerticalControl(Context, 0.004f, SecondDesiredVelocity);

	TestEqual(TEXT("First step starts from measured velocity and respects acceleration"),
		FirstDesiredVelocity, -96.0f, 1.e-4f);
	TestEqual(TEXT("Subsequent steps accumulate from the previous setpoint, not measured velocity"),
		SecondDesiredVelocity, -92.0f, 1.e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAngularDampingFeedForwardTest,
	"AircraftLab.Control.Damping.AngularTorqueFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAngularDampingFeedForwardTest::RunTest(const FString& Parameters)
{
	const FVector FeedForward = FlightControlDynamics::ComputeAngularDampingFeedForward(
		FVector(90.0f, -180.0f, 0.0f), 2.0f,
		FVector(1.0f, 2.0f, 3.0f), FVector(10.0f, 10.0f, 10.0f),
		FVector(8.0f, 8.0f, 8.0f), 1.0f);
	TestEqual(TEXT("Positive rate uses positive torque authority"),
		FeedForward.X, static_cast<double>(PI) / 10.0, 1.e-5);
	TestEqual(TEXT("Negative rate uses negative torque authority"),
		FeedForward.Y, -static_cast<double>(PI) / 2.0, 1.e-5);
	TestTrue(TEXT("Zero requested rate requires no damping torque"),
		FMath::IsNearlyZero(FeedForward.Z));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftQuaternionAttitudeUsesRigidBodyRotationTest,
	"AircraftLab.Control.Attitude.QuaternionUsesRigidBodyRotation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftQuaternionAttitudeUsesRigidBodyRotationTest::RunTest(const FString& Parameters)
{
	FControllerRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	// Deliberately stale display angles: the controller must use the rigid-body quaternion.
	Runtime.EstimatedState.State.AttitudeDegrees = FRotator(0.0f, 0.0f, 90.0f);
	FPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FQuat::Identity);
	FModeCapabilities Capabilities;
	FFlightControllerRuntimeConfig Config;
	FlightControllerConfig::InitializeDefaults(Config.Controller);
	FAutopilotMovementIntent MovementIntent;
	FAutopilotInjection Injection;
	FControlAllocator Allocator;
	FFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, MovementIntent, Injection, Allocator, false };
	FFlightControlSolver Solver;
	FFlightControlYawSetpoint YawSetpoint;
	YawSetpoint.MaxRateDegPerSec = Config.Controller.Limits.MaxYawRateDegreesPerSec;

	const FVector DesiredRates = Solver.ComputeDesiredBodyRates(
		Context, FRotator::ZeroRotator, YawSetpoint, 0.004f);
	TestTrue(TEXT("Identity rigid-body attitude has no Roll/Pitch quaternion error"),
		FVector2D(DesiredRates.X, DesiredRates.Y).IsNearlyZero(1.e-4f));

	YawSetpoint.TargetYawDegrees = 90.0f;
	const FVector YawRates = Solver.ComputeDesiredBodyRates(
		Context, FRotator::ZeroRotator, YawSetpoint, 0.004f);
	TestTrue(TEXT("Target yaw is closed by the quaternion error, not a separate angle PID"),
		FMath::Abs(YawRates.Z) > 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDampingAwareHorizontalLimitsTest,
	"AircraftLab.Control.Damping.HorizontalMotionAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDampingAwareHorizontalLimitsTest::RunTest(const FString& Parameters)
{
	const FlightControlDynamics::FDampingAwareHorizontalLimits Limited =
		FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
			800.0f, 500.0f, 1.0f, 0.2f);
	TestEqual(TEXT("Cruise speed is capped before drag consumes reserved authority"),
		Limited.MaxSpeedCmPerSec, 400.0f, 1.e-4f);
	TestEqual(TEXT("Trajectory retains the configured acceleration reserve"),
		Limited.MaxTrajectoryAccelerationCmPerSecSq, 100.0f, 1.e-4f);

	const FlightControlDynamics::FDampingAwareHorizontalLimits NoDamping =
		FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
			800.0f, 500.0f, 0.0f, 0.2f);
	TestEqual(TEXT("Without damping the requested speed remains available"),
		NoDamping.MaxSpeedCmPerSec, 800.0f, 1.e-4f);
	TestEqual(TEXT("Without damping all acceleration remains available"),
		NoDamping.MaxTrajectoryAccelerationCmPerSecSq, 500.0f, 1.e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftManualReleaseBrakesBeforeHoldingTest,
	"AircraftLab.Control.PositionHold.ManualReleaseBrakesBeforeHolding",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftManualReleaseBrakesBeforeHoldingTest::RunTest(const FString& Parameters)
{
	FControllerRuntimeState Runtime;
	Runtime.EstimatedState.State.PositionCm = FVector(100.0f, 0.0f, 0.0f);
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector(200.0f, 0.0f, 0.0f);
	Runtime.HoldTargets.HeldPositionCm = FVector::ZeroVector;
	Runtime.HoldTargets.bPositionHoldInitialized = true;
	Runtime.HoldTargets.bHorizontalBrakeBeforeHold = true;
	FPhysicsCache PhysicsCache;
	FModeCapabilities Capabilities;
	Capabilities.CanUsePositionControl = true;
	Capabilities.CanUseVelocityControl = true;
	FFlightControllerRuntimeConfig Config;
	FlightControllerConfig::InitializeDefaults(Config.Controller);
	Config.Input.HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	FAutopilotMovementIntent MovementIntent;
	FAutopilotInjection Injection;
	FControlAllocator Allocator;
	FFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, MovementIntent, Injection, Allocator, false };
	FFlightControlSolver Solver;

	Solver.ComputeDesiredHorizontalAcceleration(Context, 0.004f);
	TestTrue(TEXT("Release braking keeps the hold anchor on the moving aircraft"),
		Runtime.HoldTargets.HeldPositionCm.Equals(Runtime.EstimatedState.State.PositionCm));
	TestTrue(TEXT("Release braking commands zero horizontal velocity"),
		Solver.LastDesiredHorizontalVelocityCmPerSec.IsNearlyZero());
	TestTrue(TEXT("Position hold remains deferred while the aircraft is moving"),
		Runtime.HoldTargets.bHorizontalBrakeBeforeHold);

	Runtime.EstimatedState.State.PositionCm = FVector(180.0f, 0.0f, 0.0f);
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector(10.0f, 0.0f, 0.0f);
	Solver.ComputeDesiredHorizontalAcceleration(Context, 0.004f);
	TestFalse(TEXT("Low speed latches the final hold position"),
		Runtime.HoldTargets.bHorizontalBrakeBeforeHold);
	TestTrue(TEXT("Final hold position is where braking actually finished"),
		Runtime.HoldTargets.HeldPositionCm.Equals(FVector(180.0f, 0.0f, 0.0f)));

	Runtime.EstimatedState.State.PositionCm = FVector(230.0f, 0.0f, 0.0f);
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector::ZeroVector;
	Solver.ComputeDesiredHorizontalAcceleration(Context, 0.004f);
	TestTrue(TEXT("After latching, external drift is corrected back to the stop point"),
		Solver.LastDesiredHorizontalVelocityCmPerSec.X < 0.0f);
	return true;
}

#endif
