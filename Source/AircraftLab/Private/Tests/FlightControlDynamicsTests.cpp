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
