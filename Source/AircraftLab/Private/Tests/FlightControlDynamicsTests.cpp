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
	FAutopilotInjection Injection;
	FControlAllocator Allocator;
	FFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, Injection, Allocator, false };
	FFlightControlSolver Solver;

	const FVector Acceleration = Solver.ComputeVelocityPidAcceleration(
		Context, FVector(800.0, 0.0, 0.0), FVector::ZeroVector, 0.004f);
	TestEqual(TEXT("At zero speed error the controller still compensates linear damping"),
		Acceleration.X, 240.0, 1.e-3);
	TestEqual(TEXT("No lateral target produces no lateral acceleration"),
		Acceleration.Y, 0.0, 1.e-3);
	return true;
}

#endif
