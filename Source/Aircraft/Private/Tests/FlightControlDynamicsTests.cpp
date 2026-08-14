// 适配点：嵌套 Profile 配置 → 平铺 FAircraftFlightControllerRuntimeConfig（自带权威默认值）；
// FAutopilotMovementIntent → FAircraftManualCommand；类型前缀 FFlightControl* → FAircraftFlightControl*。

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/ControlAllocator.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftConstraintDynamicsFeedForwardTest,
	"AircraftLab.Control.Constraint.DynamicsFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftConstraintDynamicsFeedForwardTest::RunTest(const FString& Parameters)
{
	const double StrengthHz = 2.0;
	const double Stiffness = FMath::Square(StrengthHz * UE_DOUBLE_TWO_PI);
	const FVector HoverOffset =
		UE::AircraftLab::ConstraintDrive::ComputeDynamicsFeedForwardPositionOffset(
			FVector(0.0, 0.0, -980.0), FVector::ZeroVector,
			0.3, 1.0, StrengthHz, true, 100.0);
	TestEqual(TEXT("Acceleration drive offsets its target upward to cancel gravity"),
		HoverOffset.Z, 980.0 / Stiffness, 1.e-5);

	const FVector ClimbOffset =
		UE::AircraftLab::ConstraintDrive::ComputeDynamicsFeedForwardPositionOffset(
			FVector(0.0, 0.0, -980.0), FVector(0.0, 0.0, 100.0),
			0.3, 1.0, StrengthHz, true, 100.0);
	const FVector DescentOffset =
		UE::AircraftLab::ConstraintDrive::ComputeDynamicsFeedForwardPositionOffset(
			FVector(0.0, 0.0, -980.0), FVector(0.0, 0.0, -100.0),
			0.3, 1.0, StrengthHz, true, 100.0);
	TestTrue(TEXT("Climb feed-forward also overcomes linear damping"), ClimbOffset.Z > HoverOffset.Z);
	TestTrue(TEXT("Descent feed-forward removes the gravity asymmetry without opposing descent damping"),
		DescentOffset.Z < HoverOffset.Z && DescentOffset.Z > 0.0);

	const FVector ForceModeOffset =
		UE::AircraftLab::ConstraintDrive::ComputeDynamicsFeedForwardPositionOffset(
			FVector(0.0, 0.0, -980.0), FVector::ZeroVector,
			0.3, 1.0, StrengthHz, false, 100.0);
	TestEqual(TEXT("Force drive converts acceleration feed-forward using body mass"),
		ForceModeOffset.Z, HoverOffset.Z * 100.0, 1.e-4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDefaultBodyAxesTest,
	"AircraftLab.Control.Axes.DefaultForwardIsPositiveY",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDefaultBodyAxesTest::RunTest(const FString& Parameters)
{
	const FAircraftFlightControllerRuntimeConfig Config;

	TestEqual(TEXT("Default model forward axis is +Y (1)"),
		Config.ForwardAxis, static_cast<uint8>(1));
	TestTrue(TEXT("Configured forward vector is model-local +Y"),
		Config.GetForwardAxisBody().Equals(FVector::RightVector, 1.e-4f));
	TestTrue(TEXT("Configured right vector is model-local -X"),
		Config.GetRightAxisBody().Equals(-FVector::ForwardVector, 1.e-4f));
	TestTrue(TEXT("Identity model rotation exposes a +90 degree control heading"),
		FMath::IsNearlyEqual(
			Config.GetControlWorldRotation(FQuat::Identity).Rotator().Yaw,
			90.0f,
			1.e-4f));

	const FVector ControllerTorque(1.0f, 2.0f, 3.0f);
	const FVector PhysicalBodyTorque = Config.ControllerTorqueToBody(ControllerTorque);
	TestTrue(TEXT("Controller torque round-trips through the configured body axes"),
		Config.BodyTorqueToController(PhysicalBodyTorque).Equals(ControllerTorque, 1.e-4f));
	const FQuat DesiredControlWorld = FRotator(0.0f, 35.0f, 0.0f).Quaternion();
	const FQuat DesiredBodyWorld =
		DesiredControlWorld * Config.GetControlToBodyRotation().Inverse();
	TestTrue(TEXT("Kinematic body rotation preserves the requested control heading"),
		Config.GetControlWorldRotation(DesiredBodyWorld).Equals(DesiredControlWorld, 1.e-4f));

	FAircraftFlightControllerRuntimeConfig PositiveXAxes;
	PositiveXAxes.ForwardAxis = 0; // +X
	TestTrue(TEXT("+X is available as an explicit body-axis configuration"),
		PositiveXAxes.GetForwardAxisBody().Equals(FVector::ForwardVector, 1.e-4f));
	TestTrue(TEXT("+X torque mapping follows the configured body axes"),
		PositiveXAxes.ControllerTorqueToBody(ControllerTorque).Equals(
			FVector(-1.0f, -2.0f, 3.0f), 1.e-4f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftLinearDampingFeedForwardTest,
	"AircraftLab.Control.Velocity.LinearDampingFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftLinearDampingFeedForwardTest::RunTest(const FString& Parameters)
{
	const FVector DesiredVelocity(800.0, -400.0, 125.0);
	const FVector FeedForward = FlightControlDynamics::ComputeLinearDampingFeedForward(
		DesiredVelocity, FVector(0.3f), 1.0f);

	TestTrue(TEXT("Feedforward opposes modeled drag at the desired horizontal velocity"),
		FeedForward.Equals(FVector(240.0, -120.0, 0.0), 1.e-4));
	TestTrue(TEXT("Zero damping produces zero feedforward"),
		FlightControlDynamics::ComputeLinearDampingFeedForward(DesiredVelocity, FVector::ZeroVector, 1.0f).IsNearlyZero());
	TestTrue(TEXT("Negative scale is safely clamped to zero"),
		FlightControlDynamics::ComputeLinearDampingFeedForward(DesiredVelocity, FVector(0.3f), -1.0f).IsNearlyZero());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVelocityPidMaintainsTargetSpeedTest,
	"AircraftLab.Control.Velocity.ZeroErrorUsesDynamicsFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVelocityPidMaintainsTargetSpeedTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector(800.0, 0.0, 0.0);
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.LinearDampingPerSecond = FVector(0.3f);
	FAircraftModeCapabilities Capabilities;
	FAircraftFlightControllerRuntimeConfig Config;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;

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
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.EstimatedState.State.PositionCm.Z = 0.0f;
	Runtime.EstimatedState.State.VelocityCmPerSec.Z = -100.0f;
	Runtime.HoldTargets.HeldAltitudeCm = 1000.0f;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.GravityMagnitudeCmPerSecSq = 980.0f;
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldAltitude = true;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.MaxVerticalAccelerationCmPerSecSq = 1000.0f;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;

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
	FAircraftVerticalReleaseBrakesBeforeHoldingTest,
	"AircraftLab.Control.Altitude.VerticalReleaseBrakesBeforeHolding",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVerticalReleaseBrakesBeforeHoldingTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.EstimatedState.State.PositionCm.Z = 100.0f;
	Runtime.EstimatedState.State.VelocityCmPerSec.Z = 200.0f;
	Runtime.HoldTargets.HeldAltitudeCm = 0.0f;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	Runtime.HoldTargets.bVerticalBrakeBeforeHold = true;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.GravityMagnitudeCmPerSecSq = 980.0f;
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldAltitude = true;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.MaxVerticalAccelerationCmPerSecSq = 0.0f;
	Config.VerticalBrakeToHoldSpeedCmPerSec = 20.0f;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;

	float DesiredVelocity = 0.0f;
	Solver.ComputeVerticalControl(Context, 0.004f, DesiredVelocity);
	TestEqual(TEXT("Release braking commands zero vertical velocity"), DesiredVelocity, 0.0f);
	TestEqual(TEXT("Release braking moves the altitude anchor with the aircraft"),
		Runtime.HoldTargets.HeldAltitudeCm, 100.0f);
	TestTrue(TEXT("Altitude hold remains deferred while vertical speed is high"),
		Runtime.HoldTargets.bVerticalBrakeBeforeHold);

	Runtime.EstimatedState.State.PositionCm.Z = 180.0f;
	Runtime.EstimatedState.State.VelocityCmPerSec.Z = 10.0f;
	Solver.ComputeVerticalControl(Context, 0.004f, DesiredVelocity);
	TestFalse(TEXT("Low vertical speed latches the final altitude"),
		Runtime.HoldTargets.bVerticalBrakeBeforeHold);
	TestEqual(TEXT("Final altitude is where vertical braking actually finished"),
		Runtime.HoldTargets.HeldAltitudeCm, 180.0f);

	Runtime.EstimatedState.State.PositionCm.Z = 230.0f;
	Runtime.EstimatedState.State.VelocityCmPerSec.Z = 0.0f;
	Solver.ComputeVerticalControl(Context, 0.004f, DesiredVelocity);
	TestTrue(TEXT("After latching, external altitude drift is corrected toward the stop altitude"),
		DesiredVelocity < 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAngularDampingFeedForwardTest,
	"AircraftLab.Control.Damping.AngularTorqueFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAngularDampingFeedForwardTest::RunTest(const FString& Parameters)
{
	const FVector FeedForward = FlightControlDynamics::ComputeAngularDampingFeedForward(
		FVector(90.0f, -180.0f, 0.0f), FVector(2.0f),
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
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	// 故意留下过期的显示角度：控制器必须使用刚体四元数。
	Runtime.EstimatedState.State.AttitudeDegrees = FRotator(0.0f, 0.0f, 90.0f);
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FQuat::Identity);
	FAircraftModeCapabilities Capabilities;
	FAircraftFlightControllerRuntimeConfig Config;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;
	FAircraftYawSetpoint YawSetpoint;
	YawSetpoint.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;

	const FVector DesiredRates = Solver.ComputeDesiredBodyRates(
		Context, FRotator::ZeroRotator, YawSetpoint, 0.004f);
	TestTrue(TEXT("Identity rigid-body attitude has no Roll/Pitch quaternion error"),
		FVector2D(DesiredRates.X, DesiredRates.Y).IsNearlyZero(1.e-4f));

	YawSetpoint.TargetYawDegrees = 180.0f;
	const FVector YawRates = Solver.ComputeDesiredBodyRates(
		Context, FRotator::ZeroRotator, YawSetpoint, 0.004f);
	TestTrue(TEXT("Target yaw is closed from the quaternion-derived heading, not an Euler PID"),
		FMath::Abs(YawRates.Z) > 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftQuaternionHeadingDoesNotLeakIntoTiltTest,
	"AircraftLab.Control.Attitude.QuaternionHeadingDoesNotLeakIntoTilt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftQuaternionHeadingDoesNotLeakIntoTiltTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FRotator(10.0f, 45.0f, 5.0f).Quaternion());
	FAircraftModeCapabilities Capabilities;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.bEnableAttitudeReferenceModel = false;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;
	FAircraftYawSetpoint YawSetpoint;
	YawSetpoint.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;
	YawSetpoint.TargetYawDegrees = 135.0f;

	const FRotator DesiredTilt(-8.0f, 135.0f, 12.0f);
	const FVector SameHeadingRates = Solver.ComputeDesiredBodyRates(
		Context, DesiredTilt, YawSetpoint, 0.004f);

	YawSetpoint.TargetYawDegrees = -45.0f;
	const FVector OppositeHeadingRates = Solver.ComputeDesiredBodyRates(
		Context, DesiredTilt, YawSetpoint, 0.004f);

	TestTrue(TEXT("Changing only target heading does not alter Roll rate"),
		FMath::IsNearlyEqual(OppositeHeadingRates.X, SameHeadingRates.X, 1.e-4));
	TestTrue(TEXT("Changing only target heading does not alter Pitch rate"),
		FMath::IsNearlyEqual(OppositeHeadingRates.Y, SameHeadingRates.Y, 1.e-4));
	TestTrue(TEXT("Changing target heading still changes Yaw rate"),
		!FMath::IsNearlyEqual(OppositeHeadingRates.Z, SameHeadingRates.Z, 1.e-4));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawHoldInitializesFromRigidBodyQuaternionTest,
	"AircraftLab.Control.Attitude.YawHoldInitializesFromRigidBodyQuaternion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawHoldInitializesFromRigidBodyQuaternionTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	// 模拟启动期显示状态尚未刷新，但 Chaos 刚体已经具有非零出生航向。
	Runtime.EstimatedState.State.AttitudeDegrees = FRotator::ZeroRotator;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FRotator(0.0f, 73.0f, 0.0f).Quaternion());
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldYaw = true;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.bEnableAttitudeReferenceModel = false;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;

	const FAircraftYawSetpoint YawSetpoint = Solver.ComputeYawSetpoint(Context);
	const FVector DesiredRates = Solver.ComputeDesiredBodyRates(
		Context, FRotator(0.0f, 163.0f, 0.0f), YawSetpoint, 0.004f);

	TestTrue(TEXT("Initial held heading comes from the rigid-body quaternion"),
		FMath::IsNearlyEqual(YawSetpoint.TargetYawDegrees, 163.0f, 1.e-4f));
	TestTrue(TEXT("Matching initial heading produces no default Yaw rate"),
		FMath::IsNearlyZero(DesiredRates.Z, 1.e-4));
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
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.EstimatedState.State.PositionCm = FVector(100.0f, 0.0f, 0.0f);
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector(200.0f, 0.0f, 0.0f);
	Runtime.HoldTargets.HeldPositionCm = FVector::ZeroVector;
	Runtime.HoldTargets.bPositionHoldInitialized = true;
	Runtime.HoldTargets.bHorizontalBrakeBeforeHold = true;
	FAircraftPhysicsCache PhysicsCache;
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanUsePositionControl = true;
	Capabilities.CanUseVelocityControl = true;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;
	FAircraftManualCommand ManualCommand;
	FAutopilotInjection Injection;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Injection, Allocator, false };
	FAircraftFlightControlSolver Solver;

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAuthoritativePidLimitsTest,
	"AircraftLab.Control.Pid.AuthoritativeLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAuthoritativePidLimitsTest::RunTest(const FString& Parameters)
{
	const FAircraftFlightControllerRuntimeConfig Config;
	const FAircraftPidGains Position = Config.GetPositionPidGains(0);
	const FAircraftPidGains Velocity = Config.GetVelocityPidGains(0);
	const FAircraftPidGains RollRate = Config.GetRatePidGains(0);
	const FAircraftPidGains YawRate = Config.GetRatePidGains(2);
	const FAircraftPidGains Altitude = Config.GetAltitudePidGains();
	const FAircraftPidGains VerticalVelocity = Config.GetVerticalVelocityPidGains();

	TestEqual(TEXT("Position output uses the authoritative speed limit"), Position.OutputLimit, 800.0f);
	TestEqual(TEXT("Position uses authoritative velocity feed-forward"), Position.Kff, 1.0f);
	TestEqual(TEXT("Position derivative filter defaults to disabled"), Position.DerivativeCutoffHz, 0.0f);
	TestEqual(TEXT("Velocity integral uses the authoritative limit"), Velocity.IntegralLimit, 3000.0f);
	TestEqual(TEXT("Velocity output uses the authoritative acceleration limit"), Velocity.OutputLimit, 600.0f);
	TestEqual(TEXT("Velocity uses authoritative acceleration feed-forward"), Velocity.Kff, 1.0f);
	TestEqual(TEXT("Velocity derivative filter matches the authoritative cutoff"), Velocity.DerivativeCutoffHz, 12.0f);
	TestEqual(TEXT("Roll rate integral uses the authoritative limit"), RollRate.IntegralLimit, 120.0f);
	TestEqual(TEXT("Roll rate output uses the authoritative limit"), RollRate.OutputLimit, 0.35f);
	TestEqual(TEXT("Rate PID does not duplicate reference-model feed-forward"), RollRate.Kff, 0.0f);
	TestEqual(TEXT("Yaw rate output uses the authoritative limit"), YawRate.OutputLimit, 0.20f);
	TestEqual(TEXT("Altitude output uses the authoritative climb-rate limit"), Altitude.OutputLimit, 300.0f);
	TestEqual(TEXT("Altitude uses authoritative vertical-velocity feed-forward"), Altitude.Kff, 1.0f);
	TestEqual(TEXT("Vertical velocity integral uses the authoritative limit"), VerticalVelocity.IntegralLimit, 2500.0f);
	TestEqual(TEXT("Vertical velocity output uses the authoritative collective limit"), VerticalVelocity.OutputLimit, 0.30f);
	TestEqual(TEXT("Vertical velocity uses baseline-thrust instead of duplicate PID feed-forward"), VerticalVelocity.Kff, 0.0f);
	TestTrue(TEXT("All authoritative PID channels freeze integration while saturated"),
		Position.bFreezeIntegralWhenSaturated && Velocity.bFreezeIntegralWhenSaturated
		&& RollRate.bFreezeIntegralWhenSaturated && Altitude.bFreezeIntegralWhenSaturated
		&& VerticalVelocity.bFreezeIntegralWhenSaturated);
	return true;
}

#endif
