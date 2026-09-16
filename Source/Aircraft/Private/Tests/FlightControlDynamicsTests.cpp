// 适配点：嵌套 Profile 配置 → 平铺 FAircraftFlightControllerRuntimeConfig（自带权威默认值）；
// FAutopilotMovementIntent → FAircraftManualCommand；类型前缀 FFlightControl* → FAircraftFlightControl*。

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/AircraftAttitudeReference.h"
#include "Aircraft/AircraftYawReferenceDynamics.h"
#include "Aircraft/ControlAllocator.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPidExternalSaturationAntiWindupTest,
	"AircraftLab.Control.PID.ExternalSaturationDoesNotHideIntegral",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPidExternalSaturationAntiWindupTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftPidState State;
	FAircraftPidGains Gains;
	Gains.Ki = 1.0f;
	Gains.IntegralLimit = 1000.0f;

	for (int32 Step = 0; Step < 100; ++Step)
	{
		State.UpdateFromMeasurement(10.0f, 0.0f, 0.01f, Gains,
			0.0f, false);
	}
	const float OutputAfterRelease = State.UpdateFromMeasurement(
		0.0f, 0.0f, 0.01f, Gains);
	TestTrue(TEXT("External saturation must not accumulate a hidden integral"),
		FMath::IsNearlyZero(OutputAfterRelease, 1.e-4f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftConstraintDynamicsFeedForwardTest,
	"AircraftLab.Control.Constraint.DynamicsFeedForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftConstraintDynamicsFeedForwardTest::RunTest(const FString& Parameters)
{
	const double StrengthHz = 2.0;
	const double Stiffness = FMath::Square(StrengthHz * UE_DOUBLE_TWO_PI);
	const FVector HoverOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			FVector::ZeroVector, FVector::ZeroVector,
			FVector(0.0, 0.0, -980.0), 1.0, 1.0, StrengthHz, true, 100.0);
	TestEqual(TEXT("Acceleration drive offsets its target upward to cancel gravity"),
		HoverOffset.Z, 980.0 / Stiffness, 1.e-5);
	const FVector DisabledGravityOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			FVector::ZeroVector, FVector::ZeroVector,
			FVector(0.0, 0.0, -980.0), 0.0, 1.0, StrengthHz, true, 100.0);
	TestTrue(TEXT("Zero gravity feed-forward scale disables gravity compensation"),
		DisabledGravityOffset.IsNearlyZero());

	const FVector AccelerationOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			FVector(200.0, 0.0, 0.0), FVector::ZeroVector,
			FVector::ZeroVector, 0.0, 1.0, StrengthHz, true, 100.0);
	TestEqual(TEXT("Predictive control acceleration is converted into a spring target lead"),
		AccelerationOffset.X, 200.0 / Stiffness, 1.e-5);
	const FVector DynamicsOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			FVector::ZeroVector, FVector(120.0, 0.0, 0.0),
			FVector::ZeroVector, 0.0, 0.5, StrengthHz, true, 100.0);
	TestEqual(TEXT("Predictive dynamics compensation is consumed exactly once at its configured scale"),
		DynamicsOffset.X, 60.0 / Stiffness, 1.e-5);

	const FVector ForceModeOffset =
		UE::AircraftLab::ConstraintDrive::ComputeAccelerationFeedForwardPositionOffset(
			FVector::ZeroVector, FVector::ZeroVector,
			FVector(0.0, 0.0, -980.0), 1.0, 1.0, StrengthHz, false, 100.0);
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

	TestEqual(TEXT("Default model forward axis is +Y"),
		Config.FrameBinding.GetModelForwardAxis(), EAircraftModelForwardAxis::PositiveY);
	TestTrue(TEXT("Configured forward vector is model-local +Y"),
		Config.FrameBinding.GetForwardAxisModel().Equals(FVector::RightVector, 1.e-4f));
	TestTrue(TEXT("Configured right vector is model-local -X"),
		Config.FrameBinding.GetRightAxisModel().Equals(-FVector::ForwardVector, 1.e-4f));
	TestTrue(TEXT("Configured up vector is model-local +Z"),
		Config.FrameBinding.GetUpAxisModel().Equals(FVector::UpVector, 1.e-4f));
	TestTrue(TEXT("Identity root frame resolves model +Y forward to body +Y"),
		Config.GetForwardAxisBody().Equals(FVector::RightVector, 1.e-4f));
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
	const FQuat DesiredBodyWorld = Config.GetBodyWorldRotation(DesiredControlWorld);
	TestTrue(TEXT("Kinematic body rotation preserves the requested control heading"),
		Config.GetControlWorldRotation(DesiredBodyWorld).Equals(DesiredControlWorld, 1.e-4f));

	FAircraftFlightControllerRuntimeConfig PositiveXAxes;
	PositiveXAxes.FrameBinding.Configure(
		EAircraftModelForwardAxis::PositiveX, FTransform::Identity);
	TestTrue(TEXT("+X remains available as an explicit model-forward configuration"),
		PositiveXAxes.GetForwardAxisBody().Equals(FVector::ForwardVector, 1.e-4f));
	TestTrue(TEXT("+X torque mapping follows the configured body axes"),
		PositiveXAxes.ControllerTorqueToBody(ControllerTorque).Equals(
			FVector(-1.0f, -2.0f, 3.0f), 1.e-4f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftThreeAxisBodyFrameTest,
	"AircraftLab.Control.Axes.ArbitraryRootBoneFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftThreeAxisBodyFrameTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControllerRuntimeConfig Config;
	const FQuat BodyToModelRotation(FRotationMatrix::MakeFromXZ(
		FVector::UpVector, -FVector::RightVector));
	const FTransform BodyToModel(
		BodyToModelRotation, FVector(10.0, 20.0, 30.0), FVector::OneVector);
	TestTrue(TEXT("A finite RootBone reference transform configures the frame binding"),
		Config.FrameBinding.Configure(EAircraftModelForwardAxis::PositiveY, BodyToModel));

	TestTrue(TEXT("model +Y forward resolves to bone2 body -Z"),
		Config.GetForwardAxisBody().Equals(FVector(0.0, 0.0, -1.0), 1.e-4));
	TestTrue(TEXT("bone2 up is body +X"),
		Config.GetUpAxisBody().Equals(FVector(1.0, 0.0, 0.0), 1.e-4));
	TestTrue(TEXT("model -X right resolves to bone2 body +Y"),
		Config.GetRightAxisBody().Equals(FVector(0.0, 1.0, 0.0), 1.e-4));

	const FVector ControlVector(11.0, 22.0, 33.0);
	const FVector ExpectedBodyVector(33.0, 22.0, -11.0);
	TestTrue(TEXT("Control vectors map into the complete bone2 frame"),
		Config.ControlToBodyVector(ControlVector).Equals(ExpectedBodyVector, 1.e-4));
	TestTrue(TEXT("Body vectors round-trip through the complete bone2 frame"),
		Config.BodyToControlVector(ExpectedBodyVector).Equals(ControlVector, 1.e-4));

	const FQuat ControlToBody = Config.GetControlToBodyRotation();
	TestTrue(TEXT("Control rotation maps Forward to body -Z"),
		ControlToBody.RotateVector(FVector::ForwardVector).Equals(FVector(0.0, 0.0, -1.0), 1.e-4));
	TestTrue(TEXT("Control rotation maps Right to body +Y"),
		ControlToBody.RotateVector(FVector::RightVector).Equals(FVector(0.0, 1.0, 0.0), 1.e-4));
	TestTrue(TEXT("Control rotation maps Up to body +X"),
		ControlToBody.RotateVector(FVector::UpVector).Equals(FVector(1.0, 0.0, 0.0), 1.e-4));

	TestTrue(TEXT("Body-axis inertia magnitudes are reordered into Roll Pitch Yaw axes"),
		Config.BodyAxisMagnitudesToControl(FVector(1.0, 2.0, 3.0)).Equals(
			FVector(3.0, 2.0, 1.0), 1.e-4));

	const FVector ModelPoint(40.0, -15.0, 8.0);
	const FVector BodyPoint = Config.FrameBinding.ModelPositionToBody(ModelPoint);
	TestTrue(TEXT("Model and RootBone positions round-trip through the complete binding"),
		Config.FrameBinding.BodyPositionToModel(BodyPoint).Equals(ModelPoint, 1.e-4));
	const FTransform ModelWorld(
		FRotator(12.0, 47.0, -8.0), FVector(500.0, -250.0, 900.0));
	const FTransform BodyWorld = Config.FrameBinding.GetBodyWorldTransform(ModelWorld);
	TestTrue(TEXT("The compiled RootBone origin matches BodyToModel followed by ModelToWorld"),
		BodyWorld.GetLocation().Equals(
			ModelWorld.TransformPosition(BodyToModel.GetLocation()), 1.e-4));
	const FTransform RecoveredModelWorld = Config.FrameBinding.GetModelWorldTransform(BodyWorld);
	TestTrue(TEXT("Model and RootBone world transforms round-trip"),
		RecoveredModelWorld.GetLocation().Equals(ModelWorld.GetLocation(), 1.e-4)
		&& RecoveredModelWorld.GetRotation().Equals(ModelWorld.GetRotation(), 1.e-4));

	FAircraftRotorAllocationInfo UpwardRotor;
	UpwardRotor.ThrustAxisBody = Config.GetUpAxisBody();
	UpwardRotor.MaxAllocatedThrustN = 10.0f;
	const FVector4 Jacobian = FAircraftControlAllocator::BuildJacobianColumn(UpwardRotor, Config);
	TestEqual(TEXT("Collective authority follows configured body Up instead of hard-coded body +Z"),
		Jacobian[0], 10.0, 1.e-4);
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
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
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
	FAircraftTrajectoryDynamicsFeedForwardOwnershipTest,
	"AircraftLab.Control.Velocity.TrajectoryDynamicsFeedForwardAppliedOnce",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftTrajectoryDynamicsFeedForwardOwnershipTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.GravityMagnitudeCmPerSecSq = 980.0f;
	PhysicsCache.LinearDampingPerSecond = FVector(0.3f);
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanUsePositionControl = true;
	Capabilities.CanUseVelocityControl = true;
	FAircraftFlightControllerRuntimeConfig Config;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	Reference.bValid = true;
	Reference.bPositionTrackingEnabled = false;
	Reference.PositionCm = FVector(100000.0f, 0.0f, 0.0f);
	Reference.VelocityCmPerSec = FVector(100.0f, 0.0f, 0.0f);
	Reference.AccelerationCmPerSecSq = FVector(10.0f, 0.0f, 0.0f);
	Reference.ControlAccelerationCmPerSecSq = FVector(10.0f, 0.0f, 0.0f);
	Reference.DynamicsFeedForwardAccelerationCmPerSecSq = FVector(30.0f, 0.0f, 0.0f);
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, true };
	FAircraftFlightControlSolver Solver;

	const FVector Acceleration = Solver.ComputeDesiredHorizontalAcceleration(Context, 0.004f);
	TestEqual(TEXT("Velocity reference bypasses the position outer loop"),
		Solver.LastDesiredHorizontalVelocityCmPerSec.X, 100.0, 1.e-3);
	TestEqual(TEXT("Trajectory owns damping feed-forward without a second flight-control copy"),
		Solver.LastVelocityDragFeedForwardCmPerSecSq.X, 0.0, 1.e-3);
	TestEqual(TEXT("Kinematic and dynamics reference accelerations are combined once"),
		Acceleration.X, 40.0, 1.e-3);
	TestEqual(TEXT("Velocity diagnostics expose the PID feedback independently from feed-forward"),
		Solver.LastVelocityFeedbackAccelerationCmPerSecSq.X, 0.0, 1.e-3);
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
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
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
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
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
	Config.bEnableAttitudeReferenceModel = false;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
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
	FAircraftFlightControllerYawBypassesAttitudeReferenceDynamicsTest,
	"AircraftLab.Control.Attitude.YawBypassesAttitudeReferenceDynamics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftFlightControllerYawBypassesAttitudeReferenceDynamicsTest::RunTest(
	const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FQuat::Identity);
	PhysicsCache.AngularVelocityWorldRadPerSec = FVector::ZeroVector;
	FAircraftModeCapabilities Capabilities;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.bEnableAttitudeReferenceModel = true;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };

	FAircraftFlightControlSolver AngleSolver;
	FAircraftYawSetpoint AngleSetpoint;
	AngleSetpoint.TargetYawDegrees = 45.0f;
	AngleSetpoint.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;
	const FVector AngleRates = AngleSolver.ComputeDesiredBodyRates(
		Context, FRotator(20.0f, 45.0f, -15.0f), AngleSetpoint, DeltaSeconds);
	TestTrue(TEXT("The authoritative shaped yaw angle reaches the attitude loop immediately"),
		FMath::Abs(AngleRates.Z) > 80.0f);
	TestTrue(TEXT("Roll and Pitch remain inside the SO(3) reference dynamics"),
		FVector2D(AngleRates.X, AngleRates.Y).IsNearlyZero(1.e-3f));

	const FQuat TargetControlWorldRotation = FRotator(0.0f, 45.0f, 0.0f).Quaternion();
	PhysicsCache.BodyTransform.SetRotation(
		Config.GetBodyWorldRotation(TargetControlWorldRotation));
	FAircraftFlightControlSolver RateSolver;
	FAircraftYawSetpoint RateSetpoint;
	RateSetpoint.TargetYawDegrees = 45.0f;
	RateSetpoint.FeedForwardRateDegPerSec = 30.0f;
	RateSetpoint.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;
	const FVector RateCommand = RateSolver.ComputeDesiredBodyRates(
		Context, FRotator(0.0f, 45.0f, 0.0f), RateSetpoint, DeltaSeconds);
	TestEqual(TEXT("The authoritative shaped yaw rate is not delayed by the SO(3) model"),
		RateCommand.Z, 30.0, 1.e-3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftFlightControllerTiltDynamicsIgnoreYawLimitsTest,
	"AircraftLab.Control.Attitude.TiltDynamicsIgnoreYawLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftFlightControllerTiltDynamicsIgnoreYawLimitsTest::RunTest(
	const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FQuat::Identity);
	FAircraftModeCapabilities Capabilities;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.bEnableAttitudeReferenceModel = true;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
	const float CurrentYawDegrees = AircraftAttitudeReference::GetPlanarHeadingDegrees(
		PhysicsCache.BodyTransform.GetRotation(), Config);
	FAircraftYawSetpoint NoYawAuthority;
	NoYawAuthority.TargetYawDegrees = CurrentYawDegrees;
	NoYawAuthority.MaxRateDegPerSec = 0.0f;
	FAircraftYawSetpoint FullYawAuthority = NoYawAuthority;
	FullYawAuthority.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;
	FAircraftFlightControlSolver NoYawSolver;
	FAircraftFlightControlSolver FullYawSolver;
	FVector NoYawRates = FVector::ZeroVector;
	FVector FullYawRates = FVector::ZeroVector;
	for (int32 Step = 0; Step < 60; ++Step)
	{
		NoYawRates = NoYawSolver.ComputeDesiredBodyRates(
			Context, FRotator(20.0f, CurrentYawDegrees, 20.0f),
			NoYawAuthority, DeltaSeconds);
		FullYawRates = FullYawSolver.ComputeDesiredBodyRates(
			Context, FRotator(20.0f, CurrentYawDegrees, 20.0f),
			FullYawAuthority, DeltaSeconds);
	}
	TestTrue(TEXT("Roll/Pitch reference dynamics do not depend on yaw authority"),
		FVector2D(NoYawRates.X, NoYawRates.Y).Equals(
			FVector2D(FullYawRates.X, FullYawRates.Y), 1.e-3f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftQuaternionHeadingUsesCoordinatedTiltRatesTest,
	"AircraftLab.Control.Attitude.QuaternionHeadingUsesCoordinatedTiltRates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftQuaternionHeadingUsesCoordinatedTiltRatesTest::RunTest(const FString& Parameters)
{
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FRotator(10.0f, 45.0f, 5.0f).Quaternion());
	FAircraftModeCapabilities Capabilities;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.bEnableAttitudeReferenceModel = false;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
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

	TestTrue(TEXT("A tilted aircraft maps opposite planar heading corrections into different lateral rates"),
		!FVector2D(OppositeHeadingRates.X, OppositeHeadingRates.Y).Equals(
			FVector2D(SameHeadingRates.X, SameHeadingRates.Y), 1.e-4f));
	TestTrue(TEXT("Changing target heading still changes Yaw rate"),
		!FMath::IsNearlyEqual(OppositeHeadingRates.Z, SameHeadingRates.Z, 1.e-4));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftPlanarHeadingRateRejectsPureRollTest,
	"AircraftLab.Control.Attitude.PlanarHeadingRateRejectsPureRoll",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftPlanarHeadingRateRejectsPureRollTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	FAircraftFlightControllerRuntimeConfig Config;
	const FQuat ControlWorldRotation = FRotator(-20.0f, 35.0f, 12.0f).Quaternion();
	const FQuat BodyWorldRotation = Config.GetBodyWorldRotation(ControlWorldRotation);
	PhysicsCache.BodyTransform.SetRotation(BodyWorldRotation);
	const FVector PureRollBodyRadPerSec = Config.GetForwardAxisBody();
	PhysicsCache.AngularVelocityWorldRadPerSec = BodyWorldRotation.RotateVector(
		PureRollBodyRadPerSec);
	PhysicsCache.AngularVelocityControllerDegPerSec = FMath::RadiansToDegrees(
		Config.BodyAngularToController(PureRollBodyRadPerSec));

	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldYaw = true;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	Allocator.Cache.bIsValid = true;
	Allocator.Cache.PositiveTorqueAuthority[2] = 10.0;
	Allocator.Cache.NegativeTorqueAuthority[2] = 10.0;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
	FAircraftFlightControlSolver Solver;

	const FAircraftYawSetpoint Setpoint = Solver.ComputeYawSetpoint(Context, DeltaSeconds);
	TestTrue(TEXT("Rotation around the aircraft forward axis does not change planar heading"),
		FMath::Abs(Setpoint.FeedForwardRateDegPerSec) < 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftTiltedYawRateUsesFullBodyKinematicsTest,
	"AircraftLab.Control.Attitude.TiltedYawRateUsesFullBodyKinematics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftTiltedYawRateUsesFullBodyKinematicsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	FAircraftFlightControllerRuntimeConfig Config;
	Config.bEnableAttitudeReferenceModel = false;
	const FRotator ControlAttitude(-20.0f, 35.0f, 12.0f);
	const FQuat BodyWorldRotation = Config.GetBodyWorldRotation(ControlAttitude.Quaternion());
	PhysicsCache.BodyTransform.SetRotation(BodyWorldRotation);

	FAircraftModeCapabilities Capabilities;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
	FAircraftFlightControlSolver Solver;
	FAircraftYawSetpoint YawSetpoint;
	YawSetpoint.TargetYawDegrees = ControlAttitude.Yaw;
	YawSetpoint.FeedForwardRateDegPerSec = 60.0f;
	YawSetpoint.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;

	const FVector DesiredRates = Solver.ComputeDesiredBodyRates(
		Context, ControlAttitude, YawSetpoint, 1.0f / 120.0f);
	const FVector ExpectedRates = FMath::RadiansToDegrees(Config.BodyAngularToController(
		BodyWorldRotation.UnrotateVector(
			FVector::UpVector * FMath::DegreesToRadians(YawSetpoint.FeedForwardRateDegPerSec))));
	TestTrue(TEXT("Tilted world-up yaw contains controller Roll/Pitch rate components"),
		!FVector2D(ExpectedRates.X, ExpectedRates.Y).IsNearlyZero(0.1f));
	TestTrue(TEXT("Yaw feed-forward is transformed from world up into all controller axes"),
		DesiredRates.Equals(ExpectedRates, 1.e-3f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawReferenceInitializesFromRigidBodyQuaternionTest,
	"AircraftLab.Control.Attitude.YawReferenceInitializesFromRigidBodyQuaternion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawReferenceInitializesFromRigidBodyQuaternionTest::RunTest(const FString& Parameters)
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
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
	FAircraftFlightControlSolver Solver;

	const FAircraftYawSetpoint YawSetpoint = Solver.ComputeYawSetpoint(Context, 0.004f);
	const FVector DesiredRates = Solver.ComputeDesiredBodyRates(
		Context, FRotator(0.0f, 163.0f, 0.0f), YawSetpoint, 0.004f);

	TestTrue(TEXT("Initial held heading comes from the rigid-body quaternion"),
		FMath::IsNearlyEqual(YawSetpoint.TargetYawDegrees, 163.0f, 1.e-4f));
	TestTrue(TEXT("Matching initial heading produces no default Yaw rate"),
		FMath::IsNearlyZero(DesiredRates.Z, 1.e-4));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawSetpointRequiresAllocatorAuthorityTest,
	"AircraftLab.Control.Attitude.YawSetpointRequiresAllocatorAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawSetpointRequiresAllocatorAuthorityTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform.SetRotation(FQuat::Identity);
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldYaw = true;
	FAircraftFlightControllerRuntimeConfig Config;
	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference Reference;
	Reference.bValid = true;
	Reference.YawDegrees = 0.0f;
	Reference.YawRateDegPerSec = 45.0f;
	Reference.YawRateLimitDegPerSec = 90.0f;
	FAircraftControlAllocator Allocator;
	Allocator.Cache.bIsValid = true;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, true };
	FAircraftFlightControlSolver Solver;

	const FAircraftYawSetpoint Unavailable = Solver.ComputeYawSetpoint(Context, 0.004f);
	TestEqual(TEXT("Zero allocator authority disables yaw-rate limit"),
		Unavailable.MaxRateDegPerSec, 0.0f, 1.e-4f);
	TestEqual(TEXT("Zero allocator authority rejects yaw feed-forward"),
		Unavailable.FeedForwardRateDegPerSec, 0.0f, 1.e-4f);
	TestEqual(TEXT("Zero allocator authority holds measured control heading"),
		Unavailable.TargetYawDegrees, 90.0f, 1.e-4f);
	const FVector UnavailableTorqueCommand = Solver.ComputeBodyTorqueCommand(
		Context, FVector(0.0f, 0.0f, 45.0f), 0.004f);
	TestEqual(TEXT("Zero allocator authority publishes no normalized yaw command"),
		UnavailableTorqueCommand.Z, 0.0, 1.e-4);

	Allocator.Cache.PositiveTorqueAuthority[2] = 10.0;
	Allocator.Cache.NegativeTorqueAuthority[2] = 10.0;
	Reference.YawRateLimitDegPerSec = 0.0f;
	const FAircraftYawSetpoint ExplicitZeroLimit = Solver.ComputeYawSetpoint(Context, 0.004f);
	TestEqual(TEXT("Explicit zero trajectory limit cannot fall back to controller maximum"),
		ExplicitZeroLimit.MaxRateDegPerSec, 0.0f, 1.e-4f);
	TestEqual(TEXT("Explicit zero trajectory limit rejects yaw feed-forward"),
		ExplicitZeroLimit.FeedForwardRateDegPerSec, 0.0f, 1.e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftFlightControllerManualYawReleaseTest,
	"AircraftLab.Control.Attitude.ManualYawReleaseCapturesMeasuredHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftFlightControllerManualYawReleaseTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftFlightControlRuntimeState Runtime;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.BodyTransform = FTransform::Identity;
	PhysicsCache.AngularVelocityWorldRadPerSec = FVector::ZeroVector;
	PhysicsCache.AngularVelocityControllerDegPerSec = FVector::ZeroVector;
	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldYaw = true;
	FAircraftFlightControllerRuntimeConfig Config;
	FAircraftManualCommand ManualCommand;
	ManualCommand.DesiredYawRateDegPerSec = 90.0f;
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	Allocator.Cache.bIsValid = true;
	Allocator.Cache.PositiveTorqueAuthority[2] = 10.0;
	Allocator.Cache.NegativeTorqueAuthority[2] = 10.0;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
	FAircraftFlightControlSolver Solver;
	FAircraftYawSetpoint Setpoint;
	for (int32 Step = 0; Step < 60; ++Step)
	{
		Setpoint = Solver.ComputeYawSetpoint(Context, DeltaSeconds);
		PhysicsCache.BodyTransform.SetRotation(Config.GetBodyWorldRotation(
			FRotator(0.0f, Setpoint.TargetYawDegrees, 0.0f).Quaternion()));
		PhysicsCache.AngularVelocityWorldRadPerSec = FVector::UpVector
			* FMath::DegreesToRadians(Setpoint.FeedForwardRateDegPerSec);
	}
	const float ReleaseRateDegPerSec = Setpoint.FeedForwardRateDegPerSec;
	TestTrue(TEXT("Manual stick builds a positive yaw-rate reference"),
		ReleaseRateDegPerSec > 1.0f);

	ManualCommand.DesiredYawRateDegPerSec = 0.0f;
	for (int32 Step = 0; Step < 480; ++Step)
	{
		Setpoint = Solver.ComputeYawSetpoint(Context, DeltaSeconds);
		PhysicsCache.BodyTransform.SetRotation(Config.GetBodyWorldRotation(
			FRotator(0.0f, Setpoint.TargetYawDegrees, 0.0f).Quaternion()));
		PhysicsCache.AngularVelocityWorldRadPerSec = FVector::UpVector
			* FMath::DegreesToRadians(Setpoint.FeedForwardRateDegPerSec);
	}
	const float FinalMeasuredYawDegrees = Config.GetControlWorldRotation(
		PhysicsCache.BodyTransform.GetRotation()).Rotator().Yaw;
	TestTrue(TEXT("Released FlightController yaw latches the measured stop heading"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(
			Setpoint.TargetYawDegrees, FinalMeasuredYawDegrees)) < 0.1f);
	TestTrue(TEXT("Released FlightController yaw reference stops without reversing"),
		Setpoint.FeedForwardRateDegPerSec >= -UE_KINDA_SMALL_NUMBER
			&& FMath::Abs(Setpoint.FeedForwardRateDegPerSec) < 0.25f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCombinedTiltYawReleaseFrameRateTest,
	"AircraftLab.Control.Attitude.CombinedTiltYawReleaseFrameRateInvariant",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCombinedTiltYawReleaseFrameRateTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	auto Simulate = [](const float DeltaSeconds)
	{
		FAircraftFlightControlRuntimeState Runtime;
		Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;
		FAircraftPhysicsCache PhysicsCache;
		PhysicsCache.BodyTransform = FTransform::Identity;
		PhysicsCache.AngularVelocityWorldRadPerSec = FVector::ZeroVector;
		FAircraftModeCapabilities Capabilities;
		FAircraftFlightControllerRuntimeConfig Config;
		FAircraftManualCommand ManualCommand;
		FAircraftTrajectoryReference Reference;
		FAircraftControlAllocator Allocator;
		FAircraftFlightControlSolverContext Context{
			Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
		FAircraftFlightControlSolver Solver;
		FAircraftYawSetpoint YawSetpoint;
		YawSetpoint.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;

		// Initialize the reference at hover before applying the combined command.
		Solver.ComputeDesiredBodyRates(
			Context, FRotator::ZeroRotator, YawSetpoint, DeltaSeconds);

		const int32 CommandSteps = FMath::RoundToInt(0.4f / DeltaSeconds);
		YawSetpoint.TargetYawDegrees = 75.0f;
		YawSetpoint.FeedForwardRateDegPerSec = 60.0f;
		for (int32 Step = 0; Step < CommandSteps; ++Step)
		{
			Solver.ComputeDesiredBodyRates(
				Context, FRotator(-20.0f, 75.0f, 12.0f), YawSetpoint, DeltaSeconds);
		}

		const int32 ReleaseSteps = FMath::RoundToInt(0.4f / DeltaSeconds);
		YawSetpoint.FeedForwardRateDegPerSec = 0.0f;
		FVector Result = FVector::ZeroVector;
		for (int32 Step = 0; Step < ReleaseSteps; ++Step)
		{
			Result = Solver.ComputeDesiredBodyRates(
				Context, FRotator(-20.0f, 75.0f, 12.0f), YawSetpoint, DeltaSeconds);
		}
		return Result;
	};

	const FVector At30Hz = Simulate(1.0f / 30.0f);
	const FVector At60Hz = Simulate(1.0f / 60.0f);
	const FVector At120Hz = Simulate(1.0f / 120.0f);
	TestTrue(*FString::Printf(TEXT("30/120 Hz result mismatch: 30=%s 120=%s"),
		*At30Hz.ToString(), *At120Hz.ToString()), At30Hz.Equals(At120Hz, 0.1f));
	TestTrue(*FString::Printf(TEXT("60/120 Hz result mismatch: 60=%s 120=%s"),
		*At60Hz.ToString(), *At120Hz.ToString()), At60Hz.Equals(At120Hz, 0.1f));
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
	FAircraftTrajectoryReference Reference;
	FAircraftControlAllocator Allocator;
	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand, Reference, Allocator, false };
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawRateReleaseReferenceTest,
	"AircraftLab.Control.YawReference.RateReleaseBrakesToIntegratedHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawRateReleaseReferenceTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftYawReferenceLimits Limits;
	Limits.MaxRateDegPerSec = 90.0f;
	Limits.MaxAccelerationDegPerSecSq = 180.0f;
	Limits.MaxJerkDegPerSecCubed = 600.0f;
	Limits.ResponseTimeSeconds = 0.04f;
	FAircraftYawReferenceState State;
	float MeasuredYawDegrees = 0.0f;
	float MeasuredRateDegPerSec = 0.0f;

	for (int32 Step = 0; Step < 240; ++Step)
	{
		TestTrue(TEXT("Rate command update remains valid"),
			FAircraftYawReferenceDynamics::UpdateRateCommand(
				90.0f, MeasuredYawDegrees, MeasuredRateDegPerSec,
				DeltaSeconds, Limits, State));
		MeasuredRateDegPerSec = State.RateDegPerSec;
		MeasuredYawDegrees = FRotator::NormalizeAxis(
			MeasuredYawDegrees + MeasuredRateDegPerSec * DeltaSeconds);
	}
	const float ReleaseYawDegrees = State.YawDegrees;
	const float ReleaseRateDegPerSec = State.RateDegPerSec;
	TestTrue(TEXT("Held stick reaches a positive yaw rate"), ReleaseRateDegPerSec > 80.0f);

	float PreviousYawDegrees = ReleaseYawDegrees;
	float PreviousAccelerationDegPerSecSq = State.AccelerationDegPerSecSq;
	for (int32 Step = 0; Step < 1200; ++Step)
	{
		TestTrue(TEXT("Released rate command update remains valid"),
			FAircraftYawReferenceDynamics::UpdateRateCommand(
				0.0f, MeasuredYawDegrees, MeasuredRateDegPerSec,
				DeltaSeconds, Limits, State));
		const float UnwrappedYawStep = FMath::FindDeltaAngleDegrees(
			PreviousYawDegrees, State.YawDegrees);
		TestTrue(TEXT("Reference yaw continues integrating while its rate brakes"),
			UnwrappedYawStep >= -1.e-3f);
		TestTrue(TEXT("Released reference rate never reverses direction"),
			State.RateDegPerSec >= -1.e-3f);
		TestTrue(TEXT("Yaw rate respects its hard limit"),
			FMath::Abs(State.RateDegPerSec) <= Limits.MaxRateDegPerSec + 1.e-3f);
		TestTrue(TEXT("Yaw acceleration respects its hard limit"),
			FMath::Abs(State.AccelerationDegPerSecSq)
				<= Limits.MaxAccelerationDegPerSecSq + 1.e-3f);
		TestTrue(*FString::Printf(
			TEXT("Yaw jerk respects its hard limit (step=%d previous=%.6f current=%.6f)"),
			Step, PreviousAccelerationDegPerSecSq, State.AccelerationDegPerSecSq),
			FMath::Abs(State.AccelerationDegPerSecSq - PreviousAccelerationDegPerSecSq)
				<= Limits.MaxJerkDegPerSecCubed * DeltaSeconds + 1.e-3f);
		PreviousYawDegrees = State.YawDegrees;
		PreviousAccelerationDegPerSecSq = State.AccelerationDegPerSecSq;
		MeasuredRateDegPerSec = State.RateDegPerSec;
		MeasuredYawDegrees = FRotator::NormalizeAxis(
			MeasuredYawDegrees + MeasuredRateDegPerSec * DeltaSeconds);
	}

	TestTrue(TEXT("Release converges to zero yaw rate"),
		FMath::Abs(State.RateDegPerSec) < 0.1f);
	TestTrue(TEXT("Release converges to zero yaw acceleration"),
		FMath::Abs(State.AccelerationDegPerSecSq) < 0.1f);
	TestTrue(TEXT("Final hold yaw is the integrated braking endpoint"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(ReleaseYawDegrees, State.YawDegrees)) > 10.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawRateReleaseCapturesMeasuredStopHeadingTest,
	"AircraftLab.Control.YawReference.RateReleaseCapturesMeasuredStopHeading",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawRateReleaseCapturesMeasuredStopHeadingTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftYawReferenceLimits Limits;
	Limits.MaxRateDegPerSec = 90.0f;
	Limits.MaxAccelerationDegPerSecSq = 180.0f;
	Limits.MaxJerkDegPerSecCubed = 600.0f;
	Limits.ResponseTimeSeconds = 0.04f;
	FAircraftYawReferenceState State;
	float MeasuredYawDegrees = 0.0f;
	float MeasuredRateDegPerSec = 0.0f;

	for (int32 Step = 0; Step < 240; ++Step)
	{
		TestTrue(TEXT("Rate tracking remains valid"),
			FAircraftYawReferenceDynamics::UpdateRateCommand(
				90.0f, MeasuredYawDegrees, MeasuredRateDegPerSec,
				DeltaSeconds, Limits, State));
		MeasuredRateDegPerSec = State.RateDegPerSec;
		MeasuredYawDegrees = FRotator::NormalizeAxis(
			MeasuredYawDegrees + MeasuredRateDegPerSec * DeltaSeconds);
	}

	// The rigid body deliberately decelerates more slowly than the reference. Once the
	// reference reaches zero, rate mode must keep the angle loop transparent until the
	// measured body rate also stops; otherwise it pulls the aircraft back to an obsolete
	// integrated reference and creates the observed yaw rebound.
	MeasuredRateDegPerSec = 45.0f;
	bool bReferenceStoppedWhileBodyMoving = false;
	for (int32 Step = 0; Step < 360; ++Step)
	{
		MeasuredYawDegrees = FRotator::NormalizeAxis(
			MeasuredYawDegrees + MeasuredRateDegPerSec * DeltaSeconds);
		TestTrue(TEXT("Rate release remains valid"),
			FAircraftYawReferenceDynamics::UpdateRateCommand(
				0.0f, MeasuredYawDegrees, MeasuredRateDegPerSec,
				DeltaSeconds, Limits, State));
		if (FMath::Abs(State.RateDegPerSec) < 0.1f)
		{
			bReferenceStoppedWhileBodyMoving = true;
			TestTrue(TEXT("Released rate mode follows the moving body instead of pulling backward"),
				FMath::Abs(FMath::FindDeltaAngleDegrees(
					State.YawDegrees, MeasuredYawDegrees)) < 0.1f);
			break;
		}
	}
	TestTrue(TEXT("The reference can stop before the lagging rigid body"),
		bReferenceStoppedWhileBodyMoving);

	for (int32 Step = 0; Step < 240; ++Step)
	{
		MeasuredRateDegPerSec = FMath::Max(
			0.0f, MeasuredRateDegPerSec - 45.0f * DeltaSeconds);
		MeasuredYawDegrees = FRotator::NormalizeAxis(
			MeasuredYawDegrees + MeasuredRateDegPerSec * DeltaSeconds);
		TestTrue(TEXT("Measured stop capture remains valid"),
			FAircraftYawReferenceDynamics::UpdateRateCommand(
				0.0f, MeasuredYawDegrees, MeasuredRateDegPerSec,
				DeltaSeconds, Limits, State));
	}
	const float CapturedHoldYawDegrees = State.YawDegrees;
	TestTrue(TEXT("Hold captures the actual heading where the rigid body stopped"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(
			CapturedHoldYawDegrees, MeasuredYawDegrees)) < 0.25f);

	MeasuredYawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees + 10.0f);
	TestTrue(TEXT("Latched hold update remains valid"),
		FAircraftYawReferenceDynamics::UpdateRateCommand(
			0.0f, MeasuredYawDegrees, 0.0f, DeltaSeconds, Limits, State));
	TestTrue(TEXT("After capture the hold heading no longer follows external drift"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(
			State.YawDegrees, CapturedHoldYawDegrees)) < 0.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftYawAngleReferenceTest,
	"AircraftLab.Control.YawReference.AngleCommandConvergesWithoutOvershoot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftYawAngleReferenceTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	constexpr float DeltaSeconds = 1.0f / 120.0f;
	FAircraftYawReferenceLimits Limits;
	Limits.MaxRateDegPerSec = 90.0f;
	Limits.MaxAccelerationDegPerSecSq = 180.0f;
	Limits.MaxJerkDegPerSecCubed = 600.0f;
	Limits.ResponseTimeSeconds = 0.04f;
	FAircraftYawReferenceState State;
	float MaximumYawDegrees = 0.0f;
	for (int32 Step = 0; Step < 2400; ++Step)
	{
		TestTrue(TEXT("Angle command update remains valid"),
			FAircraftYawReferenceDynamics::UpdateAngleCommand(
				90.0f, 0.0f, 0.0f, DeltaSeconds, Limits, State));
		MaximumYawDegrees = FMath::Max(MaximumYawDegrees, State.YawDegrees);
	}

	TestTrue(TEXT("Angle reference does not overshoot its target"), MaximumYawDegrees <= 90.05f);
	TestTrue(TEXT("Angle reference converges to its target"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(State.YawDegrees, 90.0f)) < 0.05f);
	TestTrue(TEXT("Settled angle reference has zero rate"), FMath::Abs(State.RateDegPerSec) < 0.1f);
	return true;
}

#endif
