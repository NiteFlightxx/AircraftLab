#include "AircraftAsset/AircraftSimulationBackendMath.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationProxy.h"
#include "Aircraft/AircraftPhysicsUnits.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "Aircraft/FlightControllerRuntimeConfig.h"

#include "PhysicsEngine/ConstraintInstance.h"

#include "Misc/AutomationTest.h"
#include "UObject/UObjectGlobals.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftBackendControlStateInvalidationTest,
	"AircraftLab.Dataflow.Runtime.Backend.ControlStateInvalidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftBackendControlStateInvalidationTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	using namespace UE::AircraftLab::SimulationBackend;

	TestFalse(TEXT("Identical configuration preserves backend-local state"),
		ShouldResetBackendControlState(
			0, EAircraftSimulationDriveMode::PhysicsConstraint,
			0, EAircraftSimulationDriveMode::PhysicsConstraint, false, true));
	TestTrue(TEXT("LOD change resets backend-local state"),
		ShouldResetBackendControlState(
			0, EAircraftSimulationDriveMode::PhysicsConstraint,
			1, EAircraftSimulationDriveMode::PhysicsConstraint, false, true));
	TestTrue(TEXT("Drive-mode change resets backend-local state"),
		ShouldResetBackendControlState(
			0, EAircraftSimulationDriveMode::FlightController,
			0, EAircraftSimulationDriveMode::PhysicsConstraint, false, true));
	TestTrue(TEXT("Explicit runtime reset resets backend-local state"),
		ShouldResetBackendControlState(
			0, EAircraftSimulationDriveMode::PhysicsConstraint,
			0, EAircraftSimulationDriveMode::PhysicsConstraint, true, true));
	TestTrue(TEXT("Invalid new model resets backend-local state"),
		ShouldResetBackendControlState(
			0, EAircraftSimulationDriveMode::PhysicsConstraint,
			0, EAircraftSimulationDriveMode::PhysicsConstraint, false, false));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftConstraintAngularDriveConfigurationTest,
	"AircraftLab.Dataflow.Runtime.Constraint.NativeAngularDriveConfiguration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftConstraintAngularDriveConfigurationTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	Component->SimulationConstraint = MakeShared<FConstraintInstance>();

	FAircraftFlightControllerRuntimeConfig Config;
	Config.ConstraintAttitudeNaturalFrequencyHz = 2.0f;
	Config.ConstraintAttitudeDampingRatio = 0.75f;
	Config.ConstraintAttitudeExtraDampingPerSecond = 0.5f;
	Config.ConstraintAttitudeTorqueLimitNm = 12.0f;
	Component->UpdateConstraintDriveAuthority(Config);

	FConstraintInstance& Constraint = *Component->SimulationConstraint;
	TestEqual(TEXT("Angular drive uses the native SLERP mode"),
		Constraint.GetAngularDriveMode(), EAngularDriveMode::SLERP);
	TestTrue(TEXT("Angular orientation target drive is enabled"),
		Constraint.GetOrientationDriveSLERP());
	TestTrue(TEXT("Angular velocity target drive is enabled"),
		Constraint.GetAngularVelocityDriveSLERP());
	TestTrue(TEXT("Angular drive uses acceleration mode"),
		Constraint.ProfileInstance.AngularDrive.GetAccelerationMode());

	float Spring = 0.0f;
	float Damping = 0.0f;
	float TorqueLimit = 0.0f;
	Constraint.GetAngularDriveParams(Spring, Damping, TorqueLimit);
	float ExpectedSpring = 0.0f;
	float ExpectedDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		ExpectedSpring, ExpectedDamping,
		Config.ConstraintAttitudeNaturalFrequencyHz,
		Config.ConstraintAttitudeDampingRatio,
		Config.ConstraintAttitudeExtraDampingPerSecond);
	TestEqual(TEXT("Angular stiffness comes from the existing attitude strength"),
		Spring, ExpectedSpring);
	TestEqual(TEXT("Angular damping comes from the existing damping semantics"),
		Damping, ExpectedDamping);
	TestEqual(TEXT("Angular torque limit is converted from Nm to Chaos units"),
		TorqueLimit,
		AircraftPhysicsUnits::NewtonMetersToChaosTorque(
			Config.ConstraintAttitudeTorqueLimitNm));

	Component->DisableSimulationConstraintDrive();
	TestFalse(TEXT("Disabling the simulation drive disables angular orientation"),
		Constraint.GetOrientationDriveSLERP());
	TestFalse(TEXT("Disabling the simulation drive disables angular velocity"),
		Constraint.GetAngularVelocityDriveSLERP());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftBackendControlBoundaryLifecycleTest,
	"AircraftLab.Dataflow.Runtime.Backend.ControlBoundaryLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftBackendControlBoundaryLifecycleTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	FAircraftSimulationProxy Proxy(*Component);

	TestFalse(TEXT("Initial state has no pending boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetArmRequest_GameThread(false);
	TestTrue(TEXT("Disarm invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetArmRequest_GameThread(false);
	TestFalse(TEXT("Repeated Disarm is not another boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetArmRequest_GameThread(true);
	TestTrue(TEXT("Disarm to Arm invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());

	Proxy.SetEmergencyStop_GameThread(true);
	TestTrue(TEXT("Emergency stop invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetArmRequest_GameThread(false);
	Proxy.SetArmRequest_GameThread(true);
	TestEqual(TEXT("Arm requests cannot override an active emergency stop"),
		Proxy.GetArmState_GameThread(), EAircraftArmState::EmergencyStop);
	Proxy.ConsumeBackendLocalStateReset_GameThread();
	Proxy.SetEmergencyStop_GameThread(false);
	TestTrue(TEXT("Clearing emergency stop invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	TestEqual(TEXT("Clearing emergency stop restores the requested arm state"),
		Proxy.GetArmState_GameThread(), EAircraftArmState::Armed);

	Proxy.SetControllerEnabled_GameThread(false);
	TestTrue(TEXT("Controller disable invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetControllerEnabled_GameThread(true);
	TestTrue(TEXT("Controller enable invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());

	Proxy.SetSimulationState_GameThread(true, true);
	TestTrue(TEXT("Suspend invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetSimulationState_GameThread(true, false);
	TestTrue(TEXT("Resume invalidates backend-local state"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	Proxy.SetSimulationState_GameThread(true, false);
	TestFalse(TEXT("Repeated execution policy is not another boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftBackendResetInterleavingTest,
	"AircraftLab.Dataflow.Runtime.Backend.RuntimeResetInterleaving",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftBackendResetInterleavingTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	FAircraftSimulationProxy Proxy(*Component);

	// Deterministic ordering of the race: the runtime-reset transaction dequeues and
	// consumes an old request, then GT publishes a new control boundary while the
	// transaction is still resetting execution-domain state.
	Proxy.bPendingBackendLocalStateReset.store(true, std::memory_order_release);
	Proxy.ConsumeBackendLocalStateResetCoveredByRuntimeReset_ExecutionThread();
	Proxy.SetControllerEnabled_GameThread(false);
	Proxy.ResetBackendLocalState_ExecutionThread();

	TestTrue(TEXT("A boundary published after runtime-reset dequeue survives completion"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	TestFalse(TEXT("The surviving boundary is consumed exactly once"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftBackendControlBoundaryRotorTopologyTest,
	"AircraftLab.Dataflow.Runtime.Backend.ControlBoundaryPreservesRotorTopology",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftBackendControlBoundaryRotorTopologyTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	FAircraftSimulationProxy Proxy(*Component);
	TArray<FAircraftRotorAllocationInfo> RotorDescriptors;
	RotorDescriptors.SetNum(4);
	for (int32 RotorIndex = 0; RotorIndex < RotorDescriptors.Num(); ++RotorIndex)
	{
		RotorDescriptors[RotorIndex].RotorName = FName(
			*FString::Printf(TEXT("Rotor%d"), RotorIndex));
	}
	Proxy.ControlAllocator.SetRotorDescriptors(RotorDescriptors);
	Proxy.ControlAllocator.CommandBuffer[0] = 0.75f;
	Proxy.RotorStates.SetNum(RotorDescriptors.Num());

	Proxy.SetControllerEnabled_GameThread(false);
	TestTrue(TEXT("Controller disable publishes a control boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	TestEqual(TEXT("The control boundary preserves allocation descriptors"),
		Proxy.ControlAllocator.RotorInfoBuffer.Num(), RotorDescriptors.Num());
	TestEqual(TEXT("The control boundary preserves matching rotor runtime slots"),
		Proxy.RotorStates.Num(), RotorDescriptors.Num());
	TestTrue(TEXT("The control boundary clears stale allocation commands"),
		Proxy.ControlAllocator.CommandBuffer.IsValidIndex(0)
		&& Proxy.ControlAllocator.CommandBuffer[0] == 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicSoftResetLifecycleTest,
	"AircraftLab.Dataflow.Runtime.Backend.KinematicSoftResetLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicSoftResetLifecycleTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	const FVector ActualAngularVelocityWorldRadPerSec(0.35, -0.2, 0.5);
	Component->KinematicAttitudeMotionState.bInitialized = true;
	Component->KinematicAttitudeDiagnostics.bValid = true;
	Component->PreviousAlternativeAngularVelocityWorldRadPerSec =
		ActualAngularVelocityWorldRadPerSec;
	Component->KinematicObstruction.Revision = 41;
	Component->KinematicObstruction.bBlocked = true;
	Component->KinematicObstruction.BlockingDurationSeconds = 2.0f;

	Component->SoftResetSimulation();

	TestFalse(TEXT("Soft reset clears GT-owned Kinematic attitude state"),
		Component->KinematicAttitudeMotionState.bInitialized);
	TestFalse(TEXT("Soft reset clears Kinematic attitude diagnostics"),
		Component->KinematicAttitudeDiagnostics.bValid);
	TestTrue(TEXT("Soft reset preserves the latest actual Kinematic angular velocity"),
		Component->PreviousAlternativeAngularVelocityWorldRadPerSec.Equals(
			ActualAngularVelocityWorldRadPerSec));
	TestFalse(TEXT("Soft reset clears sticky Kinematic obstruction"),
		Component->KinematicObstruction.bBlocked);
	TestEqual(TEXT("Soft reset publishes an obstruction-state revision"),
		Component->KinematicObstruction.Revision, uint64(42));

	FAircraftFlightControllerRuntimeConfig FrameConfig;
	FAircraftAttitudeMotionConfig MotionConfig;
	FAircraftAttitudeMotionOutput Reference;
	const bool bUpdated = FAircraftAttitudeReferenceDynamics::Update(
		FVector::ZeroVector, FVector::ZeroVector, 0.0f, 0.0f, 980.0f,
		1.0f / 60.0f, FQuat::Identity,
		Component->PreviousAlternativeAngularVelocityWorldRadPerSec,
		FrameConfig, MotionConfig, Component->KinematicAttitudeMotionState, Reference);
	TestTrue(TEXT("The first post-reset attitude reference is valid"), bUpdated);
	TestTrue(TEXT("The first post-reset reference initializes from measured angular velocity"),
		Component->KinematicAttitudeMotionState.AngularVelocityControlRadPerSec.Equals(
			FrameConfig.BodyToControlVector(ActualAngularVelocityWorldRadPerSec), 1.e-5));

	Component->SetEnableSimulation(false);
	TestTrue(TEXT("Explicit simulation disable clears actual angular-velocity history"),
		Component->PreviousAlternativeAngularVelocityWorldRadPerSec.IsZero());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicControlDisableLifecycleTest,
	"AircraftLab.Dataflow.Runtime.Backend.KinematicControlDisableLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicControlDisableLifecycleTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	FAircraftSimulationProxy Proxy(*Component);
	const FVector ActiveAngularVelocityWorldRadPerSec(0.3, -0.4, 0.7);

	auto VerifyDisabledFrameClearsActualAngularVelocity = [this, Component](const TCHAR* Context)
	{
		Component->KinematicAttitudeMotionState.bInitialized = true;
		Component->PreviousAlternativeAngularVelocityWorldRadPerSec =
			FVector(0.3, -0.4, 0.7);
		Component->ResetKinematicAttitudeState();
		Component->UpdateKinematicExecutionStoppedState(1.0f / 60.0f);
		TestTrue(Context,
			Component->PreviousAlternativeAngularVelocityWorldRadPerSec.IsZero());

		FAircraftFlightControllerRuntimeConfig FrameConfig;
		FAircraftAttitudeMotionConfig MotionConfig;
		FAircraftAttitudeMotionOutput Reference;
		const bool bUpdated = FAircraftAttitudeReferenceDynamics::Update(
			FVector::ZeroVector, FVector::ZeroVector, 0.0f, 0.0f, 980.0f,
			1.0f / 60.0f, FQuat::Identity,
			Component->PreviousAlternativeAngularVelocityWorldRadPerSec,
			FrameConfig, MotionConfig, Component->KinematicAttitudeMotionState, Reference);
		TestTrue(TEXT("Restarted Kinematic reference is valid"), bUpdated);
		TestTrue(TEXT("Restart initializes from the stationary actual angular velocity"),
			Component->KinematicAttitudeMotionState.AngularVelocityControlRadPerSec.IsZero());
	};

	Component->PreviousAlternativeAngularVelocityWorldRadPerSec =
		ActiveAngularVelocityWorldRadPerSec;
	Proxy.SetArmRequest_GameThread(false);
	TestTrue(TEXT("Disarm publishes a Kinematic control boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	VerifyDisabledFrameClearsActualAngularVelocity(
		TEXT("One disarmed Kinematic frame clears stale actual angular velocity"));
	Proxy.SetArmRequest_GameThread(true);
	TestTrue(TEXT("Re-arm publishes a Kinematic control boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());

	Component->PreviousAlternativeAngularVelocityWorldRadPerSec =
		ActiveAngularVelocityWorldRadPerSec;
	Proxy.SetControllerEnabled_GameThread(false);
	TestTrue(TEXT("Controller disable publishes a Kinematic control boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	VerifyDisabledFrameClearsActualAngularVelocity(
		TEXT("One controller-disabled Kinematic frame clears stale actual angular velocity"));
	Proxy.SetControllerEnabled_GameThread(true);
	TestTrue(TEXT("Controller enable publishes a Kinematic control boundary"),
		Proxy.ConsumeBackendLocalStateReset_GameThread());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftKinematicNavigationOutputAtomicCommitTest,
	"AircraftLab.Dataflow.Runtime.Backend.KinematicNavigationOutputAtomicCommit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftKinematicNavigationOutputAtomicCommitTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	FAircraftSimulationProxy Proxy(*Component);

	FAircraftNavigationOutputSnapshot Published;
	Published.PhysicsStateSequence = 17;
	Published.ControlSequence = 23;
	Published.VehicleState.TimeSeconds = 4.5;
	Published.VehicleState.Sequence = 17;
	Published.VehicleState.PositionCm = FVector(100.0, 200.0, 300.0);
	Published.TrajectoryReference.PositionCm = FVector(400.0, 500.0, 600.0);
	Published.DynamicCapability.Revision = 17;
	Published.DynamicCapability.bValid = true;
	Published.KinematicObstruction.Revision = 9;
	Published.KinematicObstruction.bBlocked = true;
	Published.KinematicObstruction.ActualPositionCm = Published.VehicleState.PositionCm;
	FAircraftEstimatedState Estimated;
	Estimated.State.TimeSeconds = Published.VehicleState.TimeSeconds;
	Estimated.State.PositionCm = Published.VehicleState.PositionCm;
	Published.EstimatedState = Estimated;

	Proxy.CommitKinematicNavigationOutput_GameThread(Published);
	FAircraftNavigationOutputSnapshot Readback;
	Proxy.GetNavigationOutputSnapshot_GameThread(Readback);
	TestEqual(TEXT("Vehicle and obstruction share the committed sequence"),
		Readback.PhysicsStateSequence, Published.PhysicsStateSequence);
	TestTrue(TEXT("Vehicle position is from the committed execution frame"),
		Readback.VehicleState.PositionCm.Equals(Published.VehicleState.PositionCm));
	TestTrue(TEXT("Reference is from the same committed execution frame"),
		Readback.TrajectoryReference.PositionCm.Equals(
			Published.TrajectoryReference.PositionCm));
	TestEqual(TEXT("Capability is from the same committed execution frame"),
		Readback.DynamicCapability.Revision, Published.DynamicCapability.Revision);
	TestTrue(TEXT("Obstruction is from the same committed execution frame"),
		Readback.KinematicObstruction.bBlocked
		&& Readback.KinematicObstruction.ActualPositionCm.Equals(
			Readback.VehicleState.PositionCm));

	FAircraftSimulationOutputFrame FullFrame;
	Proxy.GetSimulationOutputFrame_GameThread(FullFrame);
	TestEqual(TEXT("Full diagnostics frame uses the same obstruction revision"),
		FullFrame.KinematicObstruction.Revision,
		Published.KinematicObstruction.Revision);
	return true;
}

#endif
