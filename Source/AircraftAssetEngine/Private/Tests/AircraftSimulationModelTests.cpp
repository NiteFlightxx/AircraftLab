#include "AircraftAsset/AircraftSimulationModel.h"
#include "Aircraft/ConstraintDriveUtils.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "AircraftAsset/AircraftPilotInputMapping.h"
#include "AircraftAsset/AircraftDataflowPreviewActor.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationProxy.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "Dataflow/DataflowSimulationManager.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDataflowPreviewActorProtocolsTest,
	"AircraftLab.Dataflow.Preview.ActorProtocols",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDataflowPreviewActorProtocolsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const UClass* const PreviewActorClass = AAircraftDataflowPreviewActor::StaticClass();
	TestTrue(TEXT("The preview actor receives Dataflow play, pause, and step state"),
		PreviewActorClass->ImplementsInterface(UDataflowSimulationActor::StaticClass()));
	TestTrue(TEXT("The preview actor supplies the authoritative preview movement intent"),
		PreviewActorClass->ImplementsInterface(UAircraftMovementIntentProvider::StaticClass()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftWorldChaosBackendOwnershipTest,
	"AircraftLab.Dataflow.Preview.WorldChaosBackendOwnership",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftWorldChaosBackendOwnershipTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const UAircraftComponent* const Component = GetDefault<UAircraftComponent>();
	TestEqual(TEXT("An unregistered component has no initialized backend"),
		Component->GetSimulationBackendStatus().State,
		EAircraftSimulationBackendState::Uninitialized);
	const AAircraftDataflowPreviewActor* const PreviewActor =
		GetDefault<AAircraftDataflowPreviewActor>();
	TestTrue(TEXT("The preview scenario targets world center of mass (0,0,200)"),
		PreviewActor->GetPreviewHoldTargetCm().Equals(FVector(0.0, 0.0, 200.0)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftProxyConfigurationExecutionDomainTest,
	"AircraftLab.Dataflow.Preview.ConfigurationExecutionDomain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftProxyConfigurationExecutionDomainTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	UAircraftComponent* const Component = NewObject<UAircraftComponent>();
	TestNotNull(TEXT("A transient aircraft component can host the proxy lifecycle"), Component);
	if (!Component)
	{
		return false;
	}

	FAircraftSimulationProxy PhysicsProxy(*Component);
	PhysicsProxy.Initialize_GameThread();
	TestFalse(TEXT("A queued configuration is not reported as applied"),
		PhysicsProxy.IsConfigurationApplied_GameThread());
	PhysicsProxy.TickPhysicsThread(1.0f / 60.0f, 0.0f);
	TestTrue(TEXT("The physics execution domain acknowledges its queued configuration"),
		PhysicsProxy.IsConfigurationApplied_GameThread());

	FAircraftSimulationProxy KinematicProxy(*Component);
	KinematicProxy.Initialize_GameThread();
	TestFalse(TEXT("The kinematic proxy begins with a queued configuration"),
		KinematicProxy.IsConfigurationApplied_GameThread());
	const FAircraftSimulationLodModel KinematicModel;
	KinematicProxy.TickKinematicTrajectory_GameThread(
		1.0f / 60.0f, 0.0, FTransform::Identity, FVector::ZeroVector,
		FVector::ZeroVector, FVector::ZeroVector, KinematicModel);
	TestTrue(TEXT("The kinematic execution domain acknowledges its queued configuration"),
		KinematicProxy.IsConfigurationApplied_GameThread());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDataflowDefaultAxesTest,
	"AircraftLab.Dataflow.Runtime.DefaultForwardIsPositiveY",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDataflowDefaultAxesTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const FAircraftFlightControllerRuntimeConfig Config;
	TestEqual(TEXT("The Dataflow runtime defaults to model-local +Y forward"),
		Config.FrameBinding.GetModelForwardAxis(), EAircraftModelForwardAxis::PositiveY);
	TestTrue(TEXT("Forward maps to model-local +Y"),
		Config.FrameBinding.GetForwardAxisModel().Equals(FVector::RightVector, 1.e-4f));
	TestTrue(TEXT("Right maps to model-local -X"),
		Config.FrameBinding.GetRightAxisModel().Equals(-FVector::ForwardVector, 1.e-4f));
	TestTrue(TEXT("Identity body rotation exposes a +90 degree control heading"),
		FMath::IsNearlyEqual(Config.GetControlWorldRotation(FQuat::Identity).Rotator().Yaw, 90.0f, 1.e-4f));

	const FVector ControllerTorque(1.0f, 2.0f, 3.0f);
	const FVector BodyTorque = Config.ControllerTorqueToBody(ControllerTorque);
	TestTrue(TEXT("Configured torque mapping round-trips"),
		Config.BodyAngularToController(BodyTorque).Equals(ControllerTorque, 1.e-4f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftOptionalSolverConfigTest,
	"AircraftLab.Dataflow.Runtime.OptionalSolverConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftOptionalSolverConfigTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	using namespace UE::AircraftLab::AircraftAsset;

	const TSharedRef<FManagedArrayCollection> Collection = MakeShared<FManagedArrayCollection>();
	FAircraftCollection AircraftCollection(Collection);
	AircraftCollection.DefineSchema();

	const TArray<TSharedRef<const FManagedArrayCollection>> CollectionsWithoutOverride = { Collection };
	const FAircraftSimulationModel ProjectSettingsModel(CollectionsWithoutOverride, TEXT("ProjectSettings"));
	const FAircraftSimulationLodModel* const ProjectSettingsLOD = ProjectSettingsModel.GetLodModel(0);
	TestNotNull(TEXT("A collection compiles one LOD model"), ProjectSettingsLOD);
	const FAircraftFlightControllerRuntimeConfig RuntimeDefaults;
	TestTrue(TEXT("Constraint strength is interpreted as frequency in Hz"),
		FMath::IsNearlyEqual(
			UE::AircraftLab::ConstraintDrive::StrengthToAngularFrequency(2.0),
			2.0 * UE_DOUBLE_TWO_PI, 1.e-6));
	float ConstraintStiffness = 0.0f;
	float ConstraintDamping = 0.0f;
	UE::AircraftLab::ConstraintDrive::ConvertStrengthToSpringParams(
		ConstraintStiffness, ConstraintDamping,
		RuntimeDefaults.ConstraintLinearNaturalFrequencyHz,
		RuntimeDefaults.ConstraintLinearDampingRatio,
		RuntimeDefaults.ConstraintLinearExtraDampingPerSecond);
	TestTrue(TEXT("Default constraint strength converts to the previous stiffness"),
		FMath::IsNearlyEqual(ConstraintStiffness, 100.0f, 1.e-3f));
	TestTrue(TEXT("Default constraint damping ratio converts to the previous damping"),
		FMath::IsNearlyEqual(ConstraintDamping, 20.0f, 1.e-3f));
	TestEqual(TEXT("Constraint gravity feed-forward defaults to full compensation"),
		RuntimeDefaults.ConstraintGravityFeedForwardScale, 1.0f);
	TestEqual(TEXT("Constraint linear-damping feed-forward defaults to full compensation"),
		RuntimeDefaults.ConstraintDynamicsFeedForwardScale, 1.0f);
	const double VelocityTrackingTarget =
		UE::AircraftLab::ConstraintDrive::ComputeVelocityTrackingPositionTarget(
			100.0, 200.0, 800.0, 2.0);
	TestTrue(TEXT("Constraint position lead is derived from current velocity error"),
		FMath::IsNearlyEqual(
			VelocityTrackingTarget,
			100.0 + 600.0 / (2.0 * UE_DOUBLE_TWO_PI), 1.e-6));
	TestTrue(TEXT("Constraint position lead vanishes at the requested velocity"),
		FMath::IsNearlyEqual(
			UE::AircraftLab::ConstraintDrive::ComputeVelocityTrackingPositionTarget(
				100.0, 800.0, 800.0, 2.0),
			100.0, 1.e-6));
	TestTrue(TEXT("Zero constraint strength keeps the position target finite"),
		FMath::IsNearlyEqual(
			UE::AircraftLab::ConstraintDrive::ComputeVelocityTrackingPositionTarget(
				100.0, 200.0, 800.0, 0.0),
			100.0, 1.e-6));
	TestEqual(TEXT("Schema defaults preserve horizontal manual-flight speed"),
		ProjectSettingsLOD->FlightController.MaxHorizontalSpeedCmPerSec,
		RuntimeDefaults.MaxHorizontalSpeedCmPerSec);
	TestEqual(TEXT("Schema defaults preserve climb speed"),
		ProjectSettingsLOD->FlightController.MaxClimbRateCmPerSec,
		RuntimeDefaults.MaxClimbRateCmPerSec);
	TestEqual(TEXT("Schema defaults preserve yaw rate"),
		ProjectSettingsLOD->FlightController.MaxYawRateDegreesPerSec,
		RuntimeDefaults.MaxYawRateDegreesPerSec);
	TestTrue(TEXT("Schema defaults preserve position gains"),
		ProjectSettingsLOD->FlightController.PositionKp.Equals(RuntimeDefaults.PositionKp));
	FAircraftPilotInput RollInput;
	RollInput.Roll = 1.0f;
	const FAircraftManualCommand ConstraintManualCommand =
		UE::AircraftLab::PilotInputMapping::BuildManualCommand(
			RollInput, FQuat::Identity, ProjectSettingsLOD->FlightController);
	TestTrue(TEXT("A PhysicsConstraint manual roll input produces horizontal target velocity"),
		FVector2D(ConstraintManualCommand.DesiredVelocityCmPerSec.X,
			ConstraintManualCommand.DesiredVelocityCmPerSec.Y).Size() > 1.0f);
	TestFalse(TEXT("An empty Solver group uses project physics settings"), ProjectSettingsLOD->bOverrideSolverAsyncDeltaTime);
	TestFalse(TEXT("An empty Solver group uses project iteration settings"), ProjectSettingsLOD->bOverrideSolverIterationCounts);
	TestEqual(TEXT("No override stores a zero solver delta"), ProjectSettingsLOD->SolverAsyncDeltaTime, 0.0f);

	FCollectionAircraftPropertyMutableFacade FrameProperties(Collection);
	FrameProperties.DefineSchema();
	const int32 ForwardAxisIndex = FrameProperties.AddProperty(
		TEXT("Frame.ForwardAxis"), EAircraftCollectionPropertyFlags::Enabled);
	FrameProperties.SetValue(ForwardAxisIndex, static_cast<int32>(EAircraftModelForwardAxis::NegativeY));
	const FAircraftSimulationModel FrameConfigModel(CollectionsWithoutOverride, TEXT("FrameConfig"));
	TestEqual(TEXT("Frame config owns the model-space forward selection"),
		FrameConfigModel.GetLodModel(0)->FlightController.FrameBinding.GetModelForwardAxis(),
		EAircraftModelForwardAxis::NegativeY);
	TestTrue(TEXT("Model-space Up remains fixed at +Z for every forward selection"),
		FrameConfigModel.GetLodModel(0)->FlightController.FrameBinding.GetUpAxisModel().Equals(
			FVector::UpVector, 1.e-4));
	Collection->AddElements(1, AircraftCollectionGroup::Solver);
	AircraftCollection.UpdateArrays();
	(*AircraftCollection.GetAsyncFixedTimeStepSize())[0] = 1.0f / 120.0f;

	const TArray<TSharedRef<const FManagedArrayCollection>> CollectionsWithOverride = { Collection };
	const FAircraftSimulationModel OverrideModel(CollectionsWithOverride, TEXT("Override"));
	const FAircraftSimulationLodModel* const OverrideLOD = OverrideModel.GetLodModel(0);
	TestTrue(TEXT("A Solver entry enables the aircraft override"), OverrideLOD->bOverrideSolverAsyncDeltaTime);
	TestFalse(TEXT("Iteration override remains independently disabled"), OverrideLOD->bOverrideSolverIterationCounts);
	TestTrue(TEXT("The configured async fixed time step reaches the runtime model"),
		FMath::IsNearlyEqual(OverrideLOD->SolverAsyncDeltaTime, 1.0f / 120.0f));

	(*AircraftCollection.GetOverrideIterationCounts())[0] = uint8(1);
	(*AircraftCollection.GetPositionSolverIterationCount())[0] = 12;
	(*AircraftCollection.GetVelocitySolverIterationCount())[0] = 3;
	(*AircraftCollection.GetProjectionSolverIterationCount())[0] = 2;
	const FAircraftSimulationModel IterationOverrideModel(CollectionsWithOverride, TEXT("IterationOverride"));
	const FAircraftSimulationLodModel* const IterationOverrideLOD = IterationOverrideModel.GetLodModel(0);
	TestTrue(TEXT("Iteration override can be enabled independently"), IterationOverrideLOD->bOverrideSolverIterationCounts);
	TestEqual(TEXT("Position iterations reach the runtime model"), IterationOverrideLOD->PositionSolverIterationCount, uint8(12));
	TestEqual(TEXT("Velocity iterations reach the runtime model"), IterationOverrideLOD->VelocitySolverIterationCount, uint8(3));
	TestEqual(TEXT("Projection iterations reach the runtime model"), IterationOverrideLOD->ProjectionSolverIterationCount, uint8(2));

	const TSharedRef<FManagedArrayCollection> Lod0Collection = MakeShared<FManagedArrayCollection>(*Collection);
	const TSharedRef<FManagedArrayCollection> Lod1Collection = MakeShared<FManagedArrayCollection>(*Collection);
	auto SetLodSettings = [](const TSharedRef<FManagedArrayCollection>& LodCollection, const TCHAR* Name, int32 DriveMode)
	{
		FCollectionAircraftPropertyMutableFacade Properties(LodCollection);
		Properties.DefineSchema();
		const int32 NameIndex = Properties.AddProperty(TEXT("SimulationLOD.Name"), EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetStringValue(NameIndex, Name);
		const int32 DriveIndex = Properties.AddProperty(TEXT("SimulationLOD.DriveMode"), EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(DriveIndex, DriveMode);
		const int32 CollisionIndex = Properties.AddProperty(TEXT("SimulationLOD.CollisionMode"), EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(CollisionIndex, 2);
	};
	SetLodSettings(Lod0Collection, TEXT("LOD0"),
		static_cast<int32>(EAircraftSimulationDriveMode::PhysicsConstraint));
	SetLodSettings(Lod1Collection, TEXT("LOD1"),
		static_cast<int32>(EAircraftSimulationDriveMode::FlightController));
	const TArray<TSharedRef<const FManagedArrayCollection>> TwoLodCollections = { Lod0Collection, Lod1Collection };
	const FAircraftSimulationModel TwoLodModel(TwoLodCollections, TEXT("TwoLOD"));
	TestEqual(TEXT("Each terminal collection compiles to one runtime LOD"), TwoLodModel.GetNumLods(), 2);
	TestTrue(TEXT("LOD 1 is addressable"), TwoLodModel.IsValidLodIndex(1));
	TestEqual(TEXT("LOD 0 reads its own physics-constraint profile"),
		TwoLodModel.SimulationLOD.LODs[0].DriveMode, EAircraftSimulationDriveMode::PhysicsConstraint);
	TestEqual(TEXT("LOD 1 reads its own flight-controller profile"),
		TwoLodModel.SimulationLOD.LODs[1].DriveMode, EAircraftSimulationDriveMode::FlightController);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftAutopilotConfigTest,
	"AircraftLab.Dataflow.Runtime.AutopilotConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftAutopilotConfigTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	using namespace UE::AircraftLab::AircraftAsset;

	const TSharedRef<FManagedArrayCollection> Collection = MakeShared<FManagedArrayCollection>();
	FAircraftCollection AircraftCollection(Collection);
	AircraftCollection.DefineSchema();

	// 无属性键时，Autopilot 与可选空气动力学使用权威默认语义。
	const TArray<TSharedRef<const FManagedArrayCollection>> Collections = { Collection };
	const FAircraftSimulationModel DefaultModel(Collections, TEXT("Defaults"));
	const FAircraftSimulationLodModel* const DefaultLOD = DefaultModel.GetLodModel(0);
	TestNotNull(TEXT("A collection compiles one LOD model"), DefaultLOD);
	TestTrue(TEXT("Autopilot defaults form a valid complete configuration"),
		DefaultLOD->Autopilot.IsValid());
	TestFalse(TEXT("Aerodynamics remains absent without its Dataflow node"),
		DefaultLOD->bHasAerodynamics);

	FCollectionAircraftPropertyMutableFacade Properties(Collection);
	Properties.DefineSchema();
	auto SetInt = [&Properties](const TCHAR* Key, int32 Value)
	{
		const int32 Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(Index, Value);
	};
	auto SetFloat = [&Properties](const TCHAR* Key, float Value)
	{
		const int32 Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(Index, Value);
	};

	SetFloat(TEXT("Autopilot.Path.ResampleSpacingCm"), 75.0f);
	SetFloat(TEXT("Autopilot.Path.ProjectionBacktrackToleranceCm"), 15.0f);
	SetFloat(TEXT("Autopilot.Path.ProjectionSearchDistanceCm"), 1500.0f);
	SetFloat(TEXT("Autopilot.Timing.ThrustReserveFraction"), 0.2f);
	SetFloat(TEXT("Autopilot.Timing.CurvatureAccelerationReserveFraction"), 0.22f);
	SetFloat(TEXT("Autopilot.Timing.SpeedConvergenceToleranceCmPerSec"), 0.75f);
	SetInt(TEXT("Autopilot.Mpcc.HorizonSteps"), 24);
	SetInt(TEXT("Autopilot.Mpcc.MaxOptimizationIterations"), 4);
	SetFloat(TEXT("Autopilot.Mpcc.ContourErrorWeight"), 12.0f);
	SetFloat(TEXT("Autopilot.Mpcc.CorridorViolationWeight"), 900.0f);
	SetFloat(TEXT("Autopilot.Mpcc.YawResponseTimeSeconds"), 0.08f);
	SetFloat(TEXT("Autopilot.Tracking.ContourErrorGovernorScaleCm"), 240.0f);
	SetFloat(TEXT("FlightController.MaxHorizontalDecelerationCmPerSecSq"), 550.0f);
	SetFloat(TEXT("FlightController.MaxHorizontalJerkCmPerSecCubed"), 1800.0f);
	SetFloat(TEXT("FlightController.MaxVerticalJerkCmPerSecCubed"), 1400.0f);
	SetFloat(TEXT("FlightController.MaxYawAccelerationDegPerSecSq"), 160.0f);
	SetFloat(TEXT("FlightController.MaxYawJerkDegPerSecCubed"), 500.0f);
	SetInt(TEXT("Aircraft.Initial.StartArmed"), 0);
	SetInt(TEXT("Aircraft.Initial.FlightMode"), static_cast<int32>(EAircraftFlightMode::VelocityHold));
	SetInt(TEXT("Aerodynamics.Configured"), 1);
	SetFloat(TEXT("Aerodynamics.AirDensityKgPerM3"), 1.1f);

	const FAircraftSimulationModel ConfiguredModel(Collections, TEXT("Configured"));
	const FAircraftSimulationLodModel* const LOD = ConfiguredModel.GetLodModel(0);
	TestNotNull(TEXT("Configured model compiles"), LOD);

	TestEqual(TEXT("Path spacing reaches the runtime model"), LOD->Autopilot.Path.ResampleSpacingCm, 75.0f);
	TestEqual(TEXT("Projection backtrack tolerance reaches the runtime model"),
		LOD->Autopilot.Path.ProjectionBacktrackToleranceCm, 15.0f);
	TestEqual(TEXT("Projection search distance reaches the runtime model"),
		LOD->Autopilot.Path.ProjectionSearchDistanceCm, 1500.0f);
	TestEqual(TEXT("Thrust reserve reaches the runtime model"), LOD->Autopilot.Timing.ThrustReserveFraction, 0.2f);
	TestEqual(TEXT("Curvature acceleration reserve reaches the runtime model"),
		LOD->Autopilot.Timing.CurvatureAccelerationReserveFraction, 0.22f);
	TestEqual(TEXT("Speed convergence tolerance reaches the runtime model"),
		LOD->Autopilot.Timing.SpeedConvergenceToleranceCmPerSec, 0.75f);
	TestEqual(TEXT("MPCC horizon reaches the runtime model"), LOD->Autopilot.Mpcc.HorizonSteps, 24);
	TestEqual(TEXT("MPCC iteration budget reaches the runtime model"),
		LOD->Autopilot.Mpcc.MaxOptimizationIterations, 4);
	TestEqual(TEXT("MPCC contour weight reaches the runtime model"), LOD->Autopilot.Mpcc.ContourErrorWeight, 12.0f);
	TestEqual(TEXT("MPCC corridor weight reaches the runtime model"),
		LOD->Autopilot.Mpcc.CorridorViolationWeight, 900.0f);
	TestEqual(TEXT("Yaw response time reaches the runtime model"), LOD->Autopilot.Mpcc.YawResponseTimeSeconds, 0.08f);
	TestEqual(TEXT("Contour governor scale reaches the runtime model"), LOD->Autopilot.Tracking.ContourErrorGovernorScaleCm, 240.0f);
	TestEqual(TEXT("Horizontal deceleration reaches the runtime model"),
		LOD->FlightController.MaxHorizontalDecelerationCmPerSecSq, 550.0f);
	TestEqual(TEXT("Horizontal jerk reaches the runtime model"),
		LOD->FlightController.MaxHorizontalJerkCmPerSecCubed, 1800.0f);
	TestEqual(TEXT("Vertical jerk reaches the runtime model"),
		LOD->FlightController.MaxVerticalJerkCmPerSecCubed, 1400.0f);
	TestEqual(TEXT("Yaw acceleration reaches the runtime model"),
		LOD->FlightController.MaxYawAccelerationDegPerSecSq, 160.0f);
	TestEqual(TEXT("Yaw jerk reaches the runtime model"),
		LOD->FlightController.MaxYawJerkDegPerSecCubed, 500.0f);
	TestFalse(TEXT("Initial arm state reaches the runtime model"),
		LOD->FlightController.bStartArmed);
	TestEqual(TEXT("Initial flight mode reaches the runtime model"),
		LOD->FlightController.InitialFlightMode, EAircraftFlightMode::VelocityHold);
	TestTrue(TEXT("Aerodynamics node presence reaches the runtime model"), LOD->bHasAerodynamics);
	TestEqual(TEXT("Air density reaches the runtime model"), LOD->Aerodynamics.AirDensityKgPerM3, 1.1f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCompletePidConfigCompilationTest,
	"AircraftLab.Dataflow.Runtime.CompletePidConfigCompilation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCompletePidConfigCompilationTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	using namespace UE::AircraftLab::AircraftAsset;

	const TSharedRef<FManagedArrayCollection> Collection = MakeShared<FManagedArrayCollection>();
	FAircraftCollection AircraftCollection(Collection);
	AircraftCollection.DefineSchema();
	FCollectionAircraftPropertyMutableFacade Properties(Collection);
	Properties.DefineSchema();
	auto Set = [&Properties](const TCHAR* Key, const auto& Value)
	{
		const int32 Index = Properties.AddProperty(Key, EAircraftCollectionPropertyFlags::Enabled);
		Properties.SetValue(Index, Value);
	};

	Set(TEXT("FlightController.Position.PositionKff"), FVector3f(0.7f, 0.8f, 0.0f));
	Set(TEXT("FlightController.Position.PositionDerivativeCutoffHz"), FVector3f(3.0f, 4.0f, 0.0f));
	Set(TEXT("FlightController.Position.PositionXFreezeIntegralWhenSaturated"), false);
	Set(TEXT("FlightController.Position.VelocityKff"), FVector3f(0.5f, 0.6f, 0.0f));
	Set(TEXT("FlightController.Position.VelocityDerivativeCutoffHz"), FVector3f(8.0f, 9.0f, 0.0f));
	Set(TEXT("FlightController.Attitude.RollRateFreezeIntegralWhenSaturated"), false);
	Set(TEXT("FlightController.Altitude.AltitudeKff"), 0.75f);
	Set(TEXT("FlightController.Altitude.AltitudeDerivativeCutoffHz"), 2.0f);
	Set(TEXT("FlightController.Altitude.VerticalVelocityFreezeIntegralWhenSaturated"), false);
	Set(TEXT("FlightController.Execution.ControllerEnabledByDefault"), false);
	Set(TEXT("FlightController.Constraint.Linear.GravityFeedForwardScale"), 0.8f);
	Set(TEXT("FlightController.Constraint.Linear.DynamicsFeedForwardScale"), 0.6f);

	const TArray<TSharedRef<const FManagedArrayCollection>> Collections = { Collection };
	const FAircraftSimulationModel Model(Collections, TEXT("CompletePid"));
	const FAircraftFlightControllerRuntimeConfig& Config = Model.GetLodModel(0)->FlightController;
	TestEqual(TEXT("Position X Kff compiles"), Config.GetPositionPidGains(0).Kff, 0.7f);
	TestEqual(TEXT("Position Y derivative cutoff compiles"), Config.GetPositionPidGains(1).DerivativeCutoffHz, 4.0f);
	TestFalse(TEXT("Position X anti-windup configuration compiles"), Config.GetPositionPidGains(0).bFreezeIntegralWhenSaturated);
	TestEqual(TEXT("Velocity Y Kff compiles"), Config.GetVelocityPidGains(1).Kff, 0.6f);
	TestEqual(TEXT("Velocity X derivative cutoff compiles"), Config.GetVelocityPidGains(0).DerivativeCutoffHz, 8.0f);
	TestFalse(TEXT("Roll-rate anti-windup configuration compiles"), Config.GetRatePidGains(0).bFreezeIntegralWhenSaturated);
	TestEqual(TEXT("Altitude Kff compiles"), Config.GetAltitudePidGains().Kff, 0.75f);
	TestEqual(TEXT("Altitude derivative cutoff compiles"), Config.GetAltitudePidGains().DerivativeCutoffHz, 2.0f);
	TestFalse(TEXT("Vertical-velocity anti-windup configuration compiles"), Config.GetVerticalVelocityPidGains().bFreezeIntegralWhenSaturated);
	TestFalse(TEXT("Controller execution default compiles"), Config.bControllerEnabledByDefault);
	TestEqual(TEXT("Constraint gravity feed-forward compiles"),
		Config.ConstraintGravityFeedForwardScale, 0.8f);
	TestEqual(TEXT("Constraint linear-damping feed-forward compiles"),
		Config.ConstraintDynamicsFeedForwardScale, 0.6f);
	return true;
}

#endif
