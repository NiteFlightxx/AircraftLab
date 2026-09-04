#include "AircraftAsset/AircraftCollection.h"
#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftAerodynamicsConfigNode.h"
#include "Dataflow/AircraftAutopilotMpccConfigNode.h"
#include "Dataflow/AircraftAutopilotPathConfigNode.h"
#include "Dataflow/AircraftAutopilotTimingConfigNode.h"
#include "Dataflow/AircraftFlightControlLimitsConfigNode.h"
#include "Dataflow/AircraftPositionControllerConfigNode.h"
#include "Dataflow/AircraftAttitudeControllerConfigNode.h"
#include "Dataflow/AircraftAltitudeControllerConfigNode.h"
#include "Dataflow/AircraftControlAllocatorConfigNode.h"
#include "Dataflow/AircraftConstraintSimulationConfigNode.h"
#include "Dataflow/AircraftControllerInputConfigNode.h"
#include "Dataflow/AircraftFrameConfigNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Dataflow/DataflowNodeParameters.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FManagedArrayCollection EvaluateConfigNode(const TSharedRef<const FAircraftConfigNodeBase>& Node)
	{
		UE::Dataflow::FContextSingle Context;
		const FDataflowOutput* const Output = Node->FindOutput(FName(TEXT("Collection")));
		check(Output);
		return Output->GetValue<FManagedArrayCollection>(Context, FManagedArrayCollection());
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftConfigNodeArchitectureTest,
	"AircraftLab.Dataflow.ConfigNodes.UnifiedArchitecture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftConfigNodeArchitectureTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

#define TEST_AIRCRAFT_CONFIG_NODE(NodeType) \
	TestTrue(TEXT(#NodeType " derives from FAircraftConfigNodeBase"), \
		TIsDerivedFrom<NodeType, FAircraftConfigNodeBase>::Value); \
	{ \
		const NodeType Node(UE::Dataflow::FNodeParameters{}); \
		TestEqual(TEXT(#NodeType " owns one Collection input"), Node.GetNumInputs(), 1); \
		TestEqual(TEXT(#NodeType " owns one Collection output"), Node.NumOutputs(), 1); \
		TestTrue(TEXT(#NodeType " supports Construction debug draw"), Node.CanDebugDraw()); \
	}

	TEST_AIRCRAFT_CONFIG_NODE(FAircraftFrameConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAirscrewProfileNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAerodynamicsConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftControllerInputConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftPositionControllerConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAttitudeControllerConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAltitudeControllerConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftControlAllocatorConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftFlightControlLimitsConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftConstraintSimulationConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftKinematicSimulationConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAutopilotPathConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAutopilotTimingConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftAutopilotMpccConfigNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftSimulationLODProfileNode);
	TEST_AIRCRAFT_CONFIG_NODE(FAircraftSolverConfigNode);

#undef TEST_AIRCRAFT_CONFIG_NODE
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftConfigNodeTransactionTest,
	"AircraftLab.Dataflow.ConfigNodes.TransactionalEvaluation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftConfigNodeTransactionTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	FManagedArrayCollection SentinelCollection;
	SentinelCollection.AddGroup(TEXT("Sentinel"));
	SentinelCollection.AddElements(1, TEXT("Sentinel"));
	SentinelCollection.AddAttribute<int32>(TEXT("Value"), TEXT("Sentinel"))[0] = 42;

	const TSharedRef<FAircraftFrameConfigNode> InvalidFrame =
		MakeShared<FAircraftFrameConfigNode>(UE::Dataflow::FNodeParameters{});
	InvalidFrame->Collection = SentinelCollection;
	InvalidFrame->MassKg = -1.0f;
	const FManagedArrayCollection FrameResult = EvaluateConfigNode(InvalidFrame);
	TestEqual(TEXT("Invalid frame preserves the original sentinel"),
		FrameResult.GetAttribute<int32>(TEXT("Value"), TEXT("Sentinel"))[0], 42);
	TestFalse(TEXT("Invalid frame does not leak the transaction's Frame schema"),
		FrameResult.HasGroup(UE::AircraftLab::AircraftAsset::AircraftCollectionGroup::Frame));

	const TSharedRef<FAircraftAirscrewProfileNode> FirstRotor =
		MakeShared<FAircraftAirscrewProfileNode>(UE::Dataflow::FNodeParameters{});
	FirstRotor->Collection = SentinelCollection;
	FirstRotor->Profile.Name = TEXT("RotorA");
	const FManagedArrayCollection OneRotorCollection = EvaluateConfigNode(FirstRotor);
	TestEqual(TEXT("Valid airscrew appends one motor"),
		OneRotorCollection.NumElements(UE::AircraftLab::AircraftAsset::AircraftCollectionGroup::Motors), 1);
	TestEqual(TEXT("Valid airscrew appends one propeller"),
		OneRotorCollection.NumElements(UE::AircraftLab::AircraftAsset::AircraftCollectionGroup::Propellers), 1);

	const TSharedRef<FAircraftAirscrewProfileNode> DuplicateRotor =
		MakeShared<FAircraftAirscrewProfileNode>(UE::Dataflow::FNodeParameters{});
	DuplicateRotor->Collection = OneRotorCollection;
	DuplicateRotor->Profile.Name = TEXT("RotorA");
	const FManagedArrayCollection DuplicateResult = EvaluateConfigNode(DuplicateRotor);
	TestEqual(TEXT("Duplicate airscrew does not leak a motor"),
		DuplicateResult.NumElements(UE::AircraftLab::AircraftAsset::AircraftCollectionGroup::Motors), 1);
	TestEqual(TEXT("Duplicate airscrew does not leak a propeller"),
		DuplicateResult.NumElements(UE::AircraftLab::AircraftAsset::AircraftCollectionGroup::Propellers), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftDataflowProfileDefaultsTest,
	"AircraftLab.Dataflow.Profiles.AuthoritativeDefaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftDataflowProfileDefaultsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	const FAircraftFrameConfigNode Frame(UE::Dataflow::FNodeParameters{});
	TestEqual(TEXT("Aircraft frame defaults to model +Y forward"),
		Frame.ForwardAxis, EAircraftModelForwardAxis::PositiveY);
	TestEqual(TEXT("Frame config exposes only its Collection input"), Frame.GetNumInputs(), 1);
	const FAircraftControllerInputConfig Input;
	TestEqual(TEXT("Controller input preserves authoritative horizontal hold deadband"),
		Input.HorizontalHoldStickDeadband, 0.08f);
	TestEqual(TEXT("Controller input preserves authoritative vertical hold deadband"),
		Input.VerticalHoldStickDeadband, 0.08f);
	TestEqual(TEXT("Controller input preserves authoritative yaw hold deadband"),
		Input.YawHoldStickDeadband, 0.05f);
	TestEqual(TEXT("Controller input preserves authoritative brake-to-hold speed"),
		Input.HorizontalBrakeToHoldSpeedCmPerSec, 20.0f);
	TestEqual(TEXT("Controller input preserves vertical brake-to-hold speed"),
		Input.VerticalBrakeToHoldSpeedCmPerSec, 20.0f);
	TestTrue(TEXT("Controller is enabled by default"), Input.bControllerEnabledByDefault);
	const FAircraftSolverConfigNode Solver(UE::Dataflow::FNodeParameters{});
	TestEqual(TEXT("Solver config exposes only its Collection input"), Solver.GetNumInputs(), 1);
	const FAircraftSkeletalMeshSourceNode Source(UE::Dataflow::FNodeParameters{});
	TestEqual(TEXT("Skeletal-mesh source has no configurable input pins"), Source.GetNumInputs(), 0);

	const FAircraftFlightControlLimitsConfig Limits;
	TestTrue(TEXT("Collective limits are ordered"),
		Limits.MinCollectiveCommand <= Limits.HoverCollectiveCommand
		&& Limits.HoverCollectiveCommand <= Limits.MaxCollectiveCommand);
	const FAircraftPositionControllerConfig Position;
	TestTrue(TEXT("Position controller has positive velocity derivative cutoff"),
		Position.VelocityX.DerivativeCutoffHz > 0.0f);
	TestEqual(TEXT("Position controller preserves authoritative velocity integral limit"),
		Position.VelocityX.IntegralLimit, 3000.0f);
	TestEqual(TEXT("Position controller preserves authoritative acceleration output limit"),
		Position.VelocityX.OutputLimit, 600.0f);
	const FAircraftAttitudeControllerConfig Attitude;
	TestTrue(TEXT("Attitude controller has a positive reference-model frequency"),
		Attitude.ReferenceModelNaturalFrequency > 0.0f);
	TestEqual(TEXT("Attitude controller preserves authoritative roll-rate output limit"),
		Attitude.RollRate.OutputLimit, 0.35f);
	TestEqual(TEXT("Attitude controller preserves authoritative yaw-rate output limit"),
		Attitude.YawRate.OutputLimit, 0.20f);
	const FAircraftAltitudeControllerConfig Altitude;
	TestTrue(TEXT("Altitude controller has a positive vertical-velocity derivative cutoff"),
		Altitude.VerticalVelocity.DerivativeCutoffHz > 0.0f);
	TestEqual(TEXT("Altitude controller preserves authoritative vertical integral limit"),
		Altitude.VerticalVelocity.IntegralLimit, 2500.0f);
	TestEqual(TEXT("Altitude controller preserves authoritative collective output limit"),
		Altitude.VerticalVelocity.OutputLimit, 0.30f);
	TestTrue(TEXT("Control allocator has non-negative damping"),
		FAircraftControlAllocatorConfig().DampedPseudoInverseLambda >= 0.0f);
	const FAircraftConstraintSimulationConfig Constraint;
	TestEqual(TEXT("Constraint simulation fully compensates gravity by default"),
		Constraint.GravityFeedForwardScale, 1.0f);
	TestEqual(TEXT("Constraint simulation fully compensates rigid-body linear damping by default"),
		Constraint.DynamicsFeedForwardScale, 1.0f);

	const FAircraftAirscrewProfileData Airscrew;
	TestFalse(TEXT("A single airscrew profile has an identity"), Airscrew.Name.IsNone());
	TestTrue(TEXT("A single airscrew profile is enabled by default"), Airscrew.bEnabled);
	TestTrue(TEXT("Airscrew profile has positive maximum RPM"), Airscrew.Motor.MaxRpm > Airscrew.Motor.IdleRpm);
	TestTrue(TEXT("Airscrew profile has a non-zero thrust axis"), !Airscrew.ThrustAxisLocal.IsNearlyZero());

	const FAircraftSimulationLODProfileData SimulationLOD;
	TestFalse(TEXT("A simulation LOD node has a name"), SimulationLOD.Name.IsNone());
	TestEqual(TEXT("A standalone simulation LOD node defaults to flight-controller drive"),
		SimulationLOD.DriveMode, EAircraftSimulationDriveMode::FlightController);
	TestEqual(TEXT("A standalone simulation LOD node defaults to full collision"),
		SimulationLOD.CollisionMode, EAircraftSimulationCollisionMode::QueryAndPhysics);
	return true;
}

#endif
