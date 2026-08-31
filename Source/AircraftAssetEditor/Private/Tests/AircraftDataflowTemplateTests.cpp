#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"

#include "AircraftAsset/AircraftAsset.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"
#include "Dataflow/DataflowGraph.h"
#include "Dataflow/DataflowObject.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftCompleteDefaultDataflowTemplateTest,
	"AircraftLab.Dataflow.Template.CompleteDefault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftCompleteDefaultDataflowTemplateTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	UAircraftAsset* const AircraftAsset = NewObject<UAircraftAsset>(GetTransientPackage());
	UDataflow* const Dataflow = UE::AircraftDataflowAssetEditor::Private::CreateAircraftDataflowAsset(AircraftAsset);
	if (!TestNotNull(TEXT("The complete default Dataflow template is created"), Dataflow))
	{
		return false;
	}

	const TSharedPtr<UE::Dataflow::FGraph> Graph = Dataflow->GetDataflow();
	if (!TestTrue(TEXT("The template owns a Dataflow graph"), Graph.IsValid()))
	{
		return false;
	}

	const FName RequiredNodeNames[] =
	{
		TEXT("AircraftSkeletalMeshSource"),
		TEXT("AircraftSolverConfig"),
		TEXT("AircraftFrameConfig"),
		TEXT("OptionalAircraftAerodynamicsConfig"),
		TEXT("AircraftAirscrewProfile_Rotor1_FR"),
		TEXT("AircraftAirscrewProfile_Rotor2_FL"),
		TEXT("AircraftAirscrewProfile_Rotor3_RL"),
		TEXT("AircraftAirscrewProfile_Rotor4_RR"),
		TEXT("AircraftFlightControlLimitsConfig"),
		TEXT("AircraftPositionControllerConfig"),
		TEXT("AircraftAttitudeControllerConfig"),
		TEXT("AircraftAltitudeControllerConfig"),
		TEXT("AircraftControlAllocatorConfig"),
		TEXT("AircraftControllerInputConfig"),
		TEXT("AircraftAutopilotPathConfig"),
		TEXT("AircraftAutopilotTimingConfig"),
		TEXT("AircraftAutopilotMpccConfig"),
		TEXT("AircraftConstraintSimulationConfig"),
		TEXT("AircraftKinematicSimulationConfig"),
		TEXT("AircraftSimulationLODProfile_LOD0"),
		TEXT("AircraftSimulationLODProfile_LOD1"),
		TEXT("AircraftSimulationLODProfile_LOD2"),
		TEXT("AircraftSimulationLODProfile_LOD3"),
		TEXT("AircraftAssetTerminal"),
		TEXT("ReRouteNode_v1"),
		TEXT("ReRouteNode_v1_0"),
	};

	TestEqual(TEXT("The template contains every authoring node exactly once"),
		Graph->GetNodes().Num(), static_cast<int32>(UE_ARRAY_COUNT(RequiredNodeNames)));
	for (const FName NodeName : RequiredNodeNames)
	{
		TestTrue(*FString::Printf(TEXT("Template contains %s"), *NodeName.ToString()),
			Graph->FindBaseNode(NodeName).IsValid());
	}

	TestEqual(TEXT("The shared chain and four LOD branches are fully connected"),
		Graph->GetConnections().Num(), 27);

	const TSharedPtr<FDataflowNode> AerodynamicsNode =
		Graph->FindBaseNode(TEXT("OptionalAircraftAerodynamicsConfig"));
	if (TestTrue(TEXT("The optional aerodynamics node exists"), AerodynamicsNode.IsValid()))
	{
		bool bAerodynamicsConnected = false;
		for (const UE::Dataflow::FLink& Link : Graph->GetConnections())
		{
			bAerodynamicsConnected |= Link.InputNode == AerodynamicsNode->GetGuid()
				|| Link.OutputNode == AerodynamicsNode->GetGuid();
		}
		TestFalse(TEXT("Aerodynamics remains opt-in until inserted into the Collection chain"),
			bAerodynamicsConnected);
	}

	const TSharedPtr<FDataflowNode> MpccNode =
		Graph->FindBaseNode(TEXT("AircraftAutopilotMpccConfig"));
	const TSharedPtr<FDataflowNode> ConstraintNode =
		Graph->FindBaseNode(TEXT("AircraftConstraintSimulationConfig"));
	const TSharedPtr<FDataflowNode> KinematicNode =
		Graph->FindBaseNode(TEXT("AircraftKinematicSimulationConfig"));
	const TSharedPtr<FDataflowNode> ReRouteRerouteNode =
		Graph->FindBaseNode(TEXT("ReRouteNode_v1_0"));
	const TSharedPtr<FDataflowNode> ExpectedLodSources[] =
	{
		MpccNode,
		ConstraintNode,
		KinematicNode,
		ReRouteRerouteNode,
	};
	const EAircraftSimulationDriveMode ExpectedDriveModes[] =
	{
		EAircraftSimulationDriveMode::FlightController,
		EAircraftSimulationDriveMode::PhysicsConstraint,
		EAircraftSimulationDriveMode::Kinematic,
		EAircraftSimulationDriveMode::Kinematic,
	};
	for (int32 LodIndex = 0; LodIndex < UE_ARRAY_COUNT(ExpectedDriveModes); ++LodIndex)
	{
		const FName ProfileName(*FString::Printf(TEXT("AircraftSimulationLODProfile_LOD%d"), LodIndex));
		const TSharedPtr<FDataflowNode> ProfileNode = Graph->FindBaseNode(ProfileName);
		const TSharedPtr<FDataflowNode> ExpectedSource = ExpectedLodSources[LodIndex];
		if (!TestTrue(*FString::Printf(TEXT("LOD%d source and profile exist"), LodIndex),
			ExpectedSource.IsValid() && ProfileNode.IsValid()))
		{
			continue;
		}

		bool bReceivesFromExpectedSource = false;
		for (const UE::Dataflow::FLink& Link : Graph->GetConnections())
		{
			bReceivesFromExpectedSource |= Link.OutputNode == ExpectedSource->GetGuid()
				&& Link.InputNode == ProfileNode->GetGuid();
		}
		TestTrue(*FString::Printf(TEXT("LOD%d receives its dedicated drive source"), LodIndex),
			bReceivesFromExpectedSource);

		const FAircraftSimulationLODProfileNode* const TypedProfile =
			ProfileNode->AsType<FAircraftSimulationLODProfileNode>();
		if (TestNotNull(*FString::Printf(TEXT("LOD%d has the profile node type"), LodIndex), TypedProfile))
		{
			TestEqual(*FString::Printf(TEXT("LOD%d keeps its editable initial drive mode"), LodIndex),
				TypedProfile->Profile.DriveMode, ExpectedDriveModes[LodIndex]);
		}
	}

	return !HasAnyErrors();
}

#endif
