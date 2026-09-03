#include "AircraftAsset/AircraftAssetDataflowNodesModule.h"

#include "Dataflow/DataflowCategoryRegistry.h"
#include "Dataflow/DataflowNode.h"
#include "Dataflow/DataflowRenderingViewMode.h"
#include "Dataflow/AircraftDataflowViewModes.h"

#include "Dataflow/AircraftAssetTerminalNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Dataflow/AircraftFrameConfigNode.h"
#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftFlightControlLimitsConfigNode.h"
#include "Dataflow/AircraftPositionControllerConfigNode.h"
#include "Dataflow/AircraftAttitudeControllerConfigNode.h"
#include "Dataflow/AircraftAltitudeControllerConfigNode.h"
#include "Dataflow/AircraftControlAllocatorConfigNode.h"
#include "Dataflow/AircraftControllerInputConfigNode.h"
#include "Dataflow/AircraftConstraintSimulationConfigNode.h"
#include "Dataflow/AircraftKinematicSimulationConfigNode.h"
#include "Dataflow/AircraftAerodynamicsConfigNode.h"
#include "Dataflow/AircraftAutopilotMpccConfigNode.h"
#include "Dataflow/AircraftAutopilotPathConfigNode.h"
#include "Dataflow/AircraftAutopilotTimingConfigNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/ColorScheme.h"

IMPLEMENT_MODULE(FAircraftAssetDataflowNodesModule, AircraftAssetDataflowNodes)

void FAircraftAssetDataflowNodesModule::StartupModule()
{
	using FAircraftColorScheme = UE::AircraftLab::AircraftAsset::FColorScheme;
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY(
		"Aircraft", FAircraftColorScheme::NodeHeader, FAircraftColorScheme::NodeBody);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY(
		"Terminal", FAircraftColorScheme::TerminalNodeHeader, FAircraftColorScheme::TerminalNodeBody);

	/* Source 节点 */
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSkeletalMeshSourceNode);

	/* Config 节点（按顶层 → 局部 → 控制器 → 手感的顺序，与典型图布局一致） */
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSolverConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftFrameConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAirscrewProfileNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftFlightControlLimitsConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftPositionControllerConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAttitudeControllerConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAltitudeControllerConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftControlAllocatorConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftControllerInputConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftConstraintSimulationConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftKinematicSimulationConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAerodynamicsConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAutopilotPathConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAutopilotTimingConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAutopilotMpccConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSimulationLODProfileNode);

	/* Terminal 节点 */
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAssetTerminalNode);

	UE::Dataflow::RegisterNodeFilter(FDataflowTerminalNode::StaticType());
	UE_DATAFLOW_REGISTER_CATEGORY_FORASSET_TYPE("Aircraft", UAircraftAsset);
	UE::Dataflow::FRenderingViewModeFactory::GetInstance().RegisterViewMode(
		MakeUnique<UE::AircraftLab::DataflowNodes::FAircraft3DSimViewMode>());

}

void FAircraftAssetDataflowNodesModule::ShutdownModule()
{
	UE::Dataflow::FRenderingViewModeFactory::GetInstance().DeregisterViewMode(
		UE::AircraftLab::DataflowNodes::FAircraft3DSimViewMode::Name);
	IModuleInterface::ShutdownModule();
}
