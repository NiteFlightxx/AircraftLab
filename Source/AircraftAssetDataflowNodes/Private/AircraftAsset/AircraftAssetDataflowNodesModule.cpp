#include "AircraftAsset/AircraftAssetDataflowNodesModule.h"

#include "Dataflow/DataflowCategoryRegistry.h"
#include "Dataflow/DataflowNode.h"

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
#include "Dataflow/AircraftRotorFailurePolicyConfigNode.h"
#include "Dataflow/AircraftAutopilotConfigNode.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/ColorScheme.h"

IMPLEMENT_MODULE(FAircraftAssetDataflowNodesModule, AircraftAssetDataflowNodes)

/* 视口渲染回调（AircraftRotorRenderCallbacks.cpp） */
namespace UE::AircraftLab::DataflowNodes
{
	void RegisterAircraftRenderingCallbacks();
	void DeregisterAircraftRenderingCallbacks();
}

void FAircraftAssetDataflowNodesModule::StartupModule()
{
	// 节点画布颜色统一取自 FColorScheme（对齐 ChaosClothAssetTools/ColorScheme.h 的单点定义）。
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
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftRotorFailurePolicyConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAutopilotConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSimulationLODProfileNode);

	/* Terminal 节点 */
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAssetTerminalNode);

	UE::Dataflow::RegisterNodeFilter(FDataflowTerminalNode::StaticType());
	UE_DATAFLOW_REGISTER_CATEGORY_FORASSET_TYPE("Aircraft", UAircraftAsset);

	// 视口视图模式 + 旋翼渲染回调（对齐 ChaosCloth 的 RegisterRenderingCallbacks）
	UE::AircraftLab::DataflowNodes::RegisterAircraftRenderingCallbacks();
}

void FAircraftAssetDataflowNodesModule::ShutdownModule()
{
	UE::AircraftLab::DataflowNodes::DeregisterAircraftRenderingCallbacks();
	IModuleInterface::ShutdownModule();
}
