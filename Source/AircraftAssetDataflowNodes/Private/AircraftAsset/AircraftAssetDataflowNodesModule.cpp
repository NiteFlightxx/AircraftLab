#include "AircraftAsset/AircraftAssetDataflowNodesModule.h"

#include "Dataflow/DataflowCategoryRegistry.h"
#include "Dataflow/DataflowNode.h"

#include "Dataflow/AircraftAssetTerminalNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Dataflow/AircraftFrameConfigNode.h"
#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftFlightControllerConfigNodes.h"
#include "Dataflow/AircraftSimulationLODProfileNode.h"

#include "AircraftAsset/AircraftAsset.h"

IMPLEMENT_MODULE(FAircraftAssetDataflowNodesModule, AircraftAssetDataflowNodes)

void FAircraftAssetDataflowNodesModule::StartupModule()
{
	// 节点画布颜色：Aircraft 深绿，Terminal 深红（对齐 ChaosClothAsset 节点画布颜色风格）。
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY(
		"Aircraft", FLinearColor(0.f, 0.65f, 1.f), FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY(
		"Terminal", FLinearColor(0.65f, 0.16f, 0.12f), FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));

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
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSimulationLODProfileNode);

	/* Terminal 节点 */
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAssetTerminalNode);

	UE::Dataflow::RegisterNodeFilter(FDataflowTerminalNode::StaticType());
	UE_DATAFLOW_REGISTER_CATEGORY_FORASSET_TYPE("Aircraft", UAircraftAsset);
}

void FAircraftAssetDataflowNodesModule::ShutdownModule()
{
	IModuleInterface::ShutdownModule();
}
