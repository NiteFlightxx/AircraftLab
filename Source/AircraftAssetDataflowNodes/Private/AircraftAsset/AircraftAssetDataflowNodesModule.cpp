#include "AircraftAsset/AircraftAssetDataflowNodesModule.h"

#include "Dataflow/DataflowCategoryRegistry.h"
#include "Dataflow/DataflowNode.h"
#include "Dataflow/AircraftAssetTerminalNode.h"

#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"

#include "AircraftAsset/AircraftAsset.h"


IMPLEMENT_MODULE(FAircraftAssetDataflowNodesModule, AircraftAssetDataflowNodes)

void FAircraftAssetDataflowNodesModule::StartupModule()
{
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY("Aircraft", FLinearColor(0.08f, 0.38f, 0.24f), FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY("Terminal", FLinearColor(0.65f, 0.16f, 0.12f), FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));

	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSkeletalMeshSourceNode);

	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSolverConfigNode);

	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAssetTerminalNode);

	UE::Dataflow::RegisterNodeFilter(FDataflowTerminalNode::StaticType());
	UE_DATAFLOW_REGISTER_CATEGORY_FORASSET_TYPE("Aircraft", UAircraftAsset);
}

void FAircraftAssetDataflowNodesModule::ShutdownModule()
{
	IModuleInterface::ShutdownModule();
}
