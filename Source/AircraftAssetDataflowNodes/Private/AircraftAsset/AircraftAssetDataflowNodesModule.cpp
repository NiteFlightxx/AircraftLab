#include "AircraftAsset/AircraftAssetDataflowNodesModule.h"

#include "Dataflow/DataflowCategoryRegistry.h"
#include "Dataflow/DataflowNode.h"
#include "Dataflow/AircraftAssetTerminalNode.h"
#include "Dataflow/AircraftAxleConfigNode.h"
#include "Dataflow/AircraftBrakeConfigNode.h"
#include "Dataflow/AircraftBrakeInputConfigNode.h"
#include "Dataflow/AircraftChassisConfigNode.h"
#include "Dataflow/AircraftClutchConfigNode.h"
#include "Dataflow/AircraftDifferentialConfigNode.h"
#include "Dataflow/AircraftEngineConfigNode.h"
#include "Dataflow/AircraftGearboxConfigNode.h"
#include "Dataflow/AircraftHandbrakeInputConfigNode.h"
#include "Dataflow/AircraftRigBindingNode.h"
#include "Dataflow/AircraftSkeletalMeshSourceNode.h"
#include "Dataflow/AircraftSolverConfigNode.h"
#include "Dataflow/AircraftSteeringConfigNode.h"
#include "Dataflow/AircraftSteeringInputConfigNode.h"
#include "Dataflow/AircraftSuspensionConfigNode.h"
#include "Dataflow/AircraftThrottleInputConfigNode.h"
#include "Dataflow/AircraftTireConfigNode.h"
#include "Dataflow/AircraftWheelConfigNode.h"
#include "AircraftAsset/AircraftAsset.h"


IMPLEMENT_MODULE(FAircraftAssetDataflowNodesModule, AircraftAssetDataflowNodes)

void FAircraftAssetDataflowNodesModule::StartupModule()
{
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY("Aircraft", FLinearColor(0.08f, 0.38f, 0.24f), FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY_NODE_COLORS_BY_CATEGORY("Terminal", FLinearColor(0.65f, 0.16f, 0.12f), FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));

	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSkeletalMeshSourceNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftChassisConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftEngineConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftClutchConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftGearboxConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSuspensionConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftWheelConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAxleConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSteeringConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftBrakeConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftThrottleInputConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftBrakeInputConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftHandbrakeInputConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSteeringInputConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftSolverConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftTireConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftRigBindingNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftDifferentialConfigNode);
	DATAFLOW_NODE_REGISTER_CREATION_FACTORY(FAircraftAssetTerminalNode);

	UE::Dataflow::RegisterNodeFilter(FDataflowTerminalNode::StaticType());
	UE_DATAFLOW_REGISTER_CATEGORY_FORASSET_TYPE("Aircraft", UAircraftAsset);
}

void FAircraftAssetDataflowNodesModule::ShutdownModule()
{
	IModuleInterface::ShutdownModule();
}
