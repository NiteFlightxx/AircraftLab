#include "AircraftAsset/AircraftAssetEditorToolsModule.h"

#include "AircraftAsset/AircraftMotorPlacementTool.h"
#include "AircraftAsset/AircraftPidTuningTool.h"
#include "AircraftAsset/AircraftThrustVectorOrientationTool.h"
#include "Dataflow/AircraftAirscrewProfileNode.h"
#include "Dataflow/AircraftAttitudeControllerConfigNode.h"
#include "Dataflow/AircraftPositionControllerConfigNode.h"
#include "Dataflow/DataflowToolRegistry.h"
#include "InteractiveTool.h"
#include "Styling/AppStyle.h"

IMPLEMENT_MODULE(FAircraftAssetEditorToolsModule, AircraftAssetEditorTools)

namespace
{
	/**
	 * 当前 Aircraft 工具无专属热键，保留接口挂点以便后续添加。
	 */
	class FAircraftToolActionCommandBindings : public UE::Dataflow::FDataflowToolRegistry::IDataflowToolActionCommands
	{
	public:
		virtual ~FAircraftToolActionCommandBindings() = default;

		virtual void UnbindActiveCommands(const TSharedPtr<FUICommandList>& UICommandList) const override
		{
			(void)UICommandList;
		}

		virtual void BindCommandsForCurrentTool(const TSharedPtr<FUICommandList>& UICommandList, UInteractiveTool* Tool) const override
		{
			(void)UICommandList;
			(void)Tool;
		}
	};
}

void FAircraftAssetEditorToolsModule::StartupModule()
{
	// 在 Dataflow 图编辑器中选中对应节点时提供"进入工具"按钮。
	TSharedRef<FAircraftToolActionCommandBindings> CommandBindings = MakeShared<FAircraftToolActionCommandBindings>();

	UE::Dataflow::FDataflowToolRegistry& ToolRegistry = UE::Dataflow::FDataflowToolRegistry::Get();

	ToolRegistry.AddNodeToToolMapping(
		FAircraftAirscrewProfileNode::StaticType(),
		NewObject<UAircraftMotorPlacementToolBuilder>(),
		CommandBindings,
		FSlateIcon(FAppStyle::GetAppStyleSetName(), "Icons.Move"),
		NSLOCTEXT("AircraftEditorTools", "MotorPlacementTool", "Motor Placement"),
		FName("Aircraft"));

	ToolRegistry.AddNodeToToolMapping(
		FAircraftAirscrewProfileNode::StaticType(),
		NewObject<UAircraftThrustVectorOrientationToolBuilder>(),
		CommandBindings,
		FSlateIcon(FAppStyle::GetAppStyleSetName(), "Icons.Rotate"),
		NSLOCTEXT("AircraftEditorTools", "ThrustVectorOrientationTool", "Thrust Vector Orientation"),
		FName("Aircraft"));

	ToolRegistry.AddNodeToToolMapping(
		FAircraftAttitudeControllerConfigNode::StaticType(),
		NewObject<UAircraftPidTuningToolBuilder>(),
		CommandBindings,
		FSlateIcon(FAppStyle::GetAppStyleSetName(), "Icons.Adjust"),
		NSLOCTEXT("AircraftEditorTools", "PidTuningTool", "PID Tuning"),
		FName("Aircraft"));

	ToolRegistry.AddNodeToToolMapping(
		FAircraftPositionControllerConfigNode::StaticType(),
		NewObject<UAircraftPidTuningToolBuilder>(),
		CommandBindings,
		FSlateIcon(FAppStyle::GetAppStyleSetName(), "Icons.Adjust"),
		NSLOCTEXT("AircraftEditorTools", "PidTuningTool", "PID Tuning"),
		FName("Aircraft"));
}

void FAircraftAssetEditorToolsModule::ShutdownModule()
{
	UE::Dataflow::FDataflowToolRegistry& ToolRegistry = UE::Dataflow::FDataflowToolRegistry::Get();
	for (const FName NodeType : {
		FAircraftAirscrewProfileNode::StaticType(),
		FAircraftAttitudeControllerConfigNode::StaticType(),
		FAircraftPositionControllerConfigNode::StaticType() })
	{
		if (ToolRegistry.HasToolInfoForNodeType(NodeType))
		{
			ToolRegistry.RemoveNodeToToolMapping(NodeType);
		}
	}
}
