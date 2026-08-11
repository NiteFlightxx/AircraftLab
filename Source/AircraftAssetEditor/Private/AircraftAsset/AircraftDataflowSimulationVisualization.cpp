// 组件获取路径：FDataflowSimulationScene::GetPreviewActor()->GetComponentByClass<UAircraftComponent>()。
// 旧调试绘制菜单（CenterOfMass/Rotors/ThrustVectors/Torque/Velocity 开关）已随组件侧
// DrawDebug 代码一并移除 —— 后续重新规划调试绘制功能后再恢复菜单。

#include "AircraftAsset/AircraftDataflowSimulationVisualization.h"

#include "GameFramework/Actor.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"

#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"
#include "Dataflow/DataflowSimulationViewportClient.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"

#define LOCTEXT_NAMESPACE "AircraftDataflowSimulationVisualization"

const FName FAircraftDataflowSimulationVisualization::Name = FName("AircraftDataflowSimulationVisualization");

FName FAircraftDataflowSimulationVisualization::GetName() const
{
	return Name;
}

UAircraftComponent* FAircraftDataflowSimulationVisualization::GetAircraftComponent(const FDataflowSimulationScene* SimulationScene)
{
	if (SimulationScene)
	{
		if (const TObjectPtr<AActor> PreviewActor = SimulationScene->GetPreviewActor())
		{
			return PreviewActor->GetComponentByClass<UAircraftComponent>();
		}
	}
	return nullptr;
}

void FAircraftDataflowSimulationVisualization::ExtendSimulationVisualizationMenu(
	const TSharedPtr<FDataflowSimulationViewportClient>& /*ViewportClient*/,
	FMenuBuilder& /*MenuBuilder*/)
{
	// 调试绘制功能待重新规划（旧 UAircraftComponent::DrawSimulationDebug + 调试开关已删除）。
}

FText FAircraftDataflowSimulationVisualization::GetDisplayString(const FDataflowSimulationScene* SimulationScene) const
{
	const UAircraftComponent* const Component = GetAircraftComponent(SimulationScene);
	if (!Component)
	{
		return FText::GetEmpty();
	}

	TArray<FText> Lines;

	// 仿真启停状态
	if (Component->IsSimulationSuspended())
	{
		Lines.Add(LOCTEXT("AircraftDisplaySuspended", "Simulation: Suspended"));
	}
	else if (!Component->IsSimulationEnabled())
	{
		Lines.Add(LOCTEXT("AircraftDisplayDisabled", "Simulation: Disabled"));
	}
	else
	{
		Lines.Add(LOCTEXT("AircraftDisplayRunning", "Simulation: Running"));
	}

	// 当前 LOD 与驱动模式
	const FAircraftSimulationModel* const Model = Component->GetSimulationModel();
	const int32 CurrentLOD = Component->GetCurrentSimulationLOD();
	FText LodText;
	if (Model && Model->SimulationLOD.LODs.IsValidIndex(CurrentLOD))
	{
		LodText = FText::Format(LOCTEXT("AircraftDisplayLod", "Simulation LOD: {0} ({1})"),
			FText::AsNumber(CurrentLOD),
			UEnum::GetDisplayValueAsText(Model->SimulationLOD.LODs[CurrentLOD].DriveMode));
	}
	else
	{
		LodText = LOCTEXT("AircraftDisplayLodUnknown", "Simulation LOD: -");
	}
	Lines.Add(LodText);

	// 解锁与飞行模式
	Lines.Add(FText::Format(LOCTEXT("AircraftDisplayMode", "Mode: {0} | Arm: {1}"),
		UEnum::GetDisplayValueAsText(Component->GetFlightMode()),
		UEnum::GetDisplayValueAsText(Component->GetArmState())));

	FText DisplayString;
	for (const FText& Line : Lines)
	{
		DisplayString = DisplayString.IsEmpty()
			? Line
			: FText::Format(LOCTEXT("AircraftDisplayLineJoin", "{0}\n{1}"), DisplayString, Line);
	}
	return DisplayString;
}

#undef LOCTEXT_NAMESPACE
