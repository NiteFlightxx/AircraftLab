// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/ClothDataflowSimulationVisualization.cpp。
// 菜单与状态文本逻辑迁移自已删除的 AircraftEditorSimulationVisualization.cpp（自制编辑器时代），
// 组件获取路径从 FAircraftAssetEditorPreviewScene 换成 FDataflowSimulationScene::GetPreviewActor()。

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
	const TSharedPtr<FDataflowSimulationViewportClient>& ViewportClient,
	FMenuBuilder& MenuBuilder)
{
	if (!ViewportClient)
	{
		return;
	}

	// SimulationScene 从 Toolkit 反取（对齐 ClothDataflowSimulationVisualization 的路径；
	// FDataflowSimulationViewportClient 自身不暴露 GetSimulationScene）。
	TSharedPtr<FDataflowEditorToolkit> Toolkit = ViewportClient->GetDataflowEditorToolkit().Pin();
	if (!Toolkit)
	{
		return;
	}
	const TSharedPtr<FDataflowSimulationScene>& SimulationScene = Toolkit->GetSimulationScene();
	if (!SimulationScene || !GetAircraftComponent(SimulationScene.Get()))
	{
		return;
	}

	auto AddVisualizationToggle =
		[ViewportClient, SimulationScene](FMenuBuilder& InMenuBuilder, const FText& DisplayName, const FText& ToolTip,
			TFunction<bool(const UAircraftComponent*)> IsEnabled,
			TFunction<void(UAircraftComponent*, bool)> SetEnabled)
	{
		InMenuBuilder.AddMenuEntry(
			DisplayName,
			ToolTip,
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([ViewportClient, SimulationScene, IsEnabled, SetEnabled]()
				{
					if (UAircraftComponent* const Component = GetAircraftComponent(SimulationScene.Get()))
					{
						SetEnabled(Component, !IsEnabled(Component));
						ViewportClient->Invalidate();
					}
				}),
				FCanExecuteAction::CreateLambda([SimulationScene]()
				{
					return GetAircraftComponent(SimulationScene.Get()) != nullptr;
				}),
				FIsActionChecked::CreateLambda([SimulationScene, IsEnabled]()
				{
					if (const UAircraftComponent* const Component = GetAircraftComponent(SimulationScene.Get()))
					{
						return IsEnabled(Component);
					}
					return false;
				})),
			NAME_None,
			EUserInterfaceActionType::ToggleButton);
	};

	MenuBuilder.BeginSection(TEXT("AircraftSimulationVisualization"), LOCTEXT("AircraftSimulationVisualizationSection", "Aircraft Debug Draw"));
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftVisualizationCenterOfMass", "Center Of Mass"),
		LOCTEXT("AircraftVisualizationCenterOfMassTooltip", "Draw the chassis center of mass in the simulation viewport."),
		[](const UAircraftComponent* Component) { return Component->IsCenterOfMassDebugDrawEnabled(); },
		[](UAircraftComponent* Component, bool bEnable) { Component->SetCenterOfMassDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftVisualizationRotors", "Rotors"),
		LOCTEXT("AircraftVisualizationRotorsTooltip", "Draw rotor disc positions, radius, and spin direction arrows."),
		[](const UAircraftComponent* Component) { return Component->IsRotorDebugDrawEnabled(); },
		[](UAircraftComponent* Component, bool bEnable) { Component->SetRotorDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftVisualizationThrustVectors", "Thrust Vectors"),
		LOCTEXT("AircraftVisualizationThrustVectorsTooltip", "Draw per-rotor thrust vectors at each rotor location."),
		[](const UAircraftComponent* Component) { return Component->IsThrustVectorDebugDrawEnabled(); },
		[](UAircraftComponent* Component, bool bEnable) { Component->SetThrustVectorDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftVisualizationTorque", "Body Torque"),
		LOCTEXT("AircraftVisualizationTorqueTooltip", "Draw the resulting body torque from control allocation."),
		[](const UAircraftComponent* Component) { return Component->IsTorqueDebugDrawEnabled(); },
		[](UAircraftComponent* Component, bool bEnable) { Component->SetTorqueDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftVisualizationVelocity", "Velocity"),
		LOCTEXT("AircraftVisualizationVelocityTooltip", "Draw the chassis linear velocity vector at center of mass."),
		[](const UAircraftComponent* Component) { return Component->IsVelocityDebugDrawEnabled(); },
		[](UAircraftComponent* Component, bool bEnable) { Component->SetVelocityDebugDrawEnabled(bEnable); });
	MenuBuilder.EndSection();
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
