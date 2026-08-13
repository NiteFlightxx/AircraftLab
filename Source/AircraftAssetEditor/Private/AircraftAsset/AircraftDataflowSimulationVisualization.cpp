// 组件获取路径：FDataflowSimulationScene::GetPreviewActor()->GetComponentByClass<UAircraftComponent>()。
// 与 ChaosCloth 的 Dataflow Simulation Visualization 一致：菜单只保存视口开关，
// 具体绘制由独立 FAircraftVisualization 完成，不进入飞控求解流程。

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

	TWeakPtr<FDataflowSimulationViewportClient> WeakViewportClient = ViewportClient;
	MenuBuilder.BeginSection(TEXT("AircraftSimulation_Visualizations"),
		LOCTEXT("AircraftVisualizationSection", "Aircraft Visualization"));
	auto AddToggle = [this, &MenuBuilder, WeakViewportClient](
		const FText& Label,
		const FText& ToolTip,
		bool FAircraftVisualizationFlags::* Flag)
	{
		const FExecuteAction Execute = FExecuteAction::CreateLambda([this, WeakViewportClient, Flag]()
		{
			Flags.*Flag = !(Flags.*Flag);
			if (const TSharedPtr<FDataflowSimulationViewportClient> Pinned = WeakViewportClient.Pin())
			{
				Pinned->Invalidate();
			}
		});
		const FIsActionChecked IsChecked = FIsActionChecked::CreateLambda([this, Flag]()
		{
			return Flags.*Flag;
		});
		MenuBuilder.AddMenuEntry(Label, ToolTip, FSlateIcon(),
			FUIAction(Execute, FCanExecuteAction(), IsChecked), NAME_None,
			EUserInterfaceActionType::ToggleButton);
	};

	AddToggle(LOCTEXT("AircraftVisBodyAxes", "Body Axes"),
		LOCTEXT("AircraftVisBodyAxesTip", "Draw the rigid-body coordinate system."),
		&FAircraftVisualizationFlags::bDrawBodyAxes);
	AddToggle(LOCTEXT("AircraftVisCenterOfMass", "Center of Mass"),
		LOCTEXT("AircraftVisCenterOfMassTip", "Draw the Chaos center of mass."),
		&FAircraftVisualizationFlags::bDrawCenterOfMass);
	AddToggle(LOCTEXT("AircraftVisBounds", "Bounds"),
		LOCTEXT("AircraftVisBoundsTip", "Draw the component bounds."),
		&FAircraftVisualizationFlags::bDrawBounds);
	AddToggle(LOCTEXT("AircraftVisVelocity", "Velocity"),
		LOCTEXT("AircraftVisVelocityTip", "Draw actual linear and angular velocity."),
		&FAircraftVisualizationFlags::bDrawVelocity);
	AddToggle(LOCTEXT("AircraftVisTarget", "Motion Target"),
		LOCTEXT("AircraftVisTargetTip", "Draw target position, rotation, linear velocity, and angular velocity."),
		&FAircraftVisualizationFlags::bDrawMotionTarget);
	AddToggle(LOCTEXT("AircraftVisRotors", "Rotors and Thrust"),
		LOCTEXT("AircraftVisRotorsTip", "Draw rotor locations, thrust axes, and current thrust."),
		&FAircraftVisualizationFlags::bDrawRotors);
	AddToggle(LOCTEXT("AircraftVisConstraint", "Physics Constraint"),
		LOCTEXT("AircraftVisConstraintTip", "Draw constraint reference, output force, and output torque."),
		&FAircraftVisualizationFlags::bDrawConstraint);
	MenuBuilder.EndSection();
}

void FAircraftDataflowSimulationVisualization::Draw(
	const FDataflowSimulationScene* SimulationScene,
	FPrimitiveDrawInterface* PDI)
{
	const UAircraftComponent* const Component = GetAircraftComponent(SimulationScene);
	if (!Component || !PDI)
	{
		return;
	}
	FAircraftVisualizationContext Context;
	Context.PDI = PDI;
	FAircraftVisualization::Draw(*Component, Context, Flags);
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
	Lines.Add(FText::Format(
		LOCTEXT("AircraftDisplayBackend", "Controller: {0} | Chaos Body: {1}"),
		Component->IsControllerEnabled()
			? LOCTEXT("AircraftDisplayControllerEnabled", "Enabled")
			: LOCTEXT("AircraftDisplayControllerDisabled", "Disabled"),
		Component->IsSimulatingPhysics()
			? LOCTEXT("AircraftDisplayPhysicsActive", "Simulating")
			: LOCTEXT("AircraftDisplayPhysicsInactive", "Inactive")));

	FAircraftEstimatedState EstimatedState;
	Component->GetEstimatedState(EstimatedState);
	const FVector& Velocity = EstimatedState.State.VelocityCmPerSec;
	const FRotator& Attitude = EstimatedState.State.AttitudeDegrees;
	Lines.Add(FText::Format(
		LOCTEXT("AircraftDisplayState", "Velocity: ({0}, {1}, {2}) cm/s | Attitude R/P/Y: ({3}, {4}, {5}) deg"),
		FText::AsNumber(Velocity.X), FText::AsNumber(Velocity.Y), FText::AsNumber(Velocity.Z),
		FText::AsNumber(Attitude.Roll), FText::AsNumber(Attitude.Pitch), FText::AsNumber(Attitude.Yaw)));

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
