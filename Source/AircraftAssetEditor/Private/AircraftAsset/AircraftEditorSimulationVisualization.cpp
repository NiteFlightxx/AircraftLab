// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/ClothEditorSimulationVisualization.cpp。
// 菜单内容迁移自 FAircraftAssetEditorToolkit::SpawnTab_SimulationVisualization。

#include "AircraftAsset/AircraftEditorSimulationVisualization.h"

#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Misc/Optional.h"

#define LOCTEXT_NAMESPACE "AircraftEditorSimulationVisualization"

void FAircraftEditorSimulationVisualization::ExtendViewportShowMenu(FMenuBuilder& MenuBuilder, const TSharedRef<FAircraftAssetEditorViewportClient>& ViewportClient)
{
	auto GetPreviewAircraftComponent = [ViewportClient]() -> UAircraftComponent*
	{
		if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = ViewportClient->GetPreviewScene().Pin())
		{
			return PreviewScene->GetAircraftComponent();
		}
		return nullptr;
	};

	auto AddVisualizationToggle =
		[ViewportClient, GetPreviewAircraftComponent](FMenuBuilder& InMenuBuilder, const FText& DisplayName, const FText& ToolTip,
			TFunction<bool(const UAircraftComponent*)> IsEnabled,
			TFunction<void(UAircraftComponent*, bool)> SetEnabled)
	{
		InMenuBuilder.AddMenuEntry(
			DisplayName,
			ToolTip,
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([ViewportClient, GetPreviewAircraftComponent, IsEnabled, SetEnabled]()
				{
					if (UAircraftComponent* const AircraftComponent = GetPreviewAircraftComponent())
					{
						SetEnabled(AircraftComponent, !IsEnabled(AircraftComponent));
						ViewportClient->Invalidate();
					}
				}),
				FCanExecuteAction::CreateLambda([GetPreviewAircraftComponent]()
				{
					return GetPreviewAircraftComponent() != nullptr;
				}),
				FIsActionChecked::CreateLambda([GetPreviewAircraftComponent, IsEnabled]()
				{
					if (const UAircraftComponent* const AircraftComponent = GetPreviewAircraftComponent())
					{
						return IsEnabled(AircraftComponent);
					}
					return false;
				})),
			NAME_None,
			EUserInterfaceActionType::ToggleButton);
	};

	MenuBuilder.BeginSection(TEXT("AircraftSimulationVisualization"), LOCTEXT("AircraftSimulationVisualizationSection", "Aircraft Debug Draw"));
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftSimulationVisualizationCenterOfMass", "Center Of Mass"),
		LOCTEXT("AircraftSimulationVisualizationCenterOfMassTooltip", "Draw the chassis center of mass in the preview viewport."),
		[](const UAircraftComponent* AircraftComponent) { return AircraftComponent->IsCenterOfMassDebugDrawEnabled(); },
		[](UAircraftComponent* AircraftComponent, bool bEnable) { AircraftComponent->SetCenterOfMassDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftSimulationVisualizationRotors", "Rotors"),
		LOCTEXT("AircraftSimulationVisualizationRotorsTooltip", "Draw rotor disc positions, radius, and spin direction arrows."),
		[](const UAircraftComponent* AircraftComponent) { return AircraftComponent->IsRotorDebugDrawEnabled(); },
		[](UAircraftComponent* AircraftComponent, bool bEnable) { AircraftComponent->SetRotorDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftSimulationVisualizationThrustVectors", "Thrust Vectors"),
		LOCTEXT("AircraftSimulationVisualizationThrustVectorsTooltip", "Draw per-rotor thrust vectors at each rotor location."),
		[](const UAircraftComponent* AircraftComponent) { return AircraftComponent->IsThrustVectorDebugDrawEnabled(); },
		[](UAircraftComponent* AircraftComponent, bool bEnable) { AircraftComponent->SetThrustVectorDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftSimulationVisualizationTorque", "Body Torque"),
		LOCTEXT("AircraftSimulationVisualizationTorqueTooltip", "Draw the resulting body torque from control allocation."),
		[](const UAircraftComponent* AircraftComponent) { return AircraftComponent->IsTorqueDebugDrawEnabled(); },
		[](UAircraftComponent* AircraftComponent, bool bEnable) { AircraftComponent->SetTorqueDebugDrawEnabled(bEnable); });
	AddVisualizationToggle(
		MenuBuilder,
		LOCTEXT("AircraftSimulationVisualizationVelocity", "Velocity"),
		LOCTEXT("AircraftSimulationVisualizationVelocityTooltip", "Draw the chassis linear velocity vector at center of mass."),
		[](const UAircraftComponent* AircraftComponent) { return AircraftComponent->IsVelocityDebugDrawEnabled(); },
		[](UAircraftComponent* AircraftComponent, bool bEnable) { AircraftComponent->SetVelocityDebugDrawEnabled(bEnable); });
	MenuBuilder.EndSection();
}

FText FAircraftEditorSimulationVisualization::GetDisplayString(const UAircraftComponent* AircraftComponent) const
{
	if (!AircraftComponent)
	{
		return FText::GetEmpty();
	}

	TArray<FText> Lines;

	// 仿真启停状态
	if (AircraftComponent->IsSimulationSuspended())
	{
		Lines.Add(LOCTEXT("AircraftDisplaySuspended", "Simulation: Suspended"));
	}
	else if (!AircraftComponent->IsSimulationEnabled())
	{
		Lines.Add(LOCTEXT("AircraftDisplayDisabled", "Simulation: Disabled"));
	}
	else
	{
		Lines.Add(LOCTEXT("AircraftDisplayRunning", "Simulation: Running"));
	}

	// 当前 LOD 与驱动模式
	const FAircraftSimulationModel* const Model = AircraftComponent->GetSimulationModel();
	const int32 CurrentLOD = AircraftComponent->GetCurrentSimulationLOD();
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
		UEnum::GetDisplayValueAsText(AircraftComponent->GetFlightMode()),
		UEnum::GetDisplayValueAsText(AircraftComponent->GetArmState())));

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
