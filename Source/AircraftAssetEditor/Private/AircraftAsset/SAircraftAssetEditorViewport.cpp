// 视口工具栏注入 — 完全对齐 ChaosCloth 的 SClothEditor3DViewport.cpp 第 171-255 行。
//
// ChaosCloth 的注入流程：
//   1) RegisterMenu("ChaosClothEditor.3DViewportToolbar") 创建视口工具栏；
//   2) 添加左/右两个 Section，把 Camera/ViewModes/AssetViewerProfile 等子菜单挂到右侧；
//   3) 通过 ExtendMenu(JoinMenuPaths(..., "AssetViewerProfile")) 在 AssetViewerProfile 子菜单下
//      新增 "SimulationControls" 区段；
//   4) 把 HardReset / SoftReset / ToggleSimulationSuspended 三个 Command 加进 SimulationControls，
//      并对每条 entry 设置：
//        * ToolBarData.BlockGroupName = "SimulationControlGroup"     // 让三个按钮在顶层成组
//        * ToolBarData.LabelOverride = FText::GetEmpty()              // 顶层只显图标，不显文字
//        * ToolBarData.ResizeParams.ClippingPriority = 2000           // 高优先级避免被裁切
//        * SetShowInToolbarTopLevel(true)                             // 提升到工具栏顶层
//
// 这样按钮显示位置：视口右上角工具栏，AssetViewerProfile 子菜单旁边，三个图标按钮成一组。

#include "AircraftAsset/SAircraftAssetEditorViewport.h"

#include "PreviewProfileController.h"
#include "ToolMenus.h"
#include "AircraftAsset/AircraftAssetEditorCommands.h"
#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "ViewportToolbar/UnrealEdViewportToolbar.h"

#define LOCTEXT_NAMESPACE "SAircraftAssetEditorViewport"

void SAircraftAssetEditorViewport::Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs)
{
	SAssetEditorViewport::FArguments ParentArgs;
	ParentArgs._EditorViewportClient = InArgs._EditorViewportClient;
	SAssetEditorViewport::Construct(ParentArgs, InViewportConstructionArgs);
	Client->VisibilityDelegate.BindSP(this, &SAircraftAssetEditorViewport::IsVisible);
}

void SAircraftAssetEditorViewport::BindCommands()
{
	SAssetEditorViewport::BindCommands();

	// 与 ChaosCloth 完全一致：三个 Action 全部绑定到 ViewportClient 的转发方法
	// （ViewportClient 内部再转发到 EdMode）。这种"Viewport→ViewportClient→EdMode→Component"链
	// 与 ChaosCloth 的 SClothEditor3DViewport::BindCommands 第 87-135 行一一对应。

	const FAircraftAssetEditorCommands& CommandInfos = FAircraftAssetEditorCommands::Get();

	GetCommandList()->MapAction(
		CommandInfos.SoftResetSimulation,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->SoftResetSimulation();
			}
		}),
		FCanExecuteAction::CreateLambda([this]() { return true; }),
		FIsActionChecked::CreateLambda([this]() { return false; }));

	GetCommandList()->MapAction(
		CommandInfos.HardResetSimulation,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->HardResetSimulation();
			}
		}),
		FCanExecuteAction::CreateLambda([this]() { return true; }),
		FIsActionChecked::CreateLambda([this]() { return false; }));

	GetCommandList()->MapAction(
		CommandInfos.ToggleSimulationSuspended,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				if (PreviewViewportClient->IsSimulationSuspended())
				{
					PreviewViewportClient->ResumeSimulation();
				}
				else
				{
					PreviewViewportClient->SuspendSimulation();
				}
			}
		}),
		FCanExecuteAction::CreateLambda([this]() { return true; }),
		FIsActionChecked::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				return PreviewViewportClient->IsSimulationSuspended();
			}
			return false;
		}));
}

TSharedPtr<SWidget> SAircraftAssetEditorViewport::BuildViewportToolbar()
{
	const FName ToolbarName = TEXT("AircraftAssetEditor.PreviewViewportToolbar");

	if (!UToolMenus::Get()->IsMenuRegistered(ToolbarName))
	{
		UToolMenu* const ToolbarMenu = UToolMenus::Get()->RegisterMenu(
			ToolbarName, NAME_None, EMultiBoxType::SlimHorizontalToolBar);
		ToolbarMenu->StyleName = "ViewportToolbar";

		ToolbarMenu->AddSection("Left");

		FToolMenuSection& RightSection = ToolbarMenu->AddSection("Right");
		RightSection.Alignment = EToolMenuSectionAlign::Last;

		// Camera / ViewModes / DefaultShow / AssetViewerProfile —— 与 ChaosCloth 风格一致地加到右侧
		RightSection.AddEntry(UE::UnrealEd::CreateCameraSubmenu(
			UE::UnrealEd::FViewportCameraMenuOptions().ShowCameraMovement().ShowLensControls()));
		RightSection.AddEntry(UE::UnrealEd::CreateViewModesSubmenu());
		RightSection.AddEntry(UE::UnrealEd::CreateDefaultShowSubmenu());
		RightSection.AddEntry(UE::UnrealEd::CreateAssetViewerProfileSubmenu());

		// 关键：将 HardReset / SoftReset / ToggleSuspended 注入到 AssetViewerProfile 子菜单下的
		// SimulationControls 区段，并通过 SetShowInToolbarTopLevel(true) 提到顶层。
		// 完全对齐 SClothEditor3DViewport.cpp 第 220-240 行。
		if (UToolMenu* const AssetViewerMenu =
			UToolMenus::Get()->ExtendMenu(UToolMenus::JoinMenuPaths(ToolbarName, "AssetViewerProfile")))
		{
			FToolMenuSection& SimulationSection = AssetViewerMenu->FindOrAddSection(
				"SimulationControls",
				LOCTEXT("SimulationControlsSection", "Simulation Playback Controls"));

			FToolMenuEntryToolBarData ToolBarData;
			ToolBarData.BlockGroupName = "SimulationControlGroup"; // 三个按钮成组
			ToolBarData.LabelOverride = FText::GetEmpty();          // 顶层不显文字
			ToolBarData.ResizeParams.ClippingPriority = 2000;       // 高优先级避免被裁切

			FToolMenuEntry& HardReset =
				SimulationSection.AddMenuEntry(FAircraftAssetEditorCommands::Get().HardResetSimulation);
			HardReset.ToolBarData = ToolBarData;
			HardReset.SetShowInToolbarTopLevel(true);

			FToolMenuEntry& SoftReset =
				SimulationSection.AddMenuEntry(FAircraftAssetEditorCommands::Get().SoftResetSimulation);
			SoftReset.ToolBarData = ToolBarData;
			SoftReset.SetShowInToolbarTopLevel(true);

			FToolMenuEntry& ToggleSimulation =
				SimulationSection.AddMenuEntry(FAircraftAssetEditorCommands::Get().ToggleSimulationSuspended);
			ToggleSimulation.ToolBarData = ToolBarData;
			ToggleSimulation.SetShowInToolbarTopLevel(true);
		}
	}

	FToolMenuContext Context;
	{
		Context.AppendCommandList(GetCommandList());
		Context.AddExtender(GetExtenders());

		UUnrealEdViewportToolbarContext* const ContextObject =
			UE::UnrealEd::CreateViewportToolbarDefaultContext(SharedThis(this));
		Context.AddObject(ContextObject);
	}

	return UToolMenus::Get()->GenerateWidget(ToolbarName, Context);
}

TSharedPtr<IPreviewProfileController> SAircraftAssetEditorViewport::CreatePreviewProfileController()
{
	return MakeShared<FPreviewProfileController>();
}

bool SAircraftAssetEditorViewport::IsVisible() const
{
	return ViewportWidget.IsValid();
}

void SAircraftAssetEditorViewport::OnFocusViewportToSelection()
{
	const TSharedPtr<const FAircraftAssetEditorViewportClient> PreviewViewportClient =
		StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client);
	if (!PreviewViewportClient.IsValid())
	{
		return;
	}

	const FBox PreviewBounds = PreviewViewportClient->PreviewBoundingBox();
	if (PreviewBounds.IsValid)
	{
		Client->FocusViewportOnBox(PreviewBounds);
	}
}

TSharedRef<SEditorViewport> SAircraftAssetEditorViewport::GetViewportWidget()
{
	return SharedThis(this);
}

TSharedPtr<FExtender> SAircraftAssetEditorViewport::GetExtenders() const
{
	return MakeShared<FExtender>();
}

TWeakPtr<FAircraftAssetEditorPreviewScene> SAircraftAssetEditorViewport::GetPreviewScene() const
{
	if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
		StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
	{
		return PreviewViewportClient->GetPreviewScene();
	}

	return nullptr;
}

#undef LOCTEXT_NAMESPACE
