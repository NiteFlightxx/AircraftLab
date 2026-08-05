// 视口工具栏注入 — 完全对齐 ChaosCloth 的 SClothEditor3DViewport.cpp。
//
// 1) RegisterMenu("AircraftAssetEditor.PreviewViewportToolbar") 创建视口工具栏；
// 2) 左/右两个 Section：Camera / ViewModes / Show / AssetViewerProfile 挂右侧；
// 3) ViewModes 子菜单中追加 "Aircraft" 区段（Wireframe 顶栏按钮）；
// 4) DynamicLOD 条目（预览 LOD 选择子菜单）；
// 5) AssetViewerProfile 子菜单的 "SimulationControls" 区段中注入 HardReset / SoftReset / ToggleSuspended。

#include "AircraftAsset/SAircraftAssetEditorViewport.h"

#include "AdvancedPreviewSceneMenus.h"
#include "Animation/AnimSingleNodeInstance.h"
#include "PreviewProfileController.h"
#include "ToolMenus.h"
#include "AircraftAsset/AircraftAssetEditorCommands.h"
#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "AircraftAsset/AircraftEditorMode.h"
#include "AircraftAsset/AircraftEditorSimulationVisualization.h"
#include "AircraftAsset/SAircraftAnimationScrubPanel.h"
#include "ViewportToolbar/UnrealEdViewportToolbar.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/SRichTextBlock.h"

#define LOCTEXT_NAMESPACE "SAircraftAssetEditorViewport"

void SAircraftAssetEditorViewport::Construct(const FArguments& InArgs, const FAssetEditorViewportConstructionArgs& InViewportConstructionArgs)
{
	SAssetEditorViewport::FArguments ParentArgs;
	ParentArgs._EditorViewportClient = InArgs._EditorViewportClient;
	ToolkitCommandList = InArgs._ToolkitCommandList;
	SAssetEditorViewport::Construct(ParentArgs, InViewportConstructionArgs);
	Client->VisibilityDelegate.BindSP(this, &SAircraftAssetEditorViewport::IsVisible);

	// 底部动画时间轴（对齐 SClothEditor3DViewport::Construct 的 overlay 注入）
	ViewportOverlay->AddSlot()
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot()
		.HAlign(HAlign_Fill)
		.VAlign(VAlign_Bottom)
		.FillWidth(1)
		.Padding(10.0f, 0.0f)
		[
			SNew(SBorder)
			.BorderImage(FAppStyle::Get().GetBrush("EditorViewport.OverlayBrush"))
			.Visibility(this, &SAircraftAssetEditorViewport::GetAnimControlVisibility)
			.Padding(10.0f, 2.0f)
			[
				SNew(SAircraftAnimationScrubPanel, GetPreviewScene())
				.ViewInputMin(this, &SAircraftAssetEditorViewport::GetViewMinInput)
				.ViewInputMax(this, &SAircraftAssetEditorViewport::GetViewMaxInput)
			]
		]
	];

	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = GetPreviewScene().Pin())
	{
		UE::AdvancedPreviewScene::BindDefaultOnSettingsChangedHandler(PreviewScene, Client);
	}
}

void SAircraftAssetEditorViewport::BindCommands()
{
	SAssetEditorViewport::BindCommands();

	// 与 ChaosCloth 完全一致：Action 绑定到 ToolkitCommandList（编辑器范围热键），
	// 由 ViewportClient 转发到 EdMode，再到 Component（Viewport→ViewportClient→EdMode→Component）。
	// 若 ToolkitCommandList 缺失（直接实例化视口的场景），回退到视口本地命令表。
	const TSharedPtr<FUICommandList> TargetCommandList = ToolkitCommandList.IsValid() ? ToolkitCommandList : GetCommandList();

	const FAircraftAssetEditorCommands& CommandInfos = FAircraftAssetEditorCommands::Get();

	TargetCommandList->MapAction(
		CommandInfos.TogglePreviewWireframe,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->EnableRenderMeshWireframe(!PreviewViewportClient->RenderMeshWireframeEnabled());
			}
		}),
		FCanExecuteAction::CreateLambda([this]() { return true; }),
		FIsActionChecked::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				return PreviewViewportClient->RenderMeshWireframeEnabled();
			}
			return false;
		}));

	TargetCommandList->MapAction(
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

	TargetCommandList->MapAction(
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

	TargetCommandList->MapAction(
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

	TargetCommandList->MapAction(
		CommandInfos.LODAuto,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->SetLODLevel(INDEX_NONE);
			}
		}),
		FCanExecuteAction(),
		FIsActionChecked::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				return PreviewViewportClient->IsLODSelected(INDEX_NONE);
			}
			return false;
		}));

	TargetCommandList->MapAction(
		CommandInfos.LOD0,
		FExecuteAction::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				PreviewViewportClient->SetLODLevel(0);
			}
		}),
		FCanExecuteAction(),
		FIsActionChecked::CreateLambda([this]()
		{
			if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
				StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
			{
				return PreviewViewportClient->IsLODSelected(0);
			}
			return false;
		}));

	// 其余 LOD 级别通过 FillLODCommands 动态加入工具栏菜单（见 BuildViewportToolbar 的 LODCombo 段）
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

		{
			FToolMenuSection& RightSection = ToolbarMenu->AddSection("Right");
			RightSection.Alignment = EToolMenuSectionAlign::Last;

			RightSection.AddEntry(UE::UnrealEd::CreateCameraSubmenu(
				UE::UnrealEd::FViewportCameraMenuOptions().ShowCameraMovement().ShowLensControls()));

			{
				RightSection.AddEntry(UE::UnrealEd::CreateViewModesSubmenu());
				// 在 ViewModes 子菜单中追加 "Aircraft" 区段（Wireframe 顶栏按钮对齐 ChaosCloth）
				UToolMenu* const ViewModesMenu = UToolMenus::Get()->ExtendMenu(
					UToolMenus::JoinMenuPaths(ToolbarName, "ViewModes"));
				FToolMenuSection& ViewSection = ViewModesMenu->FindOrAddSection(
					"Aircraft", LOCTEXT("AircraftViewModeSection", "Aircraft"));

				FToolMenuEntry& WireframeEntry = ViewSection.AddMenuEntry(
					FAircraftAssetEditorCommands::Get().TogglePreviewWireframe);
				WireframeEntry.SetShowInToolbarTopLevel(true);
				WireframeEntry.ToolBarData.ResizeParams.ClippingPriority = 2000;
			}

			{
				// LOD 选择（对齐 SClothEditor3DViewport 的 DynamicLOD 段）
				RightSection.AddDynamicEntry("DynamicLOD", FNewToolMenuSectionDelegate::CreateLambda([](FToolMenuSection& Section)
					{
						if (UUnrealEdViewportToolbarContext* Context = Section.FindContext<UUnrealEdViewportToolbarContext>())
						{
							if (TSharedPtr<SEditorViewport> EditorViewport = Context->Viewport.Pin())
							{
								if (TSharedPtr<FEditorViewportClient> ViewportClient = EditorViewport->GetViewportClient())
								{
									const TSharedPtr<FAircraftAssetEditorViewportClient> AircraftClient =
										StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(ViewportClient);
									Section.AddEntry(UE::UnrealEd::CreatePreviewLODSelectionSubmenu(AircraftClient));
								}
							}
						}
					}));
			}

			RightSection.AddEntry(UE::UnrealEd::CreateDefaultShowSubmenu());

			{
				RightSection.AddEntry(UE::UnrealEd::CreateAssetViewerProfileSubmenu());
				// 关键：将 HardReset / SoftReset / ToggleSuspended 注入到 AssetViewerProfile 子菜单下的
				// SimulationControls 区段，并通过 SetShowInToolbarTopLevel(true) 提到顶层。
				// 完全对齐 SClothEditor3DViewport.cpp 的 AssetViewerProfile 注入位置。
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
		}
	}

	FToolMenuContext Context;
	{
		Context.AppendCommandList(GetCommandList());
		if (ToolkitCommandList.IsValid())
		{
			Context.AppendCommandList(ToolkitCommandList);
		}
		Context.AddExtender(GetExtenders());

		UUnrealEdViewportToolbarContext* const ContextObject =
			UE::UnrealEd::CreateViewportToolbarDefaultContext(SharedThis(this));
		Context.AddObject(ContextObject);
	}

	return UToolMenus::Get()->GenerateWidget(ToolbarName, Context);
}

void SAircraftAssetEditorViewport::PopulateViewportOverlays(TSharedRef<SOverlay> Overlay)
{
	Overlay->AddSlot()
	.Padding(FMargin(4.0f, 3.0f, 0.0f, 0.0f))
	[
		SNew(SBox)
		[
			SNew(SRichTextBlock)
			.DecoratorStyleSet(&FAppStyle::Get())
			.TextStyle(&FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("AnimViewport.MessageText"))
			.Text(this, &SAircraftAssetEditorViewport::GetViewportDisplayString)
		]
	];
}

FText SAircraftAssetEditorViewport::GetViewportDisplayString() const
{
	if (const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
		StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client))
	{
		if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene = PreviewViewportClient->GetPreviewScene().Pin())
		{
			if (FAircraftEditorSimulationVisualization* const Visualization = PreviewViewportClient->GetSimulationVisualization().Pin().Get())
			{
				return Visualization->GetDisplayString(PreviewScene->GetAircraftComponent());
			}
		}
	}
	return FText::GetEmpty();
}

float SAircraftAssetEditorViewport::GetViewMinInput() const
{
	return 0.0f;
}

float SAircraftAssetEditorViewport::GetViewMaxInput() const
{
	const TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient =
		StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(Client);
	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> Scene = PreviewViewportClient
		? PreviewViewportClient->GetPreviewScene().Pin() : nullptr)
	{
		if (UAnimSingleNodeInstance* const PreviewInstance = Scene->GetPreviewAnimInstance())
		{
			return PreviewInstance->GetLength();
		}
	}
	return 0.0f;
}

EVisibility SAircraftAssetEditorViewport::GetAnimControlVisibility() const
{
	const TSharedPtr<const FAircraftAssetEditorPreviewScene> Scene = GetPreviewScene().Pin();
	if (Scene && Scene->GetPreviewAnimInstance())
	{
		return EVisibility::Visible;
	}
	return EVisibility::Collapsed;
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
