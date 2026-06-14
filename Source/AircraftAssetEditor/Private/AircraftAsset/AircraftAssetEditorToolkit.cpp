#include "AircraftAsset/AircraftAssetEditorToolkit.h"

#include "AssetEditorModeManager.h"
#include "Dataflow/DataflowContent.h"
#include "Dataflow/DataflowEditorCommands.h"
#include "Dataflow/DataflowEdNode.h"
#include "Dataflow/DataflowGraphEditor.h"
#include "Dataflow/DataflowNodeDetailExtension.h"
#include "Dataflow/DataflowObject.h"
#include "Dataflow/DataflowSchema.h"
#include "IStructureDetailsView.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Misc/ScopedSlowTask.h"
#include "PropertyEditorModule.h"
#include "SAdvancedPreviewDetailsTab.h"
#include "Styling/AppStyle.h"
#include "ToolMenus.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/SAircraftAssetEditorViewport.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftDataflowEditor.h"
#include "AircraftAsset/AircraftEditorMode.h"
#include "AircraftAsset/AircraftEditorModeUILayer.h"

#define LOCTEXT_NAMESPACE "AircraftAssetEditorToolkit"

// Same as SDataflowGraphEditor, only adds a AircraftAssetEditorToolkit pointer to correctly override GetDataflowContext().
class SAircraftAssetDataflowGraphEditor : public SDataflowGraphEditor
{
public:
	SLATE_BEGIN_ARGS(SAircraftAssetDataflowGraphEditor) {}
		SLATE_ARGUMENT_DEFAULT(UEdGraph*, GraphToEdit) = nullptr;
		SLATE_ARGUMENT(FGraphEditorEvents, GraphEvents)
		SLATE_ARGUMENT(TSharedPtr<IStructureDetailsView>, DetailsView)
		SLATE_ARGUMENT(FDataflowEditorCommands::FGraphEvaluationCallback, EvaluateGraph)
		SLATE_ARGUMENT_DEFAULT(FAircraftAssetEditorToolkit*, AircraftAssetEditorToolkit) = nullptr;
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, UObject* InAssetOwner)
	{
		check(InArgs._GraphToEdit);
		AircraftAssetEditorToolkit = InArgs._AircraftAssetEditorToolkit;

		SDataflowGraphEditor::FArguments Arguments;
		Arguments._GraphToEdit = InArgs._GraphToEdit;
		Arguments._GraphEvents = InArgs._GraphEvents;
		Arguments._DetailsView = InArgs._DetailsView;
		Arguments._EvaluateGraph = InArgs._EvaluateGraph;
		// 关键：与 ChaosCloth 的 SClothAssetDataflowGraphEditor::Construct 第 75 行一致——
		// 强制让 Dataflow 图始终可编辑。SDataflowGraphEditor 默认的 _IsEditable 会根据
		// "是否处于嵌入/锁定状态"返回 false，导致打开后整个图变只读，不能加节点也不能连线。
		Arguments._IsEditable = []()->bool { return true; };
		SDataflowGraphEditor::Construct(Arguments, InAssetOwner);
	}

	virtual TSharedPtr<UE::Dataflow::FContext> GetDataflowContext() const override
	{
		if (ensure(AircraftAssetEditorToolkit))
		{
			return StaticCastSharedPtr<UE::Dataflow::FContext>(AircraftAssetEditorToolkit->GetDataflowContext());
		}
		return TSharedPtr<UE::Dataflow::FContext>();
	}

	virtual bool NodesHaveToggleWidget() const override
	{
		return false;
	}

private:
	FAircraftAssetEditorToolkit* AircraftAssetEditorToolkit = nullptr;
};

const FName FAircraftAssetEditorToolkit::OutlinerTabId(TEXT("AircraftAssetEditor_Outliner"));
const FName FAircraftAssetEditorToolkit::SimulationVisualizationTabId(TEXT("AircraftAssetEditor_SimulationVisualization"));
const FName FAircraftAssetEditorToolkit::GraphCanvasTabId(TEXT("AircraftAssetEditor_GraphCanvas"));
const FName FAircraftAssetEditorToolkit::NodeDetailsTabId(TEXT("AircraftAssetEditor_NodeDetails"));
const FName FAircraftAssetEditorToolkit::PreviewSceneDetailsTabId(TEXT("AircraftAssetEditor_PreviewSceneDetails"));

FAircraftAssetEditorToolkit::FAircraftAssetEditorToolkit(UAssetEditor* InOwningAssetEditor)
	: FBaseCharacterFXEditorToolkit(InOwningAssetEditor, FName("AircraftAssetEditor"))
{
	StandaloneDefaultLayout = FTabManager::NewLayout(FName("AircraftAssetEditorLayout2"))
		->AddArea
		(
			FTabManager::NewPrimaryArea()->SetOrientation(Orient_Horizontal)
			->Split
			(
				FTabManager::NewSplitter()->SetOrientation(Orient_Vertical)
				->SetSizeCoefficient(0.8f)
				->Split
				(
					FTabManager::NewSplitter()->SetOrientation(Orient_Horizontal)
					->SetSizeCoefficient(0.55f)
					->Split
					(
						FTabManager::NewStack()
						->SetSizeCoefficient(0.12f)
						->SetExtensionId(UBaseCharacterFXEditorUISubsystem::EditorSidePanelAreaName)
						->SetHideTabWell(true)
					)
					->Split
					(
						FTabManager::NewStack()
						->SetSizeCoefficient(0.88f)
						->AddTab(ViewportTabID, ETabState::OpenedTab)
						->SetExtensionId("AircraftViewportArea")
						->SetHideTabWell(true)
					)
				)
				->Split
				(
					FTabManager::NewStack()
					->SetSizeCoefficient(0.45f)
					->AddTab(GraphCanvasTabId, ETabState::OpenedTab)
					->AddTab(OutlinerTabId, ETabState::OpenedTab)
					->SetExtensionId("AircraftGraphArea")
					->SetHideTabWell(false)
					->SetForegroundTab(GraphCanvasTabId)
				)
			)
			->Split
			(
				FTabManager::NewSplitter()->SetOrientation(Orient_Vertical)
				->SetSizeCoefficient(0.2f)
				->Split
				(
					FTabManager::NewStack()
					->SetSizeCoefficient(0.65f)
					->AddTab(DetailsTabID, ETabState::OpenedTab)
					->AddTab(PreviewSceneDetailsTabId, ETabState::OpenedTab)
					->AddTab(SimulationVisualizationTabId, ETabState::OpenedTab)
					->SetExtensionId("AircraftDetailsArea")
					->SetHideTabWell(true)
					->SetForegroundTab(DetailsTabID)
				)
				->Split
				(
					FTabManager::NewStack()
					->SetSizeCoefficient(0.35f)
					->AddTab(NodeDetailsTabId, ETabState::OpenedTab)
					->SetExtensionId("AircraftNodeDetailsArea")
					->SetHideTabWell(false)
				)
			)
		);

	FPreviewScene::ConstructionValues PreviewSceneArgs;
	PreviewSceneArgs.bShouldSimulatePhysics = 1;
	PreviewSceneArgs.bCreatePhysicsScene = 1;

	PreviewScene = MakeShared<FAircraftAssetEditorPreviewScene>(PreviewSceneArgs);
	ObjectScene = PreviewScene;
}

FAircraftAssetEditorToolkit::~FAircraftAssetEditorToolkit()
{
	if (GraphEditor)
	{
		GraphEditor->OnSelectionChangedMulticast.RemoveAll(this);
		GraphEditor->OnNodeDeletedMulticast.RemoveAll(this);
	}

	if (NodeDetailsEditor)
	{
		NodeDetailsEditor->GetOnFinishedChangingPropertiesDelegate().RemoveAll(this);
	}

	if (DetailsView.IsValid())
	{
		DetailsView->OnFinishedChangingProperties().RemoveAll(this);
	}
}

TSharedPtr<UE::Dataflow::FEngineContext> FAircraftAssetEditorToolkit::GetDataflowContext() const
{
	return DataflowContext;
}

const UDataflow* FAircraftAssetEditorToolkit::GetDataflow() const
{
	if (const UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		return AircraftAsset->GetDataflow();
	}

	return nullptr;
}

FName FAircraftAssetEditorToolkit::GetToolkitFName() const
{
	return TEXT("AircraftAssetEditor");
}

FText FAircraftAssetEditorToolkit::GetToolkitName() const
{
	if (const UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		return GetLabelForObject(AircraftAsset);
	}

	return LOCTEXT("ToolkitName", "Aircraft Asset");
}

FText FAircraftAssetEditorToolkit::GetBaseToolkitName() const
{
	return LOCTEXT("BaseToolkitName", "Aircraft Asset Editor");
}

FText FAircraftAssetEditorToolkit::GetToolkitToolTipText() const
{
	if (const UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		return GetToolTipTextForObject(AircraftAsset);
	}

	return LOCTEXT("ToolkitTooltip", "Aircraft Asset Editor");
}

void FAircraftAssetEditorToolkit::RegisterTabSpawners(const TSharedRef<FTabManager>& InTabManager)
{
	FBaseCharacterFXEditorToolkit::RegisterTabSpawners(InTabManager);

	EditorMenuCategory = InTabManager->AddLocalWorkspaceMenuCategory(LOCTEXT("WorkspaceMenu_AircraftAssetEditor", "Aircraft Asset Editor"));

	InTabManager->RegisterTabSpawner(GraphCanvasTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_GraphCanvas))
		.SetDisplayName(LOCTEXT("GraphCanvasTab", "Dataflow Graph"))
		.SetGroup(EditorMenuCategory.ToSharedRef());

	InTabManager->RegisterTabSpawner(OutlinerTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_Outliner))
		.SetDisplayName(LOCTEXT("OutlinerTab", "Outliner"))
		.SetGroup(EditorMenuCategory.ToSharedRef());

	InTabManager->RegisterTabSpawner(NodeDetailsTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_NodeDetails))
		.SetDisplayName(LOCTEXT("NodeDetailsTab", "Node Details"))
		.SetGroup(EditorMenuCategory.ToSharedRef());

	InTabManager->RegisterTabSpawner(PreviewSceneDetailsTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_PreviewSceneDetails))
		.SetDisplayName(LOCTEXT("PreviewSceneDetailsTab", "Preview Scene Details"))
		.SetGroup(EditorMenuCategory.ToSharedRef());

	InTabManager->RegisterTabSpawner(SimulationVisualizationTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_SimulationVisualization))
		.SetDisplayName(LOCTEXT("SimulationVisualizationTab", "Simulation Visualization"))
		.SetGroup(EditorMenuCategory.ToSharedRef());

	// 父类（FBaseCharacterFXEditorToolkit / FBaseAssetToolkit）已经为 ViewportTabID 注册了一个
	// "Viewport" 的 spawner。我们对齐 ChaosCloth 风格把 DisplayName 改为 "Simulation Viewport"，
	// 通过 unregister + 重新注册覆盖原来的实现。
	InTabManager->UnregisterTabSpawner(ViewportTabID);
	InTabManager->RegisterTabSpawner(ViewportTabID, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_Viewport))
		.SetDisplayName(LOCTEXT("AircraftSimulationViewportTab", "Simulation Viewport"))
		.SetGroup(EditorMenuCategory.ToSharedRef())
		.SetIcon(FSlateIcon(FAppStyle::GetAppStyleSetName(), "LevelEditor.Tabs.Viewports"));
}

void FAircraftAssetEditorToolkit::UnregisterTabSpawners(const TSharedRef<FTabManager>& InTabManager)
{
	FBaseCharacterFXEditorToolkit::UnregisterTabSpawners(InTabManager);
	InTabManager->UnregisterTabSpawner(OutlinerTabId);
	InTabManager->UnregisterTabSpawner(SimulationVisualizationTabId);
	InTabManager->UnregisterTabSpawner(GraphCanvasTabId);
	InTabManager->UnregisterTabSpawner(NodeDetailsTabId);
	InTabManager->UnregisterTabSpawner(PreviewSceneDetailsTabId);
}

void FAircraftAssetEditorToolkit::GetSaveableObjects(TArray<UObject*>& OutObjects) const
{
	FBaseCharacterFXEditorToolkit::GetSaveableObjects(OutObjects);

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		OutObjects.AddUnique(AircraftAsset);

		if (UDataflow* const DataflowAsset = AircraftAsset->GetDataflow())
		{
			OutObjects.AddUnique(DataflowAsset);
		}
	}
}

void FAircraftAssetEditorToolkit::Tick(float DeltaTime)
{
	(void)DeltaTime;

	TickCommands.Broadcast();
	TickCommands.Clear();

	if (DataflowContext.IsValid())
	{
		EvaluateNode(nullptr, nullptr);
	}

	InvalidateViews();
}

TStatId FAircraftAssetEditorToolkit::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(FAircraftAssetEditorToolkit, STATGROUP_Tickables);
}

FEditorModeID FAircraftAssetEditorToolkit::GetEditorModeId() const
{
	return UAircraftAssetEditorMode::EM_AircraftAssetEditorModeId;
}

void FAircraftAssetEditorToolkit::InitializeEdMode(UBaseCharacterFXEditorMode* EdMode)
{
	if (UAircraftAssetEditorMode* const AircraftMode = Cast<UAircraftAssetEditorMode>(EdMode))
	{
		AircraftMode->SetPreviewScene(PreviewScene.Get());
	}

	FBaseCharacterFXEditorToolkit::InitializeEdMode(EdMode);
}

void FAircraftAssetEditorToolkit::CreateEditorModeUILayer()
{
	TSharedPtr<IToolkitHost> PinnedToolkitHost = ToolkitHost.Pin();
	check(PinnedToolkitHost.IsValid());
	ModeUILayer = MakeShared<FAircraftAssetEditorModeUILayer>(PinnedToolkitHost.Get());
}

void FAircraftAssetEditorToolkit::CreateWidgets()
{
	FBaseCharacterFXEditorToolkit::CreateWidgets();

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		OnAircraftAssetChanged();
	}

	if (UDataflow* const DataflowAsset = GetDataflow())
	{
		DataflowAsset->Schema = UDataflowSchema::StaticClass();

		if (UAircraftAssetBase* const AircraftAsset = GetAsset())
		{
			NodeDetailsEditor = CreateNodeDetailsEditorWidget(AircraftAsset);
		}

		if (NodeDetailsEditor.IsValid())
		{
			GraphEditor = CreateGraphEditorWidget();
		}
	}
}

AssetEditorViewportFactoryFunction FAircraftAssetEditorToolkit::GetViewportDelegate()
{
	return [this](FAssetEditorViewportConstructionArgs InArgs)
	{
		return SAssignNew(PreviewViewportWidget, SAircraftAssetEditorViewport, InArgs)
			.EditorViewportClient(ViewportClient);
	};
}

TSharedPtr<FEditorViewportClient> FAircraftAssetEditorToolkit::CreateEditorViewportClient() const
{
	check(EditorModeManager.IsValid());
	return MakeShared<FAircraftAssetEditorViewportClient>(EditorModeManager.Get(), PreviewScene);
}

void FAircraftAssetEditorToolkit::PostInitAssetEditor()
{
	FBaseCharacterFXEditorToolkit::PostInitAssetEditor();

	auto SetCommonViewportClientOptions = [](FEditorViewportClient* Client)
	{
		Client->SetRealtime(true);
		Client->EngineShowFlags.SetTemporalAA(false);
		Client->EngineShowFlags.SetAntiAliasing(true);
		Client->EngineShowFlags.SetMotionBlur(false);
		Client->EngineShowFlags.SetOpaqueCompositeEditorPrimitives(true);
		Client->EngineShowFlags.SetDisableOcclusionQueries(true);
		Client->ViewFOV = 45.0f;
	};

	SetCommonViewportClientOptions(ViewportClient.Get());
	ViewportClient->SetViewportType(ELevelViewportType::LVT_Perspective);
	ViewportClient->SetViewMode(EViewModeIndex::VMI_Lit);

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		PreviewScene->SetAircraftAsset(AircraftAsset);
	}

	InitDetailsViewPanel();

	PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(ViewportClient);
	if (PreviewViewportClient.IsValid())
	{
		const FBox PreviewBounds = PreviewViewportClient->PreviewBoundingBox();
		if (PreviewBounds.IsValid)
		{
			PreviewViewportClient->FocusViewportOnBox(PreviewBounds);
		}
	}

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		if (UDataflow* const Dataflow = AircraftAsset->GetDataflow())
		{
			const TSharedPtr<UE::Dataflow::FAircraftAssetDataflowContext> AircraftDataflowContext =
				MakeShared<UE::Dataflow::FAircraftAssetDataflowContext>(AircraftAsset, Dataflow);
			DataflowContext = StaticCastSharedPtr<UE::Dataflow::FEngineContext>(AircraftDataflowContext);

			if (UAircraftDataflowEditor* const AircraftEditor = GetAircraftEditor())
			{
				if (UDataflowBaseContent* const EditorContent = AircraftEditor->GetEditorContent())
				{
					EditorContent->SetDataflowContext(DataflowContext);
					EditorContent->SetLastModifiedTimestamp(LastDataflowNodeTimestamp);

					if (UDataflowEdNode* const InitialSelectedNode = Cast<UDataflowEdNode>(GraphEditor ? GraphEditor->GetSingleSelectedNode() : nullptr))
					{
						EditorContent->SetSelectedNode(InitialSelectedNode);
					}
				}

				AircraftEditor->UpdateTerminalContents(LastDataflowNodeTimestamp);
			}
		}
	}

	// 把 FDataflowEditorCommands 中的 Evaluate / Start / Stop / Step / Reset Simulation
	// 等命令注入到资产编辑器顶部工具栏的 "DataflowTools" 区段——完全对齐 FDataflowEditorToolkit
	// 内部的 AddEvaluationWidget + AddDataflowActionWidget 行为（因为 FDataflowEditorToolkit 是
	// final 不能继承，所以我们手动复刻它的注入逻辑）。
	//
	// 每条命令的 ExecuteAction 都映射到 *我们自家* 的 UAircraftAssetEditorMode → UAircraftComponent
	// 接口（IsSimulationEnabled / SoftResetSimulation / SetEnableSimulation 等）。
	{
		auto GetEdMode = [this]() -> UAircraftAssetEditorMode*
		{
			return Cast<UAircraftAssetEditorMode>(EditorModeManager->GetActiveScriptableMode(
				UAircraftAssetEditorMode::EM_AircraftAssetEditorModeId));
		};

		// EvaluateGraph：对齐 FDataflowEditorToolkit::EvaluateGraph
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().EvaluateGraph,
			FExecuteAction::CreateLambda([this]()
			{
				if (UAircraftDataflowEditor* const AircraftEditor = Cast<UAircraftDataflowEditor>(OwningAssetEditor))
				{
					AircraftEditor->UpdateTerminalContents(LastDataflowNodeTimestamp);
				}
			}),
			FCanExecuteAction());

		// 对齐 FDataflowEditorToolkit::AddEvaluationWidget 的两条 Mode 切换命令。
		// 它们是 ToggleButton，由 FIsActionChecked 决定哪一项当前是被选中状态。
		// 注意我们 Toolkit 中暂没存 EvaluationMode，先把它做到 PreviewScene 旁的 Toolkit 成员上，
		// 否则 Auto/Manual 切换没地方落库。这里先用一个 Toolkit 私有变量表达。
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().EvaluateGraphAutomatic,
			FExecuteAction::CreateLambda([this]() { bAutomaticGraphEvaluation = true; }),
			FCanExecuteAction(),
			FIsActionChecked::CreateLambda([this]() { return bAutomaticGraphEvaluation; }));

		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().EvaluateGraphManual,
			FExecuteAction::CreateLambda([this]() { bAutomaticGraphEvaluation = false; }),
			FCanExecuteAction(),
			FIsActionChecked::CreateLambda([this]() { return !bAutomaticGraphEvaluation; }));

		// StartSimulation: 对齐 FDataflowEditorToolkit::StartDataflowSimulation
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().StartSimulation,
			FExecuteAction::CreateLambda([GetEdMode]()
			{
				if (UAircraftAssetEditorMode* const Mode = GetEdMode()) { Mode->SetEnableSimulation(true); }
			}),
			FCanExecuteAction::CreateLambda([GetEdMode]() -> bool
			{
				const UAircraftAssetEditorMode* const Mode = GetEdMode();
				return Mode && !Mode->IsSimulationEnabled();
			}));

		// StopSimulation
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().StopSimulation,
			FExecuteAction::CreateLambda([GetEdMode]()
			{
				if (UAircraftAssetEditorMode* const Mode = GetEdMode()) { Mode->SetEnableSimulation(false); }
			}),
			FCanExecuteAction::CreateLambda([GetEdMode]() -> bool
			{
				const UAircraftAssetEditorMode* const Mode = GetEdMode();
				return Mode && Mode->IsSimulationEnabled();
			}));

		// StepSimulation：硬重置一次（让 SimulationProxy 走一帧）。我们没有真正的 step-mode，
		// 暂时映射为 SoftReset 让用户能 "重新初始化一帧"。
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().StepSimulation,
			FExecuteAction::CreateLambda([GetEdMode]()
			{
				if (UAircraftAssetEditorMode* const Mode = GetEdMode()) { Mode->SoftResetSimulation(); }
			}),
			FCanExecuteAction());

		// ResetSimulation：HardReset
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().ResetSimulation,
			FExecuteAction::CreateLambda([GetEdMode]()
			{
				if (UAircraftAssetEditorMode* const Mode = GetEdMode()) { Mode->HardResetSimulation(); }
			}),
			FCanExecuteAction());

		// ToggleSimulation：bEnableSimulation 切换（用于 Toolbar 上的 Toggle Highlight 状态）
		ToolkitCommands->MapAction(FDataflowEditorCommands::Get().ToggleSimulation,
			FExecuteAction::CreateLambda([GetEdMode]()
			{
				if (UAircraftAssetEditorMode* const Mode = GetEdMode())
				{
					Mode->SetEnableSimulation(!Mode->IsSimulationEnabled());
				}
			}),
			FCanExecuteAction(),
			FIsActionChecked::CreateLambda([GetEdMode]() -> bool
			{
				const UAircraftAssetEditorMode* const Mode = GetEdMode();
				return Mode && Mode->IsSimulationEnabled();
			}));

		// 把这 6 个按钮添加到资产编辑器顶部工具栏的 DataflowTools 区段，
		// 完全对齐 FDataflowEditorToolkit::PostInitAssetEditor 第 911-919 行的注入位置。
		FName ParentToolbarName;
		const FName ToolBarName = GetToolMenuToolbarName(ParentToolbarName);
		if (UToolMenu* const AssetToolbar = UToolMenus::Get()->ExtendMenu(ToolBarName))
		{
			FToolMenuSection& Section = AssetToolbar->FindOrAddSection("DataflowTools");

			// 1) Evaluate Graph 主按钮（左半），由我们 Toolkit 的 EvaluateGraph 触发
			Section.AddEntry(FToolMenuEntry::InitToolBarButton(
				FDataflowEditorCommands::Get().EvaluateGraph,
				TAttribute<FText>(),
				TAttribute<FText>(),
				FSlateIcon(FAppStyle::GetAppStyleSetName(), "BlueprintEditor.CompileStatus.Background")));

			// 1b) Evaluate Graph 下拉菜单（右半），让用户选择 Automatic / Manual。
			// 完全对齐 FDataflowEditorToolkit::AddEvaluationWidget 第 1118-1135 行 InitComboButton 写法。
			TWeakPtr<FAircraftAssetEditorToolkit> WeakSelf = SharedThis(this);
			FToolMenuEntry EvaluationOptions = FToolMenuEntry::InitComboButton(
				"AircraftEvaluationOptions",
				FUIAction(),
				FOnGetContent::CreateLambda([WeakSelf]() -> TSharedRef<SWidget>
				{
					if (TSharedPtr<FAircraftAssetEditorToolkit> Self = WeakSelf.Pin())
					{
						return Self->GenerateEvaluationOptionsMenu();
					}
					return SNullWidget::NullWidget;
				}),
				LOCTEXT("AircraftEvaluationOptions", "Options"),
				LOCTEXT("AircraftEvaluationOptions_ToolbarTooltip", "Options to customize how the Dataflow Graph evaluates"),
				TAttribute<FSlateIcon>(),
				true);
			EvaluationOptions.StyleNameOverride = "SlimToolBar";
			Section.AddEntry(EvaluationOptions);

			// 2) Start Simulation —— 标准 Play 图标 + BackplateLeftPlay 风格
			FToolMenuEntry PlayEntry = FToolMenuEntry::InitToolBarButton(
				FDataflowEditorCommands::Get().StartSimulation,
				TAttribute<FText>(),
				TAttribute<FText>(),
				FSlateIcon(FAppStyle::GetAppStyleSetName(), "PlayWorld.PlayInViewport"));
			PlayEntry.StyleNameOverride = FName("Toolbar.BackplateLeftPlay");
			Section.AddEntry(PlayEntry);

			// 3) Step Simulation
			FToolMenuEntry StepEntry = FToolMenuEntry::InitToolBarButton(
				FDataflowEditorCommands::Get().StepSimulation,
				TAttribute<FText>(),
				TAttribute<FText>(),
				FSlateIcon(FAppStyle::GetAppStyleSetName(), "PlayWorld.SingleFrameAdvance.Small"));
			StepEntry.StyleNameOverride = FName("Toolbar.BackplateCenter");
			Section.AddEntry(StepEntry);

			// 4) Stop Simulation
			FToolMenuEntry StopEntry = FToolMenuEntry::InitToolBarButton(
				FDataflowEditorCommands::Get().StopSimulation,
				TAttribute<FText>(),
				TAttribute<FText>(),
				FSlateIcon(FAppStyle::GetAppStyleSetName(), "PlayWorld.StopPlaySession.Small"));
			StopEntry.StyleNameOverride = FName("Toolbar.BackplateCenterStop");
			Section.AddEntry(StopEntry);

			// 5) Reset Simulation
			FToolMenuEntry ResetEntry = FToolMenuEntry::InitToolBarButton(
				FDataflowEditorCommands::Get().ResetSimulation,
				TAttribute<FText>(),
				TAttribute<FText>(),
				FSlateIcon(FAppStyle::GetAppStyleSetName(), "Icons.Refresh"));
			ResetEntry.StyleNameOverride = FName("Toolbar.BackplateRight");
			Section.AddEntry(ResetEntry);
		}
	}
}

void FAircraftAssetEditorToolkit::NotifyPreChange(FEditPropertyChain* PropertyAboutToChange)
{
	if (NodeDetailsEditor.IsValid())
	{
		if (UDataflow* const DataflowAsset = GetDataflow())
		{
			FDataflowEditorCommands::OnNotifyPropertyPreChange(NodeDetailsEditor, DataflowAsset, PropertyAboutToChange);
		}
	}
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_Outliner(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == OutlinerTabId);

	SAssignNew(OutlinerDockTab, SDockTab)
		.Label(LOCTEXT("OutlinerTabTitle", "Outliner"))
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot()
			.Padding(8.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("OutlinerPlaceholder", "Aircraft Outliner panel placeholder."))
			]
		];

	return OutlinerDockTab.ToSharedRef();
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_SimulationVisualization(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == SimulationVisualizationTabId);

	SAssignNew(SimulationVisualizationDockTab, SDockTab)
		.Label(LOCTEXT("SimulationVisualizationTitle", "Simulation Visualization"));

	auto GetPreviewAircraftComponent = [this]() -> UAircraftComponent*
	{
		return PreviewScene.IsValid() ? PreviewScene->GetAircraftComponent() : nullptr;
	};

	auto AddVisualizationToggle =
		[this, &GetPreviewAircraftComponent](FMenuBuilder& MenuBuilder, const FText& DisplayName, const FText& ToolTip,
			TFunction<bool(const UAircraftComponent*)> IsEnabled,
			TFunction<void(UAircraftComponent*, bool)> SetEnabled)
	{
		MenuBuilder.AddMenuEntry(
			DisplayName,
			ToolTip,
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([this, GetPreviewAircraftComponent, IsEnabled, SetEnabled]()
				{
					if (UAircraftComponent* const AircraftComponent = GetPreviewAircraftComponent())
					{
						SetEnabled(AircraftComponent, !IsEnabled(AircraftComponent));
						InvalidateViews();
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

	FMenuBuilder MenuBuilder(false, nullptr);
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

	SimulationVisualizationDockTab->SetContent(MenuBuilder.MakeWidget());

	return SimulationVisualizationDockTab.ToSharedRef();
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_GraphCanvas(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == GraphCanvasTabId);

	SAssignNew(GraphEditorTab, SDockTab)
		.Label(LOCTEXT("GraphCanvasTabTitle", "Dataflow Graph"));

	if (GraphEditor.IsValid())
	{
		GraphEditorTab->SetContent(GraphEditor.ToSharedRef());
	}

	return GraphEditorTab.ToSharedRef();
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_NodeDetails(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == NodeDetailsTabId);

	TSharedRef<SDockTab> NodeDetailsTab = SNew(SDockTab)
		.Label(LOCTEXT("NodeDetailsTabTitle", "Node Details"));

	if (NodeDetailsEditor.IsValid())
	{
		NodeDetailsTab->SetContent(NodeDetailsEditor->GetWidget().ToSharedRef());
	}

	return NodeDetailsTab;
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_PreviewSceneDetails(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == PreviewSceneDetailsTabId);

	SAssignNew(PreviewSceneDockTab, SDockTab)
		.Label(LOCTEXT("PreviewSceneDetailsTitle", "Preview Scene Details"));

	if (AdvancedPreviewSettingsWidget.IsValid())
	{
		PreviewSceneDockTab->SetContent(AdvancedPreviewSettingsWidget.ToSharedRef());
	}

	return PreviewSceneDockTab.ToSharedRef();
}

void FAircraftAssetEditorToolkit::InitDetailsViewPanel()
{
	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		ensure(AircraftAsset->HasAnyFlags(RF_Transactional));
		SetEditingObject(AircraftAsset);
	}

	if (DetailsView.IsValid())
	{
		DetailsView->OnFinishedChangingProperties().AddSP(this, &FAircraftAssetEditorToolkit::OnFinishedChangingAssetProperties);
	}

	AdvancedPreviewSettingsWidget =
		SNew(SAdvancedPreviewDetailsTab, PreviewScene.ToSharedRef())
		.AdditionalSettings(nullptr)
		.DetailCustomizations(TArray<FAdvancedPreviewSceneModule::FDetailCustomizationInfo>())
		.PropertyTypeCustomizations(TArray<FAdvancedPreviewSceneModule::FPropertyTypeCustomizationInfo>())
		.Delegates(TArray<FAdvancedPreviewSceneModule::FDetailDelegates>());

	if (PreviewSceneDockTab.IsValid())
	{
		PreviewSceneDockTab->SetContent(AdvancedPreviewSettingsWidget.ToSharedRef());
	}
}

TSharedRef<SWidget> FAircraftAssetEditorToolkit::GenerateEvaluationOptionsMenu()
{
	// 完全对齐 FDataflowEditorToolkit::GenerateEvaluationOptionsMenu 第 1184-1197 行：
	//     MenuBuilder.AddMenuEntry(EvaluateGraphAutomatic);
	//     MenuBuilder.AddMenuEntry(EvaluateGraphManual);
	// 这里只放这两条；ClearGraphCache / TogglePerfData / ToggleAsyncEvaluation 我们暂不实现。
	FMenuBuilder MenuBuilder(true, GetToolkitCommands());
	MenuBuilder.BeginSection(TEXT("AircraftEvaluationModeSection"));
	MenuBuilder.AddMenuEntry(FDataflowEditorCommands::Get().EvaluateGraphAutomatic);
	MenuBuilder.AddMenuEntry(FDataflowEditorCommands::Get().EvaluateGraphManual);
	MenuBuilder.EndSection();
	return MenuBuilder.MakeWidget();
}

void FAircraftAssetEditorToolkit::OnFinishedChangingAssetProperties(const FPropertyChangedEvent& PropertyChangedEvent)
{
	(void)PropertyChangedEvent;
	OnAircraftAssetChanged();
}

void FAircraftAssetEditorToolkit::OnAircraftAssetChanged()
{
	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		ensure(AircraftAsset->HasAnyFlags(RF_Transactional));
		SetEditingObject(AircraftAsset);

		const UAircraftComponent* const PreviewAircraftComponent = PreviewScene->GetAircraftComponent();
		const bool bHadAircraftAsset = PreviewAircraftComponent && PreviewAircraftComponent->GetAsset() != nullptr;
		//const bool bWasSimulationEnabled = bHadAircraftAsset ? PreviewScene->IsSimulationEnabled() : true;
		//const bool bWasSimulationSuspended = bHadAircraftAsset && PreviewScene->IsSimulationSuspended();
		PreviewScene->SetAircraftAsset(AircraftAsset);

		if (bHadAircraftAsset)
		{
			/*
			 *PreviewScene->SetEnableSimulation(bWasSimulationEnabled);
			if (bWasSimulationEnabled)
			{
				if (bWasSimulationSuspended)
				{
					//PreviewScene->SuspendSimulation();
				}
				else
				{
					//PreviewScene->ResumeSimulation();
				}
			}*/
		}
	}
}

void FAircraftAssetEditorToolkit::InvalidateViews()
{
	if (ViewportClient.IsValid())
	{
		ViewportClient->Invalidate();
	}
}

void FAircraftAssetEditorToolkit::EvaluateNode(const FDataflowNode* Node, const FDataflowOutput* Output)
{
	check(DataflowContext);

	UAircraftAssetBase* const AircraftAsset = GetAsset();
	UDataflow* const Dataflow = GetDataflow();

	if (Dataflow && AircraftAsset)
	{
		constexpr float NumSteps = 1.f;
		FScopedSlowTask SlowTask(NumSteps, LOCTEXT("AircraftAssetEditorToolkitEvaluateNode", "Evaluating nodes..."));
		SlowTask.MakeDialogDelayed(1.f);
		SlowTask.EnterProgressFrame(1.f);

		const UE::Dataflow::FTimestamp OldTimestamp = LastDataflowNodeTimestamp;
		Node = FDataflowEditorCommands::EvaluateNode(
			*DataflowContext,
			LastDataflowNodeTimestamp,
			Dataflow,
			Node,
			Output,
			AircraftAsset->GetDataflowInstance().GetDataflowTerminal().ToString(),
			AircraftAsset);

		if (UAircraftDataflowEditor* const AircraftEditor = GetAircraftEditor())
		{
			if (UDataflowBaseContent* const EditorContent = AircraftEditor->GetEditorContent())
			{
				EditorContent->SetLastModifiedTimestamp(LastDataflowNodeTimestamp);
			}

			AircraftEditor->UpdateTerminalContents(LastDataflowNodeTimestamp);
		}

		if (Node && OldTimestamp < LastDataflowNodeTimestamp)
		{
			if (Node->GetName() == AircraftAsset->GetDataflowInstance().GetDataflowTerminal())
			{
				OnAircraftAssetChanged();
			}
		}
	}
}

TSharedRef<SDataflowGraphEditor> FAircraftAssetEditorToolkit::CreateGraphEditorWidget()
{
	UDataflow* const Dataflow = GetDataflow();
	check(Dataflow);

	const FDataflowEditorCommands::FGraphEvaluationCallback EvaluateGraph =
		[this](const FDataflowNode* Node, const FDataflowOutput* Output)
		{
			EvaluateNode(Node, Output);
		};

	SGraphEditor::FGraphEditorEvents GraphEditorEvents;
	GraphEditorEvents.OnVerifyTextCommit = FOnNodeVerifyTextCommit::CreateSP(this, &FAircraftAssetEditorToolkit::OnNodeVerifyTitleCommit);
	GraphEditorEvents.OnTextCommitted = FOnNodeTextCommitted::CreateSP(this, &FAircraftAssetEditorToolkit::OnNodeTitleCommitted);
	
	TSharedRef<SDataflowGraphEditor> NewGraphEditor = SNew(SAircraftAssetDataflowGraphEditor, Dataflow)
		.GraphToEdit(Dataflow)
		.GraphEvents(GraphEditorEvents)
		.DetailsView(NodeDetailsEditor)
		.EvaluateGraph(EvaluateGraph)
		.AircraftAssetEditorToolkit(this);

	NewGraphEditor->OnSelectionChangedMulticast.AddSP(this, &FAircraftAssetEditorToolkit::OnNodeSelectionChanged);
	NewGraphEditor->OnNodeDeletedMulticast.AddSP(this, &FAircraftAssetEditorToolkit::OnNodeDeleted);

	return NewGraphEditor;
}

TSharedPtr<IStructureDetailsView> FAircraftAssetEditorToolkit::CreateNodeDetailsEditorWidget(UObject* ObjectToEdit)
{
	check(ObjectToEdit);

	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>(TEXT("PropertyEditor"));

	FDetailsViewArgs DetailsViewArgs;
	DetailsViewArgs.bAllowSearch = false;
	DetailsViewArgs.bHideSelectionTip = true;
	DetailsViewArgs.bLockable = false;
	DetailsViewArgs.bSearchInitialKeyFocus = true;
	DetailsViewArgs.bUpdatesFromSelection = false;
	DetailsViewArgs.NotifyHook = this;
	DetailsViewArgs.bShowOptions = true;
	DetailsViewArgs.bShowModifiedPropertiesOption = false;
	DetailsViewArgs.bShowScrollBar = false;

	FStructureDetailsViewArgs StructureViewArgs;
	StructureViewArgs.bShowObjects = true;
	StructureViewArgs.bShowAssets = true;
	StructureViewArgs.bShowClasses = true;
	StructureViewArgs.bShowInterfaces = true;

	TSharedPtr<IStructureDetailsView> NewDetailsView = PropertyEditorModule.CreateStructureDetailView(DetailsViewArgs, StructureViewArgs, nullptr);
	NewDetailsView->GetDetailsView()->SetObject(ObjectToEdit);
	NewDetailsView->GetOnFinishedChangingPropertiesDelegate().AddSP(this, &FAircraftAssetEditorToolkit::OnPropertyValueChanged);

	NodeDetailsExtensionHandler = MakeShared<UE::Dataflow::FDataflowNodeDetailExtensionHandler>();
	NewDetailsView->GetDetailsView()->SetExtensionHandler(NodeDetailsExtensionHandler);

	return NewDetailsView;
}

void FAircraftAssetEditorToolkit::OnPropertyValueChanged(const FPropertyChangedEvent& PropertyChangedEvent)
{
	auto AsEdNodePointers = [](const TSet<UObject*>& Set)
	{
		TSet<TObjectPtr<UDataflowEdNode>> Nodes;
		for (UObject* const Elem : Set)
		{
			if (UDataflowEdNode* const Node = Cast<UDataflowEdNode>(Elem))
			{
				Nodes.Add(Node);
			}
		}
		return Nodes;
	};

	FDataflowEditorCommands::OnPropertyValueChanged(
		GetDataflow(),
		DataflowContext,
		LastDataflowNodeTimestamp,
		PropertyChangedEvent,
		GraphEditor ? AsEdNodePointers(GraphEditor->GetSelectedNodes()) : TSet<TObjectPtr<UDataflowEdNode>>());
}

bool FAircraftAssetEditorToolkit::OnNodeVerifyTitleCommit(const FText& NewText, UEdGraphNode* GraphNode, FText& OutErrorMessage) const
{
	return FDataflowEditorCommands::OnNodeVerifyTitleCommit(NewText, GraphNode, OutErrorMessage);
}

void FAircraftAssetEditorToolkit::OnNodeTitleCommitted(const FText& InNewText, ETextCommit::Type InCommitType, UEdGraphNode* GraphNode) const
{
	FDataflowEditorCommands::OnNodeTitleCommitted(InNewText, InCommitType, GraphNode);
}

void FAircraftAssetEditorToolkit::OnNodeSelectionChanged(const TSet<UObject*>& NewSelection)
{
	if (UAircraftDataflowEditor* const AircraftEditor = GetAircraftEditor())
	{
		if (UDataflowBaseContent* const EditorContent = AircraftEditor->GetEditorContent())
		{
			EditorContent->SetSelectedNode(GetOnlyFromSet(FilterDataflowEdNodesFromSet(NewSelection)));
		}
	}
}

void FAircraftAssetEditorToolkit::OnNodeDeleted(const TSet<UObject*>& DeletedNodes)
{
	(void)DeletedNodes;

	if (UAircraftDataflowEditor* const AircraftEditor = GetAircraftEditor())
	{
		if (UDataflowBaseContent* const EditorContent = AircraftEditor->GetEditorContent())
		{
			EditorContent->SetSelectedNode(GraphEditor ? Cast<UDataflowEdNode>(GraphEditor->GetSingleSelectedNode()) : nullptr);
		}
	}
}

UAircraftDataflowEditor* FAircraftAssetEditorToolkit::GetAircraftEditor() const
{
	return Cast<UAircraftDataflowEditor>(OwningAssetEditor);
}

UAircraftAssetBase* FAircraftAssetEditorToolkit::GetAsset() const
{
	if (!OwningAssetEditor)
	{
		return nullptr;
	}

	TArray<TObjectPtr<UObject>> ObjectsToEdit;
	OwningAssetEditor->GetObjectsToEdit(MutableView(ObjectsToEdit));
	return ObjectsToEdit.IsEmpty() ? nullptr : Cast<UAircraftAssetBase>(ObjectsToEdit[0]);
}

UDataflow* FAircraftAssetEditorToolkit::GetDataflow()
{
	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		return AircraftAsset->GetDataflow();
	}

	return nullptr;
}

TSet<TObjectPtr<UDataflowEdNode>> FAircraftAssetEditorToolkit::FilterDataflowEdNodesFromSet(const TSet<UObject*>& Set)
{
	TSet<TObjectPtr<UDataflowEdNode>> DataflowEdNodes;
	for (UObject* const Object : Set)
	{
		if (UDataflowEdNode* const EdNode = Cast<UDataflowEdNode>(Object))
		{
			DataflowEdNodes.Add(EdNode);
		}
	}
	return DataflowEdNodes;
}

TObjectPtr<UDataflowEdNode> FAircraftAssetEditorToolkit::GetOnlyFromSet(const TSet<TObjectPtr<UDataflowEdNode>>& Set)
{
	return Set.Num() == 1 ? *Set.CreateConstIterator() : nullptr;
}

#undef LOCTEXT_NAMESPACE
