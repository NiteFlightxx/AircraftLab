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
#include "AircraftAsset/SAircraftAssetEditorAdvancedPreviewDetailsTab.h"
#include "AircraftAsset/SAircraftCollectionOutliner.h"
#include "AircraftAsset/SAircraftSceneOutliner.h"
#include "AircraftAsset/SAircraftToolsPanel.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftAssetEditorCommands.h"
#include "AircraftAsset/AircraftAssetEditorViewportClient.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftEditorSimulationVisualization.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftDataflowEditor.h"
#include "AircraftAsset/AircraftEditorMode.h"
#include "AircraftAsset/AircraftEditorModeUILayer.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "Dataflow/DataflowContent.h"
#include "FileHelpers.h"
#include "Toolkits/AssetEditorToolkitMenuContext.h"
#include "UObject/PackageReload.h"
#include "Widgets/Layout/SSpacer.h"

#define LOCTEXT_NAMESPACE "AircraftAssetEditorToolkit"

// Same as SDataflowGraphEditor, only adds a AircraftAssetEditorToolkit pointer to correctly override GetDataflowContext().
class SAircraftAssetDataflowGraphEditor : public SDataflowGraphEditor
{
public:
	SLATE_BEGIN_ARGS(SAircraftAssetDataflowGraphEditor) {}
		SLATE_ARGUMENT_DEFAULT(UEdGraph*, GraphToEdit) = nullptr;
		SLATE_ARGUMENT(FGraphEditorEvents, GraphEvents)
		SLATE_ARGUMENT(TSharedPtr<IStructureDetailsView>, DetailsView)
		SLATE_ARGUMENT(SDataflowGraphEditor::FGraphEvaluationCallback, EvaluateGraph)
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
		return ensure(AircraftAssetEditorToolkit) ? AircraftAssetEditorToolkit->GetDataflowContext() : TSharedPtr<UE::Dataflow::FContext>();
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
const FName FAircraftAssetEditorToolkit::SceneOutlinerTabId(TEXT("AircraftAssetEditor_SceneOutliner"));
const FName FAircraftAssetEditorToolkit::ToolsPanelTabId(TEXT("AircraftAssetEditor_ToolsPanel"));

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
						->AddTab(ToolsPanelTabId, ETabState::OpenedTab)
						->SetExtensionId(UBaseCharacterFXEditorUISubsystem::EditorSidePanelAreaName)
						->SetHideTabWell(false)
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
					->AddTab(SceneOutlinerTabId, ETabState::OpenedTab)
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

	if (SelectedDataflowNode && OnNodeInvalidatedDelegateHandle.IsValid())
	{
		SelectedDataflowNode->GetOnNodeInvalidatedDelegate().Remove(OnNodeInvalidatedDelegateHandle);
	}

	FCoreUObjectDelegates::OnPackageReloaded.Remove(OnPackageReloadedDelegateHandle);
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

	InTabManager->RegisterTabSpawner(SceneOutlinerTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_SceneOutliner))
		.SetDisplayName(LOCTEXT("SceneOutlinerTab", "Scene Outliner"))
		.SetGroup(EditorMenuCategory.ToSharedRef());

	InTabManager->RegisterTabSpawner(ToolsPanelTabId, FOnSpawnTab::CreateSP(this, &FAircraftAssetEditorToolkit::SpawnTab_ToolsPanel))
		.SetDisplayName(LOCTEXT("ToolsPanelTab", "Aircraft Tools"))
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
	InTabManager->UnregisterTabSpawner(SceneOutlinerTabId);
	InTabManager->UnregisterTabSpawner(ToolsPanelTabId);
}

void FAircraftAssetEditorToolkit::GetSaveableObjects(TArray<UObject*>& OutObjects) const
{
	FBaseCharacterFXEditorToolkit::GetSaveableObjects(OutObjects);

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		OutObjects.AddUnique(AircraftAsset);

		if (UDataflow* const DataflowAsset = AircraftAsset->GetDataflow())
		{
			// 内嵌 Dataflow 随资产包一起保存；仅外部独立资产才需要单独保存
			if (DataflowAsset->IsAsset())
			{
				OutObjects.AddUnique(DataflowAsset);
			}
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
			.EditorViewportClient(ViewportClient)
			.ToolkitCommandList(GetToolkitCommands().ToSharedPtr());
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

	// 仿真可视化配置（视口左上角状态文本 + Show 菜单扩展共用）
	if (!SimulationVisualization)
	{
		SimulationVisualization = MakeShared<FAircraftEditorSimulationVisualization>();
	}

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		PreviewScene->SetAircraftAsset(AircraftAsset);
	}

	InitDetailsViewPanel();

	PreviewViewportClient = StaticCastSharedPtr<FAircraftAssetEditorViewportClient>(ViewportClient);
	if (PreviewViewportClient.IsValid())
	{
		PreviewViewportClient->SetSimulationVisualization(SimulationVisualization);
	}

	// 处理 Dataflow 包重载事件（对齐 FChaosClothAssetEditorToolkit）
	OnPackageReloadedDelegateHandle = FCoreUObjectDelegates::OnPackageReloaded.AddSP(this, &FAircraftAssetEditorToolkit::HandlePackageReloaded);

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

			FToolMenuSection& AircraftToolsSection = AssetToolbar->FindOrAddSection("AircraftInteractiveTools");
			AircraftToolsSection.AddEntry(FToolMenuEntry::InitToolBarButton(FAircraftAssetEditorCommands::Get().MotorPlacement));
			AircraftToolsSection.AddEntry(FToolMenuEntry::InitToolBarButton(FAircraftAssetEditorCommands::Get().PidTuning));
			AircraftToolsSection.AddEntry(FToolMenuEntry::InitToolBarButton(FAircraftAssetEditorCommands::Get().ThrustVectorOrientation));
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

	// Dataflow Members 面板：选中节点的输出 Collection 数据查看器（对齐 SClothCollectionOutliner），
	// 附加顶部已编译模型摘要。
	SAssignNew(OutlinerDockTab, SDockTab)
		.Label(LOCTEXT("OutlinerTabTitle", "Dataflow Members"))
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(8.0f)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("OutlinerDescription", "Compiled Aircraft Structure"))
				.Font(FAppStyle::GetFontStyle(TEXT("DetailsView.CategoryFontStyle")))
			]
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(8.0f, 0.0f, 8.0f, 4.0f)
			[
				SNew(STextBlock)
				.Text(this, &FAircraftAssetEditorToolkit::GetOutlinerSummaryText)
				.AutoWrapText(true)
			]
			+ SVerticalBox::Slot()
			.FillHeight(1.0f)
			.Padding(8.0f, 0.0f, 8.0f, 8.0f)
			[
				SAssignNew(CollectionOutliner, SAircraftCollectionOutliner)
			]
		];

	// 当前已选中节点（面板可能在选中之后才被创建）
	if (TSharedPtr<FDataflowNode> SelectedNode = GetSelectedDataflowNode())
	{
		CollectionOutliner->SetAircraftCollection(GetAircraftCollectionIfPossible(SelectedNode, DataflowContext));
	}

	return OutlinerDockTab.ToSharedRef();
}

FText FAircraftAssetEditorToolkit::GetOutlinerSummaryText() const
{
	const UAircraftAssetBase* const AircraftAsset = GetAsset();
	const TSharedPtr<const FAircraftSimulationModel> Model = AircraftAsset
		? AircraftAsset->GetAircraftSimulationModel(0)
		: nullptr;
	if (!Model.IsValid())
	{
		return LOCTEXT("OutlinerNoCompiledModel", "No compiled simulation model. Evaluate the terminal node to build the asset.");
	}
	if (Model->GetNumLods() == 0)
	{
		return LOCTEXT("OutlinerNoCompiledLOD0", "The compiled simulation model has no LOD 0.");
	}

	FString Summary = FString::Printf(TEXT("Simulation LODs (%d)"), Model->GetNumLods());
	for (int32 LodIndex = 0; LodIndex < Model->GetNumLods(); ++LodIndex)
	{
		const FAircraftSimulationLodModel& LodModel = Model->LodModels[LodIndex];
		const FAircraftSimulationLODRuntimeSettings* const Settings = Model->SimulationLOD.LODs.IsEmpty()
			? nullptr
			: &Model->SimulationLOD.LODs[FMath::Min(LodIndex, Model->SimulationLOD.LODs.Num() - 1)];
		const FString DriveMode = Settings
			? UEnum::GetDisplayValueAsText(Settings->DriveMode).ToString()
			: TEXT("Flight Controller");
		const FString AsyncPhysicsSummary = LodModel.bOverrideSolverAsyncDeltaTime
			? FString::Printf(TEXT("%.3f ms"), LodModel.SolverAsyncDeltaTime * 1000.0f)
			: TEXT("Project settings");
		Summary += FString::Printf(
			TEXT("\n\nLOD %d - %s\n  Root body: %s\n  Mass: %.3f kg\n  Async physics: %s\n  Rotors: %d"),
			LodIndex,
			*DriveMode,
			*LodModel.RootBone.ToString(),
			LodModel.Mass.MassKg,
			*AsyncPhysicsSummary,
			LodModel.Rotors.Num());
	}

	return FText::FromString(MoveTemp(Summary));
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_SimulationVisualization(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == SimulationVisualizationTabId);

	SAssignNew(SimulationVisualizationDockTab, SDockTab)
		.Label(LOCTEXT("SimulationVisualizationTitle", "Simulation Visualization"));

	// 菜单内容由 FAircraftEditorSimulationVisualization 统一维护（对齐 ClothEditorSimulationVisualization）
	FMenuBuilder MenuBuilder(false, nullptr);
	if (SimulationVisualization.IsValid() && PreviewViewportClient.IsValid())
	{
		SimulationVisualization->ExtendViewportShowMenu(MenuBuilder, PreviewViewportClient.ToSharedRef());
	}

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

	SAssignNew(NodeDetailsTab, SDockTab)
		.Label(LOCTEXT("NodeDetailsTabTitle", "Node Details"));

	if (NodeDetailsEditor.IsValid())
	{
		NodeDetailsTab->SetContent(NodeDetailsEditor->GetWidget().ToSharedRef());
	}

	return NodeDetailsTab.ToSharedRef();
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_SceneOutliner(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == SceneOutlinerTabId);

	SAssignNew(SceneOutlinerDockTab, SDockTab)
		.Label(LOCTEXT("SceneOutlinerTabTitle", "Scene Outliner"));

	SceneOutliner = SNew(SAircraftSceneOutliner, PreviewScene.ToWeakPtr());
	SceneOutliner->OnFocusRequested().BindLambda([this]()
	{
		if (PreviewViewportWidget.IsValid())
		{
			PreviewViewportWidget->OnFocusViewportToSelection();
		}
	});

	SceneOutlinerDockTab->SetContent(SceneOutliner.ToSharedRef());

	return SceneOutlinerDockTab.ToSharedRef();
}

TSharedRef<SDockTab> FAircraftAssetEditorToolkit::SpawnTab_ToolsPanel(const FSpawnTabArgs& Args)
{
	check(Args.GetTabId() == ToolsPanelTabId);

	TSharedRef<SDockTab> ToolsPanelDockTab = SNew(SDockTab)
		.Label(LOCTEXT("ToolsPanelTabTitle", "Aircraft Tools"))
		[
			SNew(SAircraftToolsPanel)
		];

	return ToolsPanelDockTab;
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
		SNew(SAircraftAssetEditorAdvancedPreviewDetailsTab, PreviewScene.ToSharedRef())
		.AdditionalSettings(PreviewScene->GetPreviewSceneDescription())
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
	// 资产 Dataflow 引用变化：重建图编辑器（对齐 ClothEditorToolkit::OnFinishedChangingAssetProperties）
	const FProperty* const ChangedProperty = PropertyChangedEvent.Property;
	if (ChangedProperty && ChangedProperty->GetFName() == TEXT("DataflowAsset"))
	{
		UDataflow* const Dataflow = GetDataflow();
		if (Dataflow)
		{
			Dataflow->Schema = UDataflowSchema::StaticClass();
			ReinitializeGraphEditorWidget();
		}
		else
		{
			// 无法在没有 UDataflow 的情况下构造 SDataflowGraphEditor：用占位 widget 清空区域
			GraphEditor.Reset();
			if (GraphEditorTab.IsValid())
			{
				GraphEditorTab->SetContent(SNew(SSpacer));
			}
			if (NodeDetailsTab.IsValid())
			{
				NodeDetailsTab->SetContent(SNew(SSpacer));
			}
		}

		if (UAircraftAssetEditorMode* const AircraftMode =
			Cast<UAircraftAssetEditorMode>(EditorModeManager->GetActiveScriptableMode(UAircraftAssetEditorMode::EM_AircraftAssetEditorModeId)))
		{
			AircraftMode->SetPreviewScene(PreviewScene.Get());
		}
	}

	OnAircraftAssetChanged();
}

void FAircraftAssetEditorToolkit::OnAircraftAssetChanged()
{
	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		ensure(AircraftAsset->HasAnyFlags(RF_Transactional));
		SetEditingObject(AircraftAsset);

		if (SceneOutliner.IsValid())
		{
			SceneOutliner->Refresh();
		}

		const UAircraftComponent* const PreviewAircraftComponent = PreviewScene->GetAircraftComponent();
		const bool bHadAircraftAsset = PreviewAircraftComponent && PreviewAircraftComponent->GetAsset() != nullptr;
		const bool bWasSimulationEnabled = bHadAircraftAsset ? PreviewScene->IsSimulationEnabled() : true;
		const bool bWasSimulationSuspended = bHadAircraftAsset && PreviewScene->IsSimulationSuspended();
		PreviewScene->SetAircraftAsset(AircraftAsset);

		if (bHadAircraftAsset)
		{
			PreviewScene->SetEnableSimulation(bWasSimulationEnabled);
			if (bWasSimulationEnabled)
			{
				if (bWasSimulationSuspended)
				{
					PreviewScene->SuspendSimulation();
				}
				else
				{
					PreviewScene->ResumeSimulation();
				}
			}
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

	const SDataflowGraphEditor::FGraphEvaluationCallback EvaluateGraph =
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

	// Dataflow Members 面板：跟踪选中节点并抓取其输出 Collection
	TSharedPtr<FDataflowNode> NewSelectedDataflowNode;
	if (UDataflowEdNode* const SelectedEdNode = GetOnlyFromSet(FilterDataflowEdNodesFromSet(NewSelection)))
	{
		if (UDataflow* const Dataflow = GetDataflow())
		{
			if (TSharedPtr<UE::Dataflow::FGraph> Graph = Dataflow->GetDataflow())
			{
				NewSelectedDataflowNode = Graph->FindBaseNode(SelectedEdNode->GetDataflowNodeGuid());
			}
		}
	}

	if (SelectedDataflowNode && OnNodeInvalidatedDelegateHandle.IsValid())
	{
		SelectedDataflowNode->GetOnNodeInvalidatedDelegate().Remove(OnNodeInvalidatedDelegateHandle);
		OnNodeInvalidatedDelegateHandle.Reset();
	}
	SelectedDataflowNode = NewSelectedDataflowNode;

	if (SelectedDataflowNode)
	{
		SelectedDataflowNodeGuid = SelectedDataflowNode->GetGuid();
		// 节点失效后重新求值并重取 Collection
		OnNodeInvalidatedDelegateHandle = SelectedDataflowNode->GetOnNodeInvalidatedDelegate().AddLambda(
			[this](FDataflowNode* InvalidatedNode)
			{
				const TSharedPtr<FDataflowNode> CurrentSelected = GetSelectedDataflowNode();
				if (CurrentSelected.Get() == InvalidatedNode && CollectionOutliner.IsValid())
				{
					CollectionOutliner->SetAircraftCollection(GetAircraftCollectionIfPossible(CurrentSelected, DataflowContext));
				}
				TickCommands.AddLambda([this]()
				{
					if (NodeDetailsEditor && NodeDetailsEditor->GetDetailsView())
					{
						NodeDetailsEditor->GetDetailsView()->InvalidateCachedState();
					}
				});
			});
	}
	else
	{
		SelectedDataflowNodeGuid.Invalidate();
	}

	if (CollectionOutliner.IsValid())
	{
		CollectionOutliner->SetAircraftCollection(
			GetAircraftCollectionIfPossible(SelectedDataflowNode, DataflowContext));
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

/* ---------------------------------------------------------------------------
 * Dataflow Members 面板 / 选中节点
 * ------------------------------------------------------------------------- */

TSharedPtr<FDataflowNode> FAircraftAssetEditorToolkit::GetSelectedDataflowNode()
{
	if (SelectedDataflowNodeGuid.IsValid())
	{
		if (UDataflow* const Dataflow = GetDataflow())
		{
			if (TSharedPtr<UE::Dataflow::FGraph> Graph = Dataflow->GetDataflow())
			{
				return Graph->FindBaseNode(SelectedDataflowNodeGuid);
			}
		}
	}
	return TSharedPtr<FDataflowNode>(nullptr);
}

TSharedPtr<const FDataflowNode> FAircraftAssetEditorToolkit::GetSelectedDataflowNode() const
{
	if (SelectedDataflowNodeGuid.IsValid())
	{
		if (const UDataflow* const Dataflow = GetDataflow())
		{
			if (TSharedPtr<const UE::Dataflow::FGraph> Graph = Dataflow->GetDataflow())
			{
				return Graph->FindBaseNode(SelectedDataflowNodeGuid);
			}
		}
	}
	return TSharedPtr<const FDataflowNode>(nullptr);
}

TSharedPtr<FManagedArrayCollection> FAircraftAssetEditorToolkit::GetAircraftCollectionIfPossible(
	const TSharedPtr<FDataflowNode> InDataflowNode, const TSharedPtr<UE::Dataflow::FEngineContext> Context) const
{
	if (InDataflowNode && Context)
	{
		for (const FDataflowOutput* const Output : InDataflowNode->GetOutputs())
		{
			if (Output->GetType() == FName("FManagedArrayCollection"))
			{
				const FManagedArrayCollection DefaultValue;
				TSharedRef<FManagedArrayCollection> Collection =
					MakeShared<FManagedArrayCollection>(Output->GetValue<FManagedArrayCollection>(*Context, DefaultValue));

				// 只接受 Aircraft Collection（Schema 完整）
				const UE::AircraftLab::AircraftAsset::FConstAircraftCollection AircraftFacade(Collection);
				if (AircraftFacade.IsValid())
				{
					return Collection;
				}
				break;
			}
		}
	}

	return TSharedPtr<FManagedArrayCollection>();
}

/* ---------------------------------------------------------------------------
 * 图编辑器韧性：Dataflow 热重载 / 包重载
 * ------------------------------------------------------------------------- */

void FAircraftAssetEditorToolkit::ReinitializeGraphEditorWidget()
{
	UDataflow* const Dataflow = GetDataflow();
	ensure(Dataflow);

	const SDataflowGraphEditor::FGraphEvaluationCallback EvaluateGraph =
		[this](const FDataflowNode* Node, const FDataflowOutput* Output)
		{
			EvaluateNode(Node, Output);
		};

	SGraphEditor::FGraphEditorEvents GraphEditorEvents;
	GraphEditorEvents.OnVerifyTextCommit = FOnNodeVerifyTextCommit::CreateSP(this, &FAircraftAssetEditorToolkit::OnNodeVerifyTitleCommit);
	GraphEditorEvents.OnTextCommitted = FOnNodeTextCommitted::CreateSP(this, &FAircraftAssetEditorToolkit::OnNodeTitleCommitted);

	if (!GraphEditor)
	{
		UAircraftAssetBase* const AircraftAsset = GetAsset();
		if (AircraftAsset)
		{
			NodeDetailsEditor = CreateNodeDetailsEditorWidget(AircraftAsset);
			if (NodeDetailsTab.IsValid())
			{
				NodeDetailsTab->SetContent(NodeDetailsEditor->GetWidget().ToSharedRef());
			}
		}

		GraphEditor = CreateGraphEditorWidget();
		if (GraphEditorTab.IsValid() && GraphEditor)
		{
			GraphEditorTab->SetContent(GraphEditor.ToSharedRef());
		}
	}
		else
		{
			UAircraftAssetBase* const AircraftAsset = GetAsset();
			const TSharedRef<SAircraftAssetDataflowGraphEditor> AircraftGraphEditor =
				StaticCastSharedRef<SAircraftAssetDataflowGraphEditor>(GraphEditor.ToSharedRef());

			SAircraftAssetDataflowGraphEditor::FArguments Args;
			Args._GraphToEdit = Dataflow;
			Args._GraphEvents = GraphEditorEvents;
			Args._DetailsView = NodeDetailsEditor;
			Args._EvaluateGraph = EvaluateGraph;
			Args._AircraftAssetEditorToolkit = this;

			AircraftGraphEditor->Construct(Args, AircraftAsset);

			GraphEditor->OnSelectionChangedMulticast.RemoveAll(this);
			GraphEditor->OnNodeDeletedMulticast.RemoveAll(this);
			GraphEditor->OnSelectionChangedMulticast.AddSP(this, &FAircraftAssetEditorToolkit::OnNodeSelectionChanged);
			GraphEditor->OnNodeDeletedMulticast.AddSP(this, &FAircraftAssetEditorToolkit::OnNodeDeleted);
		}
	}

void FAircraftAssetEditorToolkit::HandlePackageReloaded(const EPackageReloadPhase InPackageReloadPhase, FPackageReloadedEvent* InPackageReloadedEvent)
{
	// UAssetEditorSubsystem::HandlePackageReloaded 负责重启相应资产编辑器；
	// 但 Aircraft 资产编辑器内嵌的 Dataflow 不会被系统跟踪，这里自己处理：
	// 不重启整个编辑器，只重建图编辑器 Widget。

	if (InPackageReloadPhase == EPackageReloadPhase::PrePackageFixup)
	{
		checkf(InPackageReloadedEvent, TEXT("Expected a FPackageReloadedEvent object on PrePackageFixup phase"));

		for (const TPair<UObject*, UObject*>& RepointPair : InPackageReloadedEvent->GetRepointedObjects())
		{
			if (RepointPair.Key == GetDataflow())
			{
				// 清除所有持有即将重载 Dataflow 对象的引用（含节点）
				SelectedDataflowNode.Reset();
				SelectedDataflowNodeGuid.Invalidate();
				OnNodeInvalidatedDelegateHandle.Reset();
				GraphEditor.Reset();
				if (GraphEditorTab.IsValid())
				{
					GraphEditorTab->SetContent(SNew(SSpacer));
				}
			}
		}
	}
	else if (InPackageReloadPhase == EPackageReloadPhase::PostPackageFixup)
	{
		for (const TPair<UObject*, UObject*>& RepointPair : InPackageReloadedEvent->GetRepointedObjects())
		{
			if (RepointPair.Key == GetDataflow())
			{
				ReinitializeGraphEditorWidget();
			}
		}
	}
}

/* ---------------------------------------------------------------------------
 * 保存流 / 菜单上下文 / 关闭生命周期
 * ------------------------------------------------------------------------- */

void FAircraftAssetEditorToolkit::InitToolMenuContext(FToolMenuContext& MenuContext)
{
	FAssetEditorToolkit::InitToolMenuContext(MenuContext);

	UAssetEditorToolkitMenuContext* const AircraftEditorContext = NewObject<UAssetEditorToolkitMenuContext>();
	AircraftEditorContext->Toolkit = SharedThis(this);
	MenuContext.AddObject(AircraftEditorContext);
}

void FAircraftAssetEditorToolkit::OnAssetsSaved(const TArray<UObject*>& SavedObjects)
{
	// 外部 Dataflow 的引用对象也一并提示保存（迁移期兼容；内嵌时 References 为空）
	TArray<UPackage*> PackagesToSave;

	if (UAircraftAssetBase* const AircraftAsset = GetAsset())
	{
		if (UDataflow* const Dataflow = AircraftAsset->GetDataflow())
		{
			if (TSharedPtr<UE::Dataflow::FGraph> Graph = Dataflow->GetDataflow())
			{
				TArray<UObject*> References;
				FReferenceFinder ReferenceFinder(References, nullptr, false, true, false, true);
				Dataflow->Dataflow->AddReferencedObjects(ReferenceFinder);

				for (UObject* const Reference : References)
				{
					if (Reference && Reference->IsAsset())
					{
						PackagesToSave.AddUnique(Reference->GetOutermost());
					}
				}
			}
		}
	}

	if (PackagesToSave.Num() > 0)
	{
		constexpr bool bCheckDirtyOnReferenceAssetSave = true;
		constexpr bool bPromptToSave = true;
		FEditorFileUtils::PromptForCheckoutAndSave(PackagesToSave, bCheckDirtyOnReferenceAssetSave, bPromptToSave);
	}
}

void FAircraftAssetEditorToolkit::OnAssetsSavedAs(const TArray<UObject*>& SavedObjects)
{
	// "Save As" 场景：让 Aircraft 资产指向新保存的 Dataflow 对象
	UDataflow* NewDataflowAsset = nullptr;
	UAircraftAssetBase* NewAircraftAsset = nullptr;
	for (UObject* const SavedObj : SavedObjects)
	{
		if (SavedObj && SavedObj->IsA<UDataflow>())
		{
			NewDataflowAsset = Cast<UDataflow>(SavedObj);
		}
		else if (SavedObj && SavedObj->IsA<UAircraftAssetBase>())
		{
			NewAircraftAsset = Cast<UAircraftAssetBase>(SavedObj);
		}
	}

	if (NewAircraftAsset && NewDataflowAsset)
	{
		NewAircraftAsset->SetDataflow(NewDataflowAsset);

		// 属性指针已变化，再保存一次
		const TArray<UPackage*> PackagesToSave{ NewAircraftAsset->GetOutermost() };
		constexpr bool bCheckDirty = true;
		constexpr bool bPromptToSave = false;
		FEditorFileUtils::PromptForCheckoutAndSave(PackagesToSave, bCheckDirty, bPromptToSave);
	}
}

bool FAircraftAssetEditorToolkit::ShouldReopenEditorForSavedAsset(const UObject* SavedAsset) const
{
	return SavedAsset && SavedAsset->IsA<UAircraftAssetBase>();
}

bool FAircraftAssetEditorToolkit::OnRequestClose(EAssetEditorCloseReason InCloseReason)
{
	// 关闭时释放选中节点的失效监听 + 包重载监听
	if (SelectedDataflowNode && OnNodeInvalidatedDelegateHandle.IsValid())
	{
		SelectedDataflowNode->GetOnNodeInvalidatedDelegate().Remove(OnNodeInvalidatedDelegateHandle);
		OnNodeInvalidatedDelegateHandle.Reset();
	}
	SelectedDataflowNode.Reset();
	SelectedDataflowNodeGuid.Invalidate();

	FCoreUObjectDelegates::OnPackageReloaded.Remove(OnPackageReloadedDelegateHandle);
	OnPackageReloadedDelegateHandle.Reset();

	return FAssetEditorToolkit::OnRequestClose(InCloseReason);
}

void FAircraftAssetEditorToolkit::OnClose()
{
	// 给活动模式一次关闭的机会（在 ToolkitHost 仍存活时），否则重开编辑器会重复建页签。
	GetEditorModeManager().ActivateDefaultMode();

	FBaseCharacterFXEditorToolkit::OnClose();
}

#undef LOCTEXT_NAMESPACE
