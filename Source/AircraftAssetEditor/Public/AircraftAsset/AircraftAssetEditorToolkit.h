#pragma once

#include "BaseCharacterFXEditorToolkit.h"
#include "Dataflow/DataflowObjectInterface.h"
#include "Misc/NotifyHook.h"
#include "TickableEditorObject.h"

class FAircraftAssetEditorPreviewScene;
class FAircraftAssetEditorViewportClient;
class IStructureDetailsView;
class SDataflowGraphEditor;
class SDockTab;
class SAircraftAssetEditorViewport;
class UBaseCharacterFXEditorMode;
class UDataflow;
class UDataflowEdNode;
class UAircraftAssetBase;
class UAircraftDataflowEditor;
class UEdGraphNode;

struct FDataflowNode;
struct FDataflowOutput;

namespace UE::Dataflow
{
	class FDataflowNodeDetailExtensionHandler;
	class FAircraftAssetDataflowContext final : public FEngineContext
	{
	public:
		DATAFLOW_CONTEXT_INTERNAL(FEngineContext, FAircraftAssetDataflowContext);

		FAircraftAssetDataflowContext(UObject* InOwner, UDataflow* InGraph)
			: Super(InOwner)
		{
		}
	};

	struct FTimestamp;
}

class FAircraftAssetEditorToolkit : public FBaseCharacterFXEditorToolkit, public FTickableEditorObject, public FNotifyHook
{
public:
	explicit FAircraftAssetEditorToolkit(UAssetEditor* InOwningAssetEditor);
	virtual ~FAircraftAssetEditorToolkit() override;

	TSharedPtr<UE::Dataflow::FEngineContext> GetDataflowContext() const;
	const UDataflow* GetDataflow() const;

	virtual FName GetToolkitFName() const override;
	virtual FText GetToolkitName() const override;
	virtual FText GetBaseToolkitName() const override;
	virtual FText GetToolkitToolTipText() const override;

	virtual void RegisterTabSpawners(const TSharedRef<FTabManager>& InTabManager) override;
	virtual void UnregisterTabSpawners(const TSharedRef<FTabManager>& InTabManager) override;
	virtual void GetSaveableObjects(TArray<UObject*>& OutObjects) const override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickable() const override { return true; }

protected:
	virtual FEditorModeID GetEditorModeId() const override;
	virtual void InitializeEdMode(UBaseCharacterFXEditorMode* EdMode) override;
	virtual void CreateEditorModeUILayer() override;

	virtual void CreateWidgets() override;
	virtual AssetEditorViewportFactoryFunction GetViewportDelegate() override;
	virtual TSharedPtr<FEditorViewportClient> CreateEditorViewportClient() const override;
	virtual void PostInitAssetEditor() override;
	virtual void NotifyPreChange(class FEditPropertyChain* PropertyAboutToChange) override;

private:
	TSharedRef<SDockTab> SpawnTab_Outliner(const FSpawnTabArgs& Args);
	TSharedRef<SDockTab> SpawnTab_SimulationVisualization(const FSpawnTabArgs& Args);
	TSharedRef<SDockTab> SpawnTab_GraphCanvas(const FSpawnTabArgs& Args);
	TSharedRef<SDockTab> SpawnTab_NodeDetails(const FSpawnTabArgs& Args);
	TSharedRef<SDockTab> SpawnTab_PreviewSceneDetails(const FSpawnTabArgs& Args);

	void InitDetailsViewPanel();
	void OnFinishedChangingAssetProperties(const FPropertyChangedEvent& PropertyChangedEvent);
	void OnAircraftAssetChanged();
	void InvalidateViews();

	void EvaluateNode(const FDataflowNode* Node, const FDataflowOutput* Output);
	TSharedRef<SDataflowGraphEditor> CreateGraphEditorWidget();
	TSharedPtr<IStructureDetailsView> CreateNodeDetailsEditorWidget(UObject* ObjectToEdit);

	void OnPropertyValueChanged(const FPropertyChangedEvent& PropertyChangedEvent);
	bool OnNodeVerifyTitleCommit(const FText& NewText, UEdGraphNode* GraphNode, FText& OutErrorMessage) const;
	void OnNodeTitleCommitted(const FText& InNewText, ETextCommit::Type InCommitType, UEdGraphNode* GraphNode) const;
	void OnNodeSelectionChanged(const TSet<UObject*>& NewSelection);
	void OnNodeDeleted(const TSet<UObject*>& DeletedNodes);

	UAircraftDataflowEditor* GetAircraftEditor() const;
	UAircraftAssetBase* GetAsset() const;
	UDataflow* GetDataflow();

	static TSet<TObjectPtr<UDataflowEdNode>> FilterDataflowEdNodesFromSet(const TSet<UObject*>& Set);
	static TObjectPtr<UDataflowEdNode> GetOnlyFromSet(const TSet<TObjectPtr<UDataflowEdNode>>& Set);

	static const FName OutlinerTabId;
	static const FName SimulationVisualizationTabId;
	static const FName GraphCanvasTabId;
	static const FName NodeDetailsTabId;
	static const FName PreviewSceneDetailsTabId;

	TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScene;
	TSharedPtr<FAircraftAssetEditorViewportClient> PreviewViewportClient;
	TSharedPtr<SAircraftAssetEditorViewport> PreviewViewportWidget;
	TSharedPtr<SDataflowGraphEditor> GraphEditor;
	TSharedPtr<IStructureDetailsView> NodeDetailsEditor;
	TSharedPtr<UE::Dataflow::FDataflowNodeDetailExtensionHandler> NodeDetailsExtensionHandler;
	TSharedPtr<SDockTab> OutlinerDockTab;
	TSharedPtr<SDockTab> GraphEditorTab;
	TSharedPtr<SDockTab> PreviewSceneDockTab;
	TSharedPtr<SDockTab> SimulationVisualizationDockTab;
	TSharedPtr<SWidget> AdvancedPreviewSettingsWidget;

	TSharedPtr<UE::Dataflow::FEngineContext> DataflowContext;
	UE::Dataflow::FTimestamp LastDataflowNodeTimestamp = UE::Dataflow::FTimestamp::Invalid;

	DECLARE_MULTICAST_DELEGATE(FTickCommands)
	FTickCommands TickCommands;
};
