#include "AircraftAsset/AircraftDataflowSimulationVisualization.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "Dataflow/DataflowEditorToolkit.h"
#include "Dataflow/DataflowSimulationScene.h"
#include "Dataflow/DataflowSimulationViewportClient.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "GameFramework/Actor.h"

#define LOCTEXT_NAMESPACE "AircraftDataflowSimulationVisualization"

const FName FAircraftDataflowSimulationVisualization::Name(TEXT("Aircraft"));

FName FAircraftDataflowSimulationVisualization::GetName() const { return Name; }

UAircraftComponent* FAircraftDataflowSimulationVisualization::GetAircraftComponent(
	const FDataflowSimulationScene* SimulationScene)
{
	AActor* const PreviewActor = SimulationScene ? SimulationScene->GetPreviewActor() : nullptr;
	return PreviewActor ? PreviewActor->FindComponentByClass<UAircraftComponent>() : nullptr;
}

void FAircraftDataflowSimulationVisualization::PruneSessions() const
{
	for (auto It = Sessions.CreateIterator(); It; ++It)
	{
		if (!It.Value().PreviewActor.IsValid()) It.RemoveCurrent();
	}
}

FAircraftDataflowSimulationVisualization::FSession&
FAircraftDataflowSimulationVisualization::GetSession(
	const FDataflowSimulationScene* SimulationScene) const
{
	PruneSessions();
	AActor* const PreviewActor = SimulationScene ? SimulationScene->GetPreviewActor() : nullptr;
	check(PreviewActor);
	FSession& Session = Sessions.FindOrAdd(SimulationScene);
	if (Session.PreviewActor.Get() != PreviewActor)
	{
		Session.PreviewActor = PreviewActor;
		Session.CachedFrameNumber = MAX_uint64;
	}
	SynchronizeOptionState(Session);
	return Session;
}

void FAircraftDataflowSimulationVisualization::SynchronizeOptionState(FSession& Session)
{
	TArray<FAircraftDebugOptionView> Options;
	FAircraftDebugRegistry::GetOptionViews(Options);
	TSet<FName> LiveIds;
	for (const FAircraftDebugOptionView& Option : Options)
	{
		LiveIds.Add(Option.Id);
		if (!Session.KnownOptionIds.Contains(Option.Id))
		{
			Session.KnownOptionIds.Add(Option.Id);
			if (Option.bEditorEnabledByDefault) Session.EnabledOptionIds.Add(Option.Id);
		}
	}
	for (auto It = Session.KnownOptionIds.CreateIterator(); It; ++It)
	{
		if (!LiveIds.Contains(*It))
		{
			Session.EnabledOptionIds.Remove(*It);
			It.RemoveCurrent();
		}
	}
}

bool FAircraftDataflowSimulationVisualization::CaptureSnapshot(
	const FDataflowSimulationScene* SimulationScene, FSession& Session,
	const FAircraftDebugCaptureRequest& Request,
	const FAircraftDebugFrameSnapshot*& OutSnapshot)
{
	UAircraftComponent* const Component = GetAircraftComponent(SimulationScene);
	if (!Component) return false;
	if (Session.CachedFrameNumber != GFrameCounter || Session.CachedPayloads != Request.Payloads)
	{
		Component->CaptureDebugSnapshot(Request, Session.CachedSnapshot);
		if (AActor* const PreviewActor = SimulationScene->GetPreviewActor())
		{
			if (const UAutopilotComponent* const Autopilot = PreviewActor->FindComponentByClass<UAutopilotComponent>())
			{
				Autopilot->AppendDebugSnapshot(Request, Session.CachedSnapshot);
			}
		}
		Session.CachedFrameNumber = GFrameCounter;
		Session.CachedPayloads = Request.Payloads;
	}
	OutSnapshot = &Session.CachedSnapshot;
	return true;
}

void FAircraftDataflowSimulationVisualization::ExtendSimulationVisualizationMenu(
	const TSharedPtr<FDataflowSimulationViewportClient>& ViewportClient, FMenuBuilder& MenuBuilder)
{
	if (!ViewportClient) return;
	const TSharedPtr<FDataflowEditorToolkit> Toolkit = ViewportClient->GetDataflowEditorToolkit().Pin();
	const TSharedPtr<FDataflowSimulationScene> Scene = Toolkit ? Toolkit->GetSimulationScene() : nullptr;
	if (!Scene || !Scene->GetPreviewActor()) return;
	FSession& Session = GetSession(Scene.Get());
	TArray<FAircraftDebugOptionView> Options;
	FAircraftDebugRegistry::GetOptionViews(Options);
	TWeakPtr<FDataflowSimulationViewportClient> WeakViewportClient = ViewportClient;
	TWeakPtr<FDataflowSimulationScene> WeakScene = Scene;
	FName OpenCategory = NAME_None;
	for (const FAircraftDebugOptionView& Option : Options)
	{
		if (Option.Category != OpenCategory)
		{
			if (OpenCategory != NAME_None) MenuBuilder.EndSection();
			OpenCategory = Option.Category;
			MenuBuilder.BeginSection(FName(*FString::Printf(TEXT("AircraftDiagnostics_%s"), *OpenCategory.ToString())),
				Option.CategoryDisplayName);
		}
		const FName OptionId = Option.Id;
		MenuBuilder.AddMenuEntry(Option.DisplayName, Option.ToolTip, FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, WeakViewportClient, WeakScene, OptionId]()
			{
				if (const TSharedPtr<FDataflowSimulationScene> PinnedScene = WeakScene.Pin(); PinnedScene)
				{
					if (FSession* Existing = Sessions.Find(PinnedScene.Get()))
					{
						if (!Existing->EnabledOptionIds.Remove(OptionId)) Existing->EnabledOptionIds.Add(OptionId);
						Existing->CachedFrameNumber = MAX_uint64;
					}
				}
				if (const TSharedPtr<FDataflowSimulationViewportClient> Pinned = WeakViewportClient.Pin()) Pinned->Invalidate();
			}), FCanExecuteAction(), FIsActionChecked::CreateLambda([this, WeakScene, OptionId]()
			{
				const TSharedPtr<FDataflowSimulationScene> PinnedScene = WeakScene.Pin();
				const FSession* Existing = PinnedScene ? Sessions.Find(PinnedScene.Get()) : nullptr;
				return Existing != nullptr && Existing->EnabledOptionIds.Contains(OptionId);
			})), NAME_None, EUserInterfaceActionType::ToggleButton);
	}
	if (OpenCategory != NAME_None) MenuBuilder.EndSection();
}

void FAircraftDataflowSimulationVisualization::Draw(
	const FDataflowSimulationScene* SimulationScene, FPrimitiveDrawInterface* PDI)
{
	if (!PDI || !SimulationScene || !SimulationScene->GetPreviewActor()) return;
	FSession& Session = GetSession(SimulationScene);
	const FAircraftDebugCaptureRequest Request = FAircraftDebugRegistry::BuildCaptureRequest(
		Session.EnabledOptionIds, EAircraftDebugContext::PreviewSimulation);
	const FAircraftDebugFrameSnapshot* Snapshot = nullptr;
	if (!CaptureSnapshot(SimulationScene, Session, Request, Snapshot)) return;
	FAircraftSimulationDebugDrawBackend Backend(*PDI);
	FAircraftDebugDrawContext Context;
	Context.Backend = &Backend;
	FAircraftDebugRegistry::DrawSelected(*Snapshot, Context, Session.EnabledOptionIds);
}

void FAircraftDataflowSimulationVisualization::DrawCanvas(
	const FDataflowSimulationScene* SimulationScene, FCanvas* Canvas, const FSceneView* SceneView)
{
	if (!Canvas || !SimulationScene || !SimulationScene->GetPreviewActor()) return;
	FSession& Session = GetSession(SimulationScene);
	const FAircraftDebugCaptureRequest Request = FAircraftDebugRegistry::BuildCaptureRequest(
		Session.EnabledOptionIds, EAircraftDebugContext::PreviewSimulation);
	const FAircraftDebugFrameSnapshot* Snapshot = nullptr;
	if (CaptureSnapshot(SimulationScene, Session, Request, Snapshot))
	{
		FAircraftDebugRegistry::DrawCanvasSelected(*Snapshot, *Canvas, SceneView, Session.EnabledOptionIds);
	}
}

FText FAircraftDataflowSimulationVisualization::GetDisplayString(
	const FDataflowSimulationScene* SimulationScene) const
{
	if (!SimulationScene || !SimulationScene->GetPreviewActor()) return FText::GetEmpty();
	FSession& Session = GetSession(SimulationScene);
	const FAircraftDebugCaptureRequest Request = FAircraftDebugRegistry::BuildCaptureRequest(
		Session.EnabledOptionIds, EAircraftDebugContext::PreviewSimulation);
	const FAircraftDebugFrameSnapshot* Snapshot = nullptr;
	if (!CaptureSnapshot(SimulationScene, Session, Request, Snapshot))
	{
		return FText::GetEmpty();
	}
	const FText Playback = SimulationScene->IsSimulationEnabled()
		? LOCTEXT("PlaybackPlaying", "Playing")
		: LOCTEXT("PlaybackPaused", "Paused");
	return FText::Format(LOCTEXT("SimulationStatus", "Playback: {0}\n{1}"),
		Playback,
		FAircraftDebugRegistry::BuildStatusTextSelected(*Snapshot, Session.EnabledOptionIds));
}

#undef LOCTEXT_NAMESPACE
