#include "AircraftAsset/AircraftDataflowSimulationVisualization.h"

#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAutopilot/AutopilotComponent.h"
#include "AircraftDiagnostics/AircraftDebugDraw.h"
#include "Dataflow/DataflowSimulationScene.h"
#include "Dataflow/DataflowSimulationViewportClient.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "GameFramework/Actor.h"

#define LOCTEXT_NAMESPACE "AircraftDataflowSimulationVisualization"

const FName FAircraftDataflowSimulationVisualization::Name(TEXT("Aircraft"));

FName FAircraftDataflowSimulationVisualization::GetName() const
{
	return Name;
}

UAircraftComponent* FAircraftDataflowSimulationVisualization::GetAircraftComponent(
	const FDataflowSimulationScene* SimulationScene)
{
	AActor* const PreviewActor = SimulationScene ? SimulationScene->GetPreviewActor() : nullptr;
	return PreviewActor ? PreviewActor->FindComponentByClass<UAircraftComponent>() : nullptr;
}

bool FAircraftDataflowSimulationVisualization::CaptureSnapshot(
	const FDataflowSimulationScene* SimulationScene,
	FAircraftDebugFrameSnapshot& OutSnapshot)
{
	UAircraftComponent* const Component = GetAircraftComponent(SimulationScene);
	if (!Component)
	{
		return false;
	}
	Component->CaptureDebugSnapshot(OutSnapshot);
	if (AActor* const PreviewActor = SimulationScene->GetPreviewActor())
	{
		if (const UAutopilotComponent* const Autopilot =
			PreviewActor->FindComponentByClass<UAutopilotComponent>())
		{
			Autopilot->AppendDebugSnapshot(OutSnapshot);
		}
	}
	return true;
}

void FAircraftDataflowSimulationVisualization::SynchronizeOptionState() const
{
	TArray<FAircraftDebugOptionView> Options;
	FAircraftDebugRegistry::GetOptionViews(Options);
	TSet<FName> LiveIds;
	for (const FAircraftDebugOptionView& Option : Options)
	{
		LiveIds.Add(Option.Id);
		if (!KnownOptionIds.Contains(Option.Id))
		{
			KnownOptionIds.Add(Option.Id);
			if (Option.bEditorEnabledByDefault)
			{
				EnabledOptionIds.Add(Option.Id);
			}
		}
	}
	for (auto It = KnownOptionIds.CreateIterator(); It; ++It)
	{
		if (!LiveIds.Contains(*It))
		{
			EnabledOptionIds.Remove(*It);
			It.RemoveCurrent();
		}
	}
}

void FAircraftDataflowSimulationVisualization::ExtendSimulationVisualizationMenu(
	const TSharedPtr<FDataflowSimulationViewportClient>& ViewportClient,
	FMenuBuilder& MenuBuilder)
{
	if (!ViewportClient)
	{
		return;
	}
	SynchronizeOptionState();
	TArray<FAircraftDebugOptionView> Options;
	FAircraftDebugRegistry::GetOptionViews(Options);
	TWeakPtr<FDataflowSimulationViewportClient> WeakViewportClient = ViewportClient;
	FName OpenCategory = NAME_None;
	for (const FAircraftDebugOptionView& Option : Options)
	{
		if (Option.Category != OpenCategory)
		{
			if (OpenCategory != NAME_None)
			{
				MenuBuilder.EndSection();
			}
			OpenCategory = Option.Category;
			MenuBuilder.BeginSection(
				FName(*FString::Printf(TEXT("AircraftDiagnostics_%s"), *OpenCategory.ToString())),
				Option.CategoryDisplayName);
		}
		const FName OptionId = Option.Id;
		const FExecuteAction Execute = FExecuteAction::CreateLambda(
			[this, WeakViewportClient, OptionId]()
			{
				if (!EnabledOptionIds.Remove(OptionId))
				{
					EnabledOptionIds.Add(OptionId);
				}
				if (const TSharedPtr<FDataflowSimulationViewportClient> Pinned = WeakViewportClient.Pin())
				{
					Pinned->Invalidate();
				}
			});
		const FIsActionChecked IsChecked = FIsActionChecked::CreateLambda(
			[this, OptionId]() { return EnabledOptionIds.Contains(OptionId); });
		MenuBuilder.AddMenuEntry(Option.DisplayName, Option.ToolTip, FSlateIcon(),
			FUIAction(Execute, FCanExecuteAction(), IsChecked), NAME_None,
			EUserInterfaceActionType::ToggleButton);
	}
	if (OpenCategory != NAME_None)
	{
		MenuBuilder.EndSection();
	}
}

void FAircraftDataflowSimulationVisualization::Draw(
	const FDataflowSimulationScene* SimulationScene, FPrimitiveDrawInterface* PDI)
{
	if (!PDI)
	{
		return;
	}
	SynchronizeOptionState();
	FAircraftDebugFrameSnapshot Snapshot;
	if (!CaptureSnapshot(SimulationScene, Snapshot))
	{
		return;
	}
	FAircraftDebugDrawContext Context;
	Context.PDI = PDI;
	FAircraftDebugRegistry::DrawSelected(Snapshot, Context, EnabledOptionIds);
}

void FAircraftDataflowSimulationVisualization::DrawCanvas(
	const FDataflowSimulationScene* SimulationScene, FCanvas* Canvas,
	const FSceneView* SceneView)
{
	if (!Canvas)
	{
		return;
	}
	SynchronizeOptionState();
	FAircraftDebugFrameSnapshot Snapshot;
	if (CaptureSnapshot(SimulationScene, Snapshot))
	{
		FAircraftDebugRegistry::DrawCanvasSelected(
			Snapshot, *Canvas, SceneView, EnabledOptionIds);
	}
}

FText FAircraftDataflowSimulationVisualization::GetDisplayString(
	const FDataflowSimulationScene* SimulationScene) const
{
	SynchronizeOptionState();
	FAircraftDebugFrameSnapshot Snapshot;
	return CaptureSnapshot(SimulationScene, Snapshot)
		? FAircraftDebugRegistry::BuildStatusTextSelected(Snapshot, EnabledOptionIds)
		: FText::GetEmpty();
}

#undef LOCTEXT_NAMESPACE
