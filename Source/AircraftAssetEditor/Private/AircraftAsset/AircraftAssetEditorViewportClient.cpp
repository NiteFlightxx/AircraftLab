#include "AircraftAsset/AircraftAssetEditorViewportClient.h"

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftEditorMode.h"

FAircraftAssetEditorViewportClient::FAircraftAssetEditorViewportClient(
	FEditorModeTools* InModeTools,
	const TSharedPtr<FAircraftAssetEditorPreviewScene>& InPreviewScene,
	const TWeakPtr<SEditorViewport>& InEditorViewportWidget)
	: FEditorViewportClient(InModeTools, InPreviewScene.Get(), InEditorViewportWidget)
	, PreviewScene(InPreviewScene)
{
	OverrideNearClipPlane(KINDA_SMALL_NUMBER);
	EngineShowFlags.SetSelectionOutline(false);
	SetViewportType(ELevelViewportType::LVT_Perspective);
	SetViewMode(EViewModeIndex::VMI_Lit);
}

void FAircraftAssetEditorViewportClient::SoftResetSimulation()
{
	if (UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		AircraftMode->SoftResetSimulation();
	}
}

void FAircraftAssetEditorViewportClient::HardResetSimulation()
{
	if (UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		AircraftMode->HardResetSimulation();
	}
}

void FAircraftAssetEditorViewportClient::SuspendSimulation()
{
	if (UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		AircraftMode->SuspendSimulation();
	}
}

void FAircraftAssetEditorViewportClient::ResumeSimulation()
{
	if (UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		AircraftMode->ResumeSimulation();
	}
}

bool FAircraftAssetEditorViewportClient::IsSimulationSuspended() const
{
	if (const UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		return AircraftMode->IsSimulationSuspended();
	}

	return false;
}

void FAircraftAssetEditorViewportClient::SetEnableSimulation(bool bEnable)
{
	if (UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		AircraftMode->SetEnableSimulation(bEnable);
	}
}

bool FAircraftAssetEditorViewportClient::IsSimulationEnabled() const
{
	if (const UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		return AircraftMode->IsSimulationEnabled();
	}

	return false;
}

FBox FAircraftAssetEditorViewportClient::PreviewBoundingBox() const
{
	if (const TSharedPtr<FAircraftAssetEditorPreviewScene> PreviewScenePinned = PreviewScene.Pin())
	{
		if (const UAircraftComponent* const AircraftComponent = PreviewScenePinned->GetAircraftComponent())
		{
			return AircraftComponent->Bounds.GetBox();
		}
	}

	return FBox(ForceInit);
}

UAircraftAssetEditorMode* FAircraftAssetEditorViewportClient::GetAircraftEditorMode() const
{
	return ModeTools
		? Cast<UAircraftAssetEditorMode>(ModeTools->GetActiveScriptableMode(UAircraftAssetEditorMode::EM_AircraftAssetEditorModeId))
		: nullptr;
}
