#include "AircraftAsset/AircraftAssetEditorViewportClient.h"

#include "AircraftAsset/AircraftAssetEditorCommands.h"
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

void FAircraftAssetEditorViewportClient::EnableRenderMeshWireframe(bool bEnable)
{
	bRenderMeshWireframe = bEnable;
	// 与布料的 ClothComponent::SetForceWireframe 不同：无人机组件没有线框覆盖材质通道，
	// 这里切换整个预览视口的视图模式（UnrealEd 编辑器视口的标准做法）。
	SetViewMode(bRenderMeshWireframe ? VMI_BrushWireframe : VMI_Lit);
	Invalidate();
}

void FAircraftAssetEditorViewportClient::SetLODLevel(int32 LODIndex)
{
	if (UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		AircraftMode->SetLODModel(LODIndex);
		Invalidate();
	}
}

bool FAircraftAssetEditorViewportClient::IsLODSelected(int32 LODIndex) const
{
	if (const UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		return AircraftMode->IsLODModelSelected(LODIndex);
	}
	return false;
}

int32 FAircraftAssetEditorViewportClient::GetCurrentLOD() const
{
	if (const UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		return AircraftMode->GetLODModel();
	}
	return INDEX_NONE;
}

int32 FAircraftAssetEditorViewportClient::GetLODCount() const
{
	if (const UAircraftAssetEditorMode* const AircraftMode = GetAircraftEditorMode())
	{
		return AircraftMode->GetNumLODs();
	}
	return 0;
}

void FAircraftAssetEditorViewportClient::FillLODCommands(TArray<TSharedPtr<FUICommandInfo>>& Commands)
{
	Commands.Add(FAircraftAssetEditorCommands::Get().LODAuto);
	Commands.Add(FAircraftAssetEditorCommands::Get().LOD0);
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
