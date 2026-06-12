#include "AircraftAsset/AircraftEditorMode.h"

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftEditorModeToolkit.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftEditorMode)

#define LOCTEXT_NAMESPACE "AircraftAssetEditorMode"

const FEditorModeID UAircraftAssetEditorMode::EM_AircraftAssetEditorModeId = TEXT("EM_AircraftAssetEditorMode");

UAircraftAssetEditorMode::UAircraftAssetEditorMode()
{
	Info = FEditorModeInfo(
		EM_AircraftAssetEditorModeId,
		LOCTEXT("AircraftAssetEditorModeName", "Aircraft"),
		FSlateIcon(),
		false);
}

void UAircraftAssetEditorMode::SetPreviewScene(FAircraftAssetEditorPreviewScene* InPreviewScene)
{
	PreviewScene = InPreviewScene;
}

void UAircraftAssetEditorMode::SoftResetSimulation()
{
	if (PreviewScene)
	{
	//	PreviewScene->SoftResetSimulation();
	}
}

void UAircraftAssetEditorMode::HardResetSimulation()
{
	if (PreviewScene)
	{
	//	PreviewScene->HardResetSimulation();
	}
}

void UAircraftAssetEditorMode::SuspendSimulation()
{
	if (PreviewScene)
	{
	//	PreviewScene->SuspendSimulation();
	}
}

void UAircraftAssetEditorMode::ResumeSimulation()
{
	if (PreviewScene)
	{
		//PreviewScene->ResumeSimulation();
	}
}

bool UAircraftAssetEditorMode::IsSimulationSuspended() const
{
	return false;// && PreviewScene->IsSimulationSuspended();
}

void UAircraftAssetEditorMode::SetEnableSimulation(bool bEnable)
{
	if (PreviewScene)
	{
	//	PreviewScene->SetEnableSimulation(bEnable);
	}
}

bool UAircraftAssetEditorMode::IsSimulationEnabled() const
{
	return false; //&& PreviewScene->IsSimulationEnabled();
}

void UAircraftAssetEditorMode::ModeTick(float DeltaTime)
{
	Super::ModeTick(DeltaTime);

	if (PreviewScene && PreviewScene->GetWorld())
	{
		PreviewScene->GetWorld()->Tick(ELevelTick::LEVELTICK_All, DeltaTime);
	}
}

FBox UAircraftAssetEditorMode::SceneBoundingBox() const
{
	if (PreviewScene)
	{
		if (const UAircraftComponent* const AircraftComponent = PreviewScene->GetAircraftComponent())
		{
			return AircraftComponent->Bounds.GetBox();
		}
	}

	return FBox(ForceInit);
}

void UAircraftAssetEditorMode::AddToolTargetFactories()
{
}

void UAircraftAssetEditorMode::RegisterTools()
{
}

void UAircraftAssetEditorMode::CreateToolTargets(const TArray<TObjectPtr<UObject>>& AssetsIn)
{
}

void UAircraftAssetEditorMode::CreateToolkit()
{
	Toolkit = MakeShared<FAircraftAssetEditorModeToolkit>();
}

#undef LOCTEXT_NAMESPACE
