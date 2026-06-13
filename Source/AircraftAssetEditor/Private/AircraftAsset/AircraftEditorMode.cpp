#include "AircraftAsset/AircraftEditorMode.h"

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftEditorContextObject.h"
#include "AircraftAsset/AircraftEditorModeToolkit.h"
#include "AircraftAsset/AircraftMotorPlacementTool.h"
#include "AircraftAsset/AircraftPidTuningTool.h"
#include "AircraftAsset/AircraftThrustVectorOrientationTool.h"
#include "ContextObjectStore.h"
#include "InteractiveToolManager.h"

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

	// 把上下文对象加入 ToolManager 的 ContextObjectStore，让 InteractiveTool 在 Build/Setup 时可以
	// 通过 FindContext<UAircraftEditorContextObject>() 拿到当前 Component / Asset。
	if (UInteractiveToolManager* const Manager = GetToolManager())
	{
		if (UContextObjectStore* const Store = Manager->GetContextObjectStore())
		{
			UAircraftEditorContextObject* Ctx = Store->FindContext<UAircraftEditorContextObject>();
			if (!Ctx)
			{
				Ctx = NewObject<UAircraftEditorContextObject>(Manager);
				Store->AddContextObject(Ctx);
			}
			Ctx->SetAircraftComponent(PreviewScene ? PreviewScene->GetAircraftComponent() : nullptr);
		}
	}
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
	return false;
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
	return false;
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
	// 对齐 ChaosClothAssetEditorMode::RegisterTools()：把 3 个 Builder 注册到 ToolManager
	// 关联到字符串 ID，外部（Toolkit / EditorMode 命令）通过这些 ID 激活相应 Tool。
	UInteractiveToolManager* const Manager = GetToolManager();
	if (!Manager)
	{
		return;
	}

	Manager->RegisterToolType(TEXT("AircraftMotorPlacement"),
		NewObject<UAircraftMotorPlacementToolBuilder>(Manager));
	Manager->RegisterToolType(TEXT("AircraftPidTuning"),
		NewObject<UAircraftPidTuningToolBuilder>(Manager));
	Manager->RegisterToolType(TEXT("AircraftThrustVectorOrientation"),
		NewObject<UAircraftThrustVectorOrientationToolBuilder>(Manager));
}

void UAircraftAssetEditorMode::CreateToolTargets(const TArray<TObjectPtr<UObject>>& AssetsIn)
{
}

void UAircraftAssetEditorMode::CreateToolkit()
{
	Toolkit = MakeShared<FAircraftAssetEditorModeToolkit>();
}

#undef LOCTEXT_NAMESPACE
