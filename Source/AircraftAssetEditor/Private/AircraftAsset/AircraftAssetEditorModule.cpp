#include "AircraftAsset/AircraftAssetEditorModule.h"

#include "ThumbnailRendering/ThumbnailManager.h"
#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftAssetEditorCommands.h"
#include "AircraftAsset/AircraftAssetEditorStyle.h"
#include "AircraftAsset/AircraftAssetThumbnailRenderer.h"
#include "AircraftAsset/AircraftComponent.h"

IMPLEMENT_MODULE(FAircraftAssetEditorModule, AircraftAssetEditor)

namespace UE::AircraftDataflowEditor
{
	struct FAircraftAssetComponentBroker : public IComponentAssetBroker
	{
		virtual UClass* GetSupportedAssetClass() override
		{
			return UAircraftAssetBase::StaticClass();
		}

		virtual bool AssignAssetToComponent(UActorComponent* InComponent, UObject* InAsset) override
		{
			if (UAircraftComponent* AircraftComponent = Cast<UAircraftComponent>(InComponent))
			{
				if (UAircraftAssetBase* AircraftAsset = Cast<UAircraftAssetBase>(InAsset))
				{
					AircraftComponent->SetAsset(AircraftAsset);
					return true;
				}
			}

			return false;
		}

		virtual UObject* GetAssetFromComponent(UActorComponent* InComponent) override
		{
			if (const UAircraftComponent* AircraftComponent = Cast<UAircraftComponent>(InComponent))
			{
				return AircraftComponent->GetAsset();
			}

			return nullptr;
		}
	};
}

void FAircraftAssetEditorModule::StartupModule()
{
	FBaseCharacterFXEditorModule::StartupModule();

	// 主动触发自家 SlateStyle 注册（与 ChaosCloth 模块入口模式一致）。这必须在
	// FAircraftAssetEditorCommands::Register() 之前调用，因为命令构造函数会通过
	// FAircraftAssetEditorStyle::GetStyleName() 拿 StyleSet 名字。
	FAircraftAssetEditorStyle::Get();

	FAircraftAssetEditorCommands::Register();

	AircraftAssetComponentBroker = MakeShared<UE::AircraftDataflowEditor::FAircraftAssetComponentBroker>();
	FComponentAssetBrokerage::RegisterBroker(
		AircraftAssetComponentBroker,
		UAircraftComponent::StaticClass(),
		true,
		true);

	UThumbnailManager::Get().RegisterCustomRenderer(UAircraftAsset::StaticClass(), UAircraftAssetThumbnailRenderer::StaticClass());
}

void FAircraftAssetEditorModule::ShutdownModule()
{
	FAircraftAssetEditorCommands::Unregister();

	if (UObjectInitialized() && AircraftAssetComponentBroker.IsValid())
	{
		FComponentAssetBrokerage::UnregisterBroker(AircraftAssetComponentBroker);
	}

	AircraftAssetComponentBroker.Reset();
	ActiveAircraftAsset.Reset();

	FBaseCharacterFXEditorModule::ShutdownModule();
}
