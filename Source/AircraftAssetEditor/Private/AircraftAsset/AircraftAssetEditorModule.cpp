#include "AircraftAsset/AircraftAssetEditorModule.h"

#include "ThumbnailRendering/ThumbnailManager.h"
#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftDataflowSimulationVisualization.h"
#include "AircraftAsset/AircraftAssetThumbnailRenderer.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftDataflowAssetEditorUtils.h"
#include "AircraftAsset/AircraftDataflowTemplateProvider.h"
#include "Dataflow/AssetDefinition_DataflowAsset.h"
#include "Dataflow/DataflowSimulationVisualization.h"
#include "Features/IModularFeatures.h"

IMPLEMENT_MODULE(FAircraftAssetEditorModule, AircraftAssetEditor)

FAircraftAssetEditorModule::~FAircraftAssetEditorModule() = default;

namespace UE::AircraftDataflowEditor
{
	/**
	 * 工厂在 AircraftAssetTools（低层），模板生成在本模块（高层），经特性注册解耦。
	 */
	struct FAircraftDataflowTemplateProvider
		: public UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider
	{
		virtual void SetupAircraftDataflowTemplate(UAircraftAsset& InAsset) override
		{
			UE::AircraftDataflowAssetEditor::Private::EnsureAircraftDataflowAsset(&InAsset);
		}
	};

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

	AircraftAssetComponentBroker = MakeShared<UE::AircraftDataflowEditor::FAircraftAssetComponentBroker>();
	FComponentAssetBrokerage::RegisterBroker(
		AircraftAssetComponentBroker,
		UAircraftComponent::StaticClass(),
		true,
		true);

	UThumbnailManager::Get().RegisterCustomRenderer(UAircraftAsset::StaticClass(), UAircraftAssetThumbnailRenderer::StaticClass());

	// 注册程序化模板图提供者（供 AircraftAssetTools 的工厂经模块化特性调用）
	TemplateProvider = MakeUnique<UE::AircraftDataflowEditor::FAircraftDataflowTemplateProvider>();
	IModularFeatures::Get().RegisterModularFeature(
		UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider::GetFeatureName(),
		TemplateProvider.Get());

	//   - Construction 视口：旋翼几何由 AircraftAssetDataflowNodes 的 RenderingFactory 回调负责；
	//   - Simulation 视口：调试绘制开关菜单 + 视口左上角状态文本。
	UE::Dataflow::FDataflowSimulationVisualizationRegistry::GetInstance()
		.RegisterVisualization(MakeUnique<FAircraftDataflowSimulationVisualization>());

	DataflowAssetMenusHandle = UE::DataflowAssetDefinitionHelpers::RegisterDataflowAssetMenus(
		UAircraftAsset::StaticClass());
}

void FAircraftAssetEditorModule::ShutdownModule()
{
	UE::Dataflow::FDataflowSimulationVisualizationRegistry::GetInstance()
		.DeregisterVisualization(FAircraftDataflowSimulationVisualization::Name);

	if (TemplateProvider.IsValid())
	{
		IModularFeatures::Get().UnregisterModularFeature(
			UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider::GetFeatureName(),
			TemplateProvider.Get());
		TemplateProvider.Reset();
	}

	if (DataflowAssetMenusHandle.IsValid())
	{
		UE::DataflowAssetDefinitionHelpers::UnregisterDataflowAssetMenus(DataflowAssetMenusHandle);
		DataflowAssetMenusHandle.Reset();
	}

	if (UObjectInitialized() && AircraftAssetComponentBroker.IsValid())
	{
		FComponentAssetBrokerage::UnregisterBroker(AircraftAssetComponentBroker);
	}

	AircraftAssetComponentBroker.Reset();
	ActiveAircraftAsset.Reset();

	FBaseCharacterFXEditorModule::ShutdownModule();
}
