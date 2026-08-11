//
// 与 ChaosCloth 的刻意分歧：ChaosCloth 用 DF_*.uasset 二进制模板 +
// SetDataflowFromTemplatePicker；Aircraft 无随包二进制模板资产，模板图由
// IAircraftDataflowTemplateProvider 模块化特性（实现注册于 AircraftAssetEditor）
// 程序化生成。无提供者（如自动化测试/命令行）时退化为空 Dataflow，

#include "AircraftAsset/AircraftAssetFactory.h"

#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftDataflowTemplateProvider.h"
#include "Dataflow/DataflowObject.h"
#include "Features/IModularFeatures.h"
#include "Misc/AssertionMacros.h"

UAircraftAssetFactory::UAircraftAssetFactory(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bEditorImport = true;
	bEditAfterNew = true;
	SupportedClass = UAircraftAsset::StaticClass();
}

UObject* UAircraftAssetFactory::FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	UAircraftAsset* const AircraftAsset = NewObject<UAircraftAsset>(
		InParent, InClass, InName, Flags | RF_Transactional | RF_Public | RF_Standalone);
	if (!AircraftAsset)
	{
		return nullptr;
	}

	// 经模块化特性取模板图提供者（AircraftAssetEditor 注册）。
	UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider* TemplateProvider = nullptr;
	if (IModularFeatures::Get().IsModularFeatureAvailable(
		UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider::GetFeatureName()))
	{
		TemplateProvider = &IModularFeatures::Get().GetModularFeature<
			UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider>(
				UE::AircraftLab::AircraftAsset::IAircraftDataflowTemplateProvider::GetFeatureName());
	}

	if (TemplateProvider)
	{
		TemplateProvider->SetupAircraftDataflowTemplate(*AircraftAsset);
	}
	else
	{
		// 回退（自动化测试/无编辑器环境）：挂一个空 Dataflow，保证资产可序列化可编辑。
		if (AircraftAsset->GetDataflow() == nullptr)
		{
			UDataflow* const DataflowContent = NewObject<UDataflow>(
				AircraftAsset, TEXT("Dataflow"), RF_Transactional);
			AircraftAsset->SetDataflow(DataflowContent);
		}
	}

	AircraftAsset->MarkPackageDirty();
	return AircraftAsset;
}

FString UAircraftAssetFactory::GetDefaultNewAssetName() const
{
	return TEXT("VA_NewAircraftAsset");
}
