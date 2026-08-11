// （IClothingAssetFactoryProvider / IClothAssetSkeletalMeshConverterClassProvider 同位）。
//
// 低层模块（AircraftAssetTools）持有工厂，高层编辑器模块（AircraftAssetEditor）
// 注册本特性的实现来提供"程序化模板图"创建，打破向上的模块依赖。
// 与 ChaosCloth 的二进制模板资产（DF_*.uasset）不同，Aircraft 模板图由
// AircraftDataflowAssetEditorUtils 程序化生成（见该文件头注释）。

#pragma once

#include "CoreMinimal.h"
#include "Features/IModularFeature.h"

class UAircraftAsset;

namespace UE::AircraftLab::AircraftAsset
{
	/** 为新 Aircraft 资产创建并填充 Dataflow 模板图。 */
	class IAircraftDataflowTemplateProvider : public IModularFeature
	{
	public:
		virtual ~IAircraftDataflowTemplateProvider() = default;

		static FName GetFeatureName()
		{
			static const FName FeatureName = TEXT("AircraftDataflowTemplateProvider");
			return FeatureName;
		}

		/** 为新资产装配 Dataflow 图（实现方创建模板节点链并接好 Terminal）。 */
		virtual void SetupAircraftDataflowTemplate(UAircraftAsset& InAsset) = 0;
	};
}
