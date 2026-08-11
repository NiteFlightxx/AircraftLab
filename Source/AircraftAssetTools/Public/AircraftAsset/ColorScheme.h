//
// Aircraft 资产的统一配色：资产缩略图/类别色 + Dataflow 节点画布色。
// 节点注册（AircraftAssetDataflowNodes）与 AssetDefinition（AircraftAssetEditor）
// 统一取用本结构，避免多处硬编码漂移。

#pragma once

#include "CoreMinimal.h"

namespace UE::AircraftLab::AircraftAsset
{
	struct FColorScheme
	{
		/** 资产色（内容浏览器底纹/类别）：航空蓝。 */
		static constexpr FColor Asset = FColor(0, 166, 255);

		/** Dataflow 节点头/体色（"Aircraft" 类别）。 */
		static constexpr FColor NodeHeader = FColor(0, 166, 255);
		static constexpr FColor NodeBody = FColor(0, 0, 0, 115);

		static constexpr FColor TerminalNodeHeader = FColor(166, 41, 31);
		static constexpr FColor TerminalNodeBody = FColor(0, 0, 0, 115);
	};
}
