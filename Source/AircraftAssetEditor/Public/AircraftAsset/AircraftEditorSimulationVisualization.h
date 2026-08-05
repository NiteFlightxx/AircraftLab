// 对齐 ChaosClothAssetEditor/Public/ChaosClothAsset/ClothEditorSimulationVisualization.h
//
// 仿真可视化选项的持有者：向视口 Show 菜单注入调试绘制开关，
// 并为视口左上角状态文本（GetDisplayString）提供内容。

#pragma once

#include "CoreMinimal.h"

class FMenuBuilder;
class UAircraftComponent;
class FAircraftAssetEditorViewportClient;

class FAircraftEditorSimulationVisualization
{
public:
	/** 把调试绘制开关注入视口 Show 菜单。 */
	void ExtendViewportShowMenu(FMenuBuilder& MenuBuilder, const TSharedRef<FAircraftAssetEditorViewportClient>& ViewportClient);

	/** 视口左上角状态文本（当前启用选项对应的信息行）。 */
	FText GetDisplayString(const UAircraftComponent* AircraftComponent) const;
};
