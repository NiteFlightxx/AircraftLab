// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/ClothDataflowSimulationVisualization.h
//
// 引擎 FDataflowEditorToolkit Simulation 视口的多旋翼仿真可视化：
//   * ExtendSimulationVisualizationMenu —— 向 Simulation 视口菜单注入调试绘制开关
//     （CenterOfMass / Rotors / ThrustVectors / Torque / Velocity 五项；
//     逻辑继承自已删除的自制编辑器 SAir 系列面板）；
//   * GetDisplayString —— Simulation 视口左上角状态文本
//     （SDataflowSimulationViewport::GetDisplayString 聚合所有注册的可视化文本）；
//   * Draw / DrawCanvas —— 调试图元由 UAircraftComponent 自身在世界 Tick 中经
//     DrawDebug 系列接口绘制（DrawSimulationDebug），这里不重复画。
//
// 组件获取：SimulationScene->GetPreviewActor()->GetComponentByClass<UAircraftComponent>()
// （与 ClothDataflowSimulationVisualization::GetClothComponent 同一路径）。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowSimulationVisualization.h"

class UAircraftComponent;

class FAircraftDataflowSimulationVisualization : public UE::Dataflow::IDataflowSimulationVisualization
{
public:
	static const FName Name;

private:
	//~ IDataflowSimulationVisualization
	virtual FName GetName() const override;
	virtual void ExtendSimulationVisualizationMenu(const TSharedPtr<FDataflowSimulationViewportClient>& ViewportClient, FMenuBuilder& MenuBuilder) override;
	virtual FText GetDisplayString(const FDataflowSimulationScene* SimulationScene) const override;

	static UAircraftComponent* GetAircraftComponent(const FDataflowSimulationScene* SimulationScene);
};
