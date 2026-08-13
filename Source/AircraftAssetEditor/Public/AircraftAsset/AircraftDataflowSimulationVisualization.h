//
// 引擎 FDataflowEditorToolkit Simulation 视口的多旋翼仿真可视化：
//   * GetDisplayString —— Simulation 视口左上角状态文本
//     （SDataflowSimulationViewport::GetDisplayString 聚合所有注册的可视化文本）；
//   * ExtendSimulationVisualizationMenu —— 与 ChaosCloth 相同的逐项可视化开关；
//   * Draw —— 通过独立 FAircraftVisualization 在 Dataflow Simulation 视口绘制。
//
// 组件获取：SimulationScene->GetPreviewActor()->GetComponentByClass<UAircraftComponent>()
// （与 ClothDataflowSimulationVisualization::GetClothComponent 同一路径）。

#pragma once

#include "CoreMinimal.h"
#include "AircraftAsset/AircraftVisualization.h"
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
	virtual void Draw(const FDataflowSimulationScene* SimulationScene, FPrimitiveDrawInterface* PDI) override;
	virtual FText GetDisplayString(const FDataflowSimulationScene* SimulationScene) const override;

	static UAircraftComponent* GetAircraftComponent(const FDataflowSimulationScene* SimulationScene);
	FAircraftVisualizationFlags Flags;
};
