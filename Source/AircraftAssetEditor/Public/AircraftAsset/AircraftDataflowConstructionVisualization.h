// 对齐 ChaosClothAssetEditor/Private/ChaosClothAsset/ClothDataflowConstructionVisualization.h
//
// 引擎 FDataflowEditorToolkit Construction 视口的多旋翼可视化挂载点。
//
// 当前为空实现：Construction 视口的旋翼渲染已由 AircraftAssetDataflowNodes 模块的
// FAircraftRotorRenderCallbacks（UE::Dataflow::FRenderingFactory 回调）覆盖 ——
// 旋翼圆盘 / 推力轴箭头以真实几何写入渲染门面，无需再走 PDI 补充绘制。
// 保留此注册点是为了保留扩展位置（如后续加"重心标记 / LOD 分区"等视口开关）。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowConstructionVisualization.h"

class FAircraftDataflowConstructionVisualization : public UE::Dataflow::IDataflowConstructionVisualization
{
public:
	static const FName Name;

private:
	//~ IDataflowConstructionVisualization
	virtual FName GetName() const override;
};
