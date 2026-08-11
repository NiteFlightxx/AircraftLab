//
// Aircraft 资产 Dataflow 视口的构造视图模式：3D 模拟视图（机架/旋翼布局预览）。

#pragma once

#include "Dataflow/DataflowRenderingViewMode.h"

namespace UE::AircraftLab::DataflowNodes
{
	class FAircraft3DSimViewMode : public UE::Dataflow::FDataflowConstruction3DViewModeBase
	{
	public:
		static const FName Name;
		virtual ~FAircraft3DSimViewMode() = default;
	private:
		virtual FName GetName() const override;
		virtual FText GetButtonText() const override;
		virtual FText GetTooltipText() const override;
		virtual bool CanDisplayGizmo() const { return true; }
	};
}
