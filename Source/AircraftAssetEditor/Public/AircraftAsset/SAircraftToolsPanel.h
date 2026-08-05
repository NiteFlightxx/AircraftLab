// 侧边工具面板：仿布料侧边 "General"/"Cloth" 分组样式。
// 当前阶段只保留空面板框架（不含具体工具内容），供后续工具扩展填充。

#pragma once

#include "Widgets/SCompoundWidget.h"

class SAircraftToolsPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAircraftToolsPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	TSharedRef<SWidget> BuildGroup(const FText& GroupName);
};
