// 对齐 ChaosCloth 的 FChaosClothAssetEditorStyle —— 注册自家 SlateStyle，让 FUICommandInfo
// 的 GetIcon() 能查到带图标的 brush。否则 FToolMenuEntry::SetShowInToolbarTopLevel(true) 时按钮
// 会因为图标 brush 为空而完全不渲染（这就是"按钮不显示"的根因）。
//
// 我们暂时复用 FAppStyle 中现成的图标 brush（Refresh/Reset/Pause），而不像 ChaosCloth 那样使用
// 自带的 SVG 文件。等以后有自定义 SVG 资源时只需替换 Set() 的 brush 实例。

#pragma once

#include "Styling/SlateStyle.h"

class FAircraftAssetEditorStyle final : public FSlateStyleSet
{
public:
	static FAircraftAssetEditorStyle& Get();
	static const FName& GetStyleName();

private:
	FAircraftAssetEditorStyle();
	~FAircraftAssetEditorStyle();

	static const FName StyleName;
};
