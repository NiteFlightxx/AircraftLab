// 工具上下文解析（替代已删除的 UAircraftEditorContextObject）。
//
// 在引擎 UDataflowEditorMode 中，工具上下文里注入的 UDataflowContextObject 就是
// UDataflowBaseContent（= 编辑器内容），其 GetDataflowOwner() 即当前编辑的 Aircraft 资产
// （见 UDataflowEditorMode::InitializeContextObject：ContextObject = EditorContent）。
//
// Simulation 视口预览组件不在 ToolsContext 里，按 UAircraftAssetBase::GetDependentComponents
// 的同款思路经 TObjectIterator 在编辑器进程内反查。

#pragma once

#include "CoreMinimal.h"

class UAircraftAssetBase;
class UAircraftComponent;
class UInteractiveToolManager;

namespace UE::AircraftLab::AircraftEditorTools
{
	/** 从 ToolManager 的 ContextObjectStore 解析当前编辑的 Aircraft 资产；无则 nullptr。 */
	UAircraftAssetBase* ResolveAircraftAsset(UInteractiveToolManager* ToolManager);

	/** 找到当前编辑器进程里绑定该资产的全部 UAircraftComponent（含 Simulation 视口预览组件）。 */
	TArray<UAircraftComponent*> FindAircraftComponents(const UAircraftAssetBase* Asset);

	/** 写资产后刷新所有依赖组件（等价旧 ContextObject->GetAircraftComponent()->RefreshAssetState 的广播版）。 */
	void RefreshDependentComponents(const UAircraftAssetBase* Asset);
}
