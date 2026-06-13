// 对齐 ChaosClothAssetEditorTools/.../ChaosClothEditorContextObject：
// InteractiveTool 与 Editor Mode / PreviewScene 之间的共享上下文对象。Tool 在 Build/Setup 时
// 通过 ToolManager.ContextObjectStore 拿到这个对象，从而读到当前 UAircraftComponent /
// UAircraftAssetBase。
//
// 注意：上下文只持有"已解析出的 UAircraftComponent"弱引用，不直接依赖 Editor 模块的
// FAircraftAssetEditorPreviewScene 类型；Editor 侧在 SetPreviewScene 时把已解析出来的
// Component 推入 Context（保持 EditorTools 模块与 Editor 模块的依赖单向、最小）。

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"

#include "AircraftEditorContextObject.generated.h"

class UAircraftAssetBase;
class UAircraftComponent;

/**
 * Aircraft Editor 中由 Mode 注入的上下文对象。
 */
UCLASS(Transient)
class AIRCRAFTASSETEDITORTOOLS_API UAircraftEditorContextObject : public UObject
{
	GENERATED_BODY()

public:
	/** 由 EditorMode 在 SetPreviewScene 时调用，把当前 Preview 中的 UAircraftComponent 推入。 */
	void SetAircraftComponent(UAircraftComponent* InAircraftComponent);

	UAircraftComponent* GetAircraftComponent() const;
	UAircraftAssetBase* GetAircraftAsset() const;

private:
	TWeakObjectPtr<UAircraftComponent> AircraftComponentWeak;
};
