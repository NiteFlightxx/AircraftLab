//
// 编辑器选项（UDeveloperSettings 持久化 + CVar 双向绑定）：
// 控制 Aircraft 资产双击时默认打开 Dataflow 资产编辑器还是简易属性编辑器。

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"

#include "AircraftEditorOptions.generated.h"

AIRCRAFTEDITOR_API TAutoConsoleVariable<bool>& GetAircraftAssetsOpenInDataflowEditorCVar();

UCLASS(config = EditorPerProjectUserSettings)
class AIRCRAFTEDITOR_API UAircraftEditorOptions : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UAircraftEditorOptions();

	/** 双击资产时用 Dataflow 编辑器打开（false 则回退简易资产编辑器）。 */
	UPROPERTY(EditAnywhere, config, Category = "Aircraft")
	bool bAircraftAssetsOpenInDataflowEditor = true;

	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostInitProperties() override;

private:
	void UpdateCVar() const;
};
