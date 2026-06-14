#pragma once

#include "BaseCharacterFXEditorMode.h"
#include "AircraftEditorMode.generated.h"

class FToolTargetTypeRequirements;
class FAircraftAssetEditorPreviewScene;

UCLASS(Transient)
class UAircraftAssetEditorMode : public UBaseCharacterFXEditorMode
{
	GENERATED_BODY()

public:
	static const FEditorModeID EM_AircraftAssetEditorModeId;

	UAircraftAssetEditorMode();

	void SetPreviewScene(FAircraftAssetEditorPreviewScene* InPreviewScene);
	void SoftResetSimulation();
	void HardResetSimulation();
	void SuspendSimulation();
	void ResumeSimulation();
	bool IsSimulationSuspended() const;
	void SetEnableSimulation(bool bEnable);
	bool IsSimulationEnabled() const;

	virtual FBox SceneBoundingBox() const override;
	virtual void ModeTick(float DeltaTime) override;

protected:
	virtual void AddToolTargetFactories() override;
	virtual void RegisterTools() override;
	virtual void CreateToolTargets(const TArray<TObjectPtr<UObject>>& AssetsIn) override;
	virtual void CreateToolkit() override;

private:
	FAircraftAssetEditorPreviewScene* PreviewScene = nullptr;

	// 对齐 ChaosClothAssetEditorMode：Soft/Hard reset 不立刻 reset，而是写 flag，由 ModeTick
	// 在合适时机消费。HardReset 通过 FComponentReregisterContext 让组件整体重注册；SoftReset
	// 调用 Component->SoftResetSimulation()。
	bool bShouldResetSimulation = false;
	bool bHardReset = false;
};
