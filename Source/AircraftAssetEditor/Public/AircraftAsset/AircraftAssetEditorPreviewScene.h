#pragma once

#include "AdvancedPreviewScene.h"

class AActor;
class UAircraftAssetBase;
class UAircraftComponent;

class FAircraftAssetEditorPreviewScene : public FAdvancedPreviewScene
{
public:
	explicit FAircraftAssetEditorPreviewScene(FPreviewScene::ConstructionValues ConstructionValues);
	virtual ~FAircraftAssetEditorPreviewScene() override;

	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;

	void SetAircraftAsset(UAircraftAssetBase* InAircraftAsset);
	
	/*
	void SoftResetSimulation();
	void HardResetSimulation();
	void SuspendSimulation();
	void ResumeSimulation();
	bool IsSimulationSuspended() const;
	void SetEnableSimulation(bool bEnable);
	bool IsSimulationEnabled() const;
*/
	UAircraftComponent* GetAircraftComponent() const;

	//TODO:动画相关接口
private:
	void InitializePreviewAircraft();
	void UpdateInitialAircraftTransform();

	TObjectPtr<AActor> SceneActor;
	TObjectPtr<UAircraftComponent> AircraftComponent;
	FTransform InitialAircraftTransform = FTransform::Identity;
};
