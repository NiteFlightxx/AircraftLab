#pragma once

#include "AdvancedPreviewScene.h"

class AActor;
class UAircraftAssetBase;
class UAircraftComponent;
class UAircraftPreviewSceneDescription;
class UAnimSingleNodeInstance;

class FAircraftAssetEditorPreviewScene : public FAdvancedPreviewScene
{
public:
	explicit FAircraftAssetEditorPreviewScene(FPreviewScene::ConstructionValues ConstructionValues);
	virtual ~FAircraftAssetEditorPreviewScene() override;

	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;
	virtual void Tick(float DeltaT) override;

	void SetAircraftAsset(UAircraftAssetBase* InAircraftAsset);

	/** 响应预览场景描述对象（网格/动画/选项）的属性变化。 */
	void SceneDescriptionPropertyChanged(const FName& PropertyName);

	void SoftResetSimulation();
	void HardResetSimulation();
	void SuspendSimulation();
	void ResumeSimulation();
	bool IsSimulationSuspended() const;
	void SetEnableSimulation(bool bEnable);
	bool IsSimulationEnabled() const;

	UAircraftComponent* GetAircraftComponent() const;
	UAircraftPreviewSceneDescription* GetPreviewSceneDescription() const { return PreviewSceneDescription; }

	/** 预览动画实例（动画直接播放在 AircraftComponent 上；无动画时返回 nullptr）。 */
	UAnimSingleNodeInstance* GetPreviewAnimInstance();
	const UAnimSingleNodeInstance* GetPreviewAnimInstance() const;

private:
	void InitializePreviewAircraft();
	void UpdateInitialAircraftTransform();

	/** 根据描述对象刷新预览动画实例。 */
	void UpdateSkeletalMeshAnimation();

	/** 网格重载/重导入前后的动画状态快照。 */
	void SaveAnimationState();
	void RestoreSavedAnimationState();
	void HandlePackageReloaded(const EPackageReloadPhase InPackageReloadPhase, FPackageReloadedEvent* InPackageReloadedEvent);
	void HandleReimportManagerPostReimport(UObject* ReimportedObject, bool bWasSuccessful);

	TObjectPtr<AActor> SceneActor;
	TObjectPtr<UAircraftComponent> AircraftComponent;
	TObjectPtr<UAircraftPreviewSceneDescription> PreviewSceneDescription;
	FTransform InitialAircraftTransform = FTransform::Identity;

	struct FAnimState
	{
		float Time;
		bool bIsReverse;
		bool bIsLooping;
		bool bIsPlaying;
	};
	TOptional<FAnimState> SavedAnimState;

	FDelegateHandle OnPackageReloadedDelegateHandle;
	FDelegateHandle OnPostReimportDelegateHandle;
};
