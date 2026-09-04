//
// 职责：引擎 UDataflowEditor 打开 Aircraft 资产时，FDataflowSimulationScene 会 spawn 本类作为
// Simulation 视口的预览 Actor。引擎的挂接机制（见 UDataflowBaseContent::SetActorProperties 与
// UDataflowSkeletalContent::SetActorProperties，DataflowContent.cpp）通过反射查找本类上 *同名*
// 的 ObjectProperty 并写入：
//   * "DataflowAsset"  ← 数据流资产的 Owner（即 UAircraftAsset 实例本身，引擎要求属性类与值类严格相等）
//   * "SkeletalMesh"   ← 编辑器内容（UDataflowSkeletalContent）上的预览骨骼网格
//   * "AnimationAsset" ← 编辑器内容上的预览动画资产
// 三个属性在 FinishSpawning 之前被覆写，因此 OnConstruction 是唯一可靠的同步时机。
//
// 注意：布料用 BP 是因为需要美术在 BP 里挂额外道具；我们当前没有这种扩展诉求，
// C++ 直接提供即可（SpawnSimulatedActor 对非 BP 类跳过 InstancedPropertyBag 段，属正常路径）。

#pragma once

#include "CoreMinimal.h"
#include "AircraftAsset.h"
#include "AircraftComponent.h"
#include "AircraftRuntimeInterface/AircraftMovementIntentProvider.h"
#include "Dataflow/DataflowSimulationManager.h"
#include "GameFramework/Actor.h"

#include "AircraftDataflowPreviewActor.generated.h"


/**
 * Dataflow 编辑器 Simulation 视口的多旋翼预览 Actor。
 */
UCLASS()
class AIRCRAFTASSETENGINE_API AAircraftDataflowPreviewActor
	: public AActor
	, public IDataflowSimulationActor
	, public IAircraftMovementIntentProvider
{
	GENERATED_BODY()

public:
	AAircraftDataflowPreviewActor(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	UAircraftComponent* GetAircraftComponent() const { return AircraftComponent; }

	FVector GetPreviewHoldTargetCm() const { return PreviewHoldTargetCm; }
	float GetPreviewFixedYawDegrees() const { return PreviewFixedYawDegrees; }
	bool IsPreviewPlaybackEnabled() const { return bPreviewPlaybackEnabled; }
	bool HasPreviewMeshMismatch() const { return bPreviewMeshMismatch; }
	void RefreshPreviewAssetState();
	void ApplyPreviewHoldTarget(const FVector& PositionCm, float FixedYawDegrees);
	void ApplyPreviewSimulationLOD(int32 LodIndex);
	void SetPreviewArmed(bool bArmed);
	void SetPreviewFlightMode(EAircraftFlightMode FlightMode);
	bool SetPreviewRotorEffectiveness(FName RotorName, float Effectiveness);
	void ApplyPreviewForceAndTorque(const FVector& ForceWorldN, const FVector& TorqueWorldNm);

	//~ Begin AActor Interface
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void Tick(float DeltaSeconds) override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	//~ End AActor Interface

	//~ Begin IDataflowSimulationActor Interface
	virtual void OnDataflowSimulationEnabledChanged_Implementation(bool bSimulationEnabled) override;
	//~ End IDataflowSimulationActor Interface

	//~ Begin IAircraftMovementIntentProvider Interface
	virtual bool GetAircraftMovementIntent(FAircraftMovementIntent& OutIntent,
		FAircraftMovementIntentHandle& OutHandle, uint64& OutRevision) const override;
	virtual bool IsAircraftMovementIntentActive() const override;
	virtual void OnAircraftMovementIntentInterrupted(
		FAircraftMovementIntentHandle Handle,
		EAircraftMovementFailureReason Reason) override;
	//~ End IAircraftMovementIntentProvider Interface

private:
	/** 把引擎注入的三个属性同步到 UAircraftComponent。 */
	void SyncComponentFromInjectedProperties();
	void InitializeDefaultScenarioIfReady();
	void FreezePreviewSimulation();
	void ResumePreviewSimulation();
	FName GetChassisBoneName() const;

	/**
	 * 引擎注入点（UDataflowBaseContent::SetActorProperties）：
	 * 属性名必须是 "DataflowAsset"，属性类必须与 DataflowOwner 的 *实际类* 严格相等 → 声明为 UAircraftAsset。
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Dataflow", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UAircraftAsset> DataflowAsset;

	/** 引擎注入点（UDataflowSkeletalContent::SetActorProperties）：属性名必须是 "SkeletalMesh"。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Dataflow", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USkeletalMesh> SkeletalMesh;

	/** 引擎注入点（UDataflowSkeletalContent::SetActorProperties）：属性名必须是 "AnimationAsset"。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Dataflow", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UAnimationAsset> AnimationAsset;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UAircraftComponent> AircraftComponent;

	FVector PreviewHoldTargetCm = FVector(0.0, 0.0, 200.0);
	float PreviewFixedYawDegrees = 0.0f;
	FAircraftMovementIntent PreviewMovementIntent;
	FAircraftMovementIntentHandle PreviewMovementIntentHandle;
	uint64 PreviewMovementIntentRevision = 1;
	bool bPreviewIntentActive = false;
	bool bDefaultScenarioInitialized = false;
	bool bPreviewPlaybackEnabled = true;
	bool bPreviewFrozen = false;
	bool bPreviewMeshMismatch = false;
	FAircraftPhysicsStateSnapshot FrozenPhysicsState;
};
