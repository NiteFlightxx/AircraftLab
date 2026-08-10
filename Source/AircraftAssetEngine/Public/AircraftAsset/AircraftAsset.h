// 对齐 ChaosClothAssetEngine/Public/ChaosClothAsset/ClothAsset.h
//
// 职责：UAircraftAsset 是多旋翼资产的具体实现，承载骨骼网格 + 物理资产 + 多个
// FManagedArrayCollection（描述机架/电机/桨/PID/手感等），并在 Build() 时编译成
// FAircraftSimulationModel。

#pragma once

#include "CoreMinimal.h"

#include "AircraftAsset/AircraftAssetBase.h"

// EAircraftArmState / EAircraftFlightMode / EAircraftAttitudeMode 已下沉到求解器模块
// （Aircraft/Public/Aircraft/FlightControlStateTypes.h，对齐 UChaosClothConfig 归属 ChaosCloth 的做法），
// 名称不变，此处 include 复用。
#include "Aircraft/FlightControlStateTypes.h"

#include "AircraftAsset.generated.h"

class USkeleton;
class UPhysicsAsset;
class USkeletalMesh;
class FSkeletalMeshModel;
class FSkinnedAssetPostLoadContext;
struct FAircraftSimulationModel;

/**
 * 多旋翼资产
 *
 * 与 ChaosCloth 中的 UChaosClothAsset 一一对应：内部维护一组 FManagedArrayCollection（资产数据
 * 的 schema），通过 FAircraftCollection / Facade 写入；Build() 把它们编译成运行时 SimulationModel。
 */
UCLASS(hidecategories = Object, BlueprintType, PrioritizeCategories = ("Dataflow"))
class AIRCRAFTASSETENGINE_API UAircraftAsset : public UAircraftAssetBase
{
	GENERATED_BODY()

public:
	UAircraftAsset(const FObjectInitializer& ObjectInitializer);
	UAircraftAsset(FVTableHelper& Helper);
	virtual ~UAircraftAsset() override;

	//~ Begin UObject interface
	virtual void Serialize(FArchive& Ar) override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	//~ End UObject interface

	//~ Begin USkinnedAsset interface
	virtual UPhysicsAsset* GetPhysicsAsset() const override { return PhysicsAsset; }
	virtual USkeleton* GetSkeleton() override { return Skeleton; }
	virtual const USkeleton* GetSkeleton() const override { return Skeleton; }
	virtual void SetSkeleton(USkeleton* InSkeleton) override { Skeleton = InSkeleton; }

#if WITH_EDITOR
	virtual FString BuildDerivedDataKey(const ITargetPlatform* TargetPlatform) override;
	virtual bool IsInitialBuildDone() const override;
#endif

#if WITH_EDITORONLY_DATA
	virtual FSkeletalMeshModel* GetImportedModel() const override;
#endif
	//~ End USkinnedAsset interface

	/**
	 * 由 Dataflow Terminal 节点驱动：把 Collection 编译为运行时 SimulationModel。
	 *
	 * 与 UChaosClothAsset::Build() 一致——Collection 是"authoring 数据"，SimulationModel
	 * 是"运行时只读快照"。重建后会通知所有挂在该资产的 UAircraftComponent 重新初始化。
	 */
	void Build(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
		FText* ErrorText = nullptr,
		FText* VerboseText = nullptr);

	//~ Begin UAircraftAssetBase interface
	virtual bool HasValidAircraftSimulationModels() const override;
	virtual int32 GetNumAircraftSimulationModels() const override
	{
		return AircraftSimulationModel.IsValid() ? 1 : 0;
	}

	virtual TSharedPtr<const FAircraftSimulationModel> GetAircraftSimulationModel(int32 /*ModelIndex*/) const override
	{
		return AircraftSimulationModel;
	}

	virtual void SetCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InCollections) override;
	virtual const TArray<TSharedRef<const FManagedArrayCollection>>& GetCollections(int32 /*ModelIndex*/) const override;
	//~ End UAircraftAssetBase interface

	const TArray<TSharedRef<const FManagedArrayCollection>>& GetAircraftCollections() const;
	void SetAircraftCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InAircraftCollections);

protected:
	virtual USkeletalMesh* GetSourceSkeletalMesh() const override;

private:
	virtual void BeginPostLoadInternal(FSkinnedAssetPostLoadContext& Context) override;

	TArray<TSharedRef<const FManagedArrayCollection>>& GetAircraftCollectionsInternal();
	void EnsureCollectionsInitialized();
	void SynchronizeAssetStateFromCollections();
	void BuildAircraftSimulationModel();

private:
	UPROPERTY(EditAnywhere, Setter = SetSkeleton, Category = Skeleton)
	TObjectPtr<USkeleton> Skeleton;

	UPROPERTY(EditAnywhere, Category = Collision)
	TObjectPtr<UPhysicsAsset> PhysicsAsset;

	TArray<TSharedRef<const FManagedArrayCollection>> AircraftCollections;
	TSharedPtr<FAircraftSimulationModel> AircraftSimulationModel;
};
