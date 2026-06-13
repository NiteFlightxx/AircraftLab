// 对齐 ChaosClothAssetEngine/Public/ChaosClothAsset/ClothAssetBase.h
//
// 职责：UAircraftAssetBase 是 USkinnedAsset 的抽象派生，承载与 Dataflow 的对接，
// 不直接持有具体的多旋翼运行时数据。具体资产（UAircraftAsset）由 UAircraftComponent 引用。

#pragma once

#include "CoreMinimal.h"
#include "Dataflow/DataflowContent.h"
#include "Dataflow/DataflowInstance.h"
#include "Engine/SkinnedAsset.h"
#include "GeometryCollection/ManagedArrayCollection.h"

#include "AircraftAssetBase.generated.h"

class AActor;
class UActorComponent;
class UAnimationAsset;
class UDataflow;
class UPhysicsAsset;
class USkeletalMesh;
struct FPropertyChangedEvent;
struct FAircraftSimulationModel;

/**
 * 多旋翼资产基类
 *
 * 与 ChaosCloth 中的 UChaosClothAssetBase 完全对齐：作为 USkinnedAsset 的抽象派生，仅声明
 * "聚合多个 Collection / 暴露 Dataflow / 提供运行时静态模型"等接口。具体的几何/编辑器数据
 * 在 UAircraftAsset 中实现。
 */
UCLASS(Abstract, BlueprintType)
class AIRCRAFTASSETENGINE_API UAircraftAssetBase
	: public USkinnedAsset
	, public IDataflowContentOwner
	, public IDataflowInstanceInterface
{
	GENERATED_BODY()

public:
	UAircraftAssetBase(const FObjectInitializer& ObjectInitializer);

	//~ Begin UAircraftAssetBase interface
	virtual bool HasValidAircraftSimulationModels() const
	PURE_VIRTUAL(UAircraftAssetBase::HasValidAircraftSimulationModels, return false;);

	virtual int32 GetNumAircraftSimulationModels() const
	PURE_VIRTUAL(UAircraftAssetBase::GetNumAircraftSimulationModels, return 0;);

	virtual TSharedPtr<const FAircraftSimulationModel> GetAircraftSimulationModel(int32 ModelIndex) const
	PURE_VIRTUAL(UAircraftAssetBase::GetAircraftSimulationModel, return nullptr;);

	virtual void SetCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InCollections)
	PURE_VIRTUAL(UAircraftAssetBase::SetCollections, );

	virtual const TArray<TSharedRef<const FManagedArrayCollection>>& GetCollections(int32 ModelIndex) const
	PURE_VIRTUAL(UAircraftAssetBase::GetCollections, static const TArray<TSharedRef<const FManagedArrayCollection>> EmptyArray; return EmptyArray;);
	//~ End UAircraftAssetBase interface

	//~ Begin IDataflowContentOwner interface
	virtual TObjectPtr<UDataflowBaseContent> CreateDataflowContent() override;
	virtual void WriteDataflowContent(const TObjectPtr<UDataflowBaseContent>& DataflowContent) const override;
	virtual void ReadDataflowContent(const TObjectPtr<UDataflowBaseContent>& DataflowContent) override;
	//~ End IDataflowContentOwner interface

	//~ Begin IDataflowInstanceInterface interface
	virtual const FDataflowInstance& GetDataflowInstance() const override;
	virtual FDataflowInstance& GetDataflowInstance() override;
	//~ End IDataflowInstanceInterface interface

	//~ Begin UObject interface
	virtual void PostLoad() override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	//~ End UObject interface

	void SetDataflow(UDataflow* InDataflow);
	UDataflow* GetDataflow();
	const UDataflow* GetDataflow() const;

	//~ Begin USkinnedAsset interface
	virtual FReferenceSkeleton& GetRefSkeleton() override;
	virtual const FReferenceSkeleton& GetRefSkeleton() const override;
	virtual FSkeletalMeshLODInfo* GetLODInfo(int32 Index) override;
	virtual const FSkeletalMeshLODInfo* GetLODInfo(int32 Index) const override;
	virtual UPhysicsAsset* GetShadowPhysicsAsset() const override;
	virtual FMatrix GetComposedRefPoseMatrix(FName BoneName) const override;
	virtual FMatrix GetComposedRefPoseMatrix(int32 BoneIndex) const override;
	virtual const FMeshUVChannelInfo* GetUVChannelData(int32 MaterialIndex) const override;
	virtual bool GetSupportRayTracing() const override;
	virtual int32 GetRayTracingMinLOD() const override;
	virtual TArray<FMatrix44f>& GetRefBasesInvMatrix() override;
	virtual const TArray<FMatrix44f>& GetRefBasesInvMatrix() const override;
	virtual TArray<FSkeletalMeshLODInfo>& GetLODInfoArray() override;
	virtual const TArray<FSkeletalMeshLODInfo>& GetLODInfoArray() const override;
	virtual FSkeletalMeshRenderData* GetResourceForRendering() const override;
	virtual int32 GetDefaultMinLod() const override;
	virtual const FPerPlatformInt& GetMinLod() const override;
	virtual TArray<FSkeletalMaterial>& GetMaterials() override;
	virtual const TArray<FSkeletalMaterial>& GetMaterials() const override;
	virtual int32 GetLODNum() const override;
	virtual bool IsMaterialUsed(int32 MaterialIndex) const override;
	virtual FBoxSphereBounds GetBounds() const override;
	virtual TArray<class USkeletalMeshSocket*> GetActiveSocketList() const override;
	virtual USkeletalMeshSocket* FindSocket(FName InSocketName) const override;
	virtual USkeletalMeshSocket* FindSocketInfo(FName InSocketName, FTransform& OutTransform, int32& OutBoneIndex, int32& OutIndex) const override;
	virtual UMeshDeformer* GetDefaultMeshDeformer() const override;
	virtual UMeshDeformerCollection* GetTargetMeshDeformers() const override;
	virtual bool HasHalfEdgeBuffer(int32 LODIndex) const override;
	virtual UMaterialInterface* GetOverlayMaterial() const override;
	virtual float GetOverlayMaterialMaxDrawDistance() const override;
	virtual bool IsValidLODIndex(int32 Index) const override;
	virtual int32 GetMinLodIdx(bool bForceLowestLODIdx = false) const override;
	virtual bool NeedCPUData(int32 LODIndex) const override;
	virtual bool GetHasVertexColors() const override;
	virtual int32 GetPlatformMinLODIdx(const ITargetPlatform* TargetPlatform) const override;
	virtual const FPerPlatformBool& GetDisableBelowMinLodStripping() const override;

#if WITH_EDITOR
	virtual bool GetEnableLODStreaming(const ITargetPlatform* TargetPlatform) const override { return false; }
	virtual int32 GetMaxNumStreamedLODs(const ITargetPlatform* TargetPlatform) const override { return 0; }
	virtual int32 GetMaxNumOptionalLODs(const ITargetPlatform* TargetPlatform) const override { return 0; }
#endif
	//~ End USkinnedAsset interface

#if WITH_EDITORONLY_DATA
	void SetPreviewSceneSkeletalMesh(USkeletalMesh* Mesh);
	USkeletalMesh* GetPreviewSceneSkeletalMesh() const;

	void SetPreviewSceneAnimation(UAnimationAsset* Animation);
	UAnimationAsset* GetPreviewSceneAnimation() const;
#endif

protected:
	virtual USkeletalMesh* GetSourceSkeletalMesh() const
	PURE_VIRTUAL(UAircraftAssetBase::GetSourceSkeletalMesh, return nullptr;);

	void OnPropertyChanged() const;
	void OnAssetChanged(bool bReregisterComponents = true) const;
	TArray<UActorComponent*> GetDependentComponents() const;
	void UpdateSimulationActor(TObjectPtr<AActor>& SimulationActor) const;

	static FReferenceSkeleton CreateFallbackReferenceSkeleton();

	UPROPERTY(EditAnywhere, Category = "Dataflow")
	FDataflowInstance DataflowInstance;

#if WITH_EDITORONLY_DATA
	UPROPERTY(VisibleAnywhere, Category = Dataflow)
	bool bHasDataflowAsset = false;

	UPROPERTY(DuplicateTransient, AssetRegistrySearchable)
	TSoftObjectPtr<USkeletalMesh> PreviewSceneSkeletalMesh;

	UPROPERTY(DuplicateTransient, AssetRegistrySearchable)
	TSoftObjectPtr<UAnimationAsset> PreviewSceneAnimation;
#endif

private:
	FReferenceSkeleton FallbackRefSkeleton;
};
