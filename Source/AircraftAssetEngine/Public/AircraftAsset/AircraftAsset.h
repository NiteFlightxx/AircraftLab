// 对齐 ChaosClothAssetEngine/Public/ChaosClothAsset/ClothAsset.h
//
// 职责：UAircraftAsset 是多旋翼资产的具体实现，承载骨骼网格 + 物理资产 + 多个
// FManagedArrayCollection（描述机架/电机/桨/电池/PID/手感等），并在 Build() 时编译成
// FAircraftSimulationModel。

#pragma once

#include "CoreMinimal.h"

#include "AircraftAsset/AircraftAssetBase.h"

#include "AircraftAsset.generated.h"

/**
 * 解锁状态
 *
 * 与 PX4/Betaflight 等真实飞控的 ARM 状态机对齐：Disarmed → Arming → Armed；
 * Failsafe / EmergencyStop 是从任意状态可被触发的安全分支。
 */
UENUM(BlueprintType)
enum class EDroneArmState : uint8
{
	/** 上锁待机：电机不转动，不响应任何油门/姿态指令。 */
	Disarmed UMETA(DisplayName = "Disarmed"),

	/** 解锁过渡：完成自检后进入 Armed，否则退回 Disarmed。 */
	Arming UMETA(DisplayName = "Arming"),

	/** 已解锁：飞行器可以起飞。 */
	Armed UMETA(DisplayName = "Armed"),

	/** 故障保护：自动切换为返航/降落。 */
	Failsafe UMETA(DisplayName = "Failsafe"),

	/** 紧急停止：立即切断电机动力。 */
	EmergencyStop UMETA(DisplayName = "Emergency Stop")
};

/**
 * 飞行模式
 *
 * 与串级 PID 的逐级"放权"一致：
 *   Manual/Acro 直接驱动角速率；
 *   Angle 走 角度→角速率 两环；
 *   AltitudeHold/PositionHold/VelocityHold 走 位置→速度→姿态→角速率 四环；
 *   Mission/ReturnToHome/AutoLand 由上层航线规划注入位置/姿态目标后走全四环。
 */
UENUM(BlueprintType)
enum class EDroneFlightMode : uint8
{
	Manual UMETA(DisplayName = "Manual"),
	Acro UMETA(DisplayName = "Acro"),
	Angle UMETA(DisplayName = "Angle"),
	AltitudeHold UMETA(DisplayName = "Altitude Hold"),
	PositionHold UMETA(DisplayName = "Position Hold"),
	VelocityHold UMETA(DisplayName = "Velocity Hold"),
	Mission UMETA(DisplayName = "Mission"),
	ReturnToHome UMETA(DisplayName = "Return To Home"),
	AutoLand UMETA(DisplayName = "Auto Land")
};

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
