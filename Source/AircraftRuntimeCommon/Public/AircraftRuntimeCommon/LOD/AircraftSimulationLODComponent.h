#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODComponent.generated.h"

class UPrimitiveComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnAircraftLODSelectionChanged,
	int32, PreviousLODIndex,
	int32, NewLODIndex);

/** Gameplay 网络配置。数组下标与资产 LOD 下标一一对应。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMECOMMON_API FAircraftSimulationLODNetworkSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation|Networking", meta = (
		DisplayName = "网络更新频率", ClampMin = "1.0", Units = "Hz"))
	float NetUpdateFrequency = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation|Networking", meta = (
		DisplayName = "启用网络休眠"))
	bool bEnableDormancy = false;
};

/**
 * 飞行模拟 LOD 的唯一运行时选择入口。
 *
 * Dataflow 资产只定义各 LOD 被选中后的驱动、碰撞与飞控配置；何时切换完全由游戏策略决定。
 * 服务器上的 AIController、Significance Manager 或其他 Gameplay 系统显式调用 SetSimulationLOD，
 * 组件负责把选择复制到客户端并原子应用完整模拟预算。
 */
UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTRUNTIMECOMMON_API UAircraftSimulationLODComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAircraftSimulationLODComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	int32 GetCurrentSimulationLOD() const { return CurrentLODIndex; }

	/** 由服务器或 Standalone 显式选择 LOD；索引无效时不改变当前状态。 */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	bool SetSimulationLOD(int32 NewLODIndex);

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Simulation")
	FOnAircraftLODSelectionChanged OnLODSelectionChanged;

	/** 客户端仅消费服务器复制的 LOD 与刚体状态，不在本地执行飞控或位置驱动。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation|Networking", meta = (
		DisplayName = "仅服务器执行权威模拟"))
	bool bAuthoritySimulationOnly = true;

	/** 模拟代理保留物理刚体，交由 UE 物理复制执行预测插值。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Aircraft|Simulation|Networking", meta = (
		DisplayName = "客户端代理启用物理复制"))
	bool bClientProxyUsesDefaultPhysicsReplication = true;

	/** 每个 LOD 的 Actor 复制频率与休眠策略；数组下标对应资产 LOD 下标。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|Simulation|Networking", meta = (
		DisplayName = "各 LOD 网络设置"))
	TArray<FAircraftSimulationLODNetworkSettings> NetworkSettingsPerLOD;

protected:
	/** 由服务器选择并复制；没有 Gameplay 选择时确定性使用 LOD0。 */
	UPROPERTY(ReplicatedUsing = OnRep_CurrentLODIndex)
	int32 CurrentLODIndex = 0;

private:
	friend class FAircraftCollisionBudgetStateTest;

	struct FCollisionComponentState
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		ECollisionEnabled::Type OriginalCollision = ECollisionEnabled::NoCollision;
	};

	bool bHasAppliedBudget = false;
	bool bNetworkProxyBudget = false;
	TWeakObjectPtr<class UAircraftComponent> AircraftComponent;
	TArray<TWeakObjectPtr<UActorComponent>> Consumers;
	TArray<FCollisionComponentState> CollisionComponents;
	TEnumAsByte<ENetDormancy> SavedNetDormancy = DORM_Awake;
	bool bHasSavedNetDormancy = false;

	UFUNCTION()
	void OnRep_CurrentLODIndex(int32 PreviousLODIndex);

	void RefreshConsumerCache();
	void RefreshCollisionComponents();
	void ApplyCollisionBudget(const FAircraftSimulationBudget& Budget);
	bool ApplyCurrentLOD(int32 PreviousLODIndex, bool bBroadcastChange);
	void ApplyCurrentNetworkSettings();
	FAircraftSimulationLODNetworkSettings GetNetworkSettings(int32 LODIndex) const;
};
