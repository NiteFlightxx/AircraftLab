// LOD 条目直接来自 UAircraftComponent 资产的 Dataflow 编译产物。
//
// 每机适配器：所有更新由世界子系统驱动，本组件无 Tick。

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODConsumer.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODComponent.generated.h"

class UAircraftSimulationWorldSubsystem;
class UPrimitiveComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnAircraftLODSelectionChanged,
	int32, PreviousLODIndex,
	int32, NewLODIndex);

/** LOD 条目的轻量视图（Dataflow 编译产物 FAircraftSimulationLODRuntimeSettings 的镜像）。 */
struct AIRCRAFTRUNTIMECOMMON_API FAircraftSimulationLODRuntimeSettingsLite
{
	FName Name = NAME_None;
	EAircraftSimulationDriveMode DriveMode = EAircraftSimulationDriveMode::FlightController;
	EAircraftSimulationCollisionMode CollisionMode = EAircraftSimulationCollisionMode::QueryAndPhysics;
	float MaxDistanceCm = 6000.0f;
};

/** Gameplay 网络策略。数组下标与资产 LOD 下标一一对应，但不参与资产编译。 */
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

UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTRUNTIMECOMMON_API UAircraftSimulationLODComponent
	: public UActorComponent
{
	GENERATED_BODY()

public:
	UAircraftSimulationLODComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	int32 GetCurrentSimulationLOD() const { return CurrentLODIndex; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance GetSimulationImportance() const { return Importance; }

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetSimulationImportance(const FAircraftSimulationImportance& NewImportance);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetInCombat(bool bInCombat);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetFiring(bool bFiring);

	/** 在 CombatKeepAliveSeconds 内保持数组第 0 项选中。 */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void NotifyCombatActivity();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void NotifyRecentlyDamaged();

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetMustRemainPhysical(bool bMustRemainPhysical);

	/** 仅外部吊挂/世界约束强制数组第 0 项。 */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void SetHasExternalPhysicsConstraint(bool bHasConstraint);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation")
	void ForceSimulationReevaluation();

	UPROPERTY(BlueprintAssignable, Category = "Aircraft|Simulation")
	FOnAircraftLODSelectionChanged OnLODSelectionChanged;

	/* ------- 子系统驱动接口（AircraftSimulationWorldSubsystem 调用） ------- */

	FAircraftSimulationSnapshot BuildSnapshot(float NearestPlayerDistanceCm, float WorldTimeSeconds) const;
	bool IsEvaluationDue(float WorldTimeSeconds) const;
	void MarkEvaluated(float WorldTimeSeconds);
	float GetSecondsInCurrentLOD(float WorldTimeSeconds) const;
	void ApplyLODFromSubsystem(int32 NewLODIndex, float WorldTimeSeconds);

	/** 从 Owner 的 UAircraftComponent 读取 Dataflow 编译的 LOD 条目表。 */
	bool GetLODSettings(TArray<FAircraftSimulationLODRuntimeSettingsLite>& OutSettings) const;
	bool IsAuthoritySimulationOnly() const;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float EvaluationIntervalSeconds = 0.25f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float DistanceHysteresisCm = 2000.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float MinimumLODResidenceSeconds = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float CombatKeepAliveSeconds = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float DamageKeepAliveSeconds = 5.0f;

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
	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance Importance;

	UPROPERTY(ReplicatedUsing = OnRep_CurrentLODIndex)
	int32 CurrentLODIndex = 0;

private:
	friend class FAircraftCollisionBudgetStateTest;

	struct FCollisionComponentState
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		ECollisionEnabled::Type OriginalCollision = ECollisionEnabled::NoCollision;
	};

	float LastEvaluationWorldTime = -1.0f;
	float LastLODChangeWorldTime = 0.0f;
	float LastCombatActivityWorldTime = -1000.0f;
	float LastDamageWorldTime = -1000.0f;
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
	void RefreshConsumers();
	FAircraftSimulationLODNetworkSettings GetNetworkSettings(int32 LODIndex) const;
	void ApplyCurrentBudget();
};
