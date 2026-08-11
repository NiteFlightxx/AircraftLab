// 范式分歧：LOD 条目来源由 UAircraftSimulationLODProfileAsset 改为
// UAircraftComponent 资产的 Dataflow 编译产物（SimulationLOD.LODs，
// 经 IAircraftSimulationLODController 契约刷新）。
//
// 每机适配器：所有更新由世界子系统驱动，本组件无 Tick。

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODConsumer.h"
#include "AircraftRuntimeInterface/AircraftSimulationLODTypes.h"

#include "AircraftSimulationLODComponent.generated.h"

class UAircraftSimulationWorldSubsystem;

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
	bool bRunSlowLogic = true;
	float SlowLogicIntervalSeconds = 0.0f;
	float SuggestedNetUpdateFrequency = 30.0f;
	bool bEnableNetworkDormancy = false;
};

UCLASS(ClassGroup = (Aircraft), meta = (BlueprintSpawnableComponent))
class AIRCRAFTRUNTIMECOMMON_API UAircraftSimulationLODComponent
	: public UActorComponent
	, public IAircraftSimulationLODController
{
	GENERATED_BODY()

public:
	UAircraftSimulationLODComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void RefreshAircraftSimulationDrive_Implementation() override;

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

	/**
	 * 持久选择使用 DriveMode 的 LOD 条目；手动选择在 ClearManualDriveModeOverride
	 * 之前优先于自动 LOD 与运动源请求。资产中无该驱动模式条目时返回 false。
	 */
	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation|Drive")
	bool SetManualDriveModeOverride(EAircraftSimulationDriveMode DriveMode);

	UFUNCTION(BlueprintCallable, Category = "Aircraft|Simulation|Drive")
	void ClearManualDriveModeOverride();

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation|Drive")
	bool HasManualDriveModeOverride() const { return bManualDriveModeOverrideActive; }

	UFUNCTION(BlueprintPure, Category = "Aircraft|Simulation|Drive")
	EAircraftSimulationDriveMode GetManualDriveModeOverride() const { return ManualDriveModeOverride; }

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

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float EvaluationIntervalSeconds = 0.25f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float DistanceHysteresisCm = 500.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float MinimumLODResidenceSeconds = 2.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float CombatKeepAliveSeconds = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation", meta = (ClampMin = "0.0"))
	float DamageKeepAliveSeconds = 5.0f;

protected:
	UPROPERTY(EditAnywhere, Category = "Aircraft|Simulation")
	FAircraftSimulationImportance Importance;

	int32 CurrentLODIndex = 0;

public:

private:
	bool bManualDriveModeOverrideActive = false;
	EAircraftSimulationDriveMode ManualDriveModeOverride = EAircraftSimulationDriveMode::None;

	float LastEvaluationWorldTime = -1.0f;
	float LastLODChangeWorldTime = 0.0f;
	float LastCombatActivityWorldTime = -1000.0f;
	float LastDamageWorldTime = -1000.0f;
	bool bHasAppliedBudget = false;
	TWeakObjectPtr<class UAircraftComponent> AircraftComponent;
	TArray<TWeakObjectPtr<UActorComponent>> Consumers;

	void RefreshConsumerCache();
	void RefreshConsumers();
	FAircraftSimulationDriveOverride ResolveDriveOverride() const;
};
