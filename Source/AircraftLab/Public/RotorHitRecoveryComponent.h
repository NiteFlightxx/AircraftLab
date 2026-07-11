#pragma once

#include "CoreMinimal.h"
#include "FlightGameplayPolicyComponent.h"
#include "RotorHitRecoveryPolicyAsset.h"

#include "RotorHitRecoveryComponent.generated.h"

class UFlightControllerComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FOnRotorHitRecoveryStateChanged,
	ERotorHitRecoveryState, PreviousState,
	ERotorHitRecoveryState, NewState);

/**
 * 游戏化旋翼受击恢复策略。
 * 只调用 FlightController 的公开故障/模式接口，不进入 PID、Allocator 或物理线程算法。
 */
UCLASS(ClassGroup = (AircraftLab), meta = (BlueprintSpawnableComponent))
class AIRCRAFTLAB_API URotorHitRecoveryComponent : public UFlightGameplayPolicyComponent
{
	GENERATED_BODY()

public:
	URotorHitRecoveryComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** 通知策略某个旋翼被击中。支持多个旋翼同时处于恢复流程。 */
	UFUNCTION(BlueprintCallable, Category = "Flight|GameplayPolicy|RotorHit")
	bool NotifyRotorHit(int32 RotorIndex);

	/** 立即恢复所有由本策略管理的旋翼并退出恢复状态。 */
	UFUNCTION(BlueprintCallable, Category = "Flight|GameplayPolicy|RotorHit")
	void CancelRecovery(bool bRestoreRotors = true);

	UFUNCTION(BlueprintPure, Category = "Flight|GameplayPolicy|RotorHit")
	ERotorHitRecoveryState GetRecoveryState() const { return RecoveryState; }

	UFUNCTION(BlueprintPure, Category = "Flight|GameplayPolicy|RotorHit")
	int32 GetRecoveringRotorCount() const { return ActiveRecoveries.Num(); }

	UPROPERTY(BlueprintAssignable, Category = "Flight|GameplayPolicy|RotorHit")
	FOnRotorHitRecoveryStateChanged OnRecoveryStateChanged;

protected:
	void SetRecoveryState(ERotorHitRecoveryState NewState);
	void ApplyRecoveryModeIfNeeded();
	bool IsAircraftStable() const;
	void FinishRecovery();

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Flight|GameplayPolicy|RotorHit")
	TObjectPtr<URotorHitRecoveryPolicyAsset> PolicyAsset;

private:
	struct FActiveRotorRecovery
	{
		float TimeSinceHitSeconds = 0.0f;
		float TimeSinceEffectivenessUpdateSeconds = 0.0f;
	};

	UPROPERTY(Transient)
	TObjectPtr<UFlightControllerComponent> FlightController;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Flight|GameplayPolicy|RotorHit", meta = (AllowPrivateAccess = "true"))
	ERotorHitRecoveryState RecoveryState = ERotorHitRecoveryState::CombatReady;

	TMap<int32, FActiveRotorRecovery> ActiveRecoveries;
	float StableDurationSeconds = 0.0f;
	EDroneFlightMode PreviousFlightMode = EDroneFlightMode::Angle;
	bool bWasArmedBeforeDamage = false;
	bool bRecoveryModeApplied = false;
};
