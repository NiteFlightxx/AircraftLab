#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "DroneTypes.h"
#include "AircraftMovementIntent.h"

#include "RotorHitRecoveryPolicyAsset.generated.h"

/** 旋翼受击恢复策略的游戏状态。 */
UENUM(BlueprintType)
enum class ERotorHitRecoveryState : uint8
{
	CombatReady,
	DamagedUnstable,
	RecoveryRamp,
	Stabilizing
};

namespace RotorHitRecoveryPolicy
{
	AIRCRAFTLAB_API float ComputeRecoveryAlpha(float TimeSinceHitSeconds,
		float UnstableDurationSeconds, float RecoveryRampSeconds);
	AIRCRAFTLAB_API bool IsKinematicStateStable(const FDroneKinematicState& State,
		float MaximumTiltDegrees, float MaximumAngularRateDegPerSec, float MaximumVerticalSpeedCmPerSec);
}

/** 旋翼受击、短暂失稳、渐进修复与稳定确认的独立游戏策略资产。 */
UCLASS(BlueprintType)
class AIRCRAFTLAB_API URotorHitRecoveryPolicyAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	/** true=受击时完全停桨；false=降低效能，更适合四旋翼游戏表现。 */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Damage")
	bool bUseCompleteFailureOnHit = false;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Damage", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float DamagedEffectiveness = 0.35f;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Damage", meta = (ClampMin = "0.0"))
	float UnstableDurationSeconds = 1.25f;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Recovery", meta = (ClampMin = "0.0"))
	float RecoveryRampSeconds = 2.0f;

	/** 更新 Effectiveness 的间隔；限制 Allocator 重建频率。 */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Recovery", meta = (ClampMin = "0.01"))
	float EffectivenessUpdateIntervalSeconds = 0.05f;

	/** 恢复阶段提交给飞控的统一 MovementIntent；默认 Hold。 */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Recovery")
	FAutopilotMovementIntent RecoveryMovementIntent;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Recovery")
	bool bRestorePreviousFlightMode = true;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Recovery")
	bool bAutomaticallyRearmForRecovery = true;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Recovery")
	bool bResetFailurePolicyLatchForRecovery = true;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Stability", meta = (ClampMin = "0.0"))
	float MaximumStableTiltDegrees = 8.0f;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Stability", meta = (ClampMin = "0.0"))
	float MaximumStableAngularRateDegPerSec = 20.0f;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Stability", meta = (ClampMin = "0.0"))
	float MaximumStableVerticalSpeedCmPerSec = 80.0f;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Stability", meta = (ClampMin = "0.0"))
	float StabilityConfirmationSeconds = 1.0f;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Combat")
	bool bDisableCombatWhileRecovering = true;
};
