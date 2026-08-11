// Private/RotorFailureManager.cpp。
//
// 旋翼健康管理器：健康状态表、控制权限评估（相对全健康基准归一化）、失效策略判定
// （确认时间滞回 + 可选锁存）。

#pragma once

#include "CoreMinimal.h"
#include "Aircraft/ControlAllocationTypes.h"
#include "Aircraft/RotorFailureTypes.h"

#include "RotorFailureManager.generated.h"

/** 旋翼健康管理器拥有的健康状态和控制能力评估。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftRotorFailureManager
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|RotorHealth")
	TMap<FName, FAircraftRotorHealthState> HealthStatesByName;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|RotorHealth")
	FAircraftControlAuthorityInfo AuthorityInfo;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	FAircraftFailurePolicyStatus PolicyStatus;

	static void MarkRotorFailed(FAircraftRotorHealthState& State, float Timestamp);
	static void RecoverRotor(FAircraftRotorHealthState& State);
	static void SetRotorEffectiveness(
		FAircraftRotorHealthState& State, float Effectiveness, float Timestamp, double FailureEpsilon);
	void RecoverAllRotors();

	/** 以全健康基准归一化 Cache 中的有效权限，并统计健康/失效旋翼数。 */
	void UpdateAuthority(const FAircraftAllocationCache& AllocationCache,
		double BaselineCollectiveAuthority,
		double BaselineRollAuthority, double BaselinePitchAuthority, double BaselineYawAuthority,
		double AuthorityEpsilon);

	/**
	 * 评估失效策略；返回 true 表示本帧新触发 Action。
	 * 滞回：违规需持续 ConfirmationTimeSeconds 才触发；
	 * 未锁存时恢复需持续 RecoveryConfirmationTimeSeconds 才解除。
	 */
	bool EvaluatePolicy(const FAircraftFailurePolicyConfig& Policy, float DeltaSeconds,
		EAircraftFailurePolicyAction& OutAction);

	void ResetPolicyLatch();

	void ResetAuthority()
	{
		AuthorityInfo.Reset();
		PolicyStatus.bHasAuthoritySample = false;
		PolicyStatus.bViolationActive = false;
		PolicyStatus.ViolationDurationSeconds = 0.0f;
		PolicyStatus.RecoveryDurationSeconds = 0.0f;
	}
};
