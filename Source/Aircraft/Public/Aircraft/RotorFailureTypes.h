// 对应 NxGame AircraftLab 的 FlightControllerTypes.h（旋翼健康段）+
// FlightControllerProfileAsset.h（失效策略配置段）。
// 这些诊断类型对蓝图可见，供组件/调试 UI 直接回读。

#pragma once

#include "CoreMinimal.h"

#include "RotorFailureTypes.generated.h"

/** 旋翼失效模式（预留扩展）。 */
UENUM(BlueprintType)
enum class EAircraftRotorFailureMode : uint8
{
	/** 正常工作。 */
	Healthy,
	/** 完全失效。 */
	CompleteFailure,
	/** 部分损坏。 */
	PartialFailure,
	/** 响应延迟（预留）。 */
	ResponseDelay,
	/** 随机输出噪声（预留）。 */
	RandomNoise,
	/** 输出卡死（预留）。 */
	StuckOutput,
};

/**
 * 旋翼健康状态。Effectiveness 直接参与 Jacobian 构建，
 * 控制分配器自动感知旋翼失效。
 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftRotorHealthState
{
	GENERATED_BODY()

	/** 旋翼效能 0.0=完全失效 1.0=正常。 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aircraft|RotorHealth", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Effectiveness = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|RotorHealth")
	bool bIsFailed = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|RotorHealth")
	float FailureTimestamp = -1.0f;

	EAircraftRotorFailureMode FailureMode = EAircraftRotorFailureMode::Healthy;

	void MarkFailed(float Timestamp)
	{
		Effectiveness = 0.0f;
		bIsFailed = true;
		FailureTimestamp = Timestamp;
		FailureMode = EAircraftRotorFailureMode::CompleteFailure;
	}

	void Recover()
	{
		Effectiveness = 1.0f;
		bIsFailed = false;
		FailureTimestamp = -1.0f;
		FailureMode = EAircraftRotorFailureMode::Healthy;
	}

	bool IsHealthy() const { return Effectiveness >= 1.0f && !bIsFailed; }
};

/** 控制能力评估：当前各轴剩余控制能力（0~1 归一化），在分配缓存重建后更新。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftControlAuthorityInfo
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	float CollectiveAuthority = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	float RollAuthority = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	float PitchAuthority = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	float YawAuthority = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	int32 HealthyRotorCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Authority")
	int32 FailedRotorCount = 0;

	void Reset()
	{
		CollectiveAuthority = 0.0f;
		RollAuthority = 0.0f;
		PitchAuthority = 0.0f;
		YawAuthority = 0.0f;
		HealthyRotorCount = 0;
		FailedRotorCount = 0;
	}
};

/** 控制能力不足时由组件执行的降级动作。 */
UENUM(BlueprintType)
enum class EAircraftFailurePolicyAction : uint8
{
	WarningOnly UMETA(DisplayName = "仅警告"),
	SwitchFlightMode UMETA(DisplayName = "切换飞行模式"),
	Failsafe UMETA(DisplayName = "故障保护 / 停桨"),
	EmergencyStop UMETA(DisplayName = "紧急停止")
};

/**
 * 旋翼故障与控制权限降级策略。
 * 任一启用的阈值不满足并持续 ConfirmationTimeSeconds 后触发 Action。
 * 默认值对齐 NxGame 的 FFlightControllerFailurePolicyConfig。
 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftFailurePolicyConfig
{
	GENERATED_BODY()

	/** 默认关闭，确保资产化不改变现有飞行行为。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (DisplayName = "启用故障策略"))
	bool bEnabled = false;

	/** 默认只在已解锁飞行时评估，避免地面维护阶段触发降级。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (DisplayName = "仅解锁时评估"))
	bool bEvaluateOnlyWhenArmed = true;

	/** 最少健康旋翼数；0 表示不检查。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0", DisplayName = "最少健康旋翼数"))
	int32 MinimumHealthyRotorCount = 4;

	/** 各轴最小剩余控制权限；0 表示不检查该轴。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "最小总距权限"))
	float MinimumCollectiveAuthority = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "最小滚转权限"))
	float MinimumRollAuthority = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "最小俯仰权限"))
	float MinimumPitchAuthority = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "最小偏航权限"))
	float MinimumYawAuthority = 0.25f;

	/** 故障条件必须连续成立的时间，防止单帧抖动。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", DisplayName = "确认时间（秒）"))
	float ConfirmationTimeSeconds = 0.10f;

	/** 未锁存时，条件恢复后必须连续健康的时间。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", DisplayName = "恢复确认时间（秒）"))
	float RecoveryConfirmationTimeSeconds = 1.0f;

	/** 触发后保持锁存，必须显式调用 ResetPolicyLatch。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (DisplayName = "锁存触发动作"))
	bool bLatchTriggeredAction = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (DisplayName = "故障动作"))
	EAircraftFailurePolicyAction Action = EAircraftFailurePolicyAction::WarningOnly;

	/** Action=SwitchFlightMode 时切换到的模式（存 EAircraftFlightMode 的整型值，避免头文件环依赖）。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy",
		meta = (EditCondition = "Action == EAircraftFailurePolicyAction::SwitchFlightMode", EditConditionHides, DisplayName = "降级飞行模式"))
	uint8 DegradedFlightMode = 2;
};

/** FailurePolicy 的只读运行状态和最近判定原因。 */
USTRUCT(BlueprintType)
struct AIRCRAFT_API FAircraftFailurePolicyStatus
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bHasAuthoritySample = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bViolationActive = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bTriggered = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	float ViolationDurationSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	float RecoveryDurationSeconds = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	EAircraftFailurePolicyAction TriggeredAction = EAircraftFailurePolicyAction::WarningOnly;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bHealthyRotorCountViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bCollectiveAuthorityViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bRollAuthorityViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bPitchAuthorityViolation = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|FailurePolicy")
	bool bYawAuthorityViolation = false;
};
