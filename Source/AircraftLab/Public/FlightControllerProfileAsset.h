#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "DroneTypes.h"

#include "FlightControllerProfileAsset.generated.h"

/** 飞控输入解释参数。仅描述现有输入行为，不包含运行期输入状态。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControllerInputConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float HorizontalHoldStickDeadband = 0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float VerticalHoldStickDeadband = 0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float YawHoldStickDeadband = 0.05f;

};

/** 组件固定时序与启动策略。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControllerExecutionConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Execution", meta = (ClampMin = "1.0"))
	float ControlLoopRateHz = 250.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Execution")
	EDroneFlightMode InitialFlightMode = EDroneFlightMode::Angle;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Execution")
	bool bStartArmed = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Execution")
	bool bControllerEnabledByDefault = true;

};

/** 控制能力不足时由组件执行的降级动作。 */
UENUM(BlueprintType)
enum class EFlightFailurePolicyAction : uint8
{
	WarningOnly UMETA(DisplayName = "Warning Only"),
	SwitchFlightMode UMETA(DisplayName = "Switch Flight Mode"),
	Failsafe UMETA(DisplayName = "Failsafe / Stop Rotors"),
	EmergencyStop UMETA(DisplayName = "Emergency Stop")
};

/**
 * 旋翼故障与控制权限降级策略。
 * 任一启用的阈值不满足并持续 ConfirmationTimeSeconds 后触发 Action。
 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControllerFailurePolicyConfig
{
	GENERATED_BODY()

	/** 默认关闭，确保资产化不改变现有飞行行为。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy")
	bool bEnabled = false;

	/** 默认只在已解锁飞行时评估，避免地面维护阶段触发降级。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy")
	bool bEvaluateOnlyWhenArmed = true;

	/** 最少健康旋翼数；0 表示不检查。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0"))
	int32 MinimumHealthyRotorCount = 4;

	/** 各轴最小剩余控制权限；0 表示不检查该轴。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinimumCollectiveAuthority = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinimumRollAuthority = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinimumPitchAuthority = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinimumYawAuthority = 0.25f;

	/** 故障条件必须连续成立的时间，防止单帧抖动。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0"))
	float ConfirmationTimeSeconds = 0.10f;

	/** 未锁存时，条件恢复后必须连续健康的时间。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (ClampMin = "0.0"))
	float RecoveryConfirmationTimeSeconds = 1.0f;

	/** 触发后保持锁存，必须显式调用 ResetFailurePolicyLatch。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy")
	bool bLatchTriggeredAction = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy")
	EFlightFailurePolicyAction Action = EFlightFailurePolicyAction::WarningOnly;

	/** Action=SwitchFlightMode 时切换到的模式。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy",
		meta = (EditCondition = "Action == EFlightFailurePolicyAction::SwitchFlightMode", EditConditionHides))
	EDroneFlightMode DegradedFlightMode = EDroneFlightMode::Angle;
};

/**
 * 物理线程只读的配置快照。
 * 由组件在 BeginPlay 边界从必需的 Profile 一次性构建；物理线程不访问资产 UObject。
 */
struct AIRCRAFTLAB_API FFlightControllerRuntimeConfig
{
	FDroneFlightControllerConfig Controller;
	FFlightControllerInputConfig Input;
	FFlightControllerExecutionConfig Execution;
	FFlightControllerFailurePolicyConfig FailurePolicy;
};

namespace FlightControllerConfig
{
	/** 新建资产使用的唯一默认控制参数来源。 */
	AIRCRAFTLAB_API void InitializeDefaults(FDroneFlightControllerConfig& OutConfig);
}

/**
 * 可复用的独立飞控配置资产。
 * 第一版原样承载现有控制参数，避免资产化同时改变算法或默认值。
 */
UCLASS(BlueprintType)
class AIRCRAFTLAB_API UFlightControllerProfileAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UFlightControllerProfileAsset();
	
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FDroneFlightControllerConfig Controller;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FFlightControllerInputConfig Input;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FFlightControllerExecutionConfig Execution;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FFlightControllerFailurePolicyConfig FailurePolicy;

	/** 构建不含 UObject 引用的只读运行快照。 */
	FFlightControllerRuntimeConfig BuildRuntimeConfig() const;

	/** 校验参数关系；不修改资产。 */
	bool ValidateProfile(TArray<FText>& OutErrors) const;
};
