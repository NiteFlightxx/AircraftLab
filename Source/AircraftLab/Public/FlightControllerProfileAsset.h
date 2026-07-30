#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "AircraftType.h"

#include "FlightControllerProfileAsset.generated.h"

/** 飞控输入解释参数。仅描述现有输入行为，不包含运行期输入状态。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControllerInputConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "水平保持死区"))
	float HorizontalHoldStickDeadband = 0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "垂直保持死区"))
	float VerticalHoldStickDeadband = 0.08f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "偏航保持死区"))
	float YawHoldStickDeadband = 0.05f;

	/** 松开水平摇杆后，低于此速度才锁定最终悬停位置。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input",
		meta = (ClampMin = "0.0", DisplayName = "水平制动转位置保持速度（厘米/秒）"))
	float HorizontalBrakeToHoldSpeedCmPerSec = 20.0f;

};

/** 组件固定时序与启动策略。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControllerExecutionConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Execution", meta = (DisplayName = "默认启用控制器"))
	bool bControllerEnabledByDefault = true;

};

/** Generic six-degree-of-freedom physics backend owned by the flight simulation. */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightSimulationConstraintConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (ClampMin = "0.0", DisplayName = "线性位置强度"))
	float LinearPositionStrength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (ClampMin = "0.0", DisplayName = "线性速度阻尼"))
	float LinearVelocityStrength = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (ClampMin = "0.0", DisplayName = "最大线性力"))
	float LinearForceLimit = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (ClampMin = "0.0", DisplayName = "角度位置强度"))
	float AngularPositionStrength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (ClampMin = "0.0", DisplayName = "角速度阻尼"))
	float AngularVelocityStrength = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (ClampMin = "0.0", DisplayName = "最大角度力矩"))
	float AngularTorqueLimit = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Constraint", meta = (DisplayName = "使用加速度驱动"))
	bool bAccelerationMode = true;
};

/** Generic transform backend owned by the flight simulation. */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightSimulationKinematicConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Kinematic", meta = (DisplayName = "移动时检测碰撞"))
	bool bSweepMovement = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Kinematic", meta = (ClampMin = "0.0", DisplayName = "位置纠偏速率"))
	float PositionCorrectionRate = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Kinematic", meta = (ClampMin = "0.0", DisplayName = "旋转插值速度"))
	float RotationInterpSpeed = 8.0f;
};

/** 控制能力不足时由组件执行的降级动作。 */
UENUM(BlueprintType)
enum class EFlightFailurePolicyAction : uint8
{
	WarningOnly UMETA(DisplayName = "仅警告"),
	SwitchFlightMode UMETA(DisplayName = "切换飞行模式"),
	Failsafe UMETA(DisplayName = "故障保护 / 停桨"),
	EmergencyStop UMETA(DisplayName = "紧急停止")
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

	/** 触发后保持锁存，必须显式调用 ResetFailurePolicyLatch。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (DisplayName = "锁存触发动作"))
	bool bLatchTriggeredAction = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy", meta = (DisplayName = "故障动作"))
	EFlightFailurePolicyAction Action = EFlightFailurePolicyAction::WarningOnly;

	/** Action=SwitchFlightMode 时切换到的模式。 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "FailurePolicy",
		meta = (EditCondition = "Action == EFlightFailurePolicyAction::SwitchFlightMode", EditConditionHides, DisplayName = "降级飞行模式"))
	EAircraftFlightMode DegradedFlightMode = EAircraftFlightMode::Angle;
};

/**
 * 物理线程只读的配置快照。
 * 由组件在 BeginPlay 边界从必需的 Profile 一次性构建；物理线程不访问资产 UObject。
 */
struct AIRCRAFTLAB_API FFlightControllerRuntimeConfig
{
	FAircraftFlightControllerConfig Controller;
	FFlightControllerInputConfig Input;
	FFlightControllerExecutionConfig Execution;
	FFlightControllerFailurePolicyConfig FailurePolicy;
	FFlightSimulationConstraintConfig ConstraintSimulation;
	FFlightSimulationKinematicConfig KinematicSimulation;
};

namespace FlightControllerConfig
{
	/** 新建资产使用的唯一默认控制参数来源。 */
	AIRCRAFTLAB_API void InitializeDefaults(FAircraftFlightControllerConfig& OutConfig);
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
	
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "控制器配置"))
	FAircraftFlightControllerConfig Controller;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "输入配置"))
	FFlightControllerInputConfig Input;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "执行配置"))
	FFlightControllerExecutionConfig Execution;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "故障策略"))
	FFlightControllerFailurePolicyConfig FailurePolicy;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "物理约束模拟"))
	FFlightSimulationConstraintConfig ConstraintSimulation;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (DisplayName = "运动学模拟"))
	FFlightSimulationKinematicConfig KinematicSimulation;

	/** 构建不含 UObject 引用的只读运行快照。 */
	FFlightControllerRuntimeConfig BuildRuntimeConfig() const;

	/** 校验参数关系；不修改资产。 */
	bool ValidateProfile(TArray<FText>& OutErrors) const;
};
