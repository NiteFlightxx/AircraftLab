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

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Input")
	bool bCenteredThrottleUsesHoverPoint = true;
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

/** 调试输出策略，不参与控制算法。 */
USTRUCT(BlueprintType)
struct AIRCRAFTLAB_API FFlightControllerDebugConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Debug")
	bool bEnableDebugLog = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Debug")
	bool bLogRotorCommands = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Debug")
	bool bLogRotorLayout = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Debug")
	bool bLogSignDiagnostics = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Debug", meta = (ClampMin = "0.0"))
	float LogIntervalSeconds = 0.20f;
};

/**
 * 物理线程只读的配置快照。
 * 由组件在 BeginPlay 前后边界从资产或组件回退参数一次性构建；物理线程不访问资产 UObject。
 */
struct AIRCRAFTLAB_API FFlightControllerRuntimeConfig
{
	FDroneFlightControllerConfig Controller;
	FFlightControllerInputConfig Input;
	FFlightControllerExecutionConfig Execution;
	FFlightControllerDebugConfig Debug;
};

namespace FlightControllerConfig
{
	/** 组件回退配置与新建资产共同使用的唯一默认参数来源。 */
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

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile", meta = (ClampMin = "1"))
	int32 ConfigVersion = 1;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FDroneFlightControllerConfig Controller;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FFlightControllerInputConfig Input;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FFlightControllerExecutionConfig Execution;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Profile")
	FFlightControllerDebugConfig Debug;

	/** 构建不含 UObject 引用的只读运行快照。 */
	FFlightControllerRuntimeConfig BuildRuntimeConfig() const;

	/** 校验参数关系；不修改资产。 */
	bool ValidateProfile(TArray<FText>& OutErrors) const;
};
