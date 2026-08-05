// 对齐 ClothingSystemRuntimeInterface 的契约层职责。
// 对应 NxGame AircraftCore/Public/AutopilotProvider.h：
// Autopilot（AircraftRuntimeCommon 模块）→ 飞控（AircraftAssetEngine 模块）的纯数据 setpoint 契约。

#pragma once

#include "CoreMinimal.h"

#include "AutopilotProvider.generated.h"

/** 从高层制导传递到飞控的纯数据契约。 */
USTRUCT(BlueprintType)
struct AIRCRAFTRUNTIMEINTERFACE_API FAutopilotInjection
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	FVector PositionSetpointCm = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	FVector VelocitySetpointCmPerSec = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	FVector AccelerationSetpointCmPerSecSq = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float AltitudeSetpointCm = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float VerticalVelocitySetpointCmPerSec = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float ThrustFeedForward = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float YawSetpointDegrees = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float YawRateSetpointDegPerSec = 0.0f;
	/** 每个意图的偏航速率上限。为 0 时回落到飞控硬限幅。 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float YawRateLimitDegPerSec = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	float TurnRollDegrees = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Aircraft|Autopilot")
	bool bValid = false;
};

/**
 * Autopilot 设定值注入接口（UInterface）
 *
 * 架构定位：作为飞控模块与 Autopilot 模块之间的稳定公共契约：
 *   - 本接口由 AircraftRuntimeInterface 模块拥有
 *   - UAutopilotComponent（AircraftRuntimeCommon 模块）实现此接口
 *   - UAircraftComponent 持有 UObject* 并通过 Cast<IAutopilotProvider> 调用
 *   - 双方都只依赖 AircraftRuntimeInterface 中的本契约（依赖无环）
 *
 * 数据流：
 *   UAutopilotComponent.Tick（游戏线程，TG_PrePhysics）
 *     → 计算并缓存 FAutopilotInjection
 *     → UAircraftComponent.TickComponent 通过本接口拉取
 *     → 缓存后经双缓冲交给 FAircraftSimulationProxy（物理线程读）
 *
 * 线程安全：与飞行员输入缓存相同的无锁模式，
 *   依赖 TG_PrePhysics 先于物理子步的引擎调度保证。
 */
UINTERFACE(BlueprintType, meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTRUNTIMEINTERFACE_API UAutopilotProvider : public UInterface
{
	GENERATED_BODY()
};

/**
 * IAutopilotProvider —— Autopilot 设定值提供者接口
 *
 * 由 UAutopilotComponent 实现。飞控通过本接口拉取
 * 经 Motion Profile 整形 + 前馈计算后的物理可达设定值。
 */
class AIRCRAFTRUNTIMEINTERFACE_API IAutopilotProvider
{
	GENERATED_BODY()

public:
	/**
	 * 拉取本周期 Autopilot 注入的设定值。
	 * @param OutInjection 输出：经 Motion Profile 整形 + 前馈计算后的设定值集合
	 * @return 是否产出了有效注入（false 表示 Autopilot 未激活或无有效设定值）
	 */
	virtual bool GetAutopilotInjection(FAutopilotInjection& OutInjection) const = 0;

	/**
	 * 查询 Autopilot 是否处于激活状态。
	 * 飞控据此决定是否使用注入设定值。
	 */
	virtual bool IsAutopilotActive() const = 0;
};
