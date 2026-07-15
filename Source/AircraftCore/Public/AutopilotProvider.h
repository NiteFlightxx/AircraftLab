// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotProvider.generated.h"

/** Pure data contract passed from high-level guidance to a flight controller. */
USTRUCT(BlueprintType)
struct AIRCRAFTCORE_API FAutopilotInjection
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	FVector PositionSetpointCm = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	FVector VelocitySetpointCmPerSec = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	FVector AccelerationSetpointCmPerSecSq = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float AltitudeSetpointCm = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float VerticalVelocitySetpointCmPerSec = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float ThrustFeedForward = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float YawSetpointDegrees = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float YawRateSetpointDegPerSec = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	float TurnRollDegrees = 0.0f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Autopilot")
	bool bValid = false;
};

/**
 * Autopilot 设定值注入接口（UInterface）
 *
 * 架构定位：作为 AircraftLab 与 AircraftAutopilot 之间的稳定公共契约。
 *   - 本接口由 AircraftCore 模块拥有
 *   - UAutopilotComponent（在 AircraftAutopilot 模块内）实现此接口
 *   - UFlightControllerComponent 持有 UObject* 并通过 Cast<IAutopilotProvider> 调用
 *   - AircraftLab 与 AircraftAutopilot 都只依赖 AircraftCore 中的本契约
 *
 * 数据流：
 *   UAutopilotComponent.Tick（游戏线程）
 *     → 计算并缓存 FAutopilotInjection
 *     → UFlightControllerComponent.TickComponent 通过本接口拉取
 *     → CachedAutopilotInjection（游戏线程写）
 *     → AsyncPhysicsTickComponent（物理线程读）
 *
 * 线程安全：与 CachedPilotInput 相同的无锁模式，
 *   依赖 TG_PrePhysics 先于物理子步的引擎调度保证。
 */
UINTERFACE(BlueprintType, meta = (CannotImplementInterfaceInBlueprint))
class AIRCRAFTCORE_API UAutopilotProvider : public UInterface
{
	GENERATED_BODY()
};

/**
 * IAutopilotProvider —— Autopilot 设定值提供者接口
 *
 * 由 UAutopilotComponent 实现。FlightController 通过本接口拉取
 * 经 Motion Profile 整形 + 前馈计算后的物理可达设定值。
 */
class AIRCRAFTCORE_API IAutopilotProvider
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
	 * FlightController 据此决定是否使用注入设定值（灰度开关的运行时判断）。
	 */
	virtual bool IsAutopilotActive() const = 0;
};
