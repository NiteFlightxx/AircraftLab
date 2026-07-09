// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "AutopilotProvider.generated.h"

// 前向声明：FAutopilotInjection 定义在 FlightControllerComponent.h，
// 此处仅声明依赖，接口文件本身不依赖 Autopilot 模块。
struct FAutopilotInjection;

/**
 * Autopilot 设定值注入接口（UInterface）
 *
 * 架构定位：解决"AircraftLab 不能反向依赖 AircraftAutopilot"的耦合问题。
 *   - 本接口由 AircraftLab 拥有（在 AircraftLab 模块内定义）
 *   - UAutopilotComponent（在 AircraftAutopilot 模块内）实现此接口
 *   - UFlightControllerComponent 持有 UObject* 并通过 Cast<IAutopilotProvider> 调用
 *   - 单向依赖：AircraftAutopilot → AircraftLab（Autopilot 依赖 Lab），Lab 不依赖 Autopilot
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
class AIRCRAFTLAB_API UAutopilotProvider : public UInterface
{
	GENERATED_BODY()
};

/**
 * IAutopilotProvider —— Autopilot 设定值提供者接口
 *
 * 由 UAutopilotComponent 实现。FlightController 通过本接口拉取
 * 经 Motion Profile 整形 + 前馈计算后的物理可达设定值。
 */
class AIRCRAFTLAB_API IAutopilotProvider
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
