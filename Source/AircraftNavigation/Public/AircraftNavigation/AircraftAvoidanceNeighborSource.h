// 避障邻居数据源契约。
//
// 权责边界：ORCA 求解器（怎么避）属于本插件；"有哪些邻居"（数据从哪来）由
// 消费方注入——默认实现是游戏侧的全场扫描，注册表/空间索引实现（World
// 注册表或空间索引由导航/性能团队提供，经
// UNxAircraftNavigationGuidanceComponent::SetAvoidanceNeighborSource 注入。
// SelectNeighbors 承载全部纯过滤逻辑（半径剔除/TCPA 排序/截断），所有数据源
// 实现复用同一套语义，保证换数据源后 ORCA 看到的邻居序完全一致。

#pragma once

#include "CoreMinimal.h"
#include "AircraftNavigation/AircraftOrcaSolver.h"

/** 单次邻居查询参数：由消费方（GuidanceComponent）从自身状态与配置填充。 */
struct AIRCRAFTNAVIGATION_API FAircraftAvoidanceNeighborQuery
{
	FAircraftAvoidanceAgentState Self;
	int32 MaxNeighbors = 12;
	float TimeHorizonSeconds = 2.0f;
	float SeparationPaddingCm = 20.0f;
};

/** 避障邻居数据源抽象。实现必须可在游戏线程调用（求解频率 10-20Hz）。 */
class AIRCRAFTNAVIGATION_API IAircraftAvoidanceNeighborSource
{
public:
	virtual ~IAircraftAvoidanceNeighborSource() = default;

	/**
	 * 收集 Self 的避障邻居。
	 * 语义契约（由 AircraftAvoidanceNeighborSelection::SelectNeighbors 定义，
	 * 自定义实现应复用它以保证跨数据源行为一致）：
	 *   1. 采样时间补偿：候选位置按 (Self 采样时刻 - 候选采样时刻) × 候选速度前推，
	 *      补偿量钳制在 [0, TimeHorizonSeconds]；
	 *   2. 查询半径剔除：(Self 最大速度 + 候选最大速度) × TimeHorizon
	 *      + 半径和 + 跟踪预留和 + SeparationPaddingCm；
	 *   3. 按当前重叠、有限视界 3D CPA 净间距、TCPA、距离和 StableId 排序；
	 *   4. 截断保留最危险的 MaxNeighbors 个。
	 */
	virtual void GatherNeighbors(const FAircraftAvoidanceNeighborQuery& Query,
		TArray<FAircraftAvoidanceAgentState>& OutNeighbors) const = 0;
};

namespace AircraftAvoidanceNeighborSelection
{
	/**
	 * 纯过滤函数：候选集（已含 Self 的数据源实现负责剔除自身与无效状态）
	 * → 采样补偿/半径剔除/TCPA 排序/截断后的邻居集。无世界依赖，可单测。
	 */
	AIRCRAFTNAVIGATION_API void SelectNeighbors(
		const FAircraftAvoidanceNeighborQuery& Query,
		TConstArrayView<FAircraftAvoidanceAgentState> Candidates,
		TArray<FAircraftAvoidanceAgentState>& OutNeighbors);
}
