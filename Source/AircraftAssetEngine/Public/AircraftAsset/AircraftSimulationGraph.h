// 多旋翼默认 Simulation 图（程序化创建，无二进制资产依赖）。
//
//   布料链：DF_ClothSolver.uasset（二进制）+ BP_ClothPreview 类默认值引用 + 组件属性
//   我们：  本函数（瞬态单例）        + 组件 OnRegister 惰性填充     + 组件属性（可覆盖）
//
// 图内容 = 引擎最小求解调度链（与布料 DF_ClothSolver 完全同构）：
//   GetSimulationTime ─┐
//                      ├→ AdvancePhysicsSolvers → SimulationProxiesTerminal
//   GetPhysicsSolvers ─┘   SimulationGroups = {"Aircraft"}
//
// 语义：该图是 UDataflowSimulationManager 的"注册/调度壳" —— 图存在 ⇒ 组件
// SimulationAsset.DataflowAsset 非空 ⇒ OnCreatePhysicsState 的显式注册生效，
// 管理器每帧回调 PreProcessSimulation/WriteToSimulation（GT 输入桥接）。
// 控制+力注入仍在 AsyncPhysicsTickComponent（Chaos 物理子步）执行，与碰撞同一 pass；
// 图的 AdvancePhysicsSolvers 调用代理 AdvanceSolverDatas（注释化空实现）。

#pragma once

#include "CoreMinimal.h"

class UDataflow;

namespace UE::AircraftLab::AircraftAsset
{
	/** 组件默认 SimulationGroups 与图内 GetPhysicsSolvers 过滤组必须一致。 */
	extern AIRCRAFTASSETENGINE_API const FString AircraftSimulationGroupName;

	/**
	 * 获取（或首次创建）插件共享的默认 Simulation 图。
	 * 瞬态单例（GetTransientPackage / RF_Transient）：纯代码对象，不落地为资产文件。
	 */
	AIRCRAFTASSETENGINE_API UDataflow* GetOrCreateAircraftSimulationGraph();
}
