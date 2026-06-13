// 对齐 ChaosCloth 的运行时仿真器（D:/UnrealEngine/Engine/Plugins/ChaosCloth/Source/ChaosCloth）
//
// 模块定位：与 ChaosCloth 中"运行时模拟器"职责对齐——它是一个独立模块（与 ChaosClothAsset 解耦），
// 内部维护 PT 上的"求解器" + "Cloth/Collider/Solver" 等核心数据结构。AircraftLab 中我们仅需要
// 一个多旋翼版本的 FAircraftSimulationSolver 骨架；具体飞控算法（串级 PID + 控制分配 + 电机
// 一阶滞后）在 FAircraftSimulationProxy 内实现，这一层只承担"求解器寿命与配置"。
//
// Phase 1 阶段：仅给出最小骨架声明 + 默认实现，保证编译；Phase 4 中如果飞控逻辑需要扩展为多机
// 协同（编队/集群）求解，可在此模块内补充。

#pragma once

#include "CoreMinimal.h"
#include "Templates/SharedPointer.h"

struct FAircraftSimulationModel;

/**
 * 多旋翼仿真求解器
 *
 * 与 FClothingSimulation/FClothingSimulationCommonSimulator 同位：承载"全局求解参数 + 时间步"
 * 等运行时配置，可被多个 FAircraftSimulationProxy 共享（用于编队飞行的统一节拍）。
 */
class AIRCRAFT_API FAircraftSimulationSolver
{
public:
	FAircraftSimulationSolver();
	~FAircraftSimulationSolver();

	FAircraftSimulationSolver(const FAircraftSimulationSolver&) = delete;
	FAircraftSimulationSolver(FAircraftSimulationSolver&&) = delete;
	FAircraftSimulationSolver& operator=(const FAircraftSimulationSolver&) = delete;
	FAircraftSimulationSolver& operator=(FAircraftSimulationSolver&&) = delete;

	void SetGravity(const FVector& InGravity) { Gravity = InGravity; }
	const FVector& GetGravity() const { return Gravity; }

	void SetSimulationModel(const TSharedPtr<const FAircraftSimulationModel>& InModel) { SimulationModel = InModel; }
	const TSharedPtr<const FAircraftSimulationModel>& GetSimulationModel() const { return SimulationModel; }

	void Reset();

private:
	FVector Gravity = FVector(0.f, 0.f, -980.f);
	TSharedPtr<const FAircraftSimulationModel> SimulationModel;
};
