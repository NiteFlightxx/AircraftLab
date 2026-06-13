// 对齐 ChaosCloth 的运行时仿真器实现。
// Phase 1 阶段仅提供构造/析构与最小数据成员；Phase 4 在多机协同需求出现时补充。

#include "Aircraft/AircraftSimulationSolver.h"

FAircraftSimulationSolver::FAircraftSimulationSolver() = default;
FAircraftSimulationSolver::~FAircraftSimulationSolver() = default;

void FAircraftSimulationSolver::Reset()
{
	SimulationModel.Reset();
	Gravity = FVector(0.f, 0.f, -980.f);
}
