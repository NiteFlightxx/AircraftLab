// 对齐 ChaosCloth 的运行时仿真器实现。

#include "Aircraft/AircraftSimulationSolver.h"

FAircraftSimulationSolver::FAircraftSimulationSolver() = default;
FAircraftSimulationSolver::~FAircraftSimulationSolver() = default;

void FAircraftSimulationSolver::Reset()
{
	SimulationModel.Reset();
	Gravity = FVector(0.f, 0.f, -980.f);
}
