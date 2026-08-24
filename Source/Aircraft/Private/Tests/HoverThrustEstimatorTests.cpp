// 悬停推力 EKF 语义测试：收敛、门限拒绝、禁用回退、垂直通道基准替换。

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/ControlAllocator.h"
#include "Aircraft/HoverThrustEstimator.h"

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FAircraftFlightControllerRuntimeConfig MakeEstimatorTestConfig()
	{
		FAircraftFlightControllerRuntimeConfig Config;
		Config.HoverCollectiveCommand = 0.5f;
		Config.HoverThrustEstimator.bEnabled = true;
		return Config;
	}

	/** 稳态悬停加速度（cm/s²）：thrust 略高于真实悬停推力时缓慢下沉，略低时缓慢上浮。 */
	float SimulatedAccZCmPerSecSq(float TrueHoverThrust, float AppliedThrust, float GravityCmPerSecSq)
	{
		return (GravityCmPerSecSq * AppliedThrust / TrueHoverThrust - GravityCmPerSecSq);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftHoverThrustEstimatorConvergesTest,
	"AircraftLab.FlightControl.HoverThrustEstimator.Converges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftHoverThrustEstimatorConvergesTest::RunTest(const FString& Parameters)
{
	// 机体真实悬停推力是 0.7（载荷加重），初始估计从静态配置 0.5 出发
	constexpr float TrueHover = 0.7f;
	constexpr float GravityCm = 980.0f;
	const FAircraftFlightControllerRuntimeConfig Config = MakeEstimatorTestConfig();

	FAircraftFlightControlSolver Solver;
	Solver.HoverThrustEstimator.Configure(Config.HoverThrustEstimator, Config.HoverCollectiveCommand);

	// 先施加 0.5 总距：真实机体缓慢下沉，EKF 应把估计推向 0.7
	for (int32 Step = 0; Step < 2000; ++Step)
	{
		const float AccZ = SimulatedAccZCmPerSecSq(TrueHover, 0.5f, GravityCm);
		Solver.UpdateHoverThrustEstimate(Config, 1.0f / 60.0f, AccZ, 0.5f, GravityCm);
	}
	TestTrue(TEXT("Estimate converges toward the true hover thrust"),
		FMath::Abs(Solver.GetEffectiveHoverCollectiveCommand(Config) - TrueHover) < 0.05f);
	TestTrue(TEXT("Estimate stays inside the configured clamp"),
		Solver.GetEffectiveHoverCollectiveCommand(Config) <= Config.HoverThrustEstimator.MaxHoverThrust);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftHoverThrustEstimatorGateRejectsTransientTest,
	"AircraftLab.FlightControl.HoverThrustEstimator.GateRejectsTransient",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftHoverThrustEstimatorGateRejectsTransientTest::RunTest(const FString& Parameters)
{
	constexpr float GravityCm = 980.0f;
	const FAircraftFlightControllerRuntimeConfig Config = MakeEstimatorTestConfig();

	FAircraftFlightControlSolver Solver;
	Solver.HoverThrustEstimator.Configure(Config.HoverThrustEstimator, Config.HoverCollectiveCommand);

	// 与真实悬停一致：估计不应漂移
	for (int32 Step = 0; Step < 100; ++Step)
	{
		Solver.UpdateHoverThrustEstimate(Config, 1.0f / 60.0f, 0.0f, 0.5f, GravityCm);
	}
	const float Before = Solver.GetEffectiveHoverCollectiveCommand(Config);

	// 碰撞级瞬态加速度（远超 χ² 门限）：估计必须被拒绝、保持不变
	Solver.UpdateHoverThrustEstimate(Config, 1.0f / 60.0f, 50000.0f, 0.5f, GravityCm);
	TestEqual(TEXT("Transient acceleration outside the gate does not move the estimate"),
		Solver.GetEffectiveHoverCollectiveCommand(Config), Before);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftHoverThrustEstimatorDisabledFallsBackTest,
	"AircraftLab.FlightControl.HoverThrustEstimator.DisabledFallsBack",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftHoverThrustEstimatorDisabledFallsBackTest::RunTest(const FString& Parameters)
{
	constexpr float GravityCm = 980.0f;
	FAircraftFlightControllerRuntimeConfig Config = MakeEstimatorTestConfig();
	Config.HoverThrustEstimator.bEnabled = false;

	FAircraftFlightControlSolver Solver;
	Solver.HoverThrustEstimator.Configure(Config.HoverThrustEstimator, Config.HoverCollectiveCommand);

	// 持续的失配加速度也不得改变垂直通道基准（禁用 = 静态配置值）
	for (int32 Step = 0; Step < 500; ++Step)
	{
		Solver.UpdateHoverThrustEstimate(Config, 1.0f / 60.0f,
			SimulatedAccZCmPerSecSq(0.7f, 0.5f, GravityCm), 0.5f, GravityCm);
	}
	TestEqual(TEXT("Disabled estimator keeps the static hover collective"),
		Solver.GetEffectiveHoverCollectiveCommand(Config), Config.HoverCollectiveCommand);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FAircraftVerticalControlUsesEstimatedHoverTest,
	"AircraftLab.FlightControl.HoverThrustEstimator.VerticalControlUsesEstimate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FAircraftVerticalControlUsesEstimatedHoverTest::RunTest(const FString& Parameters)
{
	// 手动悬停路径：估计基准 0.6 时，零误差 PID 输出的总距必须落在 0.6 而非静态 0.5
	constexpr float GravityCm = 980.0f;
	const FAircraftFlightControllerRuntimeConfig Config = MakeEstimatorTestConfig();

	FAircraftFlightControlSolver Solver;
	FAircraftHoverThrustEstimatorConfig EstimatorConfig = Config.HoverThrustEstimator;
	EstimatorConfig.MinHoverThrust = 0.59f;
	EstimatorConfig.MaxHoverThrust = 0.61f;
	Solver.HoverThrustEstimator.Configure(EstimatorConfig, 0.6f);
	// 一次更新让估计进入"已初始化"状态（钳位范围内输入不改变 0.6）
	Solver.UpdateHoverThrustEstimate(Config, 1.0f / 60.0f, 0.0f, 0.6f, GravityCm);

	FAircraftFlightControlRuntimeState Runtime;
	Runtime.EstimatedState.State.PositionCm = FVector(0.0f, 0.0f, 1000.0f);
	Runtime.EstimatedState.State.VelocityCmPerSec = FVector::ZeroVector;
	Runtime.EstimatedState.State.AccelerationWorldCmPerSecSq = FVector::ZeroVector;
	Runtime.HoldTargets.HeldAltitudeCm = 1000.0f;
	Runtime.HoldTargets.bAltitudeHoldInitialized = true;
	Runtime.HoldTargets.bYawHoldInitialized = true;
	Runtime.AttitudeMode = EAircraftAttitudeMode::Angle;

	FAircraftPhysicsCache PhysicsCache;
	PhysicsCache.GravityMagnitudeCmPerSecSq = GravityCm;
	PhysicsCache.LinearDampingPerSecond = FVector::ZeroVector;

	FAircraftModeCapabilities Capabilities;
	Capabilities.CanHoldAltitude = true;
	Capabilities.CanUseVelocityControl = true;
	Capabilities.CanUsePositionControl = true;

	FAircraftManualCommand ManualCommand;
	FAircraftTrajectoryReference TrajectoryReference;
	FAircraftControlAllocator Allocator;

	FAircraftFlightControlSolverContext Context{
		Runtime, PhysicsCache, Capabilities, Config, ManualCommand,
		TrajectoryReference, Allocator, false };

	float OutDesiredVerticalVelocity = 0.0f;
	const float Collective = Solver.ComputeVerticalControl(
		Context, 1.0f / 60.0f, OutDesiredVerticalVelocity);
	TestTrue(TEXT("Hovering collective follows the estimated baseline (0.6), not the static config (0.5)"),
		FMath::Abs(Collective - 0.6f) < 0.01f);
	return true;
}

#endif
