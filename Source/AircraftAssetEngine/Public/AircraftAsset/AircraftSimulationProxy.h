//
// 职责：飞控运行时数据结构 + 仿真代理类（线程间数据中转 / 飞控算法执行体）。
// FAircraftSimulationProxy 与 ChaosCloth 的 FClothSimulationProxy 一一对应：
//   * GameThread API 写入双缓冲输入（摇杆 / 四级设定值 / Autopilot 注入 / 旋翼健康操作）；
//   * PhysicsThread API 在 AsyncPhysicsTickComponent 路径下消费输入，运行串级 PID/分配/电机；
//   * 通过 FChaosEngineInterface::Add*_AssumesLocked 把结果作用到 Chaos 刚体。
//
// 控制律核心（PID/求解器/分配器/旋翼模型/失效管理）位于 Aircraft 求解器模块
// 内部状态全部使用 Aircraft 模块的纯 C++ 类型（PT 零 UObject）。

#pragma once

#include <atomic>

#include "CoreMinimal.h"
#include "Dataflow/Interfaces/DataflowPhysicsSolver.h"
#include "HAL/CriticalSection.h"
#include "Templates/SharedPointer.h"

#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/ControlAllocator.h"
#include "Aircraft/RotorModel.h"
#include "Aircraft/RotorFailureManager.h"
#include "AircraftRuntimeInterface/AutopilotProvider.h"

#include "AircraftAsset/AircraftSimulationModel.h"
#include "AircraftAsset/AircraftSimulationTypes.h"

class UAircraftComponent;
struct FBodyInstance;

/* ===========================================================================
 * =========================================================================== */

/**
 * 多旋翼仿真代理
 *
 * 与 FClothSimulationProxy 同位：
 *   - GameThread → PhysicsThread 通过双缓冲（锁 + 原子）交换输入；
 *   - PhysicsThread 内单线程执行：估计状态刷新 → 串级 PID（含参考模型与阻尼前馈）
 *     → 阻尼伪逆控制分配（含失效感知与饱和回传）→ 电机一阶滞后 → Chaos 力/扭矩注入；
 *   - 旋翼失效策略在 PT 评估，触发动作经原子回传 GT 由组件执行。
 */
class AIRCRAFTASSETENGINE_API FAircraftSimulationProxy : public FDataflowPhysicsSolverProxy
{
public:
	explicit FAircraftSimulationProxy(const UAircraftComponent& InAircraftComponent);
	virtual ~FAircraftSimulationProxy() override;

	FAircraftSimulationProxy() = delete;
	FAircraftSimulationProxy(const FAircraftSimulationProxy&) = delete;
	FAircraftSimulationProxy(FAircraftSimulationProxy&&) = delete;
	FAircraftSimulationProxy& operator=(const FAircraftSimulationProxy&) = delete;
	FAircraftSimulationProxy& operator=(FAircraftSimulationProxy&&) = delete;

	/** 捕获最新模型和驱动配置；物理线程在下一子步原子消费并完成重建。 */
	virtual void PostConstructor();

	//~ Begin GameThread API
	void SetPilotInput_GameThread(const FDronePilotInput& InPilotInput);
	void SetTargets_GameThread(const FDroneControlTargets& InTargets);
	void SetFlightMode_GameThread(EAircraftFlightMode InMode);
	void SetArmRequest_GameThread(bool bArm);
	void SetEmergencyStop_GameThread(bool bStop);
	void SetControllerEnabled_GameThread(bool bEnabled);
	bool IsControllerEnabled_GameThread() const;
	void SetGravity_GameThread(float GravityCmPerSecSq);

	/** Autopilot 注入（GT 由组件从 IAutopilotProvider 拉取后写入）。 */
	void SetAutopilotInjection_GameThread(const FAutopilotInjection& InInjection);
	void SetUseAutopilotSetpoint_GameThread(bool bEnabled);
	void SetSimulationState_GameThread(bool bEnabled, bool bSuspended);

	/** 旋翼健康操作（GT 入口；经输入锁排队，PT 在下一子步消费并重建分配缓存）。 */
	void FailRotor_GameThread(FName RotorName);
	void RecoverRotor_GameThread(FName RotorName);
	void SetRotorEffectiveness_GameThread(FName RotorName, float Effectiveness);
	void RecoverAllRotors_GameThread();

	void GetEstimatedState_GameThread(FDroneEstimatedState& OutState) const;
	/** 替代驱动后端（约束/运动学，GT 执行）写回估计状态，覆盖 PT 输出槽。 */
	void SetEstimatedStateOverride_GameThread(const FDroneEstimatedState& InState);
	EAircraftArmState GetArmState_GameThread() const;
	EAircraftFlightMode GetFlightMode_GameThread() const;
	float GetCollectiveThrustCommand_GameThread() const;

	void GetControlAuthorityInfo_GameThread(FAircraftControlAuthorityInfo& OutInfo) const;
	void GetFailurePolicyStatus_GameThread(FAircraftFailurePolicyStatus& OutStatus) const;
	/** GT 消费失效策略触发的动作；无待处理动作时返回 false。 */
	bool ConsumeFailurePolicyAction_GameThread(EAircraftFailurePolicyAction& OutAction);
	/** GT 显式解除失效策略锁存。 */
	void ResetFailurePolicyLatch_GameThread();
	//~ End GameThread API

	//~ Begin PhysicsThread API
	/**
	 * 物理线程子步入口。AsyncPhysicsTickComponent 路径下 DeltaTime 是物理子步长（恒定高频），
	 * 适合直接作为 PID 的离散步长。仅在 FlightController 驱动模式下执行控制循环；
	 */
	void TickPhysicsThread(float DeltaTime, float SimTime, float ForceAccumulationScale = 1.0f);
	//~ End PhysicsThread API

	void SetAircraftBodyInstance(FBodyInstance* BodyInstance);

protected:
	/**
	 * FDataflowPhysicsSolverProxy::AdvanceSolverDatas —— 刻意保持空实现。
	 *
	 * Simulation 图每帧经 AdvancePhysicsSolvers 节点回调本函数，但无人机与布料的分工不同：
	 * 布料把整求解器（含碰撞约束）放在代理内、一帧一次自洽推进；
	 * 无人机机体是世界 Chaos 刚体，由世界物理场景积分并处理场景碰撞 ——
	 * 控制力必须与积分/碰撞同一 pass 注入，因此推进保留在 AsyncPhysicsTickComponent
	 * （物理子步、恒定 Δt）调用 TickPhysicsThread。
	 * Simulation 图在此仅作"注册/调度壳"：让组件注册进 UDataflowSimulationManager，
	 * 获得编辑器 Simulation 场景的 Play/Pause 门控与每帧 GT 桥接。
	 */
	virtual void AdvanceSolverDatas(const float DeltaTime) override
	{
		(void)DeltaTime;
	}

private:
	/** 模型/几何变化后：展开旋翼分配描述并复位全部 PT 控制状态。 */
	void RebuildRotorDescriptors_PhysicsThread();
	void ApplyPendingConfiguration_PhysicsThread();
	/** 由飞行模式推导能力缓存与姿态模式。 */
	void UpdateModeCapabilities(EAircraftFlightMode Mode);
	/** 按控制台开关限频输出权威飞控同口径的运行诊断。 */
	void MaybeEmitDebugLog_PhysicsThread(
		float DeltaTime,
		const FDronePilotInput& Pilot,
		const FAircraftManualCommand& ManualCommand,
		float CollectiveCommand,
		float DesiredVerticalVelocityCmPerSec,
		const FRotator& DesiredAttitude,
		const FVector& DesiredBodyRatesDegPerSec,
		const FVector& AxisCommands);

	const UAircraftComponent& AircraftComponent;

	TSharedPtr<const FAircraftSimulationModel> SimulationModel;
	const FAircraftSimulationLodModel* ActiveLodModel = nullptr;
	int32 ActiveLodIndex = INDEX_NONE;
	EAircraftSimulationDriveMode ActiveDriveMode = EAircraftSimulationDriveMode::FlightController;
	FString AircraftOwnerName;

	/* GT → PT 双缓冲 */
	mutable FCriticalSection InputCriticalSection;
	FDronePilotInput PendingPilotInput;
	FDroneControlTargets PendingTargets;
	FAutopilotInjection PendingAutopilotInjection;
	TSharedPtr<const FAircraftSimulationModel> PendingSimulationModel;
	int32 PendingLodIndex = INDEX_NONE;
	EAircraftSimulationDriveMode PendingDriveMode = EAircraftSimulationDriveMode::None;
	bool bPendingConfiguration = false;
	bool bArmRequest = true;
	bool bEmergencyStop = false;
	bool bRecoverAllRotors = false;
	std::atomic<uint8> PendingFlightMode{ static_cast<uint8>(EAircraftFlightMode::PositionHold) };
	std::atomic<bool> bUseAutopilotSetpoint{ false };
	std::atomic<bool> bPendingControllerReset{ false };
	std::atomic<bool> bSimulationEnabled{ true };
	std::atomic<bool> bSimulationSuspended{ false };
	std::atomic<bool> bControllerEnabled{ true };

	/** 待处理的旋翼健康操作（GT 写、PT 取）。 */
	struct FPendingRotorHealthOp
	{
		FName RotorName = NAME_None;
		/** 0=Fail 1=Recover 2=SetEffectiveness */
		uint8 Op = 0;
		float Effectiveness = 1.0f;
	};
	TArray<FPendingRotorHealthOp> PendingRotorHealthOps;

	/* PT → GT 输出缓冲 */
	mutable FCriticalSection OutputCriticalSection;
	FDroneEstimatedState LatestEstimated;
	FAircraftControlAuthorityInfo LatestAuthorityInfo;
	FAircraftFailurePolicyStatus LatestPolicyStatus;
	std::atomic<uint8> CurrentArmState{ static_cast<uint8>(EAircraftArmState::Armed) };
	std::atomic<uint8> CurrentFlightMode{ static_cast<uint8>(EAircraftFlightMode::PositionHold) };
	std::atomic<float> CurrentCollectiveThrustCommand{ 0.0f };
	std::atomic<uint8> PendingFailureAction{ static_cast<uint8>(EAircraftFailurePolicyAction::WarningOnly) };
	std::atomic<bool> bFailureActionPending{ false };
	std::atomic<bool> bPendingPolicyLatchReset{ false };

	std::atomic<FBodyInstance*> AircraftBodyInstance{ nullptr };

	std::atomic<float> GravityMagnitudeCmPerSecSq{ 980.0f };

	/* ---- PT 内部状态（只在 PT 上访问，不需要锁）---- */

	/** 级联控制解算器（PID 状态 + 参考模型状态）。 */
	FAircraftFlightControlSolver ControlSolver;
	/** 控制分配器（缓存/诊断/饱和回传）。 */
	FAircraftControlAllocator ControlAllocator;
	/** 旋翼失效管理器（健康表/权限评估/失效策略）。 */
	FAircraftRotorFailureManager RotorFailureManager;
	/** 飞控运行状态（估计/输出/保持目标/模式）。 */
	FAircraftFlightControlRuntimeState Runtime;
	/** 物理缓存（刚体真值快照）。 */
	FAircraftPhysicsCache PhysicsCache;
	/** 模式能力缓存。 */
	FAircraftModeCapabilities ModeCapabilities;
	/* 单旋翼运行时状态（与 SimulationModel.Rotors 一一对应，索引一致） */
	TArray<FAircraftRotorRuntimeState> RotorStates;

	/* 调试状态仅由 PT 访问。 */
	float DebugLogAccumulatorSeconds = 0.0f;
	FRotator DebugPreviousAttitudeDegrees = FRotator::ZeroRotator;
	bool bHasPreviousDebugSample = false;
	bool bDebugConfigurationPending = true;
};
