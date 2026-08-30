//
// 职责：飞控运行时数据结构 + 仿真代理类（线程间数据中转 / 飞控算法执行体）。
// FAircraftSimulationProxy 与 ChaosCloth 的 FClothSimulationProxy 一一对应：
//   * GameThread API 写入双缓冲输入（摇杆 / 四级设定值 / Autopilot 注入 / 旋翼效率）；
//   * PhysicsThread API 在 AsyncPhysicsTickComponent 路径下消费输入，运行串级 PID/分配/电机；
//   * 通过 FChaosEngineInterface::Add*_AssumesLocked 把结果作用到 Chaos 刚体。
//
// 控制律核心（PID/求解器/分配器/旋翼模型/执行器效能）位于 Aircraft 求解器模块
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
#include "Aircraft/RotorEffectivenessManager.h"
#include "AircraftAutopilot/AircraftTrajectoryRuntime.h"
#include "AircraftRuntimeInterface/AircraftMovementIntent.h"

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
 *     → 阻尼伪逆控制分配（含效能感知与饱和回传）→ 电机一阶滞后 → Chaos 力/扭矩注入。
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

	/** 新建机体或显式重置：读取资产初始状态并在下一物理子步重建运行时。 */
	void Initialize_GameThread();
	/** LOD/驱动切换：只替换后端配置，保留生命周期、意图进度与旋翼效率。 */
	void ReconfigureForLod_GameThread();

	//~ Begin GameThread API
	void SetPilotInput_GameThread(const FAircraftPilotInput& InPilotInput);
	void SetLowLevelTargets_GameThread(const FAircraftLowLevelControlTargets& InTargets);
	void SetFlightMode_GameThread(EAircraftFlightMode InMode);
	void SetArmRequest_GameThread(bool bArm);
	void SetEmergencyStop_GameThread(bool bStop);
	void SetControllerEnabled_GameThread(bool bEnabled);
	bool IsControllerEnabled_GameThread() const;
	void SetGravity_GameThread(float GravityCmPerSecSq);

	void SetMovementIntent_GameThread(const FAircraftMovementIntent& Intent,
		FAircraftMovementIntentHandle Handle, uint64 Revision);
	void ClearMovementIntent_GameThread(uint64 Revision);
	void GetTrajectoryReference_GameThread(FAircraftTrajectoryReference& OutReference) const;
	void GetAutopilotDiagnostics_GameThread(FAircraftAutopilotDiagnostics& OutDiagnostics) const;
	bool GetMotionPlan_GameThread(TArray<FAircraftMotionPlanSample>& OutSamples,
		float& OutDurationSeconds, float& OutLengthCm, uint64& OutPlanRevision) const;
	void TickKinematicTrajectory_GameThread(float DeltaTime, double TimeSeconds,
		const FTransform& BodyTransform, const FVector& VelocityCmPerSec,
		const FVector& AngularVelocityWorldRadPerSec,
		const FAircraftSimulationLodModel& Model);
	void SetSimulationState_GameThread(bool bEnabled, bool bSuspended);
	bool IsControlExecutionAllowed_GameThread() const;
	void InvalidateTrajectoryReference_GameThread();

	/** 设置指定旋翼的执行器效能；0 表示无输出，1 表示完整输出。 */
	bool SetRotorEffectiveness_GameThread(FName RotorName, float Effectiveness);

	void GetEstimatedState_GameThread(FAircraftEstimatedState& OutState) const;
	void GetControlOutput_GameThread(FAircraftFlightControlOutput& OutOutput) const;
	/** 替代驱动后端（约束/运动学，GT 执行）写回估计状态，覆盖 PT 输出槽。 */
	void SetEstimatedStateOverride_GameThread(const FAircraftEstimatedState& InState);
	EAircraftArmState GetArmState_GameThread() const;
	EAircraftFlightMode GetFlightMode_GameThread() const;
	float GetCollectiveThrustCommand_GameThread() const;

	void GetControlAuthorityInfo_GameThread(FAircraftControlAuthorityInfo& OutInfo) const;
	//~ End GameThread API

	//~ Begin PhysicsThread API
	/**
	 * 物理线程子步入口。AsyncPhysicsTickComponent 路径下 DeltaTime 是物理子步长（恒定高频），
	 * FlightController 在此执行 MPCC、PID、分配与旋翼；PhysicsConstraint 在此执行
	 * 确定性轨迹进度、可选空气动力和显式姿态扭矩。Kinematic 不进入本函数。
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
	/** 按 Chaos 当前真实质心展开旋翼分配描述，并复位全部 PT 控制状态。 */
	void RebuildRotorDescriptors_PhysicsThread(const FVector& CenterOfMassBodyCm);
	void QueueConfiguration_GameThread(bool bResetRuntime);
	void RefreshControlAuthority_PhysicsThread(
		const FAircraftFlightControllerRuntimeConfig& Config);
	void ApplyPendingConfiguration_PhysicsThread();
	/** 由飞行模式推导能力缓存与姿态模式。 */
	void UpdateModeCapabilities(EAircraftFlightMode Mode);
	/** 按控制台开关限频输出权威飞控同口径的运行诊断。 */
	void MaybeEmitDebugLog_PhysicsThread(
		float DeltaTime,
		const FAircraftPilotInput& Pilot,
		const FAircraftManualCommand& ManualCommand,
		float CollectiveCommand,
		float DesiredVerticalVelocityCmPerSec,
		const FRotator& DesiredAttitude,
		const FVector& DesiredBodyRatesDegPerSec,
		const FVector& AxisCommands,
		const FAircraftTrajectoryReference& TrajectoryReference);

	const UAircraftComponent& AircraftComponent;

	TSharedPtr<const FAircraftSimulationModel> SimulationModel;
	const FAircraftSimulationLodModel* ActiveLodModel = nullptr;
	int32 ActiveLodIndex = INDEX_NONE;
	EAircraftSimulationDriveMode ActiveDriveMode = EAircraftSimulationDriveMode::FlightController;
	FString AircraftOwnerName;

	/* GT → PT 双缓冲 */
	mutable FCriticalSection InputCriticalSection;
	FAircraftPilotInput PendingPilotInput;
	FAircraftLowLevelControlTargets PendingLowLevelTargets;
	FAircraftMovementIntent PendingMovementIntent;
	FAircraftMovementIntentHandle PendingMovementIntentHandle;
	uint64 PendingMovementIntentRevision = 0;
	bool bPendingMovementIntentActive = false;
	TSharedPtr<const FAircraftSimulationModel> PendingSimulationModel;
	int32 PendingLodIndex = INDEX_NONE;
	EAircraftSimulationDriveMode PendingDriveMode = EAircraftSimulationDriveMode::FlightController;
	bool bPendingConfiguration = false;
	bool bPendingRuntimeReset = false;
	bool bArmRequest = true;
	bool bEmergencyStop = false;
	TMap<FName, float> PendingRotorEffectivenessByName;
	uint64 PendingRotorEffectivenessRevision = 1;
	std::atomic<uint8> PendingFlightMode{ static_cast<uint8>(EAircraftFlightMode::PositionHold) };
	std::atomic<bool> bPendingControllerReset{ false };
	std::atomic<bool> bSimulationEnabled{ true };
	std::atomic<bool> bSimulationSuspended{ false };
	std::atomic<bool> bControllerEnabled{ true };

	/* PT → GT 输出缓冲 */
	mutable FCriticalSection OutputCriticalSection;
	FAircraftEstimatedState LatestEstimated;
	FAircraftFlightControlOutput LatestControlOutput;
	FAircraftControlAuthorityInfo LatestAuthorityInfo;
	FAircraftTrajectoryReference LatestTrajectoryReference;
	FAircraftAutopilotDiagnostics LatestAutopilotDiagnostics;
	TArray<FAircraftMotionPlanSample> LatestMotionPlanSamples;
	float LatestMotionPlanDurationSeconds = 0.0f;
	float LatestMotionPlanLengthCm = 0.0f;
	uint64 LatestMotionPlanRevision = 0;
	std::atomic<uint8> CurrentArmState{ static_cast<uint8>(EAircraftArmState::Armed) };
	std::atomic<uint8> CurrentFlightMode{ static_cast<uint8>(EAircraftFlightMode::PositionHold) };
	std::atomic<float> CurrentCollectiveThrustCommand{ 0.0f };

	std::atomic<FBodyInstance*> AircraftBodyInstance{ nullptr };

	std::atomic<float> GravityMagnitudeCmPerSecSq{ 980.0f };

	/* ---- PT 内部状态（只在 PT 上访问，不需要锁）---- */

	/** 级联控制解算器（PID 状态 + 参考模型状态）。 */
	FAircraftFlightControlSolver ControlSolver;
	/** 控制分配器（缓存/诊断/饱和回传）。 */
	FAircraftControlAllocator ControlAllocator;
	/** 旋翼执行器效能与剩余控制权限。 */
	FAircraftRotorEffectivenessManager RotorEffectivenessManager;
	uint64 AppliedRotorEffectivenessRevision = 0;
	/** 飞控运行状态（估计/输出/保持目标/模式）。 */
	FAircraftFlightControlRuntimeState Runtime;
	/** 物理缓存（刚体真值快照）。 */
	FAircraftPhysicsCache PhysicsCache;
	/** 当前分配矩阵使用的真实 Chaos 质心。 */
	FVector RotorDescriptorCenterOfMassBodyCm = FVector::ZeroVector;
	bool bHasRotorDescriptorCenterOfMass = false;
	/** 模式能力缓存。 */
	FAircraftModeCapabilities ModeCapabilities;
	FAircraftTrajectoryRuntime TrajectoryRuntime;
	uint64 ActiveMovementIntentRevision = 0;
	int64 ActiveMovementIntentId = 0;
	std::atomic<uint64> VehicleStateSequence{ 0 };
	std::atomic<bool> bTrajectoryRebindRequested{ false };
	bool bMovementIntentActive = false;
	float NativeLinearDamping = 0.0f;
	float NativeAngularDamping = 0.0f;
	bool bNativeDampingCaptured = false;
	bool bExplicitAerodynamicsApplied = false;
	/* 单旋翼运行时状态（与 SimulationModel.Rotors 一一对应，索引一致） */
	TArray<FAircraftRotorRuntimeState> RotorStates;
	bool bResetRotorRuntimeOnNextRebuild = true;

	/* 调试状态仅由 PT 访问。 */
	float DebugLogAccumulatorSeconds = 0.0f;
	bool bDebugConfigurationPending = true;
	float DriveGateDebugLogAccumulatorSeconds = 0.0f;
};
